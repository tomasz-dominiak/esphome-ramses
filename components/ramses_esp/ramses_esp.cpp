#include "ramses_esp.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"  // App.feed_wdt() w start_flood_tx()
#include "esp_rom_sys.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>

static const char *const TAG = "ramses_esp";

namespace esphome {
namespace ramses_esp {

void RamsesESPComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up RAMSES ESP component...");

  this->radio_mutex_ = xSemaphoreCreateMutex();
  this->rx_msg_queue_ = xQueueCreate(16, sizeof(RamsesMessage));
  this->tx_msg_queue_ = xQueueCreate(8, sizeof(RamsesMessage));

  if (!this->cc1101_.init(SPI2_HOST, this->sck_pin_, this->mosi_pin_, this->miso_pin_, this->cs_pin_)) {
    ESP_LOGE(TAG, "Failed to initialize CC1101 transceiver!");
    this->mark_failed();
    return;
  }

  if (!this->frame_handler_.init(this->uart_num_, this->gdo0_pin_, this->gdo2_pin_, &this->cc1101_)) {
    ESP_LOGE(TAG, "Failed to initialize RAMSES frame handler!");
    this->mark_failed();
    return;
  }

  this->frame_handler_.set_on_message_callback([this](const RamsesMessage &msg) {
    if (this->rx_msg_queue_ != nullptr) {
      xQueueSend(this->rx_msg_queue_, &msg, 0);
    }
  });

  xTaskCreatePinnedToCore(
      RamsesESPComponent::radio_task_trampoline,
      "ramses_radio",
      4096,
      this,
      10,
      &this->radio_task_handle_,
      0
  );

  this->start_tcp_server();
}

void RamsesESPComponent::radio_task_trampoline(void *arg) {
  reinterpret_cast<RamsesESPComponent *>(arg)->radio_task();
}

void RamsesESPComponent::radio_task() {
  ESP_LOGI(TAG, "RAMSES Radio task started");
  while (true) {
    if (!this->paused_) {
      if (xSemaphoreTake(this->radio_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (!this->paused_) {
          this->frame_handler_.work();
        }
        xSemaphoreGive(this->radio_mutex_);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void RamsesESPComponent::pause() {
  if (this->paused_) return;
  ESP_LOGD(TAG, "Pausing RAMSES Radio for external operation...");
  if (xSemaphoreTake(this->radio_mutex_, pdMS_TO_TICKS(200)) == pdTRUE) {
    this->paused_ = true;
    this->frame_handler_.rx_disable();
    this->cc1101_.enter_idle_mode();
  }
}

void RamsesESPComponent::resume() {
  if (!this->paused_) return;
  ESP_LOGD(TAG, "Resuming RAMSES Radio reception...");
  this->cc1101_.apply_ramses_config();
  this->frame_handler_.rx_enable();
  this->paused_ = false;
  xSemaphoreGive(this->radio_mutex_);
}

void RamsesESPComponent::start_tcp_server() {
  this->server_fd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (this->server_fd_ < 0) {
    ESP_LOGE(TAG, "Unable to create TCP socket: errno %d", errno);
    return;
  }

  int opt = 1;
  setsockopt(this->server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  fcntl(this->server_fd_, F_SETFL, O_NONBLOCK);

  struct sockaddr_in dest_addr;
  dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(this->port_);

  int err = bind(this->server_fd_, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  if (err != 0) {
    ESP_LOGE(TAG, "Socket unable to bind on port %u: errno %d", this->port_, errno);
    close(this->server_fd_);
    this->server_fd_ = -1;
    return;
  }

  err = listen(this->server_fd_, 4);
  if (err != 0) {
    ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
    close(this->server_fd_);
    this->server_fd_ = -1;
    return;
  }

  ESP_LOGI(TAG, "RAMSES HGI80 TCP Server listening on port %u", this->port_);
}

void RamsesESPComponent::loop() {
  // 1. Drain decoded incoming RAMSES messages from Radio queue
  RamsesMessage rx_msg;
  while (this->rx_msg_queue_ != nullptr && xQueueReceive(this->rx_msg_queue_, &rx_msg, 0) == pdTRUE) {
    std::string hgi80 = rx_msg.to_hgi80();
    this->broadcast_hgi80(hgi80);
    for (auto &cb : this->on_message_callbacks_) {
      cb(hgi80);
    }
  }

  // 2. Accept and manage TCP clients
  this->handle_tcp_clients();

  // 3. Process outbound RAMSES messages
  if (!this->paused_) {
    this->process_tx_queue();
  }
}

void RamsesESPComponent::handle_tcp_clients() {
  if (this->server_fd_ < 0) return;

  // Accept new clients
  struct sockaddr_in source_addr;
  socklen_t addr_len = sizeof(source_addr);
  int client_fd = accept(this->server_fd_, (struct sockaddr *)&source_addr, &addr_len);
  if (client_fd >= 0) {
    fcntl(client_fd, F_SETFL, O_NONBLOCK);
    this->client_fds_.push_back(client_fd);
    ESP_LOGI(TAG, "TCP Client connected from %s (Total clients: %d)",
             inet_ntoa(source_addr.sin_addr), (int)this->client_fds_.size());
  }

  // Read data from existing clients
  for (auto it = this->client_fds_.begin(); it != this->client_fds_.end();) {
    int fd = *it;
    char rx_buffer[256];
    int len = recv(fd, rx_buffer, sizeof(rx_buffer) - 1, 0);
    if (len > 0) {
      rx_buffer[len] = '\0';
      std::string line(rx_buffer);
      // Remove trailing CR/LF
      line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
      line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
      if (!line.empty()) {
        ESP_LOGI(TAG, "TCP Rx Command: %s", line.c_str());
        this->send_hgi80_command(line);
      }
      ++it;
    } else if (len == 0 || (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
      ESP_LOGI(TAG, "TCP Client disconnected");
      close(fd);
      it = this->client_fds_.erase(it);
    } else {
      ++it;
    }
  }
}

void RamsesESPComponent::broadcast_hgi80(const std::string &hgi80) {
  std::string line = hgi80 + "\r\n";
  for (auto it = this->client_fds_.begin(); it != this->client_fds_.end();) {
    int fd = *it;
    int sent = send(fd, line.c_str(), line.length(), 0);
    if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      close(fd);
      it = this->client_fds_.erase(it);
    } else {
      ++it;
    }
  }
}

bool RamsesESPComponent::send_hgi80_command(const std::string &cmd) {
  RamsesMessage msg;
  if (!msg.from_hgi80(cmd)) {
    ESP_LOGW(TAG, "Invalid HGI80 command format: %s", cmd.c_str());
    return false;
  }

  if (this->tx_msg_queue_ != nullptr) {
    return xQueueSend(this->tx_msg_queue_, &msg, pdMS_TO_TICKS(100)) == pdTRUE;
  }
  return false;
}

// Pełny cykl nadawania jednej ramki: FIFO, STX, wait_tx_complete, powrót do
// RX. Wołający musi już trzymać radio_mutex_. Współdzielone przez normalną
// kolejkę TX (process_tx_queue) i akcję diagnostyczną freq_sweep, żeby obie
// ścieżki nadawały identycznie.
bool RamsesESPComponent::transmit_message_locked(const RamsesMessage &tx_msg) {
  ESP_LOGI(TAG, "Transmitting RAMSES packet: %s", tx_msg.to_hgi80().c_str());

  this->frame_handler_.rx_disable();
  this->cc1101_.enter_idle_mode();

  std::vector<uint8_t> raw_frame = tx_msg.to_raw_frame();
  this->cc1101_.prepare_tx_mode();

  // Ręczna korekta częstotliwości nadawania (patrz set_tx_freq_correction) —
  // RX koryguje rozstrojenie kwarcu przez AFC, TX nie ma takiego mechanizmu,
  // więc bez tego nadajemy systematycznie obok środka pasma odbiornika.
  // Przywracane do 0x00 po transmisji, bo RX zawsze na tym polega.
  if (this->tx_freq_correction_ != 0) {
    this->cc1101_.write_reg(CC_FSCTRL0, static_cast<uint8_t>(this->tx_freq_correction_));
  }

  // Napełniamy TX FIFO PRZED strobem STX — inaczej puste FIFO wywołuje
  // TXFIFO_UNDERFLOW w czasie jednego bajtu i ramka nigdy nie jest wysłana.
  size_t sent = 0;
  size_t preload = std::min<size_t>(64, raw_frame.size());
  for (; sent < preload; sent++) {
    this->cc1101_.write_fifo(raw_frame[sent]);
  }
  uint8_t txbytes_preload = this->cc1101_.read_txbytes();
  ESP_LOGD(TAG, "TXBYTES przed STX (preload %u/%u B): 0x%02X",
           (unsigned)preload, (unsigned)raw_frame.size(), txbytes_preload);

  uint32_t tx_cycle_start_us = micros();
  this->cc1101_.start_tx();

  uint32_t start_ms = millis();
  bool mid_frame_underflow = false;
  while (sent < raw_frame.size() && (millis() - start_ms < 500)) {
    // Pelny bajt statusu z KAZDEGO zapisu SPI do FIFO, nie tylko wolne
    // miejsce: bity 6:4 to STATE automatu radia. Zapis SPI do rejestru FIFO
    // "udaje sie" niezaleznie od tego, czy chip faktycznie nadaje, wiec sam
    // fakt sent++ nie dowodzi, ze bajt poszedl w eter.
    uint8_t status = this->cc1101_.write_fifo_status(raw_frame[sent++]);
    uint8_t space = status & CC_FIFO_MASK;
    uint8_t state = CC_STATE(status);

    // start_tx() potwierdzil TX zanim tu weszlismy, wiec kazde wypadniecie z
    // TX PRZED wepchaniem calej ramki to realny przedwczesny underflow (np.
    // chwilowe opoznienie SPI/przerwanie oproznilo FIFO szybciej, niz zdazyl
    // dojsc kolejny bajt). Chip sam przeszedl do TX_UNDERFLOW/IDLE, a petla
    // pisala dalej w prozne — w eter poszla ucieta ramka. To twardy dowod,
    // logujemy ERROR z dokladnym sent/rozmiarem i przerywamy napelnianie.
    if (state != CC_STATE_TX) {
      mid_frame_underflow = true;
      ESP_LOGE(TAG, "TX PRZEDWCZESNY underflow: STATE=0x%02X po %u/%u B ramki "
                    "(zapis SPI 'udany', ale radio juz nie nadaje)",
               state, (unsigned) sent, (unsigned) raw_frame.size());
      break;
    }

    // TXFIFO (64 B) przy 38,4 kBd drenuje się w ~208 us/bajt — pełny bufor
    // daje ~13 ms zapasu. Gdy prawie pełny, trzeba tylko poczekać, aż
    // radio zwolni jeden bajt (~208 us), a NIE oddawać CPU schedulerowi:
    // vTaskDelay() tutaj czekał "co najmniej" 1-2 znaczniki (1-2 ms przy
    // domyślnym ticku 1 ms) i przy współbieżnym ruchu Wi-Fi/TCP faktyczny
    // czas potrafił być dłuższy, zjadając margines bufora — stąd
    // TXFIFO_UNDERFLOW w ~12% nadań. Krótkie zajęte oczekiwanie (jak
    // reszta sterownika, patrz cc1101_driver.cpp) usuwa to ryzyko.
    if (space < 4) {
      esp_rom_delay_us(200);
    }
  }

  // Potwierdzenie zdrowej sciezki: cala ramka wepchnieta, a chip ani razu nie
  // wypadl z TX. Logowane raz na "test" (flaga zerowana w start_flood_tx),
  // zeby przy tysiacach ramek floodu nie zasypac logu — patrz DEBUG nizej.
  if (!mid_frame_underflow && sent == raw_frame.size() && !this->tx_state_ok_logged_) {
    this->tx_state_ok_logged_ = true;
    ESP_LOGD(TAG, "TX: cala ramka (%u B) wepchnieta, chip caly czas w TX — "
                  "brak przedwczesnego underflow", (unsigned) raw_frame.size());
  }

  this->cc1101_.fifo_end();
  // Czekamy aż FIFO faktycznie się opróżni zamiast na sztywno 15 ms —
  // dla dłuższych ramek (payload >~40 B) transmisja trwa dłużej niż
  // 15 ms i była ucinana w połowie, zanim urządzenie zdążyło ją
  // odebrać, mimo że echo niżej i tak zgłaszało sukces.
  uint8_t txbytes_final = 0;
  bool ended_by_underflow = false;
  bool tx_ok = this->cc1101_.wait_tx_complete(50, &txbytes_final, &ended_by_underflow);
  // Przedwczesny underflow z petli refill jest rozstrzygajacy: wait_tx_complete
  // moze go wziac za oczekiwane zakonczenie przez underflow, ale ramka byla
  // ucieta, wiec wymuszamy porazke (echo pominiete, wolajacy dostaje false).
  if (mid_frame_underflow) {
    tx_ok = false;
  }
  ESP_LOGD(TAG, "TXBYTES po STX: 0x%02X, underflow=%s, tx_ok=%s",
           txbytes_final, ended_by_underflow ? "tak" : "nie", tx_ok ? "tak" : "nie");
  if (tx_ok) {
    // TXBYTES==0 (albo underflow) oznacza, że modulator pobrał ostatni
    // bajt z FIFO — jego fizyczne wypromieniowanie trwa jeszcze do
    // jednego okresu bajtu (8 bitów / 38 383 Bd = ~208 us). SIDLE tuż
    // po tym ucinałoby ogon ramki, niewidocznie dla nas, a dla
    // odbiorcy jako zła suma kontrolna. 300 us to pełny okres bajtu z
    // zapasem. esp_rom_delay_us, nie vTaskDelay — nie może tego
    // wywłaszczyć scheduler.
    esp_rom_delay_us(300);
  }
  if (!tx_ok) {
    ESP_LOGW(TAG, "TX ucięte, echo pominięte: %s", tx_msg.to_hgi80().c_str());
  } else {
    // Echo the transmitted frame back to TCP clients so ramses_tx sees the
    // expected self-echo and can leave its WantEcho state. Wysyłane
    // dopiero po potwierdzonym opróżnieniu FIFO — inaczej log/ramses_tx
    // widziałby poprawną ramkę nawet gdy w eter poleciał tylko urywek.
    this->broadcast_hgi80(tx_msg.to_hgi80());
  }

  if (this->tx_freq_correction_ != 0) {
    this->cc1101_.write_reg(CC_FSCTRL0, 0x00);
  }

  // Powrót do RX bez przepisywania wszystkich 47 rejestrów + PATABLE:
  // nadawanie zmienia tylko PKTCTRL0 i IOCFG0, a te dwa i tak ustawia
  // enter_rx_mode(). Skraca to okno głuchoty gateway'a po transmisji —
  // urządzenia w sieci RAMSES odpowiadają po 16-21 ms, więc każda
  // dodatkowa milisekunda martwego czasu gubi odpowiedzi.
  this->cc1101_.enter_rx_mode();
  this->frame_handler_.rx_enable();

  uint32_t tx_cycle_us = micros() - tx_cycle_start_us;
  ESP_LOGD(TAG, "Cykl STX -> z powrotem w RX: %lu us", (unsigned long)tx_cycle_us);
  return tx_ok;
}

void RamsesESPComponent::process_tx_queue() {
  RamsesMessage tx_msg;
  if (this->tx_msg_queue_ != nullptr && xQueueReceive(this->tx_msg_queue_, &tx_msg, 0) == pdTRUE) {
    if (xSemaphoreTake(this->radio_mutex_, pdMS_TO_TICKS(200)) == pdTRUE) {
      this->transmit_message_locked(tx_msg);
      xSemaphoreGive(this->radio_mutex_);
    }
  }
}

// Diagnostyka strojenia CC1101: dla tej samej ramki przemiata FSCTRL0
// (korekcja częstotliwości nadawania) od -32 do +32 co 4, nadając po każdej
// zmianie zwykłą ścieżką TX. Po zakończeniu przywraca FSCTRL0 = 0x00.
void RamsesESPComponent::freq_sweep(const std::string &cmd) {
  RamsesMessage msg;
  if (!msg.from_hgi80(cmd)) {
    ESP_LOGW(TAG, "freq_sweep: nieprawidłowy format komendy HGI80: %s", cmd.c_str());
    return;
  }

  if (xSemaphoreTake(this->radio_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
    ESP_LOGW(TAG, "freq_sweep: nie udało się przejąć radia, pomijam sweep");
    return;
  }

  static const int FREQ_SWEEP_MIN = -120;
  static const int FREQ_SWEEP_MAX = 120;
  static const int FREQ_SWEEP_STEP = 8;

  ESP_LOGI(TAG, "SWEEP: start, ramka=%s", msg.to_hgi80().c_str());
  for (int off = FREQ_SWEEP_MIN; off <= FREQ_SWEEP_MAX; off += FREQ_SWEEP_STEP) {
    this->cc1101_.write_reg(CC_FSCTRL0, static_cast<uint8_t>(off));
    ESP_LOGI(TAG, "SWEEP: FSCTRL0=%d (%.1f kHz)", off, off * 1.5869f);
    this->transmit_message_locked(msg);
    // Karmimy watchdog: wolane z lambdy API service blokuje glowny watek na
    // caly czas sweepu (~15 s), a bez tego Task WDT zresetowalby ESP.
    App.feed_wdt();
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  this->cc1101_.write_reg(CC_FSCTRL0, 0x00);
  this->cc1101_.enter_rx_mode();
  this->frame_handler_.rx_enable();
  ESP_LOGI(TAG, "SWEEP: koniec, FSCTRL0 przywrócone do 0x00");

  xSemaphoreGive(this->radio_mutex_);
}

// Test zalewania nadajnika: nadaje w kolko jedna, wbudowana na sztywno ramke
// (22F1 003 000407) tak szybko, jak pozwala normalny cykl STX->RX, przez
// duration_ms. Reuzywa DOKLADNIE tej samej sciezki wysylki co zwykla kolejka
// TX i freq_sweep — transmit_message_locked() — zamiast duplikowac logike
// nadawania. Wolane z lambdy custom API service (patrz example-c6.yaml), wiec
// biegnie na glownym watku loop(); dlatego karmimy watchdog w kazdej iteracji.
void RamsesESPComponent::start_flood_tx(uint32_t duration_ms) {
  // Zabezpieczenie: zla wartosc z API nie moze zablokowac glownej petli
  // (a z nia API/Wi-Fi) na minuty. Rozsadny zakres to 20-30 s.
  if (duration_ms < 1000) duration_ms = 1000;
  if (duration_ms > 60000) {
    ESP_LOGW(TAG, "Flood TX: duration_ms=%lu za duze, ograniczam do 60000",
             (unsigned long) duration_ms);
    duration_ms = 60000;
  }

  RamsesMessage msg;
  const char *frame = "I --- 37:220902 32:148895 --:------ 22F1 003 000407";
  if (!msg.from_hgi80(frame)) {
    ESP_LOGW(TAG, "Flood TX: nieprawidlowa ramka wbudowana, przerywam: %s", frame);
    return;
  }

  if (xSemaphoreTake(this->radio_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
    ESP_LOGW(TAG, "Flood TX: nie udalo sie przejac radia, pomijam");
    return;
  }

  ESP_LOGI(TAG, "Flood TX START: %lu ms, ramka=%s",
           (unsigned long) duration_ms, msg.to_hgi80().c_str());

  // Zerujemy flage, zeby ten przebieg floodu dal dokladnie jedno DEBUG
  // "brak przedwczesnego underflow" (albo ERROR-y, jesli underflow wystapi).
  this->tx_state_ok_logged_ = false;

  uint32_t count = 0;
  uint32_t start_ms = millis();
  uint32_t last_log_ms = start_ms;

  // Bez sztucznego opoznienia miedzy nadaniami — tempo dyktuje sam cykl
  // STX->RX w transmit_message_locked(). Kazda iteracja karmi watchdog,
  // inaczej Task WDT zresetuje ESP przy tak dlugim blokowaniu loop().
  for (uint32_t elapsed = 0; elapsed < duration_ms; elapsed = millis() - start_ms) {
    this->transmit_message_locked(msg);
    count++;
    App.feed_wdt();

    uint32_t now = millis();
    if (now - last_log_ms >= 5000) {
      last_log_ms = now;
      ESP_LOGI(TAG, "Flood TX: %lu / %lu ms, wyslano %lu razy",
               (unsigned long)(now - start_ms), (unsigned long) duration_ms,
               (unsigned long) count);
    }
  }

  // Teardown wykonywany zawsze, niezaleznie jak zakonczyla sie petla, zeby
  // chip nigdy nie zostal "zawieszony" w stanie nadawania. Wyjatki C++ sa w
  // tym buildzie wylaczone, wiec to prosty kod liniowy — a transmit_message_
  // locked() i tak konczy kazdy cykl w RX; tu wymuszamy powrot do nasluchu
  // jeszcze raz i zwalniamy radio, by radio_task/kolejka TX wrocily do pracy.
  this->cc1101_.enter_rx_mode();
  this->frame_handler_.rx_enable();
  xSemaphoreGive(this->radio_mutex_);

  ESP_LOGI(TAG, "Flood TX ZAKONCZONY, lacznie %lu ramek, powrot do normalnej pracy",
           (unsigned long) count);
}

void RamsesESPComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "RAMSES ESP Transceiver & Gateway:");
  ESP_LOGCONFIG(TAG, "  SCK Pin: GPIO%d", this->sck_pin_);
  ESP_LOGCONFIG(TAG, "  MOSI Pin: GPIO%d", this->mosi_pin_);
  ESP_LOGCONFIG(TAG, "  MISO Pin: GPIO%d", this->miso_pin_);
  ESP_LOGCONFIG(TAG, "  CS Pin: GPIO%d", this->cs_pin_);
  ESP_LOGCONFIG(TAG, "  GDO0 Pin (UART RX): GPIO%d", this->gdo0_pin_);
  if (this->gdo2_pin_ != GPIO_NUM_NC) {
    ESP_LOGCONFIG(TAG, "  GDO2 Pin: GPIO%d", this->gdo2_pin_);
  }
  ESP_LOGCONFIG(TAG, "  UART Port: UART%d", this->uart_num_);
  ESP_LOGCONFIG(TAG, "  TCP Server Port: %u", this->port_);

  // Odczyt zwrotny rejestrów CC1101 z układu (nie z tablicy CC_RAMSES_CFG w
  // RAM) — logowany tutaj, a nie z setup(), bo klient API podłącza się
  // dopiero po starcie i tylko blok [C] (dump_config) jest buforowany oraz
  // wysyłany po podłączeniu; ESP_LOGI z fazy setup() nigdy by nie dotarł.
  uint8_t freq2 = this->cc1101_.read_reg(CC_FREQ2);
  uint8_t freq1 = this->cc1101_.read_reg(CC_FREQ1);
  uint8_t freq0 = this->cc1101_.read_reg(CC_FREQ0);
  uint8_t fsctrl1 = this->cc1101_.read_reg(CC_FSCTRL1);
  uint8_t fsctrl0 = this->cc1101_.read_reg(CC_FSCTRL0);
  uint8_t mdmcfg4 = this->cc1101_.read_reg(CC_MDMCFG4);
  uint8_t mdmcfg3 = this->cc1101_.read_reg(CC_MDMCFG3);
  uint8_t mdmcfg2 = this->cc1101_.read_reg(CC_MDMCFG2);
  uint8_t deviatn = this->cc1101_.read_reg(CC_DEVIATN);
  uint8_t mcsm1 = this->cc1101_.read_reg(CC_MCSM1);
  uint8_t mcsm0 = this->cc1101_.read_reg(CC_MCSM0);
  uint8_t foccfg = this->cc1101_.read_reg(CC_FOCCFG);
  uint8_t agcctrl2 = this->cc1101_.read_reg(CC_AGCCTRL2);
  uint8_t agcctrl1 = this->cc1101_.read_reg(CC_AGCCTRL1);
  uint8_t agcctrl0 = this->cc1101_.read_reg(CC_AGCCTRL0);
  uint8_t frend1 = this->cc1101_.read_reg(CC_FREND1);
  uint8_t frend0 = this->cc1101_.read_reg(CC_FREND0);
  uint8_t pktctrl0 = this->cc1101_.read_reg(CC_PKTCTRL0);
  uint8_t iocfg0 = this->cc1101_.read_reg(CC_IOCFG0);
  uint8_t iocfg1 = this->cc1101_.read_reg(CC_IOCFG1);
  uint8_t iocfg2 = this->cc1101_.read_reg(CC_IOCFG2);
  uint8_t patable0 = this->cc1101_.read_reg(CC_PATABLE | CC_BURST);

  uint32_t freq_word = (static_cast<uint32_t>(freq2) << 16) |
                        (static_cast<uint32_t>(freq1) << 8) | freq0;
  double freq_hz = (26000000.0 / 65536.0) * freq_word;

  ESP_LOGCONFIG(TAG, "  Odczyt zwrotny rejestrow z ukladu:");
  ESP_LOGCONFIG(TAG, "  FREQ2/1/0=0x%02X/0x%02X/0x%02X -> f=%.4f MHz",
                freq2, freq1, freq0, freq_hz / 1e6);
  ESP_LOGCONFIG(TAG, "  FSCTRL1=0x%02X FSCTRL0=0x%02X", fsctrl1, fsctrl0);
  ESP_LOGCONFIG(TAG, "  MDMCFG4/3/2=0x%02X/0x%02X/0x%02X", mdmcfg4, mdmcfg3, mdmcfg2);
  ESP_LOGCONFIG(TAG, "  DEVIATN=0x%02X MCSM1=0x%02X MCSM0=0x%02X", deviatn, mcsm1, mcsm0);
  ESP_LOGCONFIG(TAG, "  FOCCFG=0x%02X", foccfg);
  ESP_LOGCONFIG(TAG, "  AGCCTRL2/1/0=0x%02X/0x%02X/0x%02X", agcctrl2, agcctrl1, agcctrl0);
  ESP_LOGCONFIG(TAG, "  FREND1=0x%02X FREND0=0x%02X", frend1, frend0);
  ESP_LOGCONFIG(TAG, "  PKTCTRL0=0x%02X", pktctrl0);
  ESP_LOGCONFIG(TAG, "  IOCFG2/1/0=0x%02X/0x%02X/0x%02X (GDO2/GDO1/GDO0, stan spoczynku)",
                iocfg2, iocfg1, iocfg0);
  ESP_LOGCONFIG(TAG, "  PATABLE[0] (burst)=0x%02X", patable0);
  ESP_LOGCONFIG(TAG, "  BUILD: freq-sweep-service-v3");
}

} // namespace ramses_esp
} // namespace esphome
