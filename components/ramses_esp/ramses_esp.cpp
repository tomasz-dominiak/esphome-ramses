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

  this->check_gdo_wiring();

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

// Pelny cykl nadawania jednej ramki — BUILD diag/uart-tx: async serial przez
// sprzetowy UART ESP32, lustrzanie do RX. Wolajacy musi trzymac radio_mutex_.
//
// Dlaczego nie FIFO: to_raw_frame() zwraca bajty na poziomie UART (te same,
// ktore RX widzi PO zdekodowaniu przez UART ESP32: 8N1, LSB-first). Stara
// sciezka ladowala je do TX FIFO CC1101, ktore nadaje gole 8 bitow MSB-first —
// w eter szedl strumien bez bitow start/stop i z odwrocona kolejnoscia bitow.
// Tutaj CC1101 w trybie async (PKTCTRL0=0x32) moduluje wprost poziom na swoim
// GDO0, a ten poziom generuje UART ESP32 (TX = gdo2_pin z configu, fizycznie
// GDO0 chipa — patrz check_gdo_wiring()), wiec start/stop i kolejnosc bitow
// robi sprzet dokladnie tak, jak po stronie RX.
//
// Zadnego odpytywania TXBYTES/SPI w trakcie nadawania: koniec wykrywa
// uart_wait_tx_done() (przerwanie TX_DONE UART-u, nie polling statusu chipa).
bool RamsesESPComponent::transmit_message_locked(const RamsesMessage &tx_msg) {
  ESP_LOGI(TAG, "Transmitting RAMSES packet (UART async): %s", tx_msg.to_hgi80().c_str());

  if (this->gdo2_pin_ == GPIO_NUM_NC) {
    ESP_LOGE(TAG, "TX UART wymaga gdo2_pin (UART TX -> GDO0 chipa), pomijam: %s",
             tx_msg.to_hgi80().c_str());
    return false;
  }

  std::vector<uint8_t> raw_frame = tx_msg.to_raw_frame();

  this->frame_handler_.rx_disable();
  this->cc1101_.prepare_tx_async_mode();

  // Reczna korekta czestotliwosci nadawania (patrz set_tx_freq_correction);
  // przywracana do 0x00 po transmisji, bo RX zawsze na tym polega.
  if (this->tx_freq_correction_ != 0) {
    this->cc1101_.write_reg(CC_FSCTRL0, static_cast<uint8_t>(this->tx_freq_correction_));
  }

  uint32_t tx_cycle_start_us = micros();
  bool tx_ok = this->cc1101_.start_tx_async();
  size_t written = 0;
  if (tx_ok) {
    // Linia UART TX stoi w spoczynku na 1, wiec przed pierwszym bitem startu
    // CC1101 nadaje po prostu nosnik "1" — nieszkodliwe, jak idle w RX.
    int w = uart_write_bytes(this->uart_num_, raw_frame.data(), raw_frame.size());
    written = w > 0 ? (size_t) w : 0;
    // 10 bitow/bajt przy 38 400 Bd = ~260 us/bajt; 162 B to ~42 ms.
    esp_err_t err = uart_wait_tx_done(this->uart_num_, pdMS_TO_TICKS(200));
    tx_ok = (err == ESP_OK) && written == raw_frame.size();
    // TX_DONE = ostatni bit stopu wyszedl z UART-u; zapas na probkowanie
    // pinu przez modulator CC1101, zanim SIDLE utnie nosnik.
    esp_rom_delay_us(100);
  }

  this->cc1101_.enter_idle_mode();
  uint32_t tx_air_us = micros() - tx_cycle_start_us;

  if (this->tx_freq_correction_ != 0) {
    this->cc1101_.write_reg(CC_FSCTRL0, 0x00);
  }

  // enter_rx_mode() ustawia z powrotem PKTCTRL0=0x32 i IOCFG0=0x2E.
  this->cc1101_.enter_rx_mode();
  this->frame_handler_.rx_enable();

  ESP_LOGD(TAG, "TX UART: %u/%u B, STX -> IDLE %lu us (oczekiwane ~%lu us), tx_ok=%s",
           (unsigned) written, (unsigned) raw_frame.size(), (unsigned long) tx_air_us,
           (unsigned long) (raw_frame.size() * 10UL * 1000000UL / 38400UL), tx_ok ? "tak" : "nie");

  if (!tx_ok) {
    ESP_LOGW(TAG, "TX nieudane, echo pominiete: %s", tx_msg.to_hgi80().c_str());
  } else {
    // Echo do klientow TCP, zeby ramses_tx dostal oczekiwane self-echo.
    this->broadcast_hgi80(tx_msg.to_hgi80());
  }
  return tx_ok;
}

// Autotest okablowania GDO: kazde wyjscie GDO chipa po kolei wymuszamy na
// stale 0 i 1 (IOCFGx = 0x2F / 0x6F, "HW to 0" + INV), trzymajac drugie na
// stalym 1, i czytamy oba piny ESP. Daje pelna mape chip GDOx -> GPIO ESP,
// niezaleznie od nazw w YAML-u. Wolane raz w setup(), zanim ruszy radio_task;
// wynik logowany w dump_config() (logi z setup() nie docieraja do API).
void RamsesESPComponent::check_gdo_wiring() {
  const uint8_t regs[2] = {CC_IOCFG0, CC_IOCFG2};
  const gpio_num_t pins[2] = {this->gdo0_pin_, this->gdo2_pin_};

  this->cc1101_.enter_idle_mode();
  // Pin UART TX na chwile jako wejscie, zeby dalo sie z niego czytac.
  if (this->gdo2_pin_ != GPIO_NUM_NC) {
    gpio_set_direction(this->gdo2_pin_, GPIO_MODE_INPUT);
  }
  for (int g = 0; g < 2; g++) {
    this->cc1101_.write_reg(regs[1 - g], 0x6F);  // drugie GDO: stale 1
    this->cc1101_.write_reg(regs[g], 0x2F);      // badane GDO: stale 0
    esp_rom_delay_us(50);
    int lo[2], hi[2];
    for (int p = 0; p < 2; p++) lo[p] = pins[p] == GPIO_NUM_NC ? -1 : gpio_get_level(pins[p]);
    this->cc1101_.write_reg(regs[g], 0x6F);      // badane GDO: stale 1
    esp_rom_delay_us(50);
    for (int p = 0; p < 2; p++) hi[p] = pins[p] == GPIO_NUM_NC ? -1 : gpio_get_level(pins[p]);
    for (int p = 0; p < 2; p++) {
      this->gdo_wiring_[g][p] = lo[p] < 0 ? -1 : ((lo[p] == 0 && hi[p] == 1) ? 1 : 0);
    }
  }
  this->cc1101_.write_reg(CC_IOCFG0, this->cc1101_.get_default_reg(CC_IOCFG0));
  this->cc1101_.write_reg(CC_IOCFG2, this->cc1101_.get_default_reg(CC_IOCFG2));

  // gpio_set_direction() odpina sygnal UART TXD z matrycy — przywracamy
  // dokladnie to samo przypisanie co RamsesFrameHandler::init().
  uart_set_pin(this->uart_num_, this->gdo2_pin_, this->gdo0_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  this->cc1101_.enter_rx_mode();
  this->frame_handler_.rx_flush();
  this->frame_handler_.rx_enable();
  this->gdo_wiring_checked_ = true;
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

  this->sweep_message_locked(msg);

  xSemaphoreGive(this->radio_mutex_);
}

// Wspolna petla sweepu FSCTRL0. Wolajacy musi trzymac radio_mutex_. Nadaje tu
// samo ramke po kazdym kroku zwykla sciezka TX (transmit_message_locked) i na
// koncu przywraca FSCTRL0=0x00 oraz RX, zeby chip nie zostal rozstrojony.
void RamsesESPComponent::sweep_message_locked(const RamsesMessage &msg) {
  static const int FREQ_SWEEP_MIN = -120;
  static const int FREQ_SWEEP_MAX = 120;
  static const int FREQ_SWEEP_STEP = 8;

  ESP_LOGI(TAG, "SWEEP: start, ramka=%s", msg.to_hgi80().c_str());
  for (int off = FREQ_SWEEP_MIN; off <= FREQ_SWEEP_MAX; off += FREQ_SWEEP_STEP) {
    this->cc1101_.write_reg(CC_FSCTRL0, static_cast<uint8_t>(off));
    ESP_LOGI(TAG, "SWEEP: FSCTRL0=%d (%.1f kHz)", off, off * 1.5869f);
    this->transmit_message_locked(msg);
    // Karmimy watchdog: sweep blokuje glowny watek na caly swoj czas (~15 s),
    // a bez tego Task WDT zresetowalby ESP.
    App.feed_wdt();
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  this->cc1101_.write_reg(CC_FSCTRL0, 0x00);
  this->cc1101_.enter_rx_mode();
  this->frame_handler_.rx_enable();
  ESP_LOGI(TAG, "SWEEP: koniec, FSCTRL0 przywrócone do 0x00");
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
  ESP_LOGCONFIG(TAG, "  BUILD: uart-tx-v1 (TX async przez UART ESP32 -> GDO0 chipa, bez FIFO)");
  if (this->gdo_wiring_checked_) {
    static const char *const gdo_names[2] = {"GDO0 (wejscie danych TX)", "GDO2 (wyjscie danych RX)"};
    static const char *const pin_names[2] = {"gdo0_pin", "gdo2_pin"};
    const gpio_num_t pins[2] = {this->gdo0_pin_, this->gdo2_pin_};
    ESP_LOGCONFIG(TAG, "  Autotest okablowania (chip GDOx -> pin ESP):");
    for (int g = 0; g < 2; g++) {
      for (int p = 0; p < 2; p++) {
        if (this->gdo_wiring_[g][p] < 0) continue;
        ESP_LOGCONFIG(TAG, "    chip %s -> %s (GPIO%d): %s", gdo_names[g], pin_names[p], pins[p],
                      this->gdo_wiring_[g][p] ? "POLACZONE" : "-");
      }
    }
    if (this->gdo_wiring_[0][1] != 1) {
      ESP_LOGE(TAG, "  GDO0 chipa NIE jest na gdo2_pin (UART TX) — TX przez UART nie zadziala!");
    }
    if (this->gdo_wiring_[1][0] != 1) {
      ESP_LOGW(TAG, "  GDO2 chipa nie wykryte na gdo0_pin (UART RX) — sprawdz okablowanie");
    }
  }
  ESP_LOGCONFIG(TAG, "  TX freq correction (FSCTRL0 na czas TX): %d", this->tx_freq_correction_);
}

} // namespace ramses_esp
} // namespace esphome
