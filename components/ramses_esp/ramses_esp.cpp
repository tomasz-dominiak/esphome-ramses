#include "ramses_esp.h"
#include "esphome/core/log.h"
#include "esp_task_wdt.h"
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

// TYMCZASOWE: brama sweepu diagnostycznego w process_tx_queue() — patrz tam.
// Dopasowuje TYLKO ramki 22F1 nadawane z 37:220902 do 32:148895; wszystko
// inne (m.in. sygnaturowy pakiet ramses_cc wysyłany zaraz po connect TCP)
// ma iść normalną, pojedynczą ścieżką TX bez dotykania FSCTRL0.
static bool ramses_addr_matches(const uint8_t addr_bytes[3], uint8_t dev_class, uint32_t id) {
  RamsesAddress addr = RamsesAddress::from_bytes(addr_bytes);
  return addr.is_valid && addr.dev_class == dev_class && addr.id == id;
}

static bool is_freq_sweep_target(const RamsesMessage &msg) {
  if (msg.opcode[0] != 0x22 || msg.opcode[1] != 0xF1) return false;
  if (!(msg.fields & RAMSES_F_ADDR0) || !ramses_addr_matches(msg.addr[0], 37, 220902)) return false;
  bool dst_match = false;
  if (msg.fields & RAMSES_F_ADDR1) dst_match |= ramses_addr_matches(msg.addr[1], 32, 148895);
  if (msg.fields & RAMSES_F_ADDR2) dst_match |= ramses_addr_matches(msg.addr[2], 32, 148895);
  return dst_match;
}

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
bool RamsesESPComponent::transmit_message_locked(const RamsesMessage &tx_msg, bool echo) {
  ESP_LOGI(TAG, "Transmitting RAMSES packet: %s", tx_msg.to_hgi80().c_str());

  this->frame_handler_.rx_disable();
  this->cc1101_.enter_idle_mode();

  std::vector<uint8_t> raw_frame = tx_msg.to_raw_frame();
  this->cc1101_.prepare_tx_mode();

  // Napełniamy TX FIFO PRZED strobem STX — inaczej puste FIFO wywołuje
  // TXFIFO_UNDERFLOW w czasie jednego bajtu i ramka nigdy nie jest wysłana.
  size_t sent = 0;
  size_t preload = std::min<size_t>(64, raw_frame.size());
  for (; sent < preload; sent++) {
    this->cc1101_.write_fifo(raw_frame[sent]);
  }

  uint32_t tx_cycle_start_us = micros();
  this->cc1101_.start_tx();

  uint32_t start_ms = millis();
  while (sent < raw_frame.size() && (millis() - start_ms < 500)) {
    uint8_t space = this->cc1101_.write_fifo(raw_frame[sent++]);
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

  this->cc1101_.fifo_end();
  // Czekamy aż FIFO faktycznie się opróżni zamiast na sztywno 15 ms —
  // dla dłuższych ramek (payload >~40 B) transmisja trwa dłużej niż
  // 15 ms i była ucinana w połowie, zanim urządzenie zdążyło ją
  // odebrać, mimo że echo niżej i tak zgłaszało sukces.
  bool tx_ok = this->cc1101_.wait_tx_complete(50);
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
  } else if (echo) {
    // Echo the transmitted frame back to TCP clients so ramses_tx sees the
    // expected self-echo and can leave its WantEcho state. Wysyłane
    // dopiero po potwierdzonym opróżnieniu FIFO — inaczej log/ramses_tx
    // widziałby poprawną ramkę nawet gdy w eter poleciał tylko urywek.
    // Wołający z echo=false (patrz process_tx_queue: tryb sweep) sam
    // decyduje, kiedy echo wysłać, żeby nie zdublować go przy wielu
    // transmisjach tej samej ramki.
    this->broadcast_hgi80(tx_msg.to_hgi80());
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
      if (!is_freq_sweep_target(tx_msg)) {
        this->transmit_message_locked(tx_msg);
      } else {
        // TYMCZASOWE: tryb diagnostyczny — tylko dla 22F1 37:220902 ->
        // 32:148895. Do cofnięcia po zakończeniu testów (przywrócić
        // pojedyncze wywołanie transmit_message_locked(tx_msg) powyżej i
        // usunąć tę gałąź razem z is_freq_sweep_target()).
        //
        // Przemiata DEVIATN (0x15), nie FSCTRL0 — FSCTRL0 zostaje na 0x00.
        static const uint8_t DEVIATN_SWEEP[] = {
            0x30, 0x34, 0x38, 0x3C, 0x40, 0x43, 0x45, 0x47, 0x50,
            0x53, 0x55, 0x57, 0x60, 0x63, 0x65, 0x67, 0x70,
        };

        bool echoed = false;
        for (uint8_t dev_reg : DEVIATN_SWEEP) {
          // Karmimy Task WDT co iterację — bez tego, przy ~10 s łącznego
          // czasu pętli, idle task na tym rdzeniu nie dostawał CPU i układ
          // resetował się po Task WDT (crash w prvIdleTask).
          esp_task_wdt_reset();

          this->cc1101_.write_reg(CC_DEVIATN, dev_reg);
          // dev = 26e6 / 2^17 * (8 + DEVIATION_M) * 2^DEVIATION_E
          // DEVIATION_E = bity 6:4, DEVIATION_M = bity 2:0.
          uint8_t dev_e = (dev_reg >> 4) & 0x07;
          uint8_t dev_m = dev_reg & 0x07;
          float dev_hz = (26000000.0f / 131072.0f) * (8 + dev_m) * (1 << dev_e);
          ESP_LOGI(TAG, "SWEEP: DEVIATN=0x%02X (dev=%.2f kHz)", dev_reg, dev_hz / 1000.0f);
          // Echo dokładnie raz, zaraz po pierwszej udanej transmisji — inaczej
          // ramses_cc dostałby 17 ech tej samej ramki i zgłosiłby błąd.
          bool tx_ok = this->transmit_message_locked(tx_msg, !echoed);
          if (tx_ok && !echoed) {
            echoed = true;
          }
          // vTaskDelay (nie aktywne czekanie na millis()) — oddaje CPU
          // schedulerowi między strzałami.
          vTaskDelay(pdMS_TO_TICKS(400));
        }

        this->cc1101_.write_reg(CC_DEVIATN, this->cc1101_.get_default_reg(CC_DEVIATN));
        this->cc1101_.enter_rx_mode();
        this->frame_handler_.rx_enable();
      }

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
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  this->cc1101_.write_reg(CC_FSCTRL0, 0x00);
  this->cc1101_.enter_rx_mode();
  this->frame_handler_.rx_enable();
  ESP_LOGI(TAG, "SWEEP: koniec, FSCTRL0 przywrócone do 0x00");

  xSemaphoreGive(this->radio_mutex_);
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
  ESP_LOGCONFIG(TAG, "  BUILD: readback-v1");
}

} // namespace ramses_esp
} // namespace esphome
