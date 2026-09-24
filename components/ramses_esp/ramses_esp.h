#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/gpio.h"
#include "cc1101_driver.h"
#include "ramses_frame.h"
#include "ramses_message.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <vector>
#include <string>

namespace esphome {
namespace ramses_esp {

class RamsesESPComponent : public Component {
 public:
  RamsesESPComponent() = default;

  void set_sck_pin(InternalGPIOPin *pin) { this->sck_pin_ = static_cast<gpio_num_t>(pin->get_pin()); }
  void set_mosi_pin(InternalGPIOPin *pin) { this->mosi_pin_ = static_cast<gpio_num_t>(pin->get_pin()); }
  void set_miso_pin(InternalGPIOPin *pin) { this->miso_pin_ = static_cast<gpio_num_t>(pin->get_pin()); }
  void set_cs_pin(InternalGPIOPin *pin) { this->cs_pin_ = static_cast<gpio_num_t>(pin->get_pin()); }
  void set_gdo0_pin(InternalGPIOPin *pin) { this->gdo0_pin_ = static_cast<gpio_num_t>(pin->get_pin()); }
  void set_gdo2_pin(InternalGPIOPin *pin) { this->gdo2_pin_ = static_cast<gpio_num_t>(pin->get_pin()); }
  void set_uart_num(uint8_t uart_num) { this->uart_num_ = static_cast<uart_port_t>(uart_num); }
  void set_port(uint16_t port) { this->port_ = port; }
  // Ręczna korekta FSCTRL0 stosowana tylko na czas TX (RX zawsze zostaje na
  // 0x00 — patrz komentarz przy CC_FSCTRL0 w cc1101_driver.cpp). Wartość w
  // jednostkach rejestru, nie kHz — patrz ramses_esp.freq_sweep do przeliczeń.
  void set_tx_freq_correction(int8_t val) { this->tx_freq_correction_ = val; }

  void add_on_message_callback(std::function<void(const std::string &)> callback) {
    this->on_message_callbacks_.push_back(callback);
  }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // High-level Actions
  bool send_hgi80_command(const std::string &cmd);
  void freq_sweep(const std::string &cmd);
  // Diagnostyka: nadaje w kolko jedna wbudowana ramke tak szybko, jak pozwala
  // cykl STX->RX, przez duration_ms. Wolane z lambdy custom API service (patrz
  // example-c6.yaml). Zawsze wraca do nasluchu RX i zwalnia radio.
  void start_flood_tx(uint32_t duration_ms);

  // Multiplexer arbitration interface
  void pause();
  void resume();
  bool is_paused() const { return this->paused_; }

 protected:
  void start_tcp_server();
  void handle_tcp_clients();
  void broadcast_hgi80(const std::string &hgi80);
  void process_tx_queue();
  bool transmit_message_locked(const RamsesMessage &tx_msg);
  // Przemiata FSCTRL0 (korekcja czestotliwosci TX) dla jednej ramki, nadajac
  // po kazdym kroku zwykla sciezka TX. Wolajacy musi trzymac radio_mutex_.
  // Uzywane tylko przez akcje freq_sweep() (start_freq_sweep).
  void sweep_message_locked(const RamsesMessage &msg);

  static void radio_task_trampoline(void *arg);
  void radio_task();

  gpio_num_t sck_pin_{GPIO_NUM_NC};
  gpio_num_t mosi_pin_{GPIO_NUM_NC};
  gpio_num_t miso_pin_{GPIO_NUM_NC};
  gpio_num_t cs_pin_{GPIO_NUM_NC};
  gpio_num_t gdo0_pin_{GPIO_NUM_NC};
  gpio_num_t gdo2_pin_{GPIO_NUM_NC};
  uart_port_t uart_num_{UART_NUM_1};
  uint16_t port_{6638};
  int8_t tx_freq_correction_{0};

  CC1101Driver cc1101_;
  RamsesFrameHandler frame_handler_;

  TaskHandle_t radio_task_handle_{nullptr};
  SemaphoreHandle_t radio_mutex_{nullptr};
  QueueHandle_t rx_msg_queue_{nullptr};
  QueueHandle_t tx_msg_queue_{nullptr};

  int server_fd_{-1};
  std::vector<int> client_fds_;

  bool paused_{false};

  // Zabezpiecza DEBUG "brak przedwczesnego underflow" przed spamem: logowany
  // raz, dopoki nie zostanie wyzerowany (start_flood_tx robi to na poczatku
  // testu, wiec dostajemy jedno potwierdzenie na przebieg floodu).
  bool tx_state_ok_logged_{false};

  std::vector<std::function<void(const std::string &)>> on_message_callbacks_;
};

template<typename... Ts>
class SendHgi80Action : public Action<Ts...> {
 public:
  SendHgi80Action(RamsesESPComponent *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, command)

  void play(Ts... x) override {
    auto cmd = this->command_.value(x...);
    this->parent_->send_hgi80_command(cmd);
  }

 protected:
  RamsesESPComponent *parent_;
};

template<typename... Ts>
class FreqSweepAction : public Action<Ts...> {
 public:
  FreqSweepAction(RamsesESPComponent *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, command)

  void play(Ts... x) override {
    auto cmd = this->command_.value(x...);
    this->parent_->freq_sweep(cmd);
  }

 protected:
  RamsesESPComponent *parent_;
};

class RamsesMessageTrigger : public Trigger<std::string> {
 public:
  explicit RamsesMessageTrigger(RamsesESPComponent *parent) {
    parent->add_on_message_callback([this](const std::string &msg) {
      this->trigger(msg);
    });
  }
};

} // namespace ramses_esp
} // namespace esphome
