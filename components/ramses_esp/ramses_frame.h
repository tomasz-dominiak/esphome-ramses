#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "ramses_message.h"
#include "cc1101_driver.h"

namespace esphome {
namespace ramses_esp {

enum FrameRxState {
  FRM_RX_OFF,
  FRM_RX_IDLE,
  FRM_RX_SYNCH,
  FRM_RX_MESSAGE,
  FRM_RX_TRAILER,
  FRM_RX_DONE,
  FRM_RX_ABORT
};

// Rozmiary buforów do tymczasowego porównania preambuły/sync/trailera
// kodera to_raw_frame() z tym, co faktycznie widać w eterze.
#define RAMSES_PREAMBLE_TAIL_CAP 32
#define RAMSES_TRAILER_CAP 32
// Przerwa bez kolejnego bajtu, po której uznajemy, że transmisja się
// skończyła (jeden bajt na 38,4 kBd trwa ~208 us — 5 ms to >20 bajtów zapasu).
#define RAMSES_TRAILER_IDLE_MS 5

class RamsesFrameHandler {
 public:
  RamsesFrameHandler() = default;

  bool init(uart_port_t uart_num, gpio_num_t gdo0_pin, gpio_num_t gdo2_pin, CC1101Driver *cc1101);
  void set_on_message_callback(std::function<void(const RamsesMessage &)> cb) {
    this->on_message_cb_ = cb;
  }

  void rx_enable();
  void rx_disable();
  void rx_flush();

  void work();

 protected:
  void process_rx_byte(uint8_t b);
  void handle_rx_done();
  void reset_rx();
  void reset_preamble_capture();

  uart_port_t uart_num_{UART_NUM_1};
  gpio_num_t gdo0_pin_{GPIO_NUM_NC};
  gpio_num_t gdo2_pin_{GPIO_NUM_NC};
  CC1101Driver *cc1101_{nullptr};
  QueueHandle_t uart_queue_{nullptr};

  FrameRxState rx_state_{FRM_RX_OFF};
  uint32_t sync_buffer_{0};
  uint8_t rx_raw_count_{0};
  uint8_t rx_msg_count_{0};
  uint8_t rx_msg_byte_{0};
  uint8_t nibble_count_{0};

  // Bufor surowych (przed dekodowaniem Manchester) bajtów bieżącej ramki,
  // od pierwszego bajtu po słowie synchronizacyjnym do bajtu przed
  // trailerem — do tymczasowego porównania kodera to_raw_frame() z
  // rzeczywistą ramką z eteru. Usunąć razem z logowaniem w handle_rx_done().
  uint8_t rx_raw_capture_[RAMSES_MAX_RAW]{0};

  // Ostatnie bajty widziane PRZED dopasowaniem słowa synchronizacyjnego
  // (bufor cykliczny) + ich łączna liczba od ostatniego resetu — obejmuje
  // też same 4 bajty sync (są dopasowywane, nie odrzucane), więc ostatnie
  // 4 bajty tego bufora to zawsze słowo sync widziane z eteru.
  uint32_t rx_preamble_count_{0};
  uint8_t rx_preamble_tail_[RAMSES_PREAMBLE_TAIL_CAP]{0};

  // Bajty widziane PO bajcie-znaczniku trailera (0x35), aż do wykrycia
  // ciszy na linii (RAMSES_TRAILER_IDLE_MS) — pierwszy zapisany bajt to
  // zawsze sam 0x35, który wywołał przejście w stan FRM_RX_TRAILER.
  uint8_t rx_trailer_capture_[RAMSES_TRAILER_CAP]{0};
  uint8_t rx_trailer_count_{0};
  uint32_t rx_trailer_last_byte_ms_{0};

  RamsesMessage current_msg_;
  uint8_t msg_parse_state_{0};
  uint8_t msg_field_count_{0};

  std::function<void(const RamsesMessage &)> on_message_cb_{nullptr};
};

} // namespace ramses_esp
} // namespace esphome
