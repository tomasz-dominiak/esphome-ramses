#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace esphome {
namespace ramses_esp {

#define RAMSES_MAX_ADDR 3
#define RAMSES_MAX_PAYLOAD 64
#define RAMSES_MAX_RAW 162

enum RamsesMsgType {
  RAMSES_MSG_RQ = 0,
  RAMSES_MSG_I  = 1,
  RAMSES_MSG_W  = 2,
  RAMSES_MSG_RP = 3,
  RAMSES_MSG_MAX = 4
};

#define RAMSES_F_MASK   0x03
#define RAMSES_F_ADDR0  0x10
#define RAMSES_F_ADDR1  0x20
#define RAMSES_F_ADDR2  0x40
#define RAMSES_F_PARAM0 0x04
#define RAMSES_F_PARAM1 0x08
#define RAMSES_F_RSSI   0x80
#define RAMSES_F_OPCODE 0x01
#define RAMSES_F_LEN    0x02

#define HDR_T_MASK 0x30
#define HDR_T_SHIFT 4
#define HDR_A_MASK 0x0C
#define HDR_A_SHIFT 2
#define HDR_PARAM0 0x02
#define HDR_PARAM1 0x01

// Preambuła (trening 0x55) którą to_raw_frame() wstawia przed słowem sync.
// Przechwyty z eteru pokazują konsekwentnie 6 bajtów u prawdziwego
// urządzenia. Jedyne źródło prawdy o długości preambuły — diagnostyka w
// ramses_frame.cpp czyta z tego samego symbolu, żeby zmiana długości nie
// rozjechała jej po cichu.
constexpr uint8_t RAMSES_TX_PREAMBLE[] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
constexpr size_t RAMSES_TX_PREAMBLE_LEN = sizeof(RAMSES_TX_PREAMBLE) / sizeof(RAMSES_TX_PREAMBLE[0]);

// Trailer, który to_raw_frame() dokłada po ostatnim bajcie danych (35 to
// terminator rozpoznawany przez RX, kolejne 55 to zapas czasowy dla
// modulatora, zanim TXFIFO_UNDERFLOW wymusi SIDLE). Jedyne źródło prawdy
// o długości trailera — diagnostyka w ramses_frame.cpp liczy z tego samego
// symbolu, żeby zmiana długości nie rozjechała jej po cichu.
constexpr uint8_t RAMSES_TX_TRAILER[] = {0x35, 0x55, 0x55, 0x55, 0x55};
constexpr size_t RAMSES_TX_TRAILER_LEN = sizeof(RAMSES_TX_TRAILER) / sizeof(RAMSES_TX_TRAILER[0]);

struct RamsesAddress {
  uint8_t dev_class{0};
  uint32_t id{0};
  bool is_valid{false};

  std::string to_string() const;
  static RamsesAddress from_bytes(const uint8_t *bytes);
  static RamsesAddress from_string(const std::string &str);
  void to_bytes(uint8_t *bytes) const;
};

struct RamsesMessage {
  RamsesMsgType type{RAMSES_MSG_I};
  uint8_t fields{0};
  uint8_t rx_fields{0};
  uint8_t error{0};

  uint8_t addr[RAMSES_MAX_ADDR][3]{{0}};
  uint8_t param[2]{0};

  uint8_t opcode[2]{0};
  uint8_t len{0};

  uint8_t csum{0};
  uint8_t rssi{0};

  uint8_t n_payload{0};
  uint8_t payload[RAMSES_MAX_PAYLOAD]{0};

  // Fixed-size buffer (not std::string): RamsesMessage instances are copied by raw
  // memcpy through FreeRTOS queues (xQueueSend/xQueueReceive), which does not run
  // C++ copy/move/destructor semantics. A heap-owning member here would let two
  // "copies" alias the same heap buffer, leading to use-after-free/double-free
  // once either copy is destroyed. Keep this struct trivially copyable.
  char timestamp[24]{0};

  void reset();
  bool is_valid() const;
  uint8_t calculate_checksum() const;
  std::string to_hgi80() const;
  bool from_hgi80(const std::string &line);
  std::vector<uint8_t> to_raw_frame() const;
};

uint8_t ramses_decode_header(uint8_t header);
uint8_t ramses_encode_header(uint8_t fields);

} // namespace ramses_esp
} // namespace esphome
