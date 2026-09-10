#include "ramses_frame.h"
#include "ramses_codec.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <sys/time.h>
#include <ctime>
#include <cstdio>

static const char *TAG = "ramses_esp.frame";

#define RAMSES_SYNC_WORD 0x00335553
#define RAMSES_TRAILER   0x35

namespace esphome {
namespace ramses_esp {

enum MsgParseState {
  STATE_HDR = 0,
  STATE_ADDR0,
  STATE_ADDR1,
  STATE_ADDR2,
  STATE_PARAM0,
  STATE_PARAM1,
  STATE_OPCODE,
  STATE_LEN,
  STATE_PAYLOAD,
  STATE_CHECKSUM,
  STATE_DONE
};

bool RamsesFrameHandler::init(uart_port_t uart_num, gpio_num_t gdo0_pin, gpio_num_t gdo2_pin, CC1101Driver *cc1101) {
  this->uart_num_ = uart_num;
  this->gdo0_pin_ = gdo0_pin;
  this->gdo2_pin_ = gdo2_pin;
  this->cc1101_ = cc1101;

  uart_config_t uart_config = {
      .baud_rate = 38400,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0,
      .source_clk = UART_SCLK_DEFAULT,
  };

  esp_err_t ret = uart_param_config(this->uart_num_, &uart_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "uart_param_config failed: %d", ret);
    return false;
  }

  ret = uart_set_pin(this->uart_num_, this->gdo2_pin_, this->gdo0_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "uart_set_pin failed: %d", ret);
    return false;
  }

  ret = uart_driver_install(this->uart_num_, 512, 0, 16, &this->uart_queue_, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "uart_driver_install failed: %d", ret);
    return false;
  }

  this->rx_enable();
  ESP_LOGI(TAG, "UART%d initialized on RX pin GPIO%d for RAMSES framing", this->uart_num_, this->gdo0_pin_);
  return true;
}

void RamsesFrameHandler::rx_enable() {
  uart_flush_input(this->uart_num_);
  uart_enable_rx_intr(this->uart_num_);
  this->rx_state_ = FRM_RX_IDLE;
  this->reset_rx();
  this->reset_preamble_capture();
}

void RamsesFrameHandler::rx_disable() {
  uart_disable_rx_intr(this->uart_num_);
  this->rx_state_ = FRM_RX_OFF;
}

void RamsesFrameHandler::rx_flush() {
  if (this->uart_queue_ != nullptr) {
    UBaseType_t n = uxQueueMessagesWaiting(this->uart_queue_);
    uart_event_t event;
    for (UBaseType_t i = 0; i < n; i++) {
      xQueueReceive(this->uart_queue_, &event, 0);
    }
  }
  uart_flush_input(this->uart_num_);
}

void RamsesFrameHandler::reset_rx() {
  this->sync_buffer_ = 0;
  this->rx_raw_count_ = 0;
  this->rx_msg_count_ = 0;
  this->rx_msg_byte_ = 0;
  this->nibble_count_ = 0;
  this->msg_parse_state_ = STATE_HDR;
  this->msg_field_count_ = 0;
  this->current_msg_.reset();
}

// Osobno od reset_rx(): reset_rx() jest wołane też w momencie dopasowania
// słowa sync (żeby przygotować stan do parsowania ciała), a wtedy właśnie
// chcemy ZACHOWAĆ policzoną preambułę do zalogowania w handle_rx_done().
// Ten reset uzbraja licznik od nowa tylko przy starcie nasłuchu na
// KOLEJNĄ ramkę (rx_enable() i po DONE/ABORT w work()).
void RamsesFrameHandler::reset_preamble_capture() {
  this->rx_preamble_count_ = 0;
}

void RamsesFrameHandler::work() {
  if (this->rx_state_ == FRM_RX_OFF || this->uart_queue_ == nullptr) return;

  uart_event_t event;
  if (xQueueReceive(this->uart_queue_, &event, pdMS_TO_TICKS(10))) {
    if (event.type == UART_DATA && event.size > 0) {
      uint8_t buf[128];
      int bytes_read = uart_read_bytes(this->uart_num_, buf, std::min((int)event.size, (int)sizeof(buf)), 0);
      for (int i = 0; i < bytes_read; i++) {
        this->process_rx_byte(buf[i]);
      }
    } else if (event.type == UART_BUFFER_FULL || event.type == UART_FIFO_OVF) {
      uart_flush_input(this->uart_num_);
      xQueueReset(this->uart_queue_);
      this->rx_state_ = FRM_RX_IDLE;
      this->reset_rx();
      this->reset_preamble_capture();
    }
  }

  // Ciało ramki się skończyło (trailer wykryty), ale czekamy jeszcze na
  // bajty PO nim, żeby zmierzyć rzeczywisty ogon transmisji. Brak nowego
  // bajtu przez RAMSES_TRAILER_IDLE_MS uznajemy za koniec transmisji.
  if (this->rx_state_ == FRM_RX_TRAILER &&
      (millis() - this->rx_trailer_last_byte_ms_) > RAMSES_TRAILER_IDLE_MS) {
    this->rx_state_ = FRM_RX_DONE;
  }

  if (this->rx_state_ == FRM_RX_DONE) {
    this->handle_rx_done();
    this->rx_state_ = FRM_RX_IDLE;
    this->reset_rx();
    this->reset_preamble_capture();
  } else if (this->rx_state_ == FRM_RX_ABORT) {
    this->rx_state_ = FRM_RX_IDLE;
    this->reset_rx();
    this->reset_preamble_capture();
  }
}

static uint8_t next_state_after_hdr(uint8_t fields) {
  if (fields & RAMSES_F_ADDR0) return STATE_ADDR0;
  if (fields & RAMSES_F_ADDR1) return STATE_ADDR1;
  if (fields & RAMSES_F_ADDR2) return STATE_ADDR2;
  if (fields & RAMSES_F_PARAM0) return STATE_PARAM0;
  if (fields & RAMSES_F_PARAM1) return STATE_PARAM1;
  return STATE_OPCODE;
}

static uint8_t next_state_after_addr(uint8_t current_addr_state, uint8_t fields) {
  if (current_addr_state == STATE_ADDR0) {
    if (fields & RAMSES_F_ADDR1) return STATE_ADDR1;
    if (fields & RAMSES_F_ADDR2) return STATE_ADDR2;
  } else if (current_addr_state == STATE_ADDR1) {
    if (fields & RAMSES_F_ADDR2) return STATE_ADDR2;
  }
  if (fields & RAMSES_F_PARAM0) return STATE_PARAM0;
  if (fields & RAMSES_F_PARAM1) return STATE_PARAM1;
  return STATE_OPCODE;
}

void RamsesFrameHandler::process_rx_byte(uint8_t b) {
  switch (this->rx_state_) {
    case FRM_RX_OFF:
      break;

    case FRM_RX_IDLE:
    case FRM_RX_SYNCH:
      // Zapisujemy KAŻDY bajt widziany przed dopasowaniem, łącznie z
      // czterema bajtami samego słowa sync — bufor cykliczny, więc po
      // dopasowaniu jego ostatnie 4 bajty to zawsze sync widziany z eteru.
      this->rx_preamble_tail_[this->rx_preamble_count_ % RAMSES_PREAMBLE_TAIL_CAP] = b;
      this->rx_preamble_count_++;
      this->sync_buffer_ = (this->sync_buffer_ << 8) | b;
      if (this->sync_buffer_ == RAMSES_SYNC_WORD) {
        this->rx_state_ = FRM_RX_MESSAGE;
        this->reset_rx();
      }
      break;

    case FRM_RX_MESSAGE:
      if (b == RAMSES_TRAILER) {
        // Nie kończymy od razu — łapiemy jeszcze bajty PO znaczniku
        // trailera, żeby zmierzyć rzeczywisty ogon transmisji (patrz work()).
        this->rx_trailer_count_ = 0;
        this->rx_trailer_capture_[this->rx_trailer_count_++] = b;
        this->rx_trailer_last_byte_ms_ = millis();
        this->rx_state_ = FRM_RX_TRAILER;
        return;
      }

      if (this->rx_raw_count_ < RAMSES_MAX_RAW) {
        this->rx_raw_capture_[this->rx_raw_count_] = b;
      }
      this->rx_raw_count_++;
      if (this->rx_raw_count_ >= RAMSES_MAX_RAW) {
        this->rx_state_ = FRM_RX_ABORT;
        return;
      }

      if (!manchester_code_valid(b)) {
        this->rx_state_ = FRM_RX_ABORT;
        return;
      }

      this->rx_msg_byte_ = (this->rx_msg_byte_ << 4) | manchester_decode(b);
      this->nibble_count_ = 1 - this->nibble_count_;

      if (this->nibble_count_ == 0) {
        uint8_t byte = this->rx_msg_byte_;
        this->rx_msg_count_++;

        switch (this->msg_parse_state_) {
          case STATE_HDR:
            this->current_msg_.fields = ramses_decode_header(byte);
            this->current_msg_.type = static_cast<RamsesMsgType>((byte & HDR_T_MASK) >> HDR_T_SHIFT);
            this->msg_parse_state_ = next_state_after_hdr(this->current_msg_.fields);
            this->msg_field_count_ = 0;
            break;

          case STATE_ADDR0:
          case STATE_ADDR1:
          case STATE_ADDR2: {
            uint8_t addr_idx = this->msg_parse_state_ - STATE_ADDR0;
            this->current_msg_.addr[addr_idx][this->msg_field_count_++] = byte;
            if (this->msg_field_count_ == 3) {
              this->current_msg_.rx_fields |= (RAMSES_F_ADDR0 << addr_idx);
              this->msg_parse_state_ = next_state_after_addr(this->msg_parse_state_, this->current_msg_.fields);
              this->msg_field_count_ = 0;
            }
            break;
          }

          case STATE_PARAM0:
            this->current_msg_.param[0] = byte;
            this->current_msg_.rx_fields |= RAMSES_F_PARAM0;
            this->msg_parse_state_ = (this->current_msg_.fields & RAMSES_F_PARAM1) ? STATE_PARAM1 : STATE_OPCODE;
            break;

          case STATE_PARAM1:
            this->current_msg_.param[1] = byte;
            this->current_msg_.rx_fields |= RAMSES_F_PARAM1;
            this->msg_parse_state_ = STATE_OPCODE;
            this->msg_field_count_ = 0;
            break;

          case STATE_OPCODE:
            this->current_msg_.opcode[this->msg_field_count_++] = byte;
            if (this->msg_field_count_ == 2) {
              this->current_msg_.rx_fields |= RAMSES_F_OPCODE;
              this->msg_parse_state_ = STATE_LEN;
              this->msg_field_count_ = 0;
            }
            break;

          case STATE_LEN:
            this->current_msg_.len = byte;
            this->current_msg_.rx_fields |= RAMSES_F_LEN;
            this->msg_field_count_ = 0;
            if (this->current_msg_.len == 0) {
              this->msg_parse_state_ = STATE_CHECKSUM;
            } else if (this->current_msg_.len > RAMSES_MAX_PAYLOAD) {
              this->rx_state_ = FRM_RX_ABORT;
            } else {
              this->msg_parse_state_ = STATE_PAYLOAD;
            }
            break;

          case STATE_PAYLOAD:
            this->current_msg_.payload[this->msg_field_count_++] = byte;
            this->current_msg_.n_payload = this->msg_field_count_;
            if (this->msg_field_count_ >= this->current_msg_.len) {
              this->msg_parse_state_ = STATE_CHECKSUM;
            }
            break;

          case STATE_CHECKSUM:
            this->current_msg_.csum = byte;
            this->msg_parse_state_ = STATE_DONE;
            break;

          case STATE_DONE:
            break;
        }
      }
      break;

    case FRM_RX_TRAILER:
      if (this->rx_trailer_count_ < RAMSES_TRAILER_CAP) {
        this->rx_trailer_capture_[this->rx_trailer_count_++] = b;
      }
      this->rx_trailer_last_byte_ms_ = millis();
      break;

    case FRM_RX_DONE:
    case FRM_RX_ABORT:
      break;
  }
}

void RamsesFrameHandler::handle_rx_done() {
  if (this->cc1101_ != nullptr) {
    this->current_msg_.rssi = this->cc1101_->read_rssi();
  }

  // Generate ISO timestamp
  char ts_buf[40];
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  struct tm *nowtm = localtime(&tv.tv_sec);
  if (nowtm != nullptr) {
    strftime(ts_buf, sizeof(ts_buf), "%Y-%m-%dT%H:%M:%S", nowtm);
    snprintf(this->current_msg_.timestamp, sizeof(this->current_msg_.timestamp), "%s.%03ld", ts_buf,
             tv.tv_usec / 1000);
  }

  // TYMCZASOWE: porównanie kodera to_raw_frame() z rzeczywistą ramką z
  // eteru. Do usunięcia po zdiagnozowaniu, dlaczego centrala ignoruje
  // ramki nadawane przez to_raw_frame() mimo poprawnej treści/mocy/czasu.
  {
    // Odtwarzamy ostatnie (do RAMSES_PREAMBLE_TAIL_CAP) bajty widziane
    // przed dopasowaniem sync, w kolejności chronologicznej — bufor jest
    // cykliczny (patrz process_rx_byte/FRM_RX_IDLE).
    uint32_t tail_len = std::min<uint32_t>(this->rx_preamble_count_, RAMSES_PREAMBLE_TAIL_CAP);
    uint8_t preamble_tail[RAMSES_PREAMBLE_TAIL_CAP];
    for (uint32_t k = 0; k < tail_len; k++) {
      uint32_t abs_pos = this->rx_preamble_count_ - tail_len + k;
      preamble_tail[k] = this->rx_preamble_tail_[abs_pos % RAMSES_PREAMBLE_TAIL_CAP];
    }

    char pre_hex[RAMSES_PREAMBLE_TAIL_CAP * 3 + 1];
    int pos = 0;
    for (uint32_t k = 0; k < tail_len && pos < (int)sizeof(pre_hex) - 3; k++) {
      pos += snprintf(pre_hex + pos, sizeof(pre_hex) - pos, "%02X ", preamble_tail[k]);
    }
    ESP_LOGD(TAG, "RX preambuła: %lu B widzianych przed sync (ostatnie %lu, w tym 4B sync): %s",
             (unsigned long)this->rx_preamble_count_, (unsigned long)tail_len, pre_hex);

    // Ostatnie 4 bajty tego ogona to zawsze dopasowane słowo sync.
    if (tail_len >= 4) {
      ESP_LOGD(TAG, "RX sync: %02X %02X %02X %02X | TX koder sync: FF 00 33 55 53 (+preambuła 20x 0x55)",
               preamble_tail[tail_len - 4], preamble_tail[tail_len - 3],
               preamble_tail[tail_len - 2], preamble_tail[tail_len - 1]);
    }

    char raw_hex[RAMSES_MAX_RAW * 3 + 1];
    pos = 0;
    for (uint8_t i = 0; i < this->rx_raw_count_ && pos < (int)sizeof(raw_hex) - 3; i++) {
      pos += snprintf(raw_hex + pos, sizeof(raw_hex) - pos, "%02X ", this->rx_raw_capture_[i]);
    }
    ESP_LOGD(TAG, "RX raw body (%u B, po sync/przed trailerem): %s", this->rx_raw_count_, raw_hex);

    char trail_hex[RAMSES_TRAILER_CAP * 3 + 1];
    pos = 0;
    for (uint8_t i = 0; i < this->rx_trailer_count_ && pos < (int)sizeof(trail_hex) - 3; i++) {
      pos += snprintf(trail_hex + pos, sizeof(trail_hex) - pos, "%02X ", this->rx_trailer_capture_[i]);
    }

    // Bajty kodera czytane z tej samej tablicy, z której to_raw_frame()
    // faktycznie buduje trailer — żeby ten log nigdy nie mógł zostać w
    // tyle za rzeczywistą zawartością po kolejnej zmianie trailera.
    char enc_trail_hex[RAMSES_TX_TRAILER_LEN * 3 + 1];
    pos = 0;
    for (size_t i = 0; i < RAMSES_TX_TRAILER_LEN && pos < (int)sizeof(enc_trail_hex) - 3; i++) {
      pos += snprintf(enc_trail_hex + pos, sizeof(enc_trail_hex) - pos, "%02X ", RAMSES_TX_TRAILER[i]);
    }

    ESP_LOGD(TAG, "RX trailer (%u B od 0x35 do ciszy >%dms): %s | TX koder trailer: %s(%u B)",
             this->rx_trailer_count_, RAMSES_TRAILER_IDLE_MS, trail_hex, enc_trail_hex,
             (unsigned)RAMSES_TX_TRAILER_LEN);
  }

  if (this->current_msg_.is_valid()) {
    std::string hgi80 = this->current_msg_.to_hgi80();
    ESP_LOGI(TAG, "RX: %s", hgi80.c_str());

    // to_raw_frame() layout: 20 B preambuły (0x55) + 5 B sync + treść
    // zakodowana Manchesterem + 2 B trailer (0x35, 0x55).
    std::vector<uint8_t> encoded = this->current_msg_.to_raw_frame();
    static const size_t PREAMBLE_SYNC_LEN = 25;
    static const size_t TRAILER_LEN = RAMSES_TX_TRAILER_LEN;
    if (encoded.size() >= PREAMBLE_SYNC_LEN + TRAILER_LEN) {
      size_t body_len = encoded.size() - PREAMBLE_SYNC_LEN - TRAILER_LEN;

      char enc_hex[RAMSES_MAX_RAW * 3 + 1];
      int pos = 0;
      for (size_t i = 0; i < body_len && pos < (int)sizeof(enc_hex) - 3; i++) {
        pos += snprintf(enc_hex + pos, sizeof(enc_hex) - pos, "%02X ", encoded[PREAMBLE_SYNC_LEN + i]);
      }
      ESP_LOGD(TAG, "TX would encode (%u B): %s", (unsigned)body_len, enc_hex);

      if (body_len != this->rx_raw_count_) {
        ESP_LOGW(TAG, "RX/TX: różna długość ciała ramki — RX=%u B, TX=%u B",
                 this->rx_raw_count_, (unsigned)body_len);
      } else {
        bool mismatch = false;
        for (uint8_t i = 0; i < this->rx_raw_count_; i++) {
          uint8_t enc_byte = encoded[PREAMBLE_SYNC_LEN + i];
          if (this->rx_raw_capture_[i] != enc_byte) {
            ESP_LOGW(TAG, "RX/TX: bajt %u różny — RX=0x%02X TX=0x%02X", i, this->rx_raw_capture_[i], enc_byte);
            mismatch = true;
          }
        }
        if (!mismatch) {
          ESP_LOGD(TAG, "RX/TX: ciało ramki identyczne (%u B)", this->rx_raw_count_);
        }
      }
    }

    if (this->on_message_cb_ != nullptr) {
      this->on_message_cb_(this->current_msg_);
    }
  } else {
    ESP_LOGD(TAG, "Dropped invalid RAMSES frame (state=%d, fields=0x%02X)",
             this->msg_parse_state_, this->current_msg_.fields);
  }
}

} // namespace ramses_esp
} // namespace esphome
