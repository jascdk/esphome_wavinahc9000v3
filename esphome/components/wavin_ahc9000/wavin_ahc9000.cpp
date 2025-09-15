#include "wavin_ahc9000.h"
#include "esphome/core/log.h"
#include <vector>

namespace esphome {
namespace wavin_ahc9000 {

static const char *const TAG = "wavin_ahc9000";

// Simple Modbus CRC16 (0xA001 poly)
static uint16_t crc16(const uint8_t *frame, size_t len) {
  uint16_t temp = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    temp ^= frame[i];
    for (uint8_t j = 0; j < 8; j++) {
      bool flag = temp & 0x0001;
      temp >>= 1;
      if (flag) temp ^= 0xA001;
    }
  }
  return temp;
}

void WavinAHC9000::setup() { ESP_LOGCONFIG(TAG, "Wavin AHC9000 hub setup"); }
void WavinAHC9000::loop() {}

void WavinAHC9000::update() {
  // First proof-of-life read: CHANNELS category, page 0 (Zone 1), PRIMARY_ELEMENT index, 1 register
  std::vector<uint16_t> out;
  bool ok = this->read_registers(CAT_CHANNELS, /*page=*/0, CH_PRIMARY_ELEMENT, /*count=*/1, out);
  if (ok) {
    if (!out.empty()) {
      ESP_LOGD(TAG, "READ ok: CH_PRIMARY_ELEMENT ch=1 value=0x%04X", (unsigned) out[0]);
    } else {
      ESP_LOGD(TAG, "READ ok: no payload words returned");
    }
  } else {
    ESP_LOGW(TAG, "READ failed (timeout or CRC)");
  }
}

void WavinAHC9000::dump_config() { ESP_LOGCONFIG(TAG, "Wavin AHC9000 (UART test read)"); }

void WavinAHC9000::add_channel_climate(WavinZoneClimate *c) { this->single_ch_climates_.push_back(c); }
void WavinAHC9000::add_group_climate(WavinZoneClimate *c) { this->group_climates_.push_back(c); }

bool WavinAHC9000::read_registers(uint8_t category, uint8_t page, uint8_t index, uint8_t count, std::vector<uint16_t> &out) {
  uint8_t msg[8];
  msg[0] = DEVICE_ADDR;
  msg[1] = FC_READ;
  msg[2] = category;
  msg[3] = index;
  msg[4] = page;
  msg[5] = count;
  uint16_t crc = crc16(msg, 6);
  msg[6] = crc & 0xFF;
  msg[7] = crc >> 8;

  if (this->tx_enable_pin_ != nullptr) this->tx_enable_pin_->digital_write(true);
  ESP_LOGV(TAG, "TX: addr=0x%02X fc=0x%02X cat=%u idx=%u page=%u cnt=%u", msg[0], msg[1], category, index, page, count);
  this->write_array(msg, 8);
  this->flush();
  delayMicroseconds(250);
  if (this->tx_enable_pin_ != nullptr) this->tx_enable_pin_->digital_write(false);

  // Receive response: [addr][fc][byte_count][payload...][crc_lo][crc_hi]
  std::vector<uint8_t> buf;
  uint32_t start = millis();
  while (millis() - start < this->receive_timeout_ms_) {
    while (this->available()) {
      int c = this->read();
      if (c < 0) break;
      buf.push_back((uint8_t) c);
      if (buf.size() >= 5) {
        uint8_t expected = (uint8_t) (buf[2] + 5); // 3 header + payload + 2 crc
        if (buf[0] == DEVICE_ADDR && buf[1] == FC_READ && buf.size() == expected) {
          uint16_t rcrc = crc16(buf.data(), buf.size());
          if (rcrc != 0) {
            ESP_LOGW(TAG, "RX: CRC mismatch (len=%u)", (unsigned) buf.size());
            return false;
          }
          uint8_t bytes = buf[2];
          out.clear();
          for (uint8_t i = 0; i + 1 < bytes; i += 2) {
            uint16_t w = (uint16_t) (buf[3 + i] << 8) | buf[3 + i + 1];
            out.push_back(w);
          }
          return true;
        }
      }
    }
    delay(1);
  }
  ESP_LOGW(TAG, "RX: timeout waiting for response");
  return false;
}

float WavinAHC9000::get_channel_current_temp(uint8_t) const { return NAN; }
float WavinAHC9000::get_channel_setpoint(uint8_t) const { return NAN; }
climate::ClimateMode WavinAHC9000::get_channel_mode(uint8_t) const { return climate::CLIMATE_MODE_HEAT; }
climate::ClimateAction WavinAHC9000::get_channel_action(uint8_t) const { return climate::CLIMATE_ACTION_OFF; }

void WavinZoneClimate::dump_config() { LOG_CLIMATE("  ", "Wavin Zone Climate (minimal)", this); }
climate::ClimateTraits WavinZoneClimate::traits() {
  climate::ClimateTraits t;
  t.set_supported_modes({climate::CLIMATE_MODE_HEAT});
  t.set_supports_current_temperature(true);
  return t;
}
void WavinZoneClimate::control(const climate::ClimateCall &) {}
void WavinZoneClimate::update_from_parent() { this->publish_state(); }

}  // namespace wavin_ahc9000
}  // namespace esphome
