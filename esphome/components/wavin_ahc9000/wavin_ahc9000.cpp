#include "wavin_ahc9000.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace wavin_ahc9000 {

static const char *const TAG = "wavin_ahc9000";

void WavinAHC9000::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Wavin AHC9000");
  if (this->tx_enable_pin_ != nullptr) {
    this->tx_enable_pin_->setup();
    this->tx_enable_pin_->digital_write(false);
  }
}

void WavinAHC9000::loop() {
  // No-op, Modbus is handled via callbacks
}

void WavinAHC9000::update() {
  // Poll a subset of channels per cycle to avoid long blocking UART waits
  for (uint8_t i = 0; i < this->poll_channels_per_cycle_; i++) {
    this->request_status_channel(this->next_channel_);
    this->next_channel_ = (uint8_t)((this->next_channel_ + 1) % 16);
  }
  this->publish_updates();
}

void WavinAHC9000::dump_config() {
  ESP_LOGCONFIG(TAG, "Wavin AHC9000");
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Temp divisor: %.2f", this->temp_divisor_);
  ESP_LOGCONFIG(TAG, "  Receive timeout: %u ms", this->receive_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  TX enable pin: %s", this->tx_enable_pin_ ? "YES" : "NO");
}

void WavinAHC9000::add_channel_climate(WavinZoneClimate *c) { this->single_ch_climates_.push_back(c); }
void WavinAHC9000::add_group_climate(WavinZoneClimate *c) { this->group_climates_.push_back(c); }

void WavinAHC9000::request_status() {
  // For each channel, gather: primary element, packed data (setpoint+mode), channels status, and element temps
  for (uint8_t ch = 0; ch < 16; ch++) {
    this->request_status_channel(ch);
  }
  this->publish_updates();
}

void WavinAHC9000::request_status_channel(uint8_t ch) {
  if (ch >= 16) return;
  // Gather: primary element, packed data (setpoint+mode), channels status, and element temps
  {
    // Primary element and lost flag
    std::vector<uint16_t> regs;
    if (this->read_registers(CAT_CHANNELS, ch, CH_PRIMARY_ELEMENT, 1, regs)) {
      uint16_t v = regs[0];
      uint16_t primary = v & CH_PRIMARY_ELEMENT_ELEMENT_MASK; // controller returns index+1
      bool all_lost = (v & CH_PRIMARY_ELEMENT_ALL_TP_LOST_MASK) != 0;
      auto &st = this->channels_[ch + 1];
      // Read mode + setpoint
      regs.clear();
      if (this->read_registers(CAT_PACKED, ch, PACKED_CONFIGURATION, 1, regs)) {
        uint16_t mode_bits = regs[0] & PACKED_CONFIGURATION_MODE_MASK;
        // Map manual -> HEAT, standby -> OFF (so users can toggle off)
        st.mode = (mode_bits == PACKED_CONFIGURATION_MODE_STANDBY) ? climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;
      }
      regs.clear();
      if (this->read_registers(CAT_PACKED, ch, PACKED_MANUAL_TEMPERATURE, 1, regs)) {
        st.setpoint_c = this->raw_to_c(regs[0]);
      }
      // Output status
      regs.clear();
      if (this->read_registers(CAT_CHANNELS, ch, CH_TIMER_EVENT, 1, regs)) {
        bool heating = (regs[0] & CH_TIMER_EVENT_OUTP_ON_MASK) != 0;
        st.action = heating ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_IDLE;
      }
      // Current temperature and battery from primary element (index-1), read 11 registers
      if (!all_lost && primary > 0) {
        regs.clear();
        if (this->read_registers(CAT_ELEMENTS, primary - 1, 0, 11, regs)) {
          if (regs.size() > ELEM_AIR_TEMPERATURE)
            st.current_temp_c = this->raw_to_c(regs[ELEM_AIR_TEMPERATURE]);
          if (regs.size() > ELEM_BATTERY_STATUS) {
            uint16_t raw = regs[ELEM_BATTERY_STATUS]; // in 10% steps
            uint8_t pct = (raw > 10 ? 100 : raw * 10);
            st.battery_pct = pct;
            auto it = this->battery_sensors_.find(ch + 1);
            if (it != this->battery_sensors_.end() && it->second != nullptr) {
              it->second->publish_state((float)pct);
            }
          }
        }
      } else {
        st.current_temp_c = NAN;
      }
    }
  }
}

void WavinAHC9000::write_channel_setpoint(uint8_t channel, float celsius) {
  uint8_t ch = channel - 1;
  uint16_t val = this->c_to_raw(celsius);
  this->write_register(CAT_PACKED, ch, PACKED_MANUAL_TEMPERATURE, val);
}

void WavinAHC9000::write_group_setpoint(const std::vector<uint8_t> &members, float celsius) {
  for (auto ch : members) this->write_channel_setpoint(ch, celsius);
}

void WavinAHC9000::write_channel_mode(uint8_t channel, climate::ClimateMode mode) {
  // Only HEAT mode is supported; to emulate standby, one might set standby mode via PACKED_CONFIGURATION
  uint8_t ch = channel - 1;
  uint16_t mode_val = PACKED_CONFIGURATION_MODE_MANUAL;
  // If we wanted to support OFF, map to STANDBY:
  if (mode == climate::CLIMATE_MODE_OFF) mode_val = PACKED_CONFIGURATION_MODE_STANDBY;
  this->write_masked_register(CAT_PACKED, ch, PACKED_CONFIGURATION, mode_val, ~PACKED_CONFIGURATION_MODE_MASK);
}

float WavinAHC9000::get_channel_current_temp(uint8_t channel) const {
  auto it = this->channels_.find(channel);
  if (it == this->channels_.end()) return NAN;
  return it->second.current_temp_c;
}

float WavinAHC9000::get_channel_setpoint(uint8_t channel) const {
  auto it = this->channels_.find(channel);
  if (it == this->channels_.end()) return NAN;
  return it->second.setpoint_c;
}

climate::ClimateMode WavinAHC9000::get_channel_mode(uint8_t channel) const {
  auto it = this->channels_.find(channel);
  if (it == this->channels_.end()) return climate::CLIMATE_MODE_HEAT;
  return it->second.mode;
}

climate::ClimateAction WavinAHC9000::get_channel_action(uint8_t channel) const {
  auto it = this->channels_.find(channel);
  if (it == this->channels_.end()) return climate::CLIMATE_ACTION_OFF;
  return it->second.action;
}

// ---- Protocol helpers ----

static uint16_t crc16(const uint8_t *frame, uint8_t len) {
  uint16_t temp = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    temp ^= frame[i];
    for (uint8_t j = 0; j < 8; j++) {
      bool flag = temp & 0x0001;
      temp >>= 1;
      if (flag) temp ^= 0xA001;
    }
  }
  return temp;
}

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
  ESP_LOGVV(TAG, "READ req: cat=%u page=%u idx=%u cnt=%u", category, page, index, count);
  this->write_array(msg, 8);
  this->flush();
  delayMicroseconds(250);
  if (this->tx_enable_pin_ != nullptr) this->tx_enable_pin_->digital_write(false);

  // Receive
  uint32_t start = millis();
  std::vector<uint8_t> buf;
  while (millis() - start < this->receive_timeout_ms_) {
    while (this->available()) {
      int c = this->read();
      if (c < 0) break;
      buf.push_back((uint8_t)c);
      if (buf.size() > 5 && buf[0] == DEVICE_ADDR && buf[1] == FC_READ && buf[2] + 5 == buf.size()) {
        uint16_t rcrc = crc16(buf.data(), (uint8_t)buf.size());
        if (rcrc != 0) return false;
        // Extract words from payload
        uint8_t bytes = buf[2];
        out.clear();
        for (uint8_t i = 0; i < bytes / 2; i++) {
          uint16_t w = (buf[3 + i * 2] << 8) | buf[4 + i * 2];
          out.push_back(w);
        }
    ESP_LOGVV(TAG, "READ rsp: %u words", (unsigned) out.size());
        return true;
      }
    }
    // brief yield
    delay(1);
  }
  ESP_LOGW(TAG, "READ timeout cat=%u page=%u idx=%u cnt=%u", category, page, index, count);
  return false;
}

bool WavinAHC9000::write_register(uint8_t category, uint8_t page, uint8_t index, uint16_t value) {
  uint8_t msg[10];
  msg[0] = DEVICE_ADDR;
  msg[1] = FC_WRITE;
  msg[2] = category;
  msg[3] = index;
  msg[4] = page;
  msg[5] = 1;
  msg[6] = value >> 8;
  msg[7] = value & 0xFF;
  uint16_t crc = crc16(msg, 8);
  msg[8] = crc & 0xFF;
  msg[9] = crc >> 8;

  if (this->tx_enable_pin_ != nullptr) this->tx_enable_pin_->digital_write(true);
  ESP_LOGVV(TAG, "WRITE req: cat=%u page=%u idx=%u val=%u", category, page, index, value);
  this->write_array(msg, 10);
  this->flush();
  delayMicroseconds(250);
  if (this->tx_enable_pin_ != nullptr) this->tx_enable_pin_->digital_write(false);

  // Read and ignore ack
  std::vector<uint8_t> buf;
  uint32_t start = millis();
  while (millis() - start < this->receive_timeout_ms_) {
    while (this->available()) {
      int c = this->read();
      if (c < 0) break;
      buf.push_back((uint8_t)c);
      if (buf.size() > 5 && buf[0] == DEVICE_ADDR && buf[1] == FC_WRITE && buf[2] + 5 == buf.size()) {
        uint16_t rcrc = crc16(buf.data(), (uint8_t)buf.size());
  bool ok = rcrc == 0;
  ESP_LOGVV(TAG, "WRITE ack: %s", ok ? "OK" : "BAD CRC");
  return ok;
      }
    }
    delay(1);
  }
  return false;
}

bool WavinAHC9000::write_masked_register(uint8_t category, uint8_t page, uint8_t index, uint16_t value, uint16_t mask) {
  uint8_t msg[12];
  msg[0] = DEVICE_ADDR;
  msg[1] = FC_WRITE_MASKED;
  msg[2] = category;
  msg[3] = index;
  msg[4] = page;
  msg[5] = 1;
  msg[6] = value >> 8;
  msg[7] = value & 0xFF;
  msg[8] = mask >> 8;
  msg[9] = mask & 0xFF;
  uint16_t crc = crc16(msg, 10);
  msg[10] = crc & 0xFF;
  msg[11] = crc >> 8;

  if (this->tx_enable_pin_ != nullptr) this->tx_enable_pin_->digital_write(true);
  ESP_LOGVV(TAG, "WMASK req: cat=%u page=%u idx=%u val=%u mask=0x%04X", category, page, index, value, mask);
  this->write_array(msg, 12);
  this->flush();
  delayMicroseconds(250);
  if (this->tx_enable_pin_ != nullptr) this->tx_enable_pin_->digital_write(false);

  // Read and ignore ack
  std::vector<uint8_t> buf;
  uint32_t start = millis();
  while (millis() - start < this->receive_timeout_ms_) {
    while (this->available()) {
      int c = this->read();
      if (c < 0) break;
      buf.push_back((uint8_t)c);
      if (buf.size() > 5 && buf[0] == DEVICE_ADDR && buf[1] == FC_WRITE_MASKED && buf[2] + 5 == buf.size()) {
        uint16_t rcrc = crc16(buf.data(), (uint8_t)buf.size());
  bool ok = rcrc == 0;
  ESP_LOGVV(TAG, "WMASK ack: %s", ok ? "OK" : "BAD CRC");
  return ok;
      }
    }
    delay(1);
  }
  return false;
}

void WavinAHC9000::publish_updates() {
  ESP_LOGV(TAG, "Publishing updates to %u single climates and %u group climates", (unsigned) this->single_ch_climates_.size(), (unsigned) this->group_climates_.size());
  for (auto *c : this->single_ch_climates_) c->update_from_parent();
  for (auto *c : this->group_climates_) c->update_from_parent();
}

// ---- Climate ----

void WavinZoneClimate::dump_config() {
  LOG_CLIMATE("  ", "Wavin Zone Climate", this);
}

climate::ClimateTraits WavinZoneClimate::traits() {
  climate::ClimateTraits t;
  t.set_supported_modes({climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_OFF});
  t.set_visual_min_temperature(5);
  t.set_visual_max_temperature(35);
  t.set_visual_temperature_step(0.5f);
  t.set_supports_current_temperature(true);
  t.set_supports_action(true);
  t.set_supports_two_point_target_temperature(false);
  return t;
}

void WavinZoneClimate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value()) {
    auto m = *call.get_mode();
    // Support OFF as standby, HEAT as manual
    if (this->single_channel_set_) {
      this->parent_->write_channel_mode(this->single_channel_, m);
    } else if (!this->members_.empty()) {
      for (auto ch : this->members_) this->parent_->write_channel_mode(ch, m);
    }
    this->mode = (m == climate::CLIMATE_MODE_OFF) ? climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;
  }

  if (call.get_target_temperature().has_value()) {
    float t = *call.get_target_temperature();
    if (this->single_channel_set_) {
      this->parent_->write_channel_setpoint(this->single_channel_, t);
    } else if (!this->members_.empty()) {
      this->parent_->write_group_setpoint(this->members_, t);
    }
    this->target_temperature = t;
    this->publish_state();
  }
}

void WavinZoneClimate::update_from_parent() {
  if (this->single_channel_set_) {
    uint8_t ch = this->single_channel_;
    this->current_temperature = this->parent_->get_channel_current_temp(ch);
    this->target_temperature = this->parent_->get_channel_setpoint(ch);
    // Use parent-provided mode (HEAT or OFF)
    this->mode = this->parent_->get_channel_mode(ch);
    this->action = this->parent_->get_channel_action(ch);
  } else if (!this->members_.empty()) {
    // Aggregate: average temps, setpoint; action = any heating => heating
    float sum_curr = 0, sum_set = 0;
    int n = 0;
    bool any_heat = false;
    bool all_off = true;
    for (auto ch : this->members_) {
      float c = this->parent_->get_channel_current_temp(ch);
      float s = this->parent_->get_channel_setpoint(ch);
      if (!std::isnan(c)) {
        sum_curr += c;
        n++;
      }
      if (!std::isnan(s)) sum_set += s;
      if (this->parent_->get_channel_action(ch) == climate::CLIMATE_ACTION_HEATING) any_heat = true;
      if (this->parent_->get_channel_mode(ch) != climate::CLIMATE_MODE_OFF) all_off = false;
    }
    if (n > 0) this->current_temperature = sum_curr / n;
    if (!this->members_.empty()) this->target_temperature = sum_set / this->members_.size();
    this->mode = all_off ? climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;
    this->action = any_heat ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_IDLE;
  }
  this->publish_state();
}

}  // namespace wavin_ahc9000
}  // namespace esphome
