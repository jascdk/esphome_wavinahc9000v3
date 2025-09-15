#include "wavin_ahc9000.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>
#include <cmath>

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
  // Round-robin staged reads across channels; each channel advances one small step per update
  std::vector<uint16_t> regs;
  for (uint8_t i = 0; i < this->poll_channels_per_cycle_; i++) {
    uint8_t ch_page = this->next_channel_;   // 0..15
    uint8_t ch_num = (uint8_t) (ch_page + 1); // 1..16
    auto &st = this->channels_[ch_num];
    uint8_t &step = this->channel_step_[ch_page];

    // Two steps per update to surface values faster
    for (int s = 0; s < 2; s++) {
      switch (step) {
        case 0: {
          if (this->read_registers(CAT_CHANNELS, ch_page, CH_PRIMARY_ELEMENT, 1, regs) && regs.size() >= 1) {
            uint16_t v = regs[0];
            st.primary_index = v & CH_PRIMARY_ELEMENT_ELEMENT_MASK;
            st.all_tp_lost = (v & CH_PRIMARY_ELEMENT_ALL_TP_LOST_MASK) != 0;
            ESP_LOGD(TAG, "CH%u primary elem=%u lost=%s", ch_num, (unsigned) st.primary_index, st.all_tp_lost ? "Y" : "N");
          } else {
            ESP_LOGW(TAG, "CH%u: primary element read failed", ch_num);
          }
          step = 1;
          break;
        }
        case 1: {
          if (this->read_registers(CAT_PACKED, ch_page, PACKED_CONFIGURATION, 1, regs) && regs.size() >= 1) {
            uint16_t mode_bits = regs[0] & PACKED_CONFIGURATION_MODE_MASK;
            st.mode = (mode_bits == PACKED_CONFIGURATION_MODE_STANDBY) ? climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;
            ESP_LOGD(TAG, "CH%u mode=%s", ch_num, st.mode == climate::CLIMATE_MODE_OFF ? "OFF" : "HEAT");
          } else {
            ESP_LOGW(TAG, "CH%u: mode read failed", ch_num);
          }
          step = 2;
          break;
        }
        case 2: {
          if (this->read_registers(CAT_PACKED, ch_page, PACKED_MANUAL_TEMPERATURE, 1, regs) && regs.size() >= 1) {
            st.setpoint_c = this->raw_to_c(regs[0]);
            ESP_LOGD(TAG, "CH%u setpoint=%.1fC", ch_num, st.setpoint_c);
          } else {
            ESP_LOGW(TAG, "CH%u: setpoint read failed", ch_num);
          }
          step = 3;
          break;
        }
        case 3: {
          if (this->read_registers(CAT_CHANNELS, ch_page, CH_TIMER_EVENT, 1, regs) && regs.size() >= 1) {
            bool heating = (regs[0] & CH_TIMER_EVENT_OUTP_ON_MASK) != 0;
            st.action = heating ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_IDLE;
            ESP_LOGD(TAG, "CH%u action=%s", ch_num, heating ? "HEATING" : "IDLE");
          } else {
            ESP_LOGW(TAG, "CH%u: action read failed", ch_num);
          }
          step = 4;
          break;
        }
        case 4: {
          if (!st.all_tp_lost && st.primary_index > 0) {
            uint8_t elem_page = (uint8_t) (st.primary_index - 1);
            if (this->read_registers(CAT_ELEMENTS, elem_page, 0x00, 11, regs) && regs.size() > ELEM_AIR_TEMPERATURE) {
              st.current_temp_c = this->raw_to_c(regs[ELEM_AIR_TEMPERATURE]);
              ESP_LOGD(TAG, "CH%u current=%.1fC", ch_num, st.current_temp_c);
              // Battery status if available (0..10 scale)
              if (regs.size() > ELEM_BATTERY_STATUS) {
                uint16_t raw = regs[ELEM_BATTERY_STATUS];
                uint8_t steps = (raw > 10) ? 10 : (uint8_t) raw;
                uint8_t pct = (uint8_t) (steps * 10);
                st.battery_pct = pct;
                auto it = this->battery_sensors_.find(ch_num);
                if (it != this->battery_sensors_.end() && it->second != nullptr) {
                  it->second->publish_state((float) pct);
                }
              }
            } else {
              ESP_LOGW(TAG, "CH%u: element temp read failed", ch_num);
            }
          } else {
            st.current_temp_c = NAN;
          }
          step = 0;
          break;
        }
      }
    }

    // advance channel
    this->next_channel_ = (uint8_t) ((this->next_channel_ + 1) % 16);
  }

  // publish once per cycle
  this->publish_updates();
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
  ESP_LOGD(TAG, "TX: addr=0x%02X fc=0x%02X cat=%u idx=%u page=%u cnt=%u", msg[0], msg[1], category, index, page, count);
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

bool WavinAHC9000::write_register(uint8_t category, uint8_t page, uint8_t index, uint16_t value) {
  uint8_t msg[10];
  msg[0] = DEVICE_ADDR;
  msg[1] = FC_WRITE;
  msg[2] = category;
  msg[3] = index;
  msg[4] = page;
  msg[5] = 1;  // count
  msg[6] = (uint8_t) (value >> 8);
  msg[7] = (uint8_t) (value & 0xFF);
  uint16_t crc = crc16(msg, 8);
  msg[8] = (uint8_t) (crc & 0xFF);
  msg[9] = (uint8_t) (crc >> 8);

  if (this->tx_enable_pin_ != nullptr) this->tx_enable_pin_->digital_write(true);
  ESP_LOGD(TAG, "TX-WR: cat=%u idx=%u page=%u val=0x%04X", category, index, page, (unsigned) value);
  this->write_array(msg, 10);
  this->flush();
  delayMicroseconds(250);
  if (this->tx_enable_pin_ != nullptr) this->tx_enable_pin_->digital_write(false);

  // Read ack
  std::vector<uint8_t> buf;
  uint32_t start = millis();
  while (millis() - start < this->receive_timeout_ms_) {
    while (this->available()) {
      int c = this->read();
      if (c < 0) break;
      buf.push_back((uint8_t) c);
      if (buf.size() >= 5) {
        uint8_t expected = (uint8_t) (buf[2] + 5);
        if (buf[0] == DEVICE_ADDR && buf[1] == FC_WRITE && buf.size() == expected) {
          uint16_t rcrc = crc16(buf.data(), buf.size());
          bool ok = (rcrc == 0);
          ESP_LOGD(TAG, "ACK-WR: %s", ok ? "OK" : "BAD-CRC");
          return ok;
        }
      }
    }
    delay(1);
  }
  ESP_LOGW(TAG, "ACK-WR: timeout");
  return false;
}

bool WavinAHC9000::write_masked_register(uint8_t category, uint8_t page, uint8_t index, uint16_t value, uint16_t mask) {
  uint8_t msg[12];
  msg[0] = DEVICE_ADDR;
  msg[1] = FC_WRITE_MASKED;
  msg[2] = category;
  msg[3] = index;
  msg[4] = page;
  msg[5] = 1;  // count
  msg[6] = (uint8_t) (value >> 8);
  msg[7] = (uint8_t) (value & 0xFF);
  msg[8] = (uint8_t) (mask >> 8);
  msg[9] = (uint8_t) (mask & 0xFF);
  uint16_t crc = crc16(msg, 10);
  msg[10] = (uint8_t) (crc & 0xFF);
  msg[11] = (uint8_t) (crc >> 8);

  if (this->tx_enable_pin_ != nullptr) this->tx_enable_pin_->digital_write(true);
  ESP_LOGD(TAG, "TX-WM: cat=%u idx=%u page=%u val=0x%04X mask=0x%04X", category, index, page, (unsigned) value, (unsigned) mask);
  this->write_array(msg, 12);
  this->flush();
  delayMicroseconds(250);
  if (this->tx_enable_pin_ != nullptr) this->tx_enable_pin_->digital_write(false);

  // Read ack
  std::vector<uint8_t> buf;
  uint32_t start = millis();
  while (millis() - start < this->receive_timeout_ms_) {
    while (this->available()) {
      int c = this->read();
      if (c < 0) break;
      buf.push_back((uint8_t) c);
      if (buf.size() >= 5) {
        uint8_t expected = (uint8_t) (buf[2] + 5);
        if (buf[0] == DEVICE_ADDR && buf[1] == FC_WRITE_MASKED && buf.size() == expected) {
          uint16_t rcrc = crc16(buf.data(), buf.size());
          bool ok = (rcrc == 0);
          ESP_LOGD(TAG, "ACK-WM: %s", ok ? "OK" : "BAD-CRC");
          return ok;
        }
      }
    }
    delay(1);
  }
  ESP_LOGW(TAG, "ACK-WM: timeout");
  return false;
}

// High-level write helpers
void WavinAHC9000::write_channel_setpoint(uint8_t channel, float celsius) {
  if (channel < 1 || channel > 16) return;
  uint8_t page = (uint8_t) (channel - 1);
  uint16_t raw = this->c_to_raw(celsius);
  if (this->write_register(CAT_PACKED, page, PACKED_MANUAL_TEMPERATURE, raw)) {
    this->channels_[channel].setpoint_c = celsius;
    this->refresh_channel_now(channel);
  }
}

void WavinAHC9000::write_group_setpoint(const std::vector<uint8_t> &members, float celsius) {
  for (auto ch : members) this->write_channel_setpoint(ch, celsius);
}

void WavinAHC9000::write_channel_mode(uint8_t channel, climate::ClimateMode mode) {
  if (channel < 1 || channel > 16) return;
  uint8_t page = (uint8_t) (channel - 1);
  uint16_t value_bits = (mode == climate::CLIMATE_MODE_OFF) ? PACKED_CONFIGURATION_MODE_STANDBY : PACKED_CONFIGURATION_MODE_MANUAL;
  if (this->write_masked_register(CAT_PACKED, page, PACKED_CONFIGURATION, value_bits, PACKED_CONFIGURATION_MODE_MASK)) {
    this->channels_[channel].mode = (mode == climate::CLIMATE_MODE_OFF) ? climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;
    this->refresh_channel_now(channel);
  }
}

void WavinAHC9000::refresh_channel_now(uint8_t channel) {
  if (channel < 1 || channel > 16) return;
  uint8_t ch_page = (uint8_t) (channel - 1);
  auto &st = this->channels_[channel];

  std::vector<uint16_t> regs;
  // Mode/config
  if (this->read_registers(CAT_PACKED, ch_page, PACKED_CONFIGURATION, 1, regs) && regs.size() >= 1) {
    uint16_t mode_bits = regs[0] & PACKED_CONFIGURATION_MODE_MASK;
    st.mode = (mode_bits == PACKED_CONFIGURATION_MODE_STANDBY) ? climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;
  }
  // Setpoint
  if (this->read_registers(CAT_PACKED, ch_page, PACKED_MANUAL_TEMPERATURE, 1, regs) && regs.size() >= 1) {
    st.setpoint_c = this->raw_to_c(regs[0]);
  }
  // Action
  if (this->read_registers(CAT_CHANNELS, ch_page, CH_TIMER_EVENT, 1, regs) && regs.size() >= 1) {
    bool heating = (regs[0] & CH_TIMER_EVENT_OUTP_ON_MASK) != 0;
    st.action = heating ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_IDLE;
  }
  // Temperature via element
  if (!st.all_tp_lost && st.primary_index > 0) {
    uint8_t elem_page = (uint8_t) (st.primary_index - 1);
    if (this->read_registers(CAT_ELEMENTS, elem_page, 0x00, 11, regs) && regs.size() > ELEM_AIR_TEMPERATURE) {
      st.current_temp_c = this->raw_to_c(regs[ELEM_AIR_TEMPERATURE]);
    }
  }
  this->publish_updates();
}

void WavinAHC9000::publish_updates() {
  ESP_LOGV(TAG, "Publishing updates: %u single climates, %u group climates",
           (unsigned) this->single_ch_climates_.size(), (unsigned) this->group_climates_.size());
  for (auto *c : this->single_ch_climates_) c->update_from_parent();
  for (auto *c : this->group_climates_) c->update_from_parent();
}

float WavinAHC9000::get_channel_current_temp(uint8_t channel) const {
  auto it = this->channels_.find(channel);
  return it == this->channels_.end() ? NAN : it->second.current_temp_c;
}
float WavinAHC9000::get_channel_setpoint(uint8_t channel) const {
  auto it = this->channels_.find(channel);
  return it == this->channels_.end() ? NAN : it->second.setpoint_c;
}
climate::ClimateMode WavinAHC9000::get_channel_mode(uint8_t channel) const {
  auto it = this->channels_.find(channel);
  return it == this->channels_.end() ? climate::CLIMATE_MODE_HEAT : it->second.mode;
}
climate::ClimateAction WavinAHC9000::get_channel_action(uint8_t channel) const {
  auto it = this->channels_.find(channel);
  return it == this->channels_.end() ? climate::CLIMATE_ACTION_OFF : it->second.action;
}

void WavinZoneClimate::dump_config() { LOG_CLIMATE("  ", "Wavin Zone Climate (minimal)", this); }
climate::ClimateTraits WavinZoneClimate::traits() {
  climate::ClimateTraits t;
  t.set_supported_modes({climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_OFF});
  t.set_supports_current_temperature(true);
  t.set_supports_action(true);
  t.set_visual_min_temperature(5);
  t.set_visual_max_temperature(35);
  t.set_visual_temperature_step(0.5f);
  return t;
}
void WavinZoneClimate::control(const climate::ClimateCall &call) {
  // Mode control
  if (call.get_mode().has_value()) {
    auto m = *call.get_mode();
    if (this->single_channel_set_) {
      this->parent_->write_channel_mode(this->single_channel_, m);
    } else if (!this->members_.empty()) {
      for (auto ch : this->members_) this->parent_->write_channel_mode(ch, m);
    }
    this->mode = (m == climate::CLIMATE_MODE_OFF) ? climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;
  }

  // Target temperature
  if (call.get_target_temperature().has_value()) {
    float t = *call.get_target_temperature();
    if (this->single_channel_set_) {
      this->parent_->write_channel_setpoint(this->single_channel_, t);
    } else if (!this->members_.empty()) {
      this->parent_->write_group_setpoint(this->members_, t);
    }
    this->target_temperature = t;
  }

  this->publish_state();
}
void WavinZoneClimate::update_from_parent() {
  if (this->single_channel_set_) {
    uint8_t ch = this->single_channel_;
    this->current_temperature = this->parent_->get_channel_current_temp(ch);
    this->target_temperature = this->parent_->get_channel_setpoint(ch);
    this->mode = this->parent_->get_channel_mode(ch);
    // Action: derive from temperatures with a small deadband, fallback to controller bit
    const float db = 0.3f;  // hysteresis in °C
    auto raw_action = this->parent_->get_channel_action(ch);
    if (!std::isnan(this->current_temperature) && !std::isnan(this->target_temperature)) {
      if (this->current_temperature > this->target_temperature + db) {
        this->action = climate::CLIMATE_ACTION_IDLE;
      } else if (this->current_temperature < this->target_temperature - db) {
        this->action = climate::CLIMATE_ACTION_HEATING;
      } else {
        this->action = raw_action;
      }
    } else {
      this->action = raw_action;
    }
  } else if (!this->members_.empty()) {
    float sum_curr = 0.0f, sum_set = 0.0f;
    int n_curr = 0;
    bool any_heat = false;
    bool all_off = true;
    for (auto ch : this->members_) {
      float c = this->parent_->get_channel_current_temp(ch);
      if (!std::isnan(c)) {
        sum_curr += c;
        n_curr++;
      }
      float s = this->parent_->get_channel_setpoint(ch);
      if (!std::isnan(s)) sum_set += s;
      if (this->parent_->get_channel_action(ch) == climate::CLIMATE_ACTION_HEATING) any_heat = true;
      if (this->parent_->get_channel_mode(ch) != climate::CLIMATE_MODE_OFF) all_off = false;
    }
    if (n_curr > 0) this->current_temperature = sum_curr / n_curr;
    if (!this->members_.empty()) this->target_temperature = sum_set / this->members_.size();
    this->mode = all_off ? climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;
    // Group action: prefer temperature comparison with deadband, fallback to any member heating
    const float db = 0.3f;
    if (!std::isnan(this->current_temperature) && !std::isnan(this->target_temperature)) {
      if (this->current_temperature > this->target_temperature + db) {
        this->action = climate::CLIMATE_ACTION_IDLE;
      } else if (this->current_temperature < this->target_temperature - db) {
        this->action = climate::CLIMATE_ACTION_HEATING;
      } else {
        this->action = any_heat ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_IDLE;
      }
    } else {
      this->action = any_heat ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_IDLE;
    }
  }
  this->publish_state();
}

}  // namespace wavin_ahc9000
}  // namespace esphome
