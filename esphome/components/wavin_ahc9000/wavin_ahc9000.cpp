#include "wavin_ahc9000.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>
#include <cmath>
#include <algorithm>

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
  // If polling is temporarily suspended (after a write), skip until window expires
  if (this->suspend_polling_until_ != 0 && millis() < this->suspend_polling_until_) {
    ESP_LOGV(TAG, "Polling suspended for %u ms more", (unsigned) (this->suspend_polling_until_ - millis()));
    return;
  }

  // Process any urgent channels first (scheduled due to a write)
  std::vector<uint16_t> regs;
  uint8_t urgent_processed = 0;
  while (!this->urgent_channels_.empty() && urgent_processed < this->poll_channels_per_cycle_) {
    uint8_t ch = this->urgent_channels_.front();
    this->urgent_channels_.erase(this->urgent_channels_.begin());
    uint8_t ch_page = (uint8_t) (ch - 1);
    auto &st = this->channels_[ch];
    // Perform a compact refresh sequence for the channel
    if (this->read_registers(CAT_PACKED, ch_page, PACKED_CONFIGURATION, 1, regs) && regs.size() >= 1) {
      uint16_t raw_cfg = regs[0];
      uint16_t mode_bits = raw_cfg & PACKED_CONFIGURATION_MODE_MASK;
      bool is_off = (mode_bits == PACKED_CONFIGURATION_MODE_STANDBY) || (mode_bits == PACKED_CONFIGURATION_MODE_STANDBY_ALT);
      st.mode = is_off ? climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;
      ESP_LOGD(TAG, "CH%u cfg=0x%04X mode=%s", (unsigned) ch, (unsigned) raw_cfg, is_off ? "OFF" : "HEAT");
      // Reconcile desired mode if pending and mismatch
      auto it_des = this->desired_mode_.find(ch);
      if (it_des != this->desired_mode_.end()) {
        auto want = it_des->second;
        if (want != st.mode) {
          uint16_t current = raw_cfg;
          // Enforce standard OFF bits or MANUAL
          uint16_t new_bits = (want == climate::CLIMATE_MODE_OFF) ? PACKED_CONFIGURATION_MODE_STANDBY : PACKED_CONFIGURATION_MODE_MANUAL;
          uint16_t next = (uint16_t) ((current & ~PACKED_CONFIGURATION_MODE_MASK) | (new_bits & PACKED_CONFIGURATION_MODE_MASK));
          ESP_LOGW(TAG, "Reconciling mode for ch=%u cur=0x%04X next=0x%04X", (unsigned) ch, (unsigned) current, (unsigned) next);
          if (this->write_register(CAT_PACKED, ch_page, PACKED_CONFIGURATION, next)) {
            // Schedule another quick check
            this->urgent_channels_.push_back(ch);
            this->suspend_polling_until_ = millis() + 100;
          }
        } else {
          // Achieved desired mode; clear desire
          this->desired_mode_.erase(it_des);
        }
      }
    }
    if (this->read_registers(CAT_PACKED, ch_page, PACKED_MANUAL_TEMPERATURE, 1, regs) && regs.size() >= 1) {
      st.setpoint_c = this->raw_to_c(regs[0]);
    }
    if (this->read_registers(CAT_CHANNELS, ch_page, CH_TIMER_EVENT, 1, regs) && regs.size() >= 1) {
      bool heating = (regs[0] & CH_TIMER_EVENT_OUTP_ON_MASK) != 0;
      st.action = heating ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_IDLE;
    }
    if (!st.all_tp_lost && st.primary_index > 0) {
      uint8_t elem_page = (uint8_t) (st.primary_index - 1);
      if (this->read_registers(CAT_ELEMENTS, elem_page, 0x00, 11, regs) && regs.size() > ELEM_AIR_TEMPERATURE) {
        st.current_temp_c = this->raw_to_c(regs[ELEM_AIR_TEMPERATURE]);
      }
    }
    urgent_processed++;
  }

  // Round-robin staged reads across active channels; each advances one step per update
  if (this->active_channels_.empty()) {
    // Default to all 1..16 if none explicitly configured
    this->active_channels_.reserve(16);
    for (uint8_t ch = 1; ch <= 16; ch++) this->active_channels_.push_back(ch);
  }

  for (uint8_t i = urgent_processed; i < this->poll_channels_per_cycle_ && !this->active_channels_.empty(); i++) {
    // Wrap active index
    if (this->next_active_index_ >= this->active_channels_.size()) this->next_active_index_ = 0;
    uint8_t ch_num = this->active_channels_[this->next_active_index_]; // 1..16
    uint8_t ch_page = (uint8_t) (ch_num - 1);
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
            uint16_t raw_cfg = regs[0];
            uint16_t mode_bits = raw_cfg & PACKED_CONFIGURATION_MODE_MASK;
            bool is_off = (mode_bits == PACKED_CONFIGURATION_MODE_STANDBY) || (mode_bits == PACKED_CONFIGURATION_MODE_STANDBY_ALT);
            st.mode = is_off ? climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;
            ESP_LOGD(TAG, "CH%u cfg=0x%04X mode=%s", ch_num, (unsigned) raw_cfg, is_off ? "OFF" : "HEAT");
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
              // Publish to per-channel temperature sensor if configured
              auto it_t = this->temperature_sensors_.find(ch_num);
              if (it_t != this->temperature_sensors_.end() && it_t->second != nullptr && !std::isnan(st.current_temp_c)) {
                it_t->second->publish_state(st.current_temp_c);
              }
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

  // advance to next active channel
  this->next_active_index_ = (uint8_t) ((this->next_active_index_ + 1) % this->active_channels_.size());
  }

  // publish once per cycle
  this->publish_updates();
}

void WavinAHC9000::dump_config() { ESP_LOGCONFIG(TAG, "Wavin AHC9000 (UART test read)"); }

void WavinAHC9000::add_channel_climate(WavinZoneClimate *c) { this->single_ch_climates_.push_back(c); }
void WavinAHC9000::add_group_climate(WavinZoneClimate *c) { this->group_climates_.push_back(c); }
void WavinAHC9000::add_active_channel(uint8_t ch) {
  if (ch < 1 || ch > 16) return;
  if (std::find(this->active_channels_.begin(), this->active_channels_.end(), ch) == this->active_channels_.end()) {
    this->active_channels_.push_back(ch);
  }
}

// Repair functions removed; use normalize_channel_config via API service

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

bool WavinAHC9000::write_masked_register(uint8_t category, uint8_t page, uint8_t index, uint16_t and_mask, uint16_t or_mask) {
  uint8_t msg[12];
  msg[0] = DEVICE_ADDR;
  msg[1] = FC_WRITE_MASKED;
  msg[2] = category;
  msg[3] = index;
  msg[4] = page;
  msg[5] = 1;  // count
  msg[6] = (uint8_t) (and_mask >> 8);
  msg[7] = (uint8_t) (and_mask & 0xFF);
  msg[8] = (uint8_t) (or_mask >> 8);
  msg[9] = (uint8_t) (or_mask & 0xFF);
  uint16_t crc = crc16(msg, 10);
  msg[10] = (uint8_t) (crc & 0xFF);
  msg[11] = (uint8_t) (crc >> 8);

  if (this->tx_enable_pin_ != nullptr) this->tx_enable_pin_->digital_write(true);
  ESP_LOGD(TAG, "TX-WM: cat=%u idx=%u page=%u and=0x%04X or=0x%04X", category, index, page, (unsigned) and_mask, (unsigned) or_mask);
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
  // Schedule a quick refresh on next cycle and briefly suspend normal polling to avoid collisions
  this->urgent_channels_.push_back(channel);
  this->suspend_polling_until_ = millis() + 100; // 100 ms guard
  }
}

void WavinAHC9000::write_group_setpoint(const std::vector<uint8_t> &members, float celsius) {
  for (auto ch : members) this->write_channel_setpoint(ch, celsius);
}

void WavinAHC9000::write_channel_mode(uint8_t channel, climate::ClimateMode mode) {
  if (channel < 1 || channel > 16) return;
  uint8_t page = (uint8_t) (channel - 1);
  this->desired_mode_[channel] = mode;
  // Always use strict baseline to 0x4000/0x4001 for reliable OFF/HEAT
  bool ok = false;
  {
    uint16_t strict_val = (uint16_t) (0x4000 | (mode == climate::CLIMATE_MODE_OFF ? PACKED_CONFIGURATION_MODE_STANDBY : PACKED_CONFIGURATION_MODE_MANUAL));
    ok = this->write_register(CAT_PACKED, page, PACKED_CONFIGURATION, strict_val);
  }
  if (!ok) {
    // Fallback: read-modify-write full register (update only mode bits)
    std::vector<uint16_t> regs;
    if (this->read_registers(CAT_PACKED, page, PACKED_CONFIGURATION, 1, regs) && regs.size() >= 1) {
      uint16_t current = regs[0];
      // Prefer standard standby bits for OFF; otherwise use MANUAL
      uint16_t new_bits = (mode == climate::CLIMATE_MODE_OFF) ? PACKED_CONFIGURATION_MODE_STANDBY : PACKED_CONFIGURATION_MODE_MANUAL;
      uint16_t next = (uint16_t) ((current & ~PACKED_CONFIGURATION_MODE_MASK) | (new_bits & PACKED_CONFIGURATION_MODE_MASK));
      ESP_LOGW(TAG, "WM fallback: PACKED_CONFIGURATION ch=%u cur=0x%04X next=0x%04X", (unsigned) channel, (unsigned) current, (unsigned) next);
      ok = this->write_register(CAT_PACKED, page, PACKED_CONFIGURATION, next);
  // No alternate OFF attempt to avoid special thermostat modes
    } else {
      ESP_LOGW(TAG, "WM fallback: read PACKED_CONFIGURATION failed for ch=%u", (unsigned) channel);
    }
  }
  if (ok) {
    this->channels_[channel].mode = (mode == climate::CLIMATE_MODE_OFF) ? climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;
    this->urgent_channels_.push_back(channel);
    this->suspend_polling_until_ = millis() + 100; // 100 ms guard
  } else {
    ESP_LOGW(TAG, "Mode write failed for ch=%u", (unsigned) channel);
  }
}

void WavinAHC9000::set_strict_mode_write(uint8_t channel, bool enable) {
  if (channel < 1 || channel > 16) return;
  if (enable) this->strict_mode_channels_.insert(channel);
  else this->strict_mode_channels_.erase(channel);
}
bool WavinAHC9000::is_strict_mode_write(uint8_t channel) const {
  return this->strict_mode_channels_.find(channel) != this->strict_mode_channels_.end();
}

void WavinAHC9000::refresh_channel_now(uint8_t channel) {
  if (channel < 1 || channel > 16) return;
  // Just schedule urgent refresh; actual reads happen in update()
  this->urgent_channels_.push_back(channel);
}

void WavinAHC9000::normalize_channel_config(uint8_t channel, bool off) {
  if (channel < 1 || channel > 16) return;
  uint8_t page = (uint8_t) (channel - 1);
  // Force PACKED_CONFIGURATION to exact baseline used by healthy channels
  uint16_t value = (uint16_t) (0x4000 | (off ? PACKED_CONFIGURATION_MODE_STANDBY : PACKED_CONFIGURATION_MODE_MANUAL));
  if (this->write_register(CAT_PACKED, page, PACKED_CONFIGURATION, value)) {
    ESP_LOGW(TAG, "Normalize (strict) applied: ch=%u -> 0x%04X", (unsigned) channel, (unsigned) value);
    this->urgent_channels_.push_back(channel);
    this->suspend_polling_until_ = millis() + 100;
  } else {
    ESP_LOGW(TAG, "Normalize (strict) failed: write not acknowledged for ch=%u", (unsigned) channel);
  }
}

void WavinAHC9000::generate_yaml_suggestion() {
  // One-shot discovery sweep: detect active channels immediately (independent of background polling)
  std::vector<uint8_t> active;
  active.reserve(16);
  std::vector<uint16_t> regs;
  for (uint8_t ch = 1; ch <= 16; ch++) {
    uint8_t page = (uint8_t) (ch - 1);
    if (this->read_registers(CAT_CHANNELS, page, CH_PRIMARY_ELEMENT, 1, regs) && regs.size() >= 1) {
      uint16_t v = regs[0];
      uint16_t primary_index = v & CH_PRIMARY_ELEMENT_ELEMENT_MASK;
      bool all_tp_lost = (v & CH_PRIMARY_ELEMENT_ALL_TP_LOST_MASK) != 0;
      if (primary_index > 0 && !all_tp_lost) {
        active.push_back(ch);
        // Opportunistically fill cache (does not change behavior)
        auto &st = this->channels_[ch];
        st.primary_index = primary_index;
        st.all_tp_lost = all_tp_lost;
        // Read basic mode + setpoint so climates look sensible in cache
        if (this->read_registers(CAT_PACKED, page, PACKED_CONFIGURATION, 1, regs) && regs.size() >= 1) {
          uint16_t raw_cfg = regs[0];
          uint16_t mode_bits = raw_cfg & PACKED_CONFIGURATION_MODE_MASK;
          bool is_off = (mode_bits == PACKED_CONFIGURATION_MODE_STANDBY) || (mode_bits == PACKED_CONFIGURATION_MODE_STANDBY_ALT);
          st.mode = is_off ? climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;
        }
        if (this->read_registers(CAT_PACKED, page, PACKED_MANUAL_TEMPERATURE, 1, regs) && regs.size() >= 1) {
          st.setpoint_c = this->raw_to_c(regs[0]);
        }
      }
    }
  }

  // Persist active channels for chunk helpers
  this->yaml_active_channels_ = active;

  // Build three YAML sections separately, then compose
  std::string yaml_climate;
  yaml_climate += "climate:\n";
  for (auto ch : active) {
    yaml_climate += "  - platform: wavin_ahc9000\n";
    yaml_climate += "    wavin_ahc9000_id: wavin\n";
    yaml_climate += "    name: \"Zone " + std::to_string((int) ch) + "\"\n";
    yaml_climate += "    channel: " + std::to_string((int) ch) + "\n";
  }

  std::string yaml_batt;
  yaml_batt += "sensor:\n";
  for (auto ch : active) {
    yaml_batt += "  - platform: wavin_ahc9000\n";
    yaml_batt += "    wavin_ahc9000_id: wavin\n";
    yaml_batt += "    name: \"Zone " + std::to_string((int) ch) + " Battery\"\n";
    yaml_batt += "    channel: " + std::to_string((int) ch) + "\n";
    yaml_batt += "    type: battery\n";
  }

  std::string yaml_temp;
  yaml_temp += "sensor:\n";
  for (auto ch : active) {
    yaml_temp += "  - platform: wavin_ahc9000\n";
    yaml_temp += "    wavin_ahc9000_id: wavin\n";
    yaml_temp += "    name: \"Zone " + std::to_string((int) ch) + " Temperature\"\n";
    yaml_temp += "    channel: " + std::to_string((int) ch) + "\n";
    yaml_temp += "    type: temperature\n";
  }

  std::string out = yaml_climate + "\n" + yaml_batt + "\n" + yaml_temp;

  // Save last YAML and publish to optional text sensor (HA may truncate state >255 chars)
  this->yaml_last_suggestion_ = out;
  this->yaml_last_climate_ = yaml_climate;
  this->yaml_last_battery_ = yaml_batt;
  this->yaml_last_temperature_ = yaml_temp;
  if (this->yaml_text_sensor_ != nullptr) {
    this->yaml_text_sensor_->publish_state(out);
  }


  // Also print with banners (and ANSI color if viewer supports it)
  const char *CYAN = "\x1b[36m";
  const char *GREEN = "\x1b[32m";
  const char *RESET = "\x1b[0m";
  ESP_LOGI(TAG, "%s==================== Wavin YAML SUGGESTION BEGIN ====================%s", CYAN, RESET);
  {
    // Print line by line to avoid single-message truncation in logger
    const char *p = out.c_str();
    const char *line_start = p;
    while (*p) {
      if (*p == '\n') {
        std::string line(line_start, p - line_start);
        ESP_LOGI(TAG, "%s%s%s", GREEN, line.c_str(), RESET);
        ++p;
        line_start = p;
      } else {
        ++p;
      }
    }
    // Last line if not newline-terminated
    if (line_start != p) {
      std::string line(line_start, p - line_start);
      ESP_LOGI(TAG, "%s%s%s", GREEN, line.c_str(), RESET);
    }
  }
  ESP_LOGI(TAG, "%s===================== Wavin YAML SUGGESTION END =====================%s", CYAN, RESET);
}

// --- YAML chunk helpers (whole-entity, not byte size) ---
static std::string build_climate_yaml_for(const std::vector<uint8_t> &chs) {
  // Return only entity blocks, no leading 'climate:' header
  std::string y;
  if (chs.empty()) return y;
  for (auto ch : chs) {
    y += "- platform: wavin_ahc9000\n";
    y += "  wavin_ahc9000_id: wavin\n";
    y += "  name: \"Zone " + std::to_string((int) ch) + "\"\n";
    y += "  channel: " + std::to_string((int) ch) + "\n";
  }
  return y;
}
static std::string build_battery_yaml_for(const std::vector<uint8_t> &chs) {
  // Return only entity blocks, no leading 'sensor:' header
  std::string y;
  if (chs.empty()) return y;
  for (auto ch : chs) {
    y += "- platform: wavin_ahc9000\n";
    y += "  wavin_ahc9000_id: wavin\n";
    y += "  name: \"Zone " + std::to_string((int) ch) + " Battery\"\n";
    y += "  channel: " + std::to_string((int) ch) + "\n";
    y += "  type: battery\n";
  }
  return y;
}
static std::string build_temperature_yaml_for(const std::vector<uint8_t> &chs) {
  // Return only entity blocks, no leading 'sensor:' header
  std::string y;
  if (chs.empty()) return y;
  for (auto ch : chs) {
    y += "- platform: wavin_ahc9000\n";
    y += "  wavin_ahc9000_id: wavin\n";
    y += "  name: \"Zone " + std::to_string((int) ch) + " Temperature\"\n";
    y += "  channel: " + std::to_string((int) ch) + "\n";
    y += "  type: temperature\n";
  }
  return y;
}

std::string WavinAHC9000::get_yaml_climate_chunk(uint8_t start, uint8_t count) const {
  if (start >= this->yaml_active_channels_.size() || count == 0) return std::string("");
  uint8_t end = (uint8_t) std::min<size_t>(this->yaml_active_channels_.size(), (size_t) start + count);
  std::vector<uint8_t> chs(this->yaml_active_channels_.begin() + start, this->yaml_active_channels_.begin() + end);
  return build_climate_yaml_for(chs);
}
std::string WavinAHC9000::get_yaml_battery_chunk(uint8_t start, uint8_t count) const {
  if (start >= this->yaml_active_channels_.size() || count == 0) return std::string("");
  uint8_t end = (uint8_t) std::min<size_t>(this->yaml_active_channels_.size(), (size_t) start + count);
  std::vector<uint8_t> chs(this->yaml_active_channels_.begin() + start, this->yaml_active_channels_.begin() + end);
  return build_battery_yaml_for(chs);
}
std::string WavinAHC9000::get_yaml_temperature_chunk(uint8_t start, uint8_t count) const {
  if (start >= this->yaml_active_channels_.size() || count == 0) return std::string("");
  uint8_t end = (uint8_t) std::min<size_t>(this->yaml_active_channels_.size(), (size_t) start + count);
  std::vector<uint8_t> chs(this->yaml_active_channels_.begin() + start, this->yaml_active_channels_.begin() + end);
  return build_temperature_yaml_for(chs);
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
  ESP_LOGD(TAG, "CTRL: mode=%s for %s", (m == climate::CLIMATE_MODE_OFF ? "OFF" : "HEAT"), this->get_name().c_str());
  if (this->parent_->get_allow_mode_writes()) {
      if (this->single_channel_set_) {
        this->parent_->write_channel_mode(this->single_channel_, m);
      } else if (!this->members_.empty()) {
        for (auto ch : this->members_) this->parent_->write_channel_mode(ch, m);
      }
    } else {
      ESP_LOGW(TAG, "Mode writes disabled by config; skipping write for %s", this->get_name().c_str());
    }
    this->mode = (m == climate::CLIMATE_MODE_OFF) ? climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;
  }

  // Target temperature
  if (call.get_target_temperature().has_value()) {
    float t = *call.get_target_temperature();
  ESP_LOGD(TAG, "CTRL: target=%.1fC for %s", t, this->get_name().c_str());
    // If we're turning OFF in the same call, skip setpoint write to avoid switching back to MANUAL
    bool turning_off = call.get_mode().has_value() && (*call.get_mode() == climate::CLIMATE_MODE_OFF);
    if (!turning_off) {
      if (this->single_channel_set_) {
        this->parent_->write_channel_setpoint(this->single_channel_, t);
      } else if (!this->members_.empty()) {
        this->parent_->write_group_setpoint(this->members_, t);
      }
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
