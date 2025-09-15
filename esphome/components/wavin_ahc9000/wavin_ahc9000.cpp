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
  // Poll a subset of channels per cycle; each channel advances one small step
  bool any_wrap = false;
  for (uint8_t i = 0; i < this->poll_channels_per_cycle_; i++) {
    uint8_t ch = this->next_channel_;
    this->request_status_channel(ch);
    this->next_channel_ = (uint8_t)((this->next_channel_ + 1) % 16);
    if (this->next_channel_ == 0) any_wrap = true;
  }
  // Publish periodically; also publish every 5 cycles to keep HA fresh
  this->publish_updates();
}

void WavinAHC9000::dump_config() {
  #include "wavin_ahc9000.h"
  #include "esphome/core/log.h"

  namespace esphome {
  namespace wavin_ahc9000 {

  static const char *const TAG = "wavin_ahc9000";

  void WavinAHC9000::setup() { ESP_LOGCONFIG(TAG, "Wavin AHC9000 hub setup"); }
  void WavinAHC9000::loop() {}
  void WavinAHC9000::update() {}
  void WavinAHC9000::dump_config() { ESP_LOGCONFIG(TAG, "Wavin AHC9000 (minimal scaffold)"); }

  void WavinAHC9000::add_channel_climate(WavinZoneClimate *c) { this->single_ch_climates_.push_back(c); }
  void WavinAHC9000::add_group_climate(WavinZoneClimate *c) { this->group_climates_.push_back(c); }

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
