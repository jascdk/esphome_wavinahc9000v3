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
