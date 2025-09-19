#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/component.h"

#include <vector>
#include <map>
#include <set>
#include <cmath>
#include <string>

namespace esphome {
namespace sensor { class Sensor; }
namespace text_sensor { class TextSensor; }
namespace binary_sensor { class BinarySensor; }
namespace wavin_ahc9000 {

// Forward
class WavinZoneClimate;

class WavinAHC9000 : public PollingComponent, public uart::UARTDevice {
 public:
  void set_temp_divisor(float d) { this->temp_divisor_ = d; }
  void set_receive_timeout_ms(uint32_t t) { this->receive_timeout_ms_ = t; }
  void set_tx_enable_pin(GPIOPin *p) { this->tx_enable_pin_ = p; }
  void set_poll_channels_per_cycle(uint8_t n) { this->poll_channels_per_cycle_ = n == 0 ? 1 : (n > 16 ? 16 : n); }
  void set_allow_mode_writes(bool v) { this->allow_mode_writes_ = v; }
  bool get_allow_mode_writes() const { return this->allow_mode_writes_; }

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  void add_channel_climate(WavinZoneClimate *c);
  void add_group_climate(WavinZoneClimate *c);
  void add_channel_battery_sensor(uint8_t ch, sensor::Sensor *s);
  void add_channel_temperature_sensor(uint8_t ch, sensor::Sensor *s);
  void add_channel_comfort_setpoint_sensor(uint8_t ch, sensor::Sensor *s);
  void add_channel_floor_temperature_sensor(uint8_t ch, sensor::Sensor *s);
  // New read-only floor limit sensors
  void add_channel_floor_min_temperature_sensor(uint8_t ch, sensor::Sensor *s);
  void add_channel_floor_max_temperature_sensor(uint8_t ch, sensor::Sensor *s);
  void add_active_channel(uint8_t ch);

  // Send commands
  void write_channel_setpoint(uint8_t channel, float celsius);
  void write_group_setpoint(const std::vector<uint8_t> &members, float celsius);
  void write_channel_mode(uint8_t channel, climate::ClimateMode mode);
  // Write floor temperature limits (Celsius), clamped to sane bounds
  void write_channel_floor_min_temperature(uint8_t channel, float celsius);
  void write_channel_floor_max_temperature(uint8_t channel, float celsius);
  void refresh_channel_now(uint8_t channel);
  void set_strict_mode_write(uint8_t channel, bool enable);
  bool is_strict_mode_write(uint8_t channel) const;
  void request_status();
  void request_status_channel(uint8_t ch_index);
  void normalize_channel_config(uint8_t channel, bool off);
  void generate_yaml_suggestion();
  void set_yaml_ready_binary_sensor(binary_sensor::BinarySensor *s) { this->yaml_ready_binary_sensor_ = s; }
  void set_yaml_text_sensor(text_sensor::TextSensor *s) { this->yaml_text_sensor_ = s; }
  // Debug helper to dump registers for a channel (to identify floor min/max addresses)
  void dump_channel_floor_limits(uint8_t channel);
  // Accessor for last generated YAML (for HA notifications via lambda)
  std::string get_yaml_suggestion() const { return this->yaml_last_suggestion_; }
  std::string get_yaml_climate() const { return this->yaml_last_climate_; }
  std::string get_yaml_battery() const { return this->yaml_last_battery_; }
  std::string get_yaml_temperature() const { return this->yaml_last_temperature_; }
  std::string get_yaml_floor_temperature() const { return this->yaml_last_floor_temperature_; }
  // Chunk helpers: return YAML entity blocks (complete entities only, NO section header)
  // start is 0-based entity index among discovered active channels; count is number of entities to include
  std::string get_yaml_climate_chunk(uint8_t start, uint8_t count) const;
  std::string get_yaml_comfort_climate_chunk(uint8_t start, uint8_t count) const;
  std::string get_yaml_battery_chunk(uint8_t start, uint8_t count) const;
  std::string get_yaml_temperature_chunk(uint8_t start, uint8_t count) const;
  std::string get_yaml_floor_temperature_chunk(uint8_t start, uint8_t count) const;
  std::string get_yaml_floor_min_temperature_chunk(uint8_t start, uint8_t count) const;
  std::string get_yaml_floor_max_temperature_chunk(uint8_t start, uint8_t count) const;
  uint8_t get_yaml_active_count() const { return (uint8_t) this->yaml_active_channels_.size(); }

  // Data access
  float get_channel_current_temp(uint8_t channel) const;
  float get_channel_setpoint(uint8_t channel) const;
  float get_channel_floor_temp(uint8_t channel) const;
  float get_channel_floor_min_temp(uint8_t channel) const;
  float get_channel_floor_max_temp(uint8_t channel) const;
  climate::ClimateMode get_channel_mode(uint8_t channel) const;
  climate::ClimateAction get_channel_action(uint8_t channel) const;

 protected:
  // Low-level protocol helpers (dkjonas framing)
  bool read_registers(uint8_t category, uint8_t page, uint8_t index, uint8_t count, std::vector<uint16_t> &out);
  bool write_register(uint8_t category, uint8_t page, uint8_t index, uint16_t value);
  // Masked write: apply (reg & and_mask) | or_mask semantics
  bool write_masked_register(uint8_t category, uint8_t page, uint8_t index, uint16_t and_mask, uint16_t or_mask);

  void publish_updates();

  // Helpers
  float raw_to_c(float raw) const { return raw / this->temp_divisor_; }
  uint16_t c_to_raw(float c) const { return static_cast<uint16_t>(c * this->temp_divisor_ + 0.5f); }

  // Simple cache per channel
  struct ChannelState {
    float current_temp_c{NAN};
    float floor_temp_c{NAN};
    // New read-only floor limits (Celsius)
    float floor_min_c{NAN};
    float floor_max_c{NAN};
    float setpoint_c{NAN};
    climate::ClimateMode mode{climate::CLIMATE_MODE_HEAT};
    climate::ClimateAction action{climate::CLIMATE_ACTION_OFF};
    uint8_t battery_pct{255}; // 0..100; 255=unknown
  uint16_t primary_index{0};
  bool all_tp_lost{false};
    bool has_floor_sensor{false};
  };

  std::map<uint8_t, ChannelState> channels_;
  std::vector<WavinZoneClimate *> single_ch_climates_;
  std::vector<WavinZoneClimate *> group_climates_;
  std::map<uint8_t, sensor::Sensor *> battery_sensors_;
  std::map<uint8_t, sensor::Sensor *> temperature_sensors_;
  std::map<uint8_t, sensor::Sensor *> floor_temperature_sensors_;
  // New read-only floor limit sensor maps
  std::map<uint8_t, sensor::Sensor *> floor_min_temperature_sensors_;
  std::map<uint8_t, sensor::Sensor *> floor_max_temperature_sensors_;
  std::map<uint8_t, sensor::Sensor *> comfort_setpoint_sensors_;
  binary_sensor::BinarySensor *yaml_ready_binary_sensor_{nullptr};
  text_sensor::TextSensor *yaml_text_sensor_{nullptr};
  std::string yaml_last_suggestion_{};
  std::string yaml_last_climate_{};
  std::string yaml_last_battery_{};
  std::string yaml_last_temperature_{};
  std::string yaml_last_floor_temperature_{};
  std::vector<uint8_t> yaml_active_channels_{}; // active channels discovered during last YAML generation
  std::vector<uint8_t> yaml_floor_channels_{}; // subset with detected floor sensors during last YAML generation
  std::vector<uint8_t> yaml_comfort_climate_channels_{}; // same as floor subset; for comfort climate generation
  std::vector<uint8_t> active_channels_;
  std::map<uint8_t, climate::ClimateMode> desired_mode_; // desired mode to reconcile after refresh
  std::set<uint8_t> strict_mode_channels_; // channels opting into strict baseline writes

  float temp_divisor_{10.0f};
  uint32_t last_poll_ms_{0};
  uint32_t receive_timeout_ms_{1000};
  uint32_t suspend_polling_until_{0};
  GPIOPin *tx_enable_pin_{nullptr};
  uint8_t poll_channels_per_cycle_{2};
  uint8_t next_active_index_{0};
  uint8_t channel_step_[16] = {0};
  std::vector<uint8_t> urgent_channels_{}; // channels scheduled for immediate refresh on next update
  bool allow_mode_writes_{true};

  // YAML readiness tracking: which channels are present and which had an element block read at least once
  uint16_t yaml_primary_present_mask_{0};  // bit i set when channel (i+1) has a primary element and no tp lost
  uint16_t yaml_elem_read_mask_{0};        // bit i set when we've successfully read the element block for channel (i+1)

  // Protocol constants
  static constexpr uint8_t DEVICE_ADDR = 0x01;
  static constexpr uint8_t FC_READ = 0x43;
  static constexpr uint8_t FC_WRITE = 0x44;
  static constexpr uint8_t FC_WRITE_MASKED = 0x45;

  // Categories & indices (from dkjonas repo)
  static constexpr uint8_t CAT_CHANNELS = 0x03;
  static constexpr uint8_t CAT_ELEMENTS = 0x01;
  static constexpr uint8_t CAT_PACKED = 0x02;

  static constexpr uint8_t CH_TIMER_EVENT = 0x00; // status incl. output bit
  static constexpr uint16_t CH_TIMER_EVENT_OUTP_ON_MASK = 0x0010;
  static constexpr uint8_t CH_PRIMARY_ELEMENT = 0x02;
  static constexpr uint16_t CH_PRIMARY_ELEMENT_ELEMENT_MASK = 0x003f;
  static constexpr uint16_t CH_PRIMARY_ELEMENT_ALL_TP_LOST_MASK = 0x0400;

  static constexpr uint8_t ELEM_AIR_TEMPERATURE = 0x04; // index within block
  static constexpr uint8_t ELEM_FLOOR_TEMPERATURE = 0x05; // index for floor probe
  static constexpr uint8_t ELEM_BATTERY_STATUS = 0x0A;  // not used yet

  static constexpr uint8_t PACKED_MANUAL_TEMPERATURE = 0x00;
  static constexpr uint8_t PACKED_STANDBY_TEMPERATURE = 0x04;
  static constexpr uint8_t PACKED_CONFIGURATION = 0x07;
  // Inferred from field dump: floor min/max setpoints exposed in PACKED page
  // Updated mapping based on user dump for channel 10:
  //   PACKED[10] = 0x00D7 (215 -> 21.5C) => MIN
  //   PACKED[11] = 0x00FF (255 -> 25.5C) => MAX
  static constexpr uint8_t PACKED_FLOOR_MIN_TEMPERATURE = 0x0A; // 21.5C example
  static constexpr uint8_t PACKED_FLOOR_MAX_TEMPERATURE = 0x0B; // 25.5C example
  static constexpr uint16_t PACKED_CONFIGURATION_MODE_MASK = 0x07;
  static constexpr uint16_t PACKED_CONFIGURATION_MODE_MANUAL = 0x00;
  static constexpr uint16_t PACKED_CONFIGURATION_MODE_STANDBY = 0x01;
  static constexpr uint16_t PACKED_CONFIGURATION_MODE_STANDBY_ALT = 0x04; // fallback for variant firmwares
  static constexpr uint16_t PACKED_CONFIGURATION_PROGRAM_BIT = 0x0008; // suspected schedule/program flag
  static constexpr uint16_t PACKED_CONFIGURATION_PROGRAM_MASK = 0x0018; // extended clear: bits 3 and 4
  static constexpr uint16_t PACKED_CONFIGURATION_STRICT_UNLOCK_MASK = 0x0078; // bits 3..6 (avoid touching mode bits 0..2)

  // I/O reliability: number of attempts for read/write before escalating to WARN
  static constexpr uint8_t IO_RETRY_ATTEMPTS = 2; // first failure logged at DEBUG, final at WARN
};

// Inline helpers for configuring sensors
inline void WavinAHC9000::add_channel_battery_sensor(uint8_t ch, sensor::Sensor *s) {
  this->battery_sensors_[ch] = s;
}

inline void WavinAHC9000::add_channel_temperature_sensor(uint8_t ch, sensor::Sensor *s) {
  this->temperature_sensors_[ch] = s;
}

inline void WavinAHC9000::add_channel_comfort_setpoint_sensor(uint8_t ch, sensor::Sensor *s) {
  this->comfort_setpoint_sensors_[ch] = s;
}

inline void WavinAHC9000::add_channel_floor_temperature_sensor(uint8_t ch, sensor::Sensor *s) {
  this->floor_temperature_sensors_[ch] = s;
}

inline void WavinAHC9000::add_channel_floor_min_temperature_sensor(uint8_t ch, sensor::Sensor *s) {
  this->floor_min_temperature_sensors_[ch] = s;
}

inline void WavinAHC9000::add_channel_floor_max_temperature_sensor(uint8_t ch, sensor::Sensor *s) {
  this->floor_max_temperature_sensors_[ch] = s;
}

// numeric yaml_ready sensor removed

class WavinZoneClimate : public climate::Climate, public Component {
 public:
  void set_parent(WavinAHC9000 *p) { this->parent_ = p; }
  void set_single_channel(uint8_t ch) {
  this->single_channel_ = ch;
  this->single_channel_set_ = true;
  this->members_.clear();
  }
  void set_use_floor_temperature(bool v) { this->use_floor_temperature_ = v; }
  void set_members(const std::vector<int> &members) {
    this->members_.clear();
    for (int m : members) this->members_.push_back(static_cast<uint8_t>(m));
    this->single_channel_set_ = false;
  }

  void dump_config() override;

  void update_from_parent();

 protected:
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

  WavinAHC9000 *parent_{nullptr};
  uint8_t single_channel_{0};
  bool single_channel_set_{false};
  std::vector<uint8_t> members_{};
  bool use_floor_temperature_{false};
};

// Repair button removed; use API service to normalize

}  // namespace wavin_ahc9000
}  // namespace esphome
