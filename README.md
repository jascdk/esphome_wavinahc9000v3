# ESPHome component: Wavin AHC 9000 / Jablotron AC-116 (UART protocol)

This repo contains an ESPHome external component to integrate a Wavin AHC 9000 (aka Jablotron AC-116) floor heating controller using its serial protocol (Modbus-like with custom function codes 0x43/0x44/0x45). It exposes one Home Assistant Climate entity per zone/channel and supports grouping multiple channels into one climate entity when they are paired.

Status: working scaffold using the dkjonas mapping (categories/pages/indices). Adjust indices if your firmware differs.

## Features
- 16 channels (zones)
- Climate entity per channel
- Optional groups that aggregate multiple channels into one entity
- Read current temperature, setpoint, mode/action
- Set target temperature (writes setpoint)
- Optional battery sensors per channel (percentage)

## Wiring
- RS-485 connection to the controller.
- Use an ESP32 or ESP8266 with a TTL↔RS485 adapter.

## Example ESPHome YAML
```yaml
esphome:
  name: wavin-gateway
  platform: ESP32
  board: esp32dev

external_components:
  - source: github://heinekmadsen/esphome_wavinahc9000v3@main
    components: [wavin_ahc9000]

uart:
  id: uart_wavin
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 38400
  stop_bits: 1
  parity: NONE

wavin_ahc9000:
  id: wavin
  uart_id: uart_wavin
  tx_enable_pin: GPIO5  # Optional, for RS-485 DE/RE
  channels: [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
  groups:
    - name: "Living Area"
      members: [1,2]
    - name: "Bedrooms"
      members: [3,4,5]
  channel_names:
    - channel: 1
      name: "Kitchen"
    - channel: 2
      name: "Dining"
    - channel: 3
      name: "Living Room"
  temp_divisor: 10.0  # raw temperature scaling; adjust per spec
  receive_timeout_ms: 1000
  update_interval: 5s
  battery_sensors: true

logger:
  level: DEBUG
```

Note: entities are created automatically by this component; there's no separate `climate:` platform block needed.

## Implementation Plan
- Protocol per dkjonas reference:
  - Categories: CHANNELS (primary element, timer event), ELEMENTS (air temperature), PACKED (manual temp, standby temp, configuration/mode)
  - Function codes: 0x43 (read), 0x44 (write), 0x45 (write masked), CRC16 Modbus
- Scaling: `temp_divisor` default 10.0 for 0.1°C values
- Entities are created automatically; no extra `climate:` block needed
  - Battery sensors are published as Sensor entities named "Zone N Battery" (0-100)

## Aggregation semantics
- Current temperature: average across member channels
- Setpoint: average across member channels
- Action: heating if any member is heating; otherwise idle

## Development tips
- Start with one known-good channel.
- Enable `logger:` at DEBUG to see frames.
- If using RS-485 transceiver, set `tx_enable_pin` for DE/RE control.

## More config examples

1) Minimal single-controller, all 16 channels, no groups, custom names on a few channels:
```yaml
uart:
  id: uart_wavin
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 38400

wavin_ahc9000:
  uart_id: uart_wavin
  channels: [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
  channel_names:
    - channel: 1
      name: "Hall"
    - channel: 5
      name: "Master Bedroom"
  update_interval: 5s
```

2) Only use selected channels and one group of paired zones:
```yaml
wavin_ahc9000:
  uart_id: uart_wavin
  channels: [1,2,3,6]
  groups:
    - name: "Living Area"
      members: [1,2]
  channel_names:
    - channel: 3
      name: "Bathroom"
  battery_sensors: true
```

3) RS-485 transceiver with TX enable and tighter polling:
```yaml
wavin_ahc9000:
  uart_id: uart_wavin
  tx_enable_pin: GPIO5
  channels: [1,2,3]
  update_interval: 3s
  receive_timeout_ms: 800
```

## Disclaimer
This is an independent, community-driven integration. Use at your own risk.