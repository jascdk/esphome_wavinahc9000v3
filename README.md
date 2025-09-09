# ESPHome component: Wavin AHC 9000 / Jablotron AC-116 (UART protocol)

This repo contains an ESPHome external component to integrate a Wavin AHC 9000 (aka Jablotron AC-116) floor heating controller using its serial protocol (Modbus-like with custom function codes 0x43/0x44/0x45). It exposes one Home Assistant Climate entity per zone/channel and supports grouping multiple channels into one climate entity when they are paired.

Status: working scaffold using the dkjonas mapping (categories/pages/indices). Adjust indices if your firmware differs.

## Features
- 16 channels (zones)
- Climate entity per channel
- Optional grouped climates that aggregate multiple channels into one entity
- Read current temperature, setpoint, mode/action
- Set target temperature (writes setpoint)
- Optional battery sensors per channel (percentage)

## Wiring
- RS-485 connection to the controller.
- Use an ESP32 or ESP8266 with a TTL↔RS485 adapter.

## Example ESPHome YAML (explicit platforms)
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
  temp_divisor: 10.0  # raw temperature scaling; adjust per spec
  receive_timeout_ms: 1000
  update_interval: 5s

climate:
  - platform: wavin_ahc9000
    wavin_ahc9000_id: wavin
    name: "Zone 1"
    channel: 1
  - platform: wavin_ahc9000
    wavin_ahc9000_id: wavin
    name: "Living Area"
    members: [2, 3]

sensor:
  - platform: wavin_ahc9000
    wavin_ahc9000_id: wavin
    name: "Zone 1 Battery"
    channel: 1

logger:
  level: DEBUG
```

Note: climates are defined explicitly under the `climate:` section using platform `wavin_ahc9000` (single channel or grouped). Optional per-channel battery sensors use `sensor: - platform: wavin_ahc9000`.

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
  update_interval: 5s

climate:
  - platform: wavin_ahc9000
    wavin_ahc9000_id: wavin
    name: "Zone 1"
    channel: 1
  - platform: wavin_ahc9000
    wavin_ahc9000_id: wavin
    name: "Zone 2"
    channel: 2
```

2) Only use selected channels and one group of paired zones:
```yaml
wavin_ahc9000:
  uart_id: uart_wavin

climate:
  - platform: wavin_ahc9000
    wavin_ahc9000_id: wavin
    name: "Living Area"
    members: [1,2]
  - platform: wavin_ahc9000
    wavin_ahc9000_id: wavin
    name: "Bathroom"
    channel: 3

sensor:
  - platform: wavin_ahc9000
    wavin_ahc9000_id: wavin
    name: "Zone 1 Battery"
    channel: 1
```

3) RS-485 transceiver with TX enable and tighter polling:
```yaml
wavin_ahc9000:
  uart_id: uart_wavin
  tx_enable_pin: GPIO5
  update_interval: 3s
  receive_timeout_ms: 800
```

## Disclaimer
This is an independent, community-driven integration. Use at your own risk.