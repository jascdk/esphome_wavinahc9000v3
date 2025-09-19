# ESPHome component: Wavin AHC 9000 / Jablotron AC-116 (restart, minimal scaffold)

This repo contains an ESPHome external component to integrate a Wavin AHC 9000 (aka Jablotron AC-116) floor heating controller using its serial protocol (Modbus-like with custom function codes 0x43/0x44/0x45).

Status: restarted and functional basics in place: multi-channel staged polling, read current/setpoint/mode/action, and write setpoint/mode. Battery sensors supported per channel.

## Goals (to be reintroduced incrementally)
- 16 channels (zones)
- Climate entity per channel
- Optional grouped climates that aggregate multiple channels into one entity
- Read current temperature, setpoint, mode/action
- Set target temperature (writes setpoint)
- Optional battery sensors per channel (percentage)

## Wiring
- RS-485 connection to the controller.
- Use an ESP32 or ESP8266 with a TTL↔RS485 adapter.

## Example ESPHome YAML (explicit platforms, minimal)
```yaml
esphome:
  name: wavin-gateway

esp32:
  board: esp32dev
  framework:
    type: esp-idf

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
  # tx_enable_pin: GPIO5  # Optional, for RS-485 DE/RE
  # temp_divisor: 10.0    # raw temperature scaling; adjust per spec
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

## Implementation Plan (phased)
1) Minimal compile and entities visible (this commit)
2) Add UART framing + CRC, read a single register (log-only)
3) Add staged polling for per-channel primary element, setpoint, temps, action
4) Publish values to climates; wire write path for mode/setpoint
5) Add battery sensors

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

3) RS-485 transceiver with TX enable and tighter polling (when protocol returns):
```yaml
wavin_ahc9000:
  uart_id: uart_wavin
  tx_enable_pin: GPIO5
  update_interval: 3s
  receive_timeout_ms: 800
```

## Disclaimer
This is an independent, community-driven integration. Use at your own risk.

## YAML generator and HA Jinja stitching

This component can generate suggested YAML for discovered channels. Because Home Assistant limits text sensor sizes, the suggestion is also exposed as chunked text sensors (each containing only entity blocks, without section headers). Use the included `jinjatemplate.txt` to stitch them back together.

### Enabling the generator (manual include)

Files involved:
- `packages/yaml_generator.on.yaml`  (full set of text sensors + services)
- `packages/yaml_generator.off.yaml` (empty stub – mostly for clarity)
- `packages/yaml_generator.yaml`     (documentation only)

In your node YAML add (uncomment while needed):

```yaml
packages:
  # Enable during onboarding to generate YAML suggestions
  yaml_generator: !include packages/yaml_generator.on.yaml
  # Optional: debug register dump service
  # wavin_debug: !include packages/wavin_debug_services.yaml
```

After you've copied the generated YAML into your permanent config, simply comment the `yaml_generator` line again to remove the helper entities and services.

### Generating & collecting the YAML
1. Recompile & upload with the generator enabled.
2. In Home Assistant call the service: `esphome.<node_name>_wavin_publish_yaml_text_sensors` (or first `wavin_generate_yaml`).
3. Inspect sensors:
   - `sensor.wavin_yaml_climate_1` .. `sensor.wavin_yaml_climate_8`
   - `sensor.wavin_yaml_comfort_climate_1` .. (if comfort climates apply)
   - `sensor.wavin_yaml_temperature_1` .. `sensor.wavin_yaml_temperature_8`
   - `sensor.wavin_yaml_battery_1` .. `sensor.wavin_yaml_battery_8`
   - `sensor.wavin_yaml_floor_temperature_1` .. (only if floor probes detected)
4. Optionally view the single full suggestion sensor: `sensor.wavin_yaml_suggestion` (may be truncated for very large outputs in HA – use chunks when in doubt).
5. Open `jinjatemplate.txt` and use the “All-in-one” Jinja macro to stitch chunks back into a cohesive YAML block (adds section headers + indentation).

### Why no substitution toggle?
Early versions used a substitution (`YAML_GEN_STATE`) to switch between on/off files. Remote GitHub package includes cannot use substitutions in the `files:` list, leading to confusion. The project now favors a simple comment/uncomment pattern for reliability both locally and in remote includes.

### Floor probe driven additions
If at least one floor probe is plausibly detected (>1.0°C and <90°C after startup), the generator adds the relevant floor temperature sensor entries. If you trigger the service very early and they are missing, trigger again later.

### Readiness indicator
You can optionally add a `yaml_ready` binary sensor to know when discovery has progressed enough to get stable YAML suggestions:

```yaml
binary_sensor:
  - platform: wavin_ahc9000
    wavin_ahc9000_id: wavin
    type: yaml_ready
    name: "Wavin YAML Ready"
```

Once you have migrated the suggested YAML entities you want, disable the generator include to declutter.

### Comfort Setpoint Read-Only Sensor

If you do not want writable number entities but still want to surface the current comfort (manual) setpoint as a sensor for dashboards or historical graphs, you can add a `comfort_setpoint` sensor type:

```yaml
sensor:
  - platform: wavin_ahc9000
    wavin_ahc9000_id: wavin
    channel: 3
    type: comfort_setpoint
    name: "Zone 3 Comfort Setpoint"
```

This sensor publishes the latest manual/comfort setpoint (register 0x00) that the integration already reads for the climate entity. It is read-only; changing it requires using the climate entity or a number entity if enabled in another branch.

### Floor Temperature Sensor

If your thermostat provides a floor probe value, you can expose it separately:

```yaml
sensor:
  - platform: wavin_ahc9000
    wavin_ahc9000_id: wavin
    channel: 3
    type: floor_temperature
    name: "Zone 3 Floor Temperature"
```

This reads element index 0x05 (same scaling as air). A plausibility filter (>1..90°C) is applied; invalid or missing values result in the sensor staying unavailable.

Automatic detection: The component now marks a channel as having a floor sensor only after at least one plausible, non-zero floor reading is observed (> 1.0°C and < 90°C). Until then, any configured `floor_temperature` sensor will remain unavailable (rather than showing 0.0). The YAML generation services (`_wavin_generate_yaml` / `_wavin_publish_yaml_text_sensors`) will include floor temperature sensor entity suggestions only for channels where a valid floor probe has already been detected. If you generate YAML immediately after boot and a floor probe wasn’t yet detected, just trigger the service again later after some polling cycles.

Chunked YAML: If at least one floor probe is detected, an additional chunk set is exposed via the `get_yaml_floor_temperature_chunk` API internally (usable in future text sensors). If you add corresponding text sensors (e.g. `wavin_yaml_floor_temperature_1..8`), you can stitch them with the same Jinja approach as other sensor chunks by adding an extra `sensor:` header and their content.

### YAML Readiness Indicator (binary_sensor)

To know when it's reasonable to generate and copy the suggested YAML, a readiness indicator is exposed as a binary sensor. It turns on once the hub has:
- Discovered at least one active channel (primary element present and no TP lost)
- Completed at least one full element block read for all discovered channels

Add this to your config if desired:

```yaml
binary_sensor:
  - platform: wavin_ahc9000
    wavin_ahc9000_id: wavin
    type: yaml_ready
    name: "Wavin YAML Ready"
```

This readiness indicator is available as a binary_sensor in this branch.