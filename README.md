# Wavin AHC 9000 / Jablotron AC-116 for ESPHome

This custom component integrates Wavin AHC 9000 and Jablotron AC-116 floor heating controllers into Home Assistant using ESPHome. It offers full bidirectional control, fast sensor updates, and access to advanced configuration settings.

## ✨ Key Features

### 🔥 Climate Control
*   **Thermostat Entity:** Full control over target temperature and operation mode (Heat/Off).
*   **Zone Grouping:** Combine multiple physical channels (e.g., a large living room) into a single thermostat control.
*   **Floor Protection:** Supports floor sensors with min/max surface temperature limits.

### 📊 Sensors & Monitoring
*   **Real-time Data:** Monitors Air Temperature, Floor Temperature, and Current Setpoint.
*   **Battery Status:** Reports battery levels (%) for wireless thermostats.
*   **Signal Quality (RSSI):** See signal strength for both the Thermostat (Element) and the Controller (CU) to diagnose range issues.
*   **System Info:** Reports Hardware version, Software version, and Device Name.

### ⚙️ Switches & Configuration
*   **Standby Switch:** Dedicated switch to toggle a zone ON/OFF without losing settings.
*   **Child Lock:** Remotely lock/unlock the physical buttons on the wall thermostats.
*   **Hysteresis Control:** Fine-tune the switching differential (0.1°C to 1.0°C) via a number entity.
*   **Floor Limits:** View the configured minimum and maximum floor temperatures.

### 🚀 Performance & Stability
*   **Smart Polling:** Configurable `poll_channels_per_cycle` to speed up updates (e.g., refresh 4 channels at once).
*   **Anti-Revert Logic:** Automatically clears internal "Program/Schedule" bits when you change modes, ensuring the controller doesn't override your commands.
*   **Safety Mode:** Optional `allow_mode_writes: false` setting to prevent accidental heating shutdowns from the dashboard.

---

## 📝 Example Configuration

### 1. Main Hub Setup
Define the hub with a specific `id` so other entities can reference it.

```yaml
wavinahc9000v3:
  id: wavin_hub  # <--- ID for the controller
  uart_id: uart_bus
  update_interval: 5s
  poll_channels_per_cycle: 4
  allow_mode_writes: true
```

### 2. Thermostat (Climate)
```yaml
climate:
  - platform: wavinahc9000v3
    id: climate_living_room
    wavinahc9000v3_id: wavin_hub  # <--- Links to the hub ID above
    name: "Living Room"
    channel: 1
    # Optional: Use floor sensor logic
    # use_floor_temperature: true 
```

### 3. Switches (Standby & Lock)
```yaml
switch:
  - platform: wavinahc9000v3
    id: switch_standby_living_room
    wavinahc9000v3_id: wavin_hub
    name: "Living Room Standby"
    channel: 1
    type: standby

  - platform: wavinahc9000v3
    id: switch_lock_living_room
    wavinahc9000v3_id: wavin_hub
    name: "Living Room Child Lock"
    channel: 1
    type: child_lock
```

### 4. Sensors (Battery & Signal)
```yaml
sensor:
  - platform: wavinahc9000v3
    id: sensor_battery_living_room
    wavinahc9000v3_id: wavin_hub
    channel: 1
    type: battery
    name: "Living Room Battery"

  - platform: wavinahc9000v3
    id: sensor_rssi_living_room
    wavinahc9000v3_id: wavin_hub
    channel: 1
    type: rssi_element
    name: "Living Room Signal"
```

### 5. Advanced Settings (Hysteresis)
```yaml
number:
  - platform: wavinahc9000v3
    id: number_hysteresis_living_room
    wavinahc9000v3_id: wavin_hub
    channel: 1
    type: hysteresis
    name: "Living Room Hysteresis"
```