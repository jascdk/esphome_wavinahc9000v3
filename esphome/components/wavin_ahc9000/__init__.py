import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, climate, sensor
from esphome.const import CONF_ID
from esphome import pins

CODEOWNERS = ["@you"]
AUTO_LOAD = ["climate", "uart", "sensor"]

ns = cg.esphome_ns.namespace("wavin_ahc9000")
WavinAHC9000 = ns.class_("WavinAHC9000", cg.PollingComponent, uart.UARTDevice)
WavinZoneClimate = ns.class_("WavinZoneClimate", climate.Climate, cg.Component)

CONF_UART_ID = "uart_id"
CONF_TX_ENABLE_PIN = "tx_enable_pin"
CONF_TEMP_DIVISOR = "temp_divisor"
CONF_RECEIVE_TIMEOUT_MS = "receive_timeout_ms"
CONF_POLL_CHANNELS_PER_CYCLE = "poll_channels_per_cycle"
CONF_ALLOW_MODE_WRITES = "allow_mode_writes"
CONF_AUTO_CLIMATES = "auto_climates"
CONF_TEMPERATURE_SENSORS = "temperature_sensors"
CONF_BATTERY_SENSORS = "battery_sensors"
CONF_AUTO_CHANNELS = "auto_channels"
CONF_NAME_PREFIX = "name_prefix"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WavinAHC9000),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Optional(CONF_TX_ENABLE_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_TEMP_DIVISOR, default=10.0): cv.positive_float,
        cv.Optional(CONF_RECEIVE_TIMEOUT_MS, default=1000): cv.positive_int,
        cv.Optional(CONF_POLL_CHANNELS_PER_CYCLE, default=2): cv.int_range(min=1, max=16),
    cv.Optional(CONF_ALLOW_MODE_WRITES, default=True): cv.boolean,
    # Auto-generation options
    cv.Optional(CONF_AUTO_CLIMATES, default=True): cv.boolean,
    cv.Optional(CONF_TEMPERATURE_SENSORS, default=False): cv.boolean,
    cv.Optional(CONF_BATTERY_SENSORS, default=False): cv.boolean,
    cv.Optional(CONF_AUTO_CHANNELS, default="all"): cv.Any(cv.one_of("all", lower=True), cv.ensure_list(cv.int_range(min=1, max=16))),
    cv.Optional(CONF_NAME_PREFIX, default="Zone "): cv.string,
    }
).extend(uart.UART_DEVICE_SCHEMA).extend(cv.polling_component_schema("5s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await uart.register_uart_device(var, config)
    await cg.register_component(var, config)
    if CONF_TX_ENABLE_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_TX_ENABLE_PIN])
        cg.add(var.set_tx_enable_pin(pin))
    if CONF_TEMP_DIVISOR in config:
        cg.add(var.set_temp_divisor(config[CONF_TEMP_DIVISOR]))
    if CONF_RECEIVE_TIMEOUT_MS in config:
        cg.add(var.set_receive_timeout_ms(config[CONF_RECEIVE_TIMEOUT_MS]))
    if CONF_POLL_CHANNELS_PER_CYCLE in config:
        cg.add(var.set_poll_channels_per_cycle(config[CONF_POLL_CHANNELS_PER_CYCLE]))
    if CONF_ALLOW_MODE_WRITES in config:
        cg.add(var.set_allow_mode_writes(config[CONF_ALLOW_MODE_WRITES]))

    # Auto-generate entities if requested
    channels = list(range(1, 17)) if (isinstance(config.get(CONF_AUTO_CHANNELS, "all"), str) and str(config.get(CONF_AUTO_CHANNELS)).lower() == "all") else list(config.get(CONF_AUTO_CHANNELS, []))
    prefix = config.get(CONF_NAME_PREFIX, "Zone ")

    # Climates auto-generation deferred (requires typed IDs); keep manual climate blocks for now.

    # Temperature and battery sensors per channel
    if config.get(CONF_TEMPERATURE_SENSORS, False) or config.get(CONF_BATTERY_SENSORS, False):
        for ch in channels:
            if config.get(CONF_TEMPERATURE_SENSORS, False):
                s_cfg = {"name": f"{prefix}{ch} Temperature"}
                s = await sensor.new_sensor(s_cfg)
                # Temperature defaults
                from esphome.const import DEVICE_CLASS_TEMPERATURE, UNIT_CELSIUS
                cg.add(s.set_device_class(DEVICE_CLASS_TEMPERATURE))
                cg.add(s.set_unit_of_measurement(UNIT_CELSIUS))
                cg.add(s.set_accuracy_decimals(1))
                cg.add(var.add_channel_temperature_sensor(ch, s))
                cg.add(var.add_active_channel(ch))
            if config.get(CONF_BATTERY_SENSORS, False):
                b_cfg = {"name": f"{prefix}{ch} Battery"}
                b = await sensor.new_sensor(b_cfg)
                # Battery defaults
                from esphome.const import DEVICE_CLASS_BATTERY, UNIT_PERCENT
                cg.add(b.set_device_class(DEVICE_CLASS_BATTERY))
                cg.add(b.set_unit_of_measurement(UNIT_PERCENT))
                cg.add(b.set_accuracy_decimals(0))
                cg.add(var.add_channel_battery_sensor(ch, b))
                cg.add(var.add_active_channel(ch))
