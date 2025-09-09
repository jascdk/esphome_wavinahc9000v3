import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_CHANNELS,
    CONF_UPDATE_INTERVAL,
)

from esphome.components import climate, uart, sensor
from esphome import pins

CODEOWNERS = ["@you"]

AUTO_LOAD = ["climate", "uart", "sensor"]

namespace = cg.esphome_ns.namespace("wavin_ahc9000")
WavinAHC9000 = namespace.class_("WavinAHC9000", cg.PollingComponent, uart.UARTDevice)
WavinZoneClimate = namespace.class_("WavinZoneClimate", climate.Climate, cg.Component)

CONF_GROUPS = "groups"
CONF_MEMBERS = "members"
CONF_TEMP_DIVISOR = "temp_divisor"
CONF_UART_ID = "uart_id"
CONF_TX_ENABLE_PIN = "tx_enable_pin"
CONF_RECEIVE_TIMEOUT_MS = "receive_timeout_ms"
CONF_BATTERY_SENSORS = "battery_sensors"
CONF_CHANNEL_NAMES = "channel_names"


def _channels_list(value):
    return cv.ensure_list(cv.int_range(min=1, max=16))(value)


GROUP_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_NAME): cv.string,
        cv.Required(CONF_MEMBERS): _channels_list,
    }
)

CHANNEL_NAME_SCHEMA = cv.Schema(
    {
        cv.Required("channel"): cv.int_range(min=1, max=16),
        cv.Required(CONF_NAME): cv.string,
    }
)


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(WavinAHC9000),
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Optional(CONF_TX_ENABLE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_CHANNELS, default=[i for i in range(1, 17)]): _channels_list,
            cv.Optional(CONF_GROUPS, default=[]): cv.ensure_list(GROUP_SCHEMA),
            cv.Optional(CONF_TEMP_DIVISOR, default=10.0): cv.positive_float,
            cv.Optional(CONF_RECEIVE_TIMEOUT_MS, default=1000): cv.positive_int,
            cv.Optional(CONF_UPDATE_INTERVAL, default="5s"): cv.update_interval,
            cv.Optional(CONF_BATTERY_SENSORS, default=True): cv.boolean,
            cv.Optional(CONF_CHANNEL_NAMES, default=[]): cv.ensure_list(CHANNEL_NAME_SCHEMA),
        }
    )
    .extend(cv.polling_component_schema("5s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    uart_component = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    # Configure device-level params
    cg.add(var.set_temp_divisor(config[CONF_TEMP_DIVISOR]))
    cg.add(var.set_receive_timeout_ms(config[CONF_RECEIVE_TIMEOUT_MS]))
    if CONF_TX_ENABLE_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_TX_ENABLE_PIN])
        cg.add(var.set_tx_enable_pin(pin))

    # Track which channels are part of groups to avoid duplicates
    grouped = set()
    for g in config[CONF_GROUPS]:
        members = g[CONF_MEMBERS]
        for ch in members:
            grouped.add(ch)

    cl = cg.new_Pvariable(cg.auto_id(), WavinZoneClimate)
    await climate.register_climate(cl, {CONF_NAME: g[CONF_NAME]})
    cg.add(cl.set_parent(var))
    cg.add(cl.set_members(members))
    cg.add(var.add_group_climate(cl))

    # Channel name overrides map
    override_names = {}
    for entry in config[CONF_CHANNEL_NAMES]:
        override_names[entry["channel"]] = entry[CONF_NAME]

    # Create per-channel climates for any channels not grouped
    for ch in config[CONF_CHANNELS]:
        if ch in grouped:
            continue
    cl = cg.new_Pvariable(cg.auto_id(), WavinZoneClimate)
    name = override_names.get(ch, f"Zone {ch}")
    await climate.register_climate(cl, {CONF_NAME: name})
    cg.add(cl.set_parent(var))
    cg.add(cl.set_single_channel(ch))
    cg.add(var.add_channel_climate(cl))

    # Battery sensors: expose for all channels in channels + groups
    if config[CONF_BATTERY_SENSORS]:
        all_channels = set(config[CONF_CHANNELS])
        for g in config[CONF_GROUPS]:
            for ch in g[CONF_MEMBERS]:
                all_channels.add(ch)
        for ch in sorted(all_channels):
            s = cg.new_Pvariable(cg.auto_id(), sensor.Sensor)
            await sensor.register_sensor(s, {CONF_NAME: f"Zone {ch} Battery"})
            cg.add(var.add_channel_battery_sensor(ch, s))
