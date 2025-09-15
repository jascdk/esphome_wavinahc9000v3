import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    UNIT_PERCENT,
    DEVICE_CLASS_BATTERY,
    ICON_BATTERY,
    DEVICE_CLASS_TEMPERATURE,
    UNIT_CELSIUS,
)

from . import WavinAHC9000

CONF_PARENT_ID = "wavin_ahc9000_id"
CONF_CHANNEL = "channel"
CONF_AUTO_CHANNELS = "auto_channels"
CONF_NAME_PREFIX = "name_prefix"


CONF_TYPE = "type"

CONFIG_SCHEMA = sensor.sensor_schema().extend(
    {
        cv.GenerateID(CONF_PARENT_ID): cv.use_id(WavinAHC9000),
        # Either a single channel or auto_channels
        cv.Optional(CONF_CHANNEL): cv.int_range(min=1, max=16),
        cv.Optional(CONF_AUTO_CHANNELS): cv.Any(cv.one_of("all", lower=True), cv.ensure_list(cv.int_range(min=1, max=16))),
        cv.Required(CONF_TYPE): cv.one_of("battery", "temperature", lower=True),
        cv.Optional(CONF_NAME_PREFIX, default="Zone "): cv.string,
    }
).extend(cv.only_with_ensure_required(CONF_CHANNEL, [CONF_TYPE]))


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PARENT_ID])
    # Single channel definition
    if CONF_CHANNEL in config:
        sens = await sensor.new_sensor(config)
        if config[CONF_TYPE] == "battery":
            cg.add(sens.set_device_class(DEVICE_CLASS_BATTERY))
            cg.add(sens.set_unit_of_measurement(UNIT_PERCENT))
            cg.add(sens.set_icon(ICON_BATTERY))
            cg.add(sens.set_accuracy_decimals(0))
            cg.add(hub.add_channel_battery_sensor(config[CONF_CHANNEL], sens))
        else:
            cg.add(sens.set_device_class(DEVICE_CLASS_TEMPERATURE))
            cg.add(sens.set_unit_of_measurement(UNIT_CELSIUS))
            cg.add(sens.set_accuracy_decimals(1))
            cg.add(hub.add_channel_temperature_sensor(config[CONF_CHANNEL], sens))
        cg.add(hub.add_active_channel(config[CONF_CHANNEL]))
        return

    # Auto-generate sensors across multiple channels
    if CONF_AUTO_CHANNELS in config:
        auto = config[CONF_AUTO_CHANNELS]
        channels = list(range(1, 17)) if (isinstance(auto, str) and auto.lower() == "all") else list(auto)
        prefix = config.get(CONF_NAME_PREFIX, "Zone ")
        is_batt = config[CONF_TYPE] == "battery"
        for ch in channels:
            child = {
                CONF_PARENT_ID: config[CONF_PARENT_ID],
                "name": f"{prefix}{ch} {'Battery' if is_batt else 'Temperature'}",
            }
            s = await sensor.new_sensor(child)
            if is_batt:
                cg.add(s.set_device_class(DEVICE_CLASS_BATTERY))
                cg.add(s.set_unit_of_measurement(UNIT_PERCENT))
                cg.add(s.set_icon(ICON_BATTERY))
                cg.add(s.set_accuracy_decimals(0))
                cg.add(hub.add_channel_battery_sensor(ch, s))
            else:
                cg.add(s.set_device_class(DEVICE_CLASS_TEMPERATURE))
                cg.add(s.set_unit_of_measurement(UNIT_CELSIUS))
                cg.add(s.set_accuracy_decimals(1))
                cg.add(hub.add_channel_temperature_sensor(ch, s))
            cg.add(hub.add_active_channel(ch))
