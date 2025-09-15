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


CONF_TYPE = "type"

def _schema_for(t: str):
    if t == "battery":
        return sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_BATTERY,
            icon=ICON_BATTERY,
        )
    elif t == "temperature":
        return sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
        )
    else:
        raise cv.Invalid("Unsupported sensor type; use 'battery' or 'temperature'")

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_PARENT_ID): cv.use_id(WavinAHC9000),
            cv.Required(CONF_CHANNEL): cv.int_range(min=1, max=16),
            cv.Required(CONF_TYPE): cv.one_of("battery", "temperature", lower=True),
        }
    ).extend(cv.COMPONENT_SCHEMA),
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PARENT_ID])
    sens = await sensor.new_sensor(_schema_for(config[CONF_TYPE]))
    if config[CONF_TYPE] == "battery":
        cg.add(hub.add_channel_battery_sensor(config[CONF_CHANNEL], sens))
    else:
        cg.add(hub.add_channel_temperature_sensor(config[CONF_CHANNEL], sens))
    cg.add(hub.add_active_channel(config[CONF_CHANNEL]))
