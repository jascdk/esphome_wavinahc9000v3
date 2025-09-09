import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import UNIT_PERCENT, DEVICE_CLASS_BATTERY, ICON_BATTERY

from . import WavinAHC9000

CONF_PARENT_ID = "wavin_ahc9000_id"
CONF_CHANNEL = "channel"


CONFIG_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_PERCENT,
    accuracy_decimals=0,
    device_class=DEVICE_CLASS_BATTERY,
    icon=ICON_BATTERY,
).extend(
    {
        cv.GenerateID(CONF_PARENT_ID): cv.use_id(WavinAHC9000),
        cv.Required(CONF_CHANNEL): cv.int_range(min=1, max=16),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PARENT_ID])
    sens = await sensor.new_sensor(config)
    cg.add(hub.add_channel_battery_sensor(config[CONF_CHANNEL], sens))
