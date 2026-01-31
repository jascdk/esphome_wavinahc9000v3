import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor

from . import WavinAHC9000

CONF_PARENT_ID = "wavinahc9000v3_id"
CONF_TYPE = "type"

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend(
    {
        cv.GenerateID("wavinahc9000v3_id"): cv.use_id(WavinAHC9000),
        # No types left
    }
)

async def to_code(config):
    pass
