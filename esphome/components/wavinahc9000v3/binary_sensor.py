import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_CHANNEL, CONF_TYPE

from . import WavinAHC9000

CONF_PARENT_ID = "wavinahc9000v3_id"

TYPE_OUTPUT = "output"
TYPE_PROBLEM = "problem"

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend(
    {
        cv.GenerateID("wavinahc9000v3_id"): cv.use_id(WavinAHC9000),
        cv.Required(CONF_CHANNEL): cv.int_range(min=1, max=16),
        cv.Required(CONF_TYPE): cv.one_of(TYPE_OUTPUT, TYPE_PROBLEM, lower=True),
    }
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_PARENT_ID])
    var = await binary_sensor.new_binary_sensor(config)
    
    if config[CONF_TYPE] == TYPE_OUTPUT:
        cg.add(hub.add_channel_output_binary_sensor(config[CONF_CHANNEL], var))
    elif config[CONF_TYPE] == TYPE_PROBLEM:
        cg.add(hub.add_channel_problem_binary_sensor(config[CONF_CHANNEL], var))