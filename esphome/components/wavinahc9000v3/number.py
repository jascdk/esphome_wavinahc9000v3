import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_ID, CONF_NAME
from . import WavinAHC9000, WavinSetpointNumber

CONF_PARENT_ID = "wavinahc9000v3_id"
CONF_CHANNEL = "channel"
CONF_TYPE = "type"

SETPOINT_TYPES = ["comfort", "standby", "hysteresis"]

# Match the static constexpr values in WavinSetpointNumber
SETPOINT_TYPE_VALUES = {
    "comfort": 0,
    "standby": 1,
    "hysteresis": 2,
}

CONFIG_SCHEMA = number.number_schema(WavinSetpointNumber).extend(
    {
        cv.GenerateID("wavinahc9000v3_id"): cv.use_id(WavinAHC9000),
        cv.Required(CONF_CHANNEL): cv.int_range(min=1, max=16),
        cv.Required(CONF_TYPE): cv.one_of(*SETPOINT_TYPES, lower=True),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PARENT_ID])
    var = cg.new_Pvariable(config[CONF_ID])

    if config[CONF_TYPE] == "hysteresis":
        await number.register_number(var, config, min_value=0.1, max_value=1.0, step=0.1)
    else:
        await number.register_number(var, config, min_value=5, max_value=35, step=0.5)

    cg.add(var.set_parent(hub))
    cg.add(var.set_channel(config[CONF_CHANNEL]))
    cg.add(var.set_type(SETPOINT_TYPE_VALUES[config[CONF_TYPE]]))

    if config[CONF_TYPE] == "comfort":
        cg.add(hub.add_comfort_number(var))
    elif config[CONF_TYPE] == "standby":
        cg.add(hub.add_standby_number(var))
    else:
        cg.add(hub.add_hysteresis_number(var))

    cg.add(hub.add_active_channel(config[CONF_CHANNEL]))
