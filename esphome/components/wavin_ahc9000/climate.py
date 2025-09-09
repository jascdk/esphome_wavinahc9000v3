import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate
from esphome.const import CONF_ID, CONF_NAME

from . import WavinAHC9000, WavinZoneClimate

CONF_PARENT_ID = "wavin_ahc9000_id"
CONF_CHANNEL = "channel"
CONF_MEMBERS = "members"


def _channels_list(value):
    return cv.ensure_list(cv.int_range(min=1, max=16))(value)


BASE_SCHEMA = climate.CLIMATE_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(WavinZoneClimate),
        cv.GenerateID(CONF_PARENT_ID): cv.use_id(WavinAHC9000),
        cv.Optional(CONF_CHANNEL): cv.int_range(min=1, max=16),
        cv.Optional(CONF_MEMBERS): _channels_list,
    }
).extend(cv.COMPONENT_SCHEMA)


def _validate(cfg):
    if (CONF_CHANNEL in cfg) == (CONF_MEMBERS in cfg):
        raise cv.Invalid("Specify either 'channel' for a single zone or 'members' for a group, not both")
    return cfg


CONFIG_SCHEMA = cv.All(BASE_SCHEMA, _validate)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PARENT_ID])

    var = cg.new_Pvariable(config[CONF_ID])
    await climate.register_climate(var, config)
    cg.add(var.set_parent(hub))

    # Single channel or group
    if CONF_CHANNEL in config:
        cg.add(var.set_single_channel(config[CONF_CHANNEL]))
        cg.add(hub.add_channel_climate(var))
    elif CONF_MEMBERS in config:
        members = config.get(CONF_MEMBERS, [])
        vec = cg.RawExpression("std::vector<int>{%s}" % ",".join(str(m) for m in members))
        cg.add(var.set_members(vec))
        cg.add(hub.add_group_climate(var))
