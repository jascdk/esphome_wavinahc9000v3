import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_CHANNEL

from . import WavinAHC9000, ns

CONF_PARENT_ID = "wavinahc9000v3_id"
CONF_TYPE = "type"

WavinSwitch = ns.class_("WavinSwitch", switch.Switch)

SWITCH_TYPE_VALUES = {
    "child_lock": 0,
    "standby": 1,
}

CONFIG_SCHEMA = switch.switch_schema(WavinSwitch).extend(
    {
        cv.GenerateID("wavinahc9000v3_id"): cv.use_id(WavinAHC9000),
        cv.Required(CONF_CHANNEL): cv.int_range(min=1, max=16),
        cv.Optional(CONF_TYPE, default="child_lock"): cv.one_of("child_lock", "standby", lower=True),
    }
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_PARENT_ID])
    ch = config[CONF_CHANNEL]
    var = await switch.new_switch(config)
    cg.add(var.set_parent(hub))
    cg.add(var.set_channel(ch))
    
    typ = config[CONF_TYPE]
    cg.add(var.set_type(SWITCH_TYPE_VALUES[typ]))
    
    if typ == "child_lock":
        cg.add(hub.add_channel_child_lock_switch(ch, var))
    elif typ == "standby":
        cg.add(hub.add_channel_standby_switch(ch, var))

    cg.add(hub.add_active_channel(ch))
