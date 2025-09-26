import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_CHANNEL, CONF_ID

from . import WavinAHC9000, ns

CONF_PARENT_ID = "wavin_ahc9000_id"
CONF_TYPE = "type"

WavinChildLockSwitch = ns.class_("WavinChildLockSwitch", switch.Switch)

CONFIG_SCHEMA = cv.All(
    switch.switch_schema(
        {
            cv.GenerateID(CONF_PARENT_ID): cv.use_id(WavinAHC9000),
            cv.Required(CONF_CHANNEL): cv.int_range(min=1, max=16),
            cv.Required(CONF_TYPE): cv.one_of("child_lock", lower=True),
        }
    ),
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_PARENT_ID])
    ch = config[CONF_CHANNEL]
    # Create subclass instance
    sw = cg.new_Pvariable(config[CONF_ID], WavinChildLockSwitch)
    await switch.register_switch(sw, config)
    cg.add(sw.set_parent(hub))
    cg.add(sw.set_channel(ch))
    cg.add(hub.add_channel_child_lock_switch(ch, sw))
    # State updates are handled by publish_updates; optimistic state already published by switch
