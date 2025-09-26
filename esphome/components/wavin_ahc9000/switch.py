import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_CHANNEL, CONF_ID

from . import WavinAHC9000

# We use the base Switch class without custom C++ subclass: actions call hub directly via lambda.

CONF_PARENT_ID = "wavin_ahc9000_id"
CONF_TYPE = "type"

CONFIG_SCHEMA = switch.switch_schema(
    {
        cv.GenerateID(CONF_PARENT_ID): cv.use_id(WavinAHC9000),
        cv.Required(CONF_CHANNEL): cv.int_range(min=1, max=16),
        cv.Required(CONF_TYPE): cv.one_of("child_lock", lower=True),
    }
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_PARENT_ID])
    sw = await switch.new_switch(config)
    ch = config[CONF_CHANNEL]
    if config[CONF_TYPE] == "child_lock":
        cg.add(hub.add_channel_child_lock_switch(ch, sw))
        # Inject on_turn_on/off lambdas to call hub method
        cg.add(sw.add_on_turn_on_trigger(cg.RawExpression(f"[](){{ wavin->write_channel_child_lock({ch}, true); }}")))
        cg.add(sw.add_on_turn_off_trigger(cg.RawExpression(f"[](){{ wavin->write_channel_child_lock({ch}, false); }}")))
        # publish_updates will correct state after urgent refresh
