import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import CONF_ID

from . import WavinAHC9000

CONF_PARENT_ID = "wavin_ahc9000_id"
CONF_CHANNEL = "channel"

WavinRepairButton = cg.esphome_ns.namespace("wavin_ahc9000").class_("WavinRepairButton", button.Button, cg.Component)

# Optional extended repair clears additional flags that may lock keypad
CONF_EXTENDED = "extended"

CONFIG_SCHEMA = button.button_schema(WavinRepairButton).extend(
    {
        cv.GenerateID(CONF_PARENT_ID): cv.use_id(WavinAHC9000),
        cv.Required(CONF_CHANNEL): cv.int_range(min=1, max=16),
    cv.Optional(CONF_EXTENDED, default=False): cv.boolean,
    }
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_PARENT_ID])
    btn = await button.new_button(config)
    cg.add(btn.set_parent(hub))
    cg.add(btn.set_channel(config[CONF_CHANNEL]))
    cg.add(btn.set_extended(config[CONF_EXTENDED]))
    cg.add(hub.add_active_channel(config[CONF_CHANNEL]))
