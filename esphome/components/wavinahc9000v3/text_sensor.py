import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import WavinAHC9000

CONF_PARENT_ID = "wavinahc9000v3_id"
CONF_TYPE = "type"

TYPE_YAML = "yaml_suggestion"
TYPE_SOFTWARE_VERSION = "software_version"
TYPE_HARDWARE_VERSION = "hardware_version"
TYPE_DEVICE_NAME = "device_name"

CONFIG_SCHEMA = text_sensor.text_sensor_schema().extend(
    {
        cv.GenerateID("wavinahc9000v3_id"): cv.use_id(WavinAHC9000),
        cv.Optional(CONF_TYPE, default=TYPE_YAML): cv.one_of(TYPE_YAML, TYPE_SOFTWARE_VERSION, TYPE_HARDWARE_VERSION, TYPE_DEVICE_NAME, lower=True),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PARENT_ID])
    ts = await text_sensor.new_text_sensor(config)

    typ = config[CONF_TYPE]
    if typ == TYPE_YAML:
        cg.add(hub.set_yaml_text_sensor(ts))
    elif typ == TYPE_SOFTWARE_VERSION:
        cg.add(hub.set_software_version_sensor(ts))
    elif typ == TYPE_HARDWARE_VERSION:
        cg.add(hub.set_hardware_version_sensor(ts))
    elif typ == TYPE_DEVICE_NAME:
        cg.add(hub.set_device_name_sensor(ts))
