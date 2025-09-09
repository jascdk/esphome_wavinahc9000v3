import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_UPDATE_INTERVAL,
)

from esphome.components import climate, uart
from esphome import pins

CODEOWNERS = ["@you"]

AUTO_LOAD = ["climate", "uart"]

namespace = cg.esphome_ns.namespace("wavin_ahc9000")
WavinAHC9000 = namespace.class_("WavinAHC9000", cg.PollingComponent, uart.UARTDevice)
WavinZoneClimate = namespace.class_("WavinZoneClimate", climate.Climate, cg.Component)
CONF_TEMP_DIVISOR = "temp_divisor"
CONF_UART_ID = "uart_id"
CONF_TX_ENABLE_PIN = "tx_enable_pin"
CONF_RECEIVE_TIMEOUT_MS = "receive_timeout_ms"
CONF_POLL_CHANNELS_PER_CYCLE = "poll_channels_per_cycle"
def _channels_list(value):
    return cv.ensure_list(cv.int_range(min=1, max=16))(value)


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(WavinAHC9000),
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Optional(CONF_TX_ENABLE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_TEMP_DIVISOR, default=10.0): cv.positive_float,
            cv.Optional(CONF_RECEIVE_TIMEOUT_MS, default=1000): cv.positive_int,
            cv.Optional(CONF_UPDATE_INTERVAL, default="5s"): cv.update_interval,
            cv.Optional(CONF_POLL_CHANNELS_PER_CYCLE, default=2): cv.int_range(min=1, max=16),
        }
    )
    .extend(cv.polling_component_schema("5s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    uart_component = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    # Configure device-level params
    cg.add(var.set_temp_divisor(config[CONF_TEMP_DIVISOR]))
    cg.add(var.set_receive_timeout_ms(config[CONF_RECEIVE_TIMEOUT_MS]))
    if CONF_TX_ENABLE_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_TX_ENABLE_PIN])
        cg.add(var.set_tx_enable_pin(pin))
    if CONF_POLL_CHANNELS_PER_CYCLE in config:
        cg.add(var.set_poll_channels_per_cycle(config[CONF_POLL_CHANNELS_PER_CYCLE]))

    # Entities are defined under climate:/sensor: platforms (see climate.py and sensor/) 
