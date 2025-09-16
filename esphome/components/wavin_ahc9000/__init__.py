import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, climate, number
from esphome.const import CONF_ID
from esphome import pins

CODEOWNERS = ["@you"]
AUTO_LOAD = ["climate", "uart", "number"]

ns = cg.esphome_ns.namespace("wavin_ahc9000")
WavinAHC9000 = ns.class_("WavinAHC9000", cg.PollingComponent, uart.UARTDevice)
WavinZoneClimate = ns.class_("WavinZoneClimate", climate.Climate, cg.Component)
WavinSetpointNumber = ns.class_("WavinSetpointNumber", number.Number, cg.Component)

CONF_UART_ID = "uart_id"
CONF_TX_ENABLE_PIN = "tx_enable_pin"
CONF_TEMP_DIVISOR = "temp_divisor"
CONF_RECEIVE_TIMEOUT_MS = "receive_timeout_ms"
CONF_POLL_CHANNELS_PER_CYCLE = "poll_channels_per_cycle"
CONF_ALLOW_MODE_WRITES = "allow_mode_writes"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WavinAHC9000),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Optional(CONF_TX_ENABLE_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_TEMP_DIVISOR, default=10.0): cv.positive_float,
        cv.Optional(CONF_RECEIVE_TIMEOUT_MS, default=1000): cv.positive_int,
        cv.Optional(CONF_POLL_CHANNELS_PER_CYCLE, default=2): cv.int_range(min=1, max=16),
    cv.Optional(CONF_ALLOW_MODE_WRITES, default=True): cv.boolean,
    }
).extend(uart.UART_DEVICE_SCHEMA).extend(cv.polling_component_schema("5s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await uart.register_uart_device(var, config)
    await cg.register_component(var, config)
    if CONF_TX_ENABLE_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_TX_ENABLE_PIN])
        cg.add(var.set_tx_enable_pin(pin))
    if CONF_TEMP_DIVISOR in config:
        cg.add(var.set_temp_divisor(config[CONF_TEMP_DIVISOR]))
    if CONF_RECEIVE_TIMEOUT_MS in config:
        cg.add(var.set_receive_timeout_ms(config[CONF_RECEIVE_TIMEOUT_MS]))
    if CONF_POLL_CHANNELS_PER_CYCLE in config:
        cg.add(var.set_poll_channels_per_cycle(config[CONF_POLL_CHANNELS_PER_CYCLE]))
    if CONF_ALLOW_MODE_WRITES in config:
        cg.add(var.set_allow_mode_writes(config[CONF_ALLOW_MODE_WRITES]))
