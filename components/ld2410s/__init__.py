import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID


# Ownership and dependencies
CODEOWNERS = ["@projectzed"]
DEPENDENCIES = ["uart"]
MULTI_CONF = True


# Namespace and component class
ld2410s_ns = cg.esphome_ns.namespace("ld2410s")
LD2410S = ld2410s_ns.class_(
    "LD2410S",
    cg.Component,
    uart.UARTDevice,
)


# ID constant
CONF_LD2410S_ID = "ld2410s_id"


# Configuration schema
CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LD2410S),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


# Enforce UART requirements
FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "ld2410s_uart",
    require_tx=True,
    require_rx=True,
    parity="NONE",
    stop_bits=1,
)


# Code generation
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
