import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import ENTITY_CATEGORY_CONFIG

from .. import CONF_LD2410S_ID, LD2410S, ld2410s_ns


# Config keys and options
CONF_RESPONSE_SPEED = "response_speed"
RESPONSE_SPEED_OPTIONS = ["Normal", "Fast"]


# Select class declaration
LD2410SResponseSpeedSelect = ld2410s_ns.class_(
    "LD2410SResponseSpeedSelect",
    cg.Component,
)


# Config schema
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_LD2410S_ID): cv.use_id(LD2410S),
        cv.Required(CONF_RESPONSE_SPEED): select.select_schema(
            LD2410SResponseSpeedSelect,
            entity_category=ENTITY_CATEGORY_CONFIG,
        ),
    }
)


# Code generation
async def to_code(config):
    ld2410s = await cg.get_variable(config[CONF_LD2410S_ID])

    if response_speed_cfg := config.get(CONF_RESPONSE_SPEED):
        sel = await select.new_select(
            response_speed_cfg,
            options=RESPONSE_SPEED_OPTIONS,
        )
        await cg.register_parented(sel, config[CONF_LD2410S_ID])
        cg.add(ld2410s.set_response_speed_select(sel))
