import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import (
    DEVICE_CLASS_IDENTIFY,
    DEVICE_CLASS_RESTART,
    DEVICE_CLASS_UPDATE,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_BUG,
    ICON_RESTART_ALERT,
)

from .. import CONF_LD2410S_ID, LD2410S, ld2410s_ns


# Button class declarations
LD2410SEnableConfigButton = ld2410s_ns.class_(
    "LD2410SEnableConfigButton",
    button.Button,
)
LD2410SDisableConfigButton = ld2410s_ns.class_(
    "LD2410SDisableConfigButton",
    button.Button,
)
LD2410SApplyConfigButton = ld2410s_ns.class_(
    "LD2410SApplyConfigButton",
    button.Button,
)
LD2410SAutoConfigThreshold = ld2410s_ns.class_(
    "LD2410SAutoConfigThreshold",
    button.Button,
)


# Config keys
CONF_ENABLE_CONFIG = "enable_config"
CONF_DISABLE_CONFIG = "disable_config"
CONF_APPLY_CONFIG = "apply_config"
CONF_AUTO_THRESHOLD = "auto_threshold"


# Config schema
CONFIG_SCHEMA = {
    cv.GenerateID(CONF_LD2410S_ID): cv.use_id(LD2410S),
    cv.Optional(CONF_ENABLE_CONFIG): button.button_schema(
        LD2410SEnableConfigButton,
        device_class=DEVICE_CLASS_IDENTIFY,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon=ICON_BUG,
    ),
    cv.Optional(CONF_DISABLE_CONFIG): button.button_schema(
        LD2410SDisableConfigButton,
        device_class=DEVICE_CLASS_IDENTIFY,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon=ICON_BUG,
    ),
    cv.Required(CONF_APPLY_CONFIG): button.button_schema(
        LD2410SApplyConfigButton,
        device_class=DEVICE_CLASS_RESTART,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon=ICON_RESTART_ALERT,
    ),
    cv.Required(CONF_AUTO_THRESHOLD): button.button_schema(
        LD2410SAutoConfigThreshold,
        device_class=DEVICE_CLASS_UPDATE,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:refresh",
    ),
}


async def to_code(config):
    ld2410s_component = await cg.get_variable(config[CONF_LD2410S_ID])

    button_mappings = [
        (CONF_ENABLE_CONFIG, ld2410s_component.set_enable_config_button),
        (CONF_DISABLE_CONFIG, ld2410s_component.set_disable_config_button),
        (CONF_APPLY_CONFIG, ld2410s_component.set_apply_config_button),
        (CONF_AUTO_THRESHOLD, ld2410s_component.set_auto_threshold_button),
    ]

    for conf_key, setter in button_mappings:
        if btn_conf := config.get(conf_key):
            btn = await button.new_button(btn_conf)
            await cg.register_parented(btn, config[CONF_LD2410S_ID])
            cg.add(setter(btn))
