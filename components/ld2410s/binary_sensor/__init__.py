import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (
    CONF_HAS_TARGET,
    CONF_ID,
    DEVICE_CLASS_EMPTY,
    DEVICE_CLASS_OCCUPANCY,
)

from .. import CONF_LD2410S_ID, LD2410S, ld2410s_ns


# Binary sensor component class
LD2410SBinarySensor = ld2410s_ns.class_(
    "LD2410SBinarySensor",
    binary_sensor.BinarySensor,
    cg.Component,
)


# Config keys
CONF_THRESHOLD_UPDATE = "has_threshold_update"


# Config schema
CONFIG_SCHEMA = cv.All(
    cv.COMPONENT_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(LD2410SBinarySensor),
            cv.GenerateID(CONF_LD2410S_ID): cv.use_id(LD2410S),
            cv.Optional(CONF_HAS_TARGET): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_OCCUPANCY
            ),
            cv.Optional(CONF_THRESHOLD_UPDATE): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_EMPTY
            ),
        }
    )
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Map config keys to setter functions
    sensor_mappings = [
        (CONF_HAS_TARGET, var.set_presence_sensor),
        (CONF_THRESHOLD_UPDATE, var.set_threshold_update_sensor),
    ]

    for conf_key, setter in sensor_mappings:
        if sensor_conf := config.get(conf_key):
            sens = await binary_sensor.new_binary_sensor(sensor_conf)
            cg.add(setter(sens))

    # Register this binary sensor as a listener to the parent LD2410S component
    ld2410s = await cg.get_variable(config[CONF_LD2410S_ID])
    cg.add(ld2410s.register_listener(var))
