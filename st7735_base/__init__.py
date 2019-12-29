import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import display
from esphome.const import CONF_LAMBDA, CONF_MODEL, CONF_RESET_PIN
from esphome.core import coroutine

st7735_base_ns = cg.esphome_ns.namespace('st7735_base')
ST7735 = st7735_base_ns.class_('ST7735', cg.PollingComponent, display.DisplayBuffer)
ST7735Model = st7735_base_ns.enum('ST7735Model')

MODELS = {
    'ST7735_128X128': ST7735Model.ST7735_MODEL_128_128,
}

ST7735_MODEL = cv.enum(MODELS, upper=True, space="_")

ST7735_SCHEMA = display.FULL_DISPLAY_SCHEMA.extend({
    cv.Required(CONF_MODEL): ST7735_MODEL,
    cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema
}).extend(cv.polling_component_schema('1s'))


@coroutine
def setup_st7735(var, config):
    yield cg.register_component(var, config)
    yield display.register_display(var, config)

    cg.add(var.set_model(config[CONF_MODEL]))
    if CONF_RESET_PIN in config:
        reset = yield cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(reset))
    if CONF_LAMBDA in config:
        lambda_ = yield cg.process_lambda(
            config[CONF_LAMBDA], [(display.DisplayBufferRef, 'it')], return_type=cg.void)
        cg.add(var.set_writer(lambda_))
