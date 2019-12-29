import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import spi, st7735_base
from esphome.const import CONF_DC_PIN, CONF_ID, CONF_LAMBDA, CONF_PAGES

AUTO_LOAD = ['st7735_base']
DEPENDENCIES = ['spi']

st7735_spi = cg.esphome_ns.namespace('st7735_spi')
SPIST7735 = st7735_spi.class_('SPIST7735', st7735_base.ST7735, spi.SPIDevice)

CONFIG_SCHEMA = cv.All(st7735_base.ST7735_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(SPIST7735),
    cv.Required(CONF_DC_PIN): pins.gpio_output_pin_schema,
}).extend(cv.COMPONENT_SCHEMA).extend(spi.SPI_DEVICE_SCHEMA),
                       cv.has_at_most_one_key(CONF_PAGES, CONF_LAMBDA))


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield st7735_base.setup_st7735(var, config)
    yield spi.register_spi_device(var, config)

    dc = yield cg.gpio_pin_expression(config[CONF_DC_PIN])
    cg.add(var.set_dc_pin(dc))
