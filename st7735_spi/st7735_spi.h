#pragma once

#include "esphome/core/component.h"
#include "esphome/components/st7735_base/st7735_base.h"
#include "esphome/components/spi/spi.h"

namespace esphome {
namespace st7735_spi {

class SPIST7735 : public st7735_base::ST7735,
                   public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING,
                                         spi::DATA_RATE_8MHZ> {
 public:
  void set_dc_pin(GPIOPin *dc_pin) { dc_pin_ = dc_pin; }

  void setup() override;

  void dump_config() override;

 protected:
  void command(uint8_t value) override;
  void data(uint8_t value) override;
  void data16(uint16_t value) override;
  void data32(uint32_t value) override;
  void Sendcommand(uint8_t cmd, const uint8_t* dataBytes, uint8_t numDataBytes) override;
  void Senddata(const uint8_t* dataBytes, uint8_t numDataBytes) override;
  void blank() override;
  void write_display_data() override;

  GPIOPin *dc_pin_;
};

}  // namespace st7735_spi
}  // namespace esphome
