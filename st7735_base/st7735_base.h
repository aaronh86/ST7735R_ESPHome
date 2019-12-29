#pragma once

#include "esphome/core/component.h"
#include "esphome/core/esphal.h"
#include "esphome/components/display/display_buffer.h"

namespace esphome {
namespace st7735_base {

#define ST7735_TFTWIDTH_128   128 // for 1.44 and mini
#define ST7735_TFTWIDTH_80     80 // for mini
#define ST7735_TFTHEIGHT_128  128 // for 1.44" display
#define ST7735_TFTHEIGHT_160  160 // for 1.8" and mini display

// Some ready-made 16-bit ('565') color settings:
#define	ST77XX_BLACK      0x0000
#define ST77XX_WHITE      0xFFFF
#define	ST77XX_RED        0xF800
#define	ST77XX_GREEN      0x07E0
#define	ST77XX_BLUE       0x001F
#define ST77XX_CYAN       0x07FF
#define ST77XX_MAGENTA    0xF81F
#define ST77XX_YELLOW     0xFFE0
#define	ST77XX_ORANGE     0xFC00

// Some ready-made 16-bit ('565') color settings:
#define ST7735_BLACK      ST77XX_BLACK
#define ST7735_WHITE      ST77XX_WHITE
#define ST7735_RED        ST77XX_RED
#define ST7735_GREEN      ST77XX_GREEN
#define ST7735_BLUE       ST77XX_BLUE
#define ST7735_CYAN       ST77XX_CYAN
#define ST7735_MAGENTA    ST77XX_MAGENTA
#define ST7735_YELLOW     ST77XX_YELLOW
#define ST7735_ORANGE     ST77XX_ORANGE

enum ST7735Model {
  ST7735_MODEL_128_128 = 0
};


class ST7735 : public PollingComponent, public display::DisplayBuffer {
 public:
  void setup() override;

  void display();

  void update() override;

  void set_model(ST7735Model model) { this->model_ = model; }
  void set_reset_pin(GPIOPin *reset_pin) { this->reset_pin_ = reset_pin; }

  float get_setup_priority() const override { return setup_priority::PROCESSOR; }
  void fill(int color) override;

 protected:
  virtual void command(uint8_t value) = 0;
  virtual void Sendcommand(uint8_t cmd, const uint8_t* dataBytes, uint8_t numDataBytes) = 0;
  virtual void Senddata(const uint8_t* dataBytes, uint8_t numDataBytes) = 0;
  virtual void data(uint8_t value) = 0;
  virtual void data16(uint16_t value) = 0;
  virtual void data32(uint32_t value) = 0;
  virtual void write_display_data() = 0;
  virtual void blank() = 0;

  void init_reset_();
  void displayInit(const uint8_t *addr);
  void setAddrWindow(uint16_t x, uint16_t y, uint16_t w,  uint16_t h);
  void draw_absolute_pixel_internal(int x, int y, int color) override;
  void writeColor(uint16_t color);
  
  int get_height_internal() override;
  int get_width_internal() override;
  size_t get_buffer_length_();
  const char *model_str_();

  ST7735Model model_{ST7735_MODEL_128_128};
  uint8_t _colstart = 2, //needs this or crashes
          _rowstart = 3; //needs this or crashes
  int16_t _width = ST7735_TFTWIDTH_128,         ///< Display width as modified by current rotation
          _height = ST7735_TFTHEIGHT_128;        ///< Display height as modified by current rotation
  
  GPIOPin *reset_pin_{nullptr};
};

}  // namespace ssd1325_base
}  // namespace esphome