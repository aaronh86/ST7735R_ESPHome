#include "st7735_base.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace st7735_base {

#define ST_CMD_DELAY      0x80    // special signifier for command lists

#define ST77XX_NOP        0x00
#define ST77XX_SWRESET    0x01
#define ST77XX_RDDID      0x04
#define ST77XX_RDDST      0x09

#define ST77XX_SLPIN      0x10
#define ST77XX_SLPOUT     0x11
#define ST77XX_PTLON      0x12
#define ST77XX_NORON      0x13

#define ST77XX_INVOFF     0x20
#define ST77XX_INVON      0x21
#define ST77XX_DISPOFF    0x28
#define ST77XX_DISPON     0x29
#define ST77XX_CASET      0x2A
#define ST77XX_RASET      0x2B
#define ST77XX_RAMWR      0x2C
#define ST77XX_RAMRD      0x2E

#define ST77XX_PTLAR      0x30
#define ST77XX_TEOFF      0x34
#define ST77XX_TEON       0x35
#define ST77XX_MADCTL     0x36
#define ST77XX_COLMOD     0x3A

#define ST77XX_MADCTL_MY  0x80
#define ST77XX_MADCTL_MX  0x40
#define ST77XX_MADCTL_MV  0x20
#define ST77XX_MADCTL_ML  0x10
#define ST77XX_MADCTL_RGB 0x00

#define ST77XX_RDID1      0xDA
#define ST77XX_RDID2      0xDB
#define ST77XX_RDID3      0xDC
#define ST77XX_RDID4      0xDD


// some flags for initR() :(
#define INITR_GREENTAB    0x00
#define INITR_REDTAB      0x01
#define INITR_BLACKTAB    0x02
#define INITR_18GREENTAB  INITR_GREENTAB
#define INITR_18REDTAB    INITR_REDTAB
#define INITR_18BLACKTAB  INITR_BLACKTAB
#define INITR_144GREENTAB 0x01
#define INITR_MINI160x80  0x04
#define INITR_HALLOWING   0x05

// Some register settings
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH  0x04

#define ST7735_FRMCTR1    0xB1
#define ST7735_FRMCTR2    0xB2
#define ST7735_FRMCTR3    0xB3
#define ST7735_INVCTR     0xB4
#define ST7735_DISSET5    0xB6

#define ST7735_PWCTR1     0xC0
#define ST7735_PWCTR2     0xC1
#define ST7735_PWCTR3     0xC2
#define ST7735_PWCTR4     0xC3
#define ST7735_PWCTR5     0xC4
#define ST7735_VMCTR1     0xC5

#define ST7735_PWCTR6     0xFC

#define ST7735_GMCTRP1    0xE0
#define ST7735_GMCTRN1    0xE1

static const uint8_t PROGMEM
    Rcmd1[] = {                       // 7735R init, part 1 (red or green tab)
        15,                             // 15 commands in list:
        ST77XX_SWRESET,   ST_CMD_DELAY, //  1: Software reset, 0 args, w/delay
        150,                          //     150 ms delay
        ST77XX_SLPOUT,    ST_CMD_DELAY, //  2: Out of sleep mode, 0 args, w/delay
        255,                          //     500 ms delay
        ST7735_FRMCTR1, 3,              //  3: Framerate ctrl - normal mode, 3 arg:
        0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
        ST7735_FRMCTR2, 3,              //  4: Framerate ctrl - idle mode, 3 args:
        0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
        ST7735_FRMCTR3, 6,              //  5: Framerate - partial mode, 6 args:
        0x01, 0x2C, 0x2D,             //     Dot inversion mode
        0x01, 0x2C, 0x2D,             //     Line inversion mode
        ST7735_INVCTR,  1,              //  6: Display inversion ctrl, 1 arg:
        0x07,                         //     No inversion
        ST7735_PWCTR1,  3,              //  7: Power control, 3 args, no delay:
        0xA2,
        0x02,                         //     -4.6V
        0x84,                         //     AUTO mode
        ST7735_PWCTR2,  1,              //  8: Power control, 1 arg, no delay:
        0xC5,                         //     VGH25=2.4C VGSEL=-10 VGH=3 * AVDD
        ST7735_PWCTR3,  2,              //  9: Power control, 2 args, no delay:
        0x0A,                         //     Opamp current small
        0x00,                         //     Boost frequency
        ST7735_PWCTR4,  2,              // 10: Power control, 2 args, no delay:
        0x8A,                         //     BCLK/2,
        0x2A,                         //     opamp current small & medium low
        ST7735_PWCTR5,  2,              // 11: Power control, 2 args, no delay:
        0x8A, 0xEE,
        ST7735_VMCTR1,  1,              // 12: Power control, 1 arg, no delay:
        0x0E,
        ST77XX_INVOFF,  0,              // 13: Don't invert display, no args
        ST77XX_MADCTL,  1,              // 14: Mem access ctl (directions), 1 arg:
        0xC8,                         //     row/col addr, bottom-top refresh
        ST77XX_COLMOD,  1,              // 15: set color mode, 1 arg, no delay:
        0x05 },                       //     16-bit color

    Rcmd2green[] = {                  // 7735R init, part 2 (green tab only)
        2,                              //  2 commands in list:
        ST77XX_CASET,   4,              //  1: Column addr set, 4 args, no delay:
        0x00, 0x02,                   //     XSTART = 0
        0x00, 0x7F+0x02,              //     XEND = 127
        ST77XX_RASET,   4,              //  2: Row addr set, 4 args, no delay:
        0x00, 0x01,                   //     XSTART = 0
        0x00, 0x9F+0x01 },            //     XEND = 159
    Rcmd2green144[] = {               // 7735R init, part 2 (green 1.44 tab)
        2,                              //  2 commands in list:
        ST77XX_CASET,   4,              //  1: Column addr set, 4 args, no delay:
        0x00, 0x00,                   //     XSTART = 0
        0x00, 0x7F,                   //     XEND = 127
        ST77XX_RASET,   4,              //  2: Row addr set, 4 args, no delay:
        0x00, 0x00,                   //     XSTART = 0
        0x00, 0x7F },                 //     XEND = 127
    Rcmd3[] = {                       // 7735R init, part 3 (red or green tab)
        4,                              //  4 commands in list:
        ST7735_GMCTRP1, 16      ,       //  1: Gamma Adjustments (pos. polarity), 16 args + delay:
        0x02, 0x1c, 0x07, 0x12,       //     (Not entirely necessary, but provides
        0x37, 0x32, 0x29, 0x2d,       //      accurate colors)
        0x29, 0x25, 0x2B, 0x39,
        0x00, 0x01, 0x03, 0x10,
        ST7735_GMCTRN1, 16      ,       //  2: Gamma Adjustments (neg. polarity), 16 args + delay:
        0x03, 0x1d, 0x07, 0x06,       //     (Not entirely necessary, but provides
        0x2E, 0x2C, 0x29, 0x2D,       //      accurate colors)
        0x2E, 0x2E, 0x37, 0x3F,
        0x00, 0x00, 0x02, 0x10,
        ST77XX_NORON,     ST_CMD_DELAY, //  3: Normal display on, no args, w/delay
        10,                           //     10 ms delay
        ST77XX_DISPON,    ST_CMD_DELAY, //  4: Main screen turn on, no args w/delay
        100 };                        //     100 ms delay        

static const char *TAG = "st7735";

void ST7735::setup() {
  this->init_internal_(this->get_buffer_length_());

  this->displayInit(Rcmd1);
  this->displayInit(Rcmd2green144);
  this->displayInit(Rcmd3);

  this->fill(0);
}

void ST7735::display() {
  this->command(ST77XX_CASET); /* set column address */
  this->command(_colstart);               /* set column start address */
  this->command(_width);               /* set column end address */
  this->command(ST77XX_RASET); /* set row address */
  this->command(_rowstart);               /* set row start address */
  this->command(_height);               /* set row end address */

  this->write_display_data();
}

void ST7735::update() {
  this->do_update_();
  this->display();
}

int ST7735::get_height_internal() {
  switch (this->model_) {
    case ST7735_MODEL_128_128:
      return 128;
    default:
      return 0;
  }
}

int ST7735::get_width_internal() {
  switch (this->model_) {
    case ST7735_MODEL_128_128:
      return 128;
    default:
      return 0;
  }
}

size_t ST7735::get_buffer_length_() {
  return size_t(this->get_width_internal()) * size_t(this->get_height_internal()) / 8u;
}

void HOT ST7735::draw_absolute_pixel_internal(int x, int y, int color) {
  this->setAddrWindow(x,y,0,0); 
  this->writeColor(ST7735_WHITE); //conert int to uint16
}

void ST7735::fill(int color) {
  uint16_t fill = color ? ST7735_WHITE : ST7735_BLACK;
  this->setAddrWindow(1,1,_width,_height);
  this->blank();
}

void ST7735::init_reset_() {
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(true); 
    delay(10);
    this->reset_pin_->digital_write(false); // Trigger Reset
    delay(150);
    this->reset_pin_->digital_write(true);
    delay(150);
    ESP_LOGCONFIG(TAG, "SPI Reset");
  }
}

const char *ST7735::model_str_() {
  switch (this->model_) {
    case ST7735_MODEL_128_128:
      return "ST7735 128x128";
    default:
      return "Unknown";
  }
}

void ST7735::setAddrWindow(uint16_t x, uint16_t y, uint16_t w,
  uint16_t h) {
  x += _colstart;
  y += _rowstart;
  uint32_t xa = ((uint32_t)x << 16) | (x+w-1);
  uint32_t ya = ((uint32_t)y << 16) | (y+h-1); 

  this->command(ST77XX_CASET); // Column addr set
  this->data32(xa);

  this->command(ST77XX_RASET); // Row addr set
  this->data32(ya);

  this->command(ST77XX_RAMWR); // write to RAM
}

void ST7735::writeColor(uint16_t color){
  this->data16(color);    
  }

void ST7735::displayInit(const uint8_t *addr) {

  uint8_t  numCommands, cmd, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    cmd = pgm_read_byte(addr++);         // Read command
    numArgs  = pgm_read_byte(addr++);    // Number of args to follow
    ms       = numArgs & ST_CMD_DELAY;   // If hibit set, delay follows args
    numArgs &= ~ST_CMD_DELAY;            // Mask out delay bit
    this->Sendcommand(cmd, addr, numArgs);  
    addr += numArgs;

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delay(ms);
    }
  }
}


}
}

