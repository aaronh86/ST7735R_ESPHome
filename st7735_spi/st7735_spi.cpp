#include "st7735_spi.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace st7735_spi {

static const char *TAG = "st7735_spi";

void SPIST7735::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SPI ST7735...");
  this->spi_setup();
  
  this->dc_pin_->setup();  // OUTPUT
  this->cs_->setup();      // OUTPUT

  this->dc_pin_->digital_write(true);
  this->cs_->digital_write(true);
  
  this->init_reset_();
  delay(100);  // NOLINT
  ST7735::setup();
}
void SPIST7735::dump_config() {
  LOG_DISPLAY("", "SPI ST7735", this);
  ESP_LOGCONFIG(TAG, "  Model: %s", this->model_str_());
  LOG_PIN("  CS Pin: ", this->cs_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_UPDATE_INTERVAL(this);
}

void SPIST7735::command(uint8_t value) {
  this->dc_pin_->digital_write(false); //pull DC low to indicate Command
  this->cs_->digital_write(false);
  this->enable();
  this->transfer_byte(value);  //write byte - SPI library
  this->cs_->digital_write(true);
  this->disable();
}

void SPIST7735::Sendcommand(uint8_t cmd, const uint8_t* dataBytes, uint8_t numDataBytes) { 
  this->command(cmd);  //write command - SPI library 
  this->Senddata(dataBytes,numDataBytes);
}

void SPIST7735::Senddata(const uint8_t* dataBytes, uint8_t numDataBytes) {
  this->dc_pin_->digital_write(true); //pull DC high to indicate data
  this->cs_->digital_write(false);
  this->enable();
  for (uint8_t i=0; i<numDataBytes; i++) {
    this->transfer_byte(pgm_read_byte(dataBytes++));  //write byte - SPI library
  }
  this->cs_->digital_write(true);
  this->disable();

}

void SPIST7735::data(uint8_t value) {
  this->dc_pin_->digital_write(true); //pull DC high to indicate data
  this->cs_->digital_write(false);
  this->enable();
  this->transfer_byte(value);  //write byte - SPI library
  this->cs_->digital_write(true);
  this->disable();
}

void SPIST7735::data16(uint16_t w){
  this->dc_pin_->digital_write(true); //pull DC high to indicate data
  this->cs_->digital_write(false);
  this->enable();
  this->transfer_byte(w >> 8);  //write MSB - SPI library
  this->transfer_byte(w);  //write LSB - SPI library
  this->cs_->digital_write(true);
  this->disable();
}

void SPIST7735::data32(uint32_t w){
  this->dc_pin_->digital_write(true); //pull DC high to indicate data
  this->cs_->digital_write(false); //set CS low to send data
  this->enable();
  this->transfer_byte(w >> 24);  //write MSB - SPI library
  this->transfer_byte(w >> 16);  //write byte - SPI library
  this->transfer_byte(w >> 8);  //write byte - SPI library
  this->transfer_byte(w);  //write LSB - SPI library
  this->cs_->digital_write(true); //set CS high to send data
  this->disable();
}

void HOT SPIST7735::write_display_data() {
  this->cs_->digital_write(true);
  this->dc_pin_->digital_write(true);
  this->cs_->digital_write(false);
  this->enable();
  for (uint16_t x = _colstart ; x < this->get_width_internal(); x += 2) {
    for (uint16_t y = _rowstart; y < this->get_height_internal(); y += 8) {  // we write 8 pixels at once
      uint8_t left8 = this->buffer_[y * 16 + x];
      uint8_t right8 = this->buffer_[y * 16 + x + 1];
      for (uint8_t p = 0; p < 8; p++) {
        uint8_t d = 0;
        if (left8 & (1 << p))
          d |= 0xF0;
        if (right8 & (1 << p))
          d |= 0x0F;
        this->write_byte(d);
      }
    }
  }
  this->cs_->digital_write(true);
  this->disable();
}


void SPIST7735::blank(){
  this->dc_pin_->digital_write(true); //pull DC high to indicate data
  this->cs_->digital_write(false);

  uint16_t w;

  this->enable();
  for (int i= 0; i<_width; i++) {
    for (int j= 0; j<_height; j++) {  
      w = ST7735_BLACK;    
      this->transfer_byte(w >> 8);  //write MSB - SPI library
      this->transfer_byte(w);  //write LSB - SPI library
    }
  }
  
  this->cs_->digital_write(true); 
  this->disable();
}

}  // namespace ssd1325_spi
}  // namespace esphome
