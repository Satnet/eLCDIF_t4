#include "eLCDIF_t4.h"
eLCDIF_t4 lcd;
eLCDIF_t4_config lcd_config = {480, 8, 4, 4, 800, 8, 4, 4, 25, 24, 0, 0};

void setup() {
  // put your setup code here, to run once:
  lcd.begin(lcd_config, BUS_16BIT, WORD_16BIT, COLOR_DEPTH_RGB565);

}

void loop() {
  // put your main code here, to run repeatedly:

}