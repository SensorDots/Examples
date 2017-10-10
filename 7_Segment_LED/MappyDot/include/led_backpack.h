/*
 * led_backpack.h
 *
 * Created: 9/10/2017 11:28:19 PM
 * Adapted from https://github.com/adafruit/Adafruit_LED_Backpack
 */ 


#ifndef LED_BACKPACK_H_
#define LED_BACKPACK_H_

#include <stdbool.h>

#define HT16K33_BLINK_CMD 0x80
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_BLINK_OFF 0
#define HT16K33_BLINK_2HZ  1
#define HT16K33_BLINK_1HZ  2
#define HT16K33_BLINK_HALFHZ  3

#define HT16K33_CMD_BRIGHTNESS 0xE0

void backpack_setBrightness(uint8_t b);

void backpack_blinkRate(uint8_t b);

//Default addr = 0xE0 (twi code modifies R/W bit, rather than adds like Arduino)
void backpack_begin(uint8_t address);

void backpack_writeDisplay(void);

void backpack_clear(void);

uint8_t backpack_write(uint8_t c);

void backpack_writeDigitRaw(uint8_t d, uint8_t bitmask);

void backpack_drawColon(bool state);

void backpack_writeColon(void);

void backpack_writeDigitNum(uint8_t d, uint8_t num, bool dot);

void backpack_print(long n, int base);

void backpack_printNumber(long n, uint8_t base);

void backpack_printFloat(double n, uint8_t fracDigits, uint8_t base);


#endif /* LED_BACKPACK_H_ */