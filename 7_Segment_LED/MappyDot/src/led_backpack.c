/*
 * led_backpack.c
 *
 * Created: 9/10/2017 11:27:57 PM
 * Adapted from https://github.com/adafruit/Adafruit_LED_Backpack
 */ 

#include <stdint.h>
#include "led_backpack.h"
#include "i2c_master_external.h"

uint8_t i2c_addr;

uint16_t displaybuffer[8]; 

uint8_t position;

const uint8_t numbertable[] = {
	 0x3F, /* 0 */
	 0x06, /* 1 */
	 0x5B, /* 2 */
	 0x4F, /* 3 */
	 0x66, /* 4 */
	 0x6D, /* 5 */
	 0x7D, /* 6 */
	 0x07, /* 7 */
	 0x7F, /* 8 */
	 0x6F, /* 9 */
	 0x77, /* a */
	 0x7C, /* b */
	 0x39, /* C */
	 0x5E, /* d */
	 0x79, /* E */
	 0x71, /* F */
 };



void backpack_setBrightness(uint8_t b) {
	 if (b > 15) b = 15;
	 i2c_start_external(i2c_addr);
	 i2c_write_external(HT16K33_CMD_BRIGHTNESS | b);
	 i2c_stop_external();
 }

 void backpack_blinkRate(uint8_t b) {
	 if (b > 3) b = 0; // turn off if not sure
	 i2c_start_external(i2c_addr);
	 i2c_write_external(HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1));
	 i2c_stop_external();
}

//Default addr = 0xE0 (twi code modifies R/W bit, rather than adds like Arduino)
void backpack_begin(uint8_t address) {
	i2c_addr = address;
	position = 0;

	i2c_init_external();

	i2c_start_external(i2c_addr);
	i2c_write_external(0x21);  // turn on oscillator
	i2c_stop_external();
	backpack_blinkRate(HT16K33_BLINK_OFF);

	backpack_setBrightness(10);
}

void backpack_writeDisplay(void) {
    i2c_start_external(i2c_addr);
	i2c_write_external((uint8_t)0x00); // start at address $00

	for (uint8_t i=0; i<8; i++) {
		i2c_write_external(displaybuffer[i] & 0xFF);
		i2c_write_external(displaybuffer[i] >> 8);
	}
	i2c_stop_external();
}

void backpack_clear(void) {
	for (uint8_t i=0; i<8; i++) {
		displaybuffer[i] = 0;
	}
}

uint8_t backpack_write(uint8_t c) {

	uint8_t r = 0;

	if (c == '\n') position = 0;
	if (c == '\r') position = 0;

	if ((c >= '0') && (c <= '9')) {
		backpack_writeDigitNum(position, c-'0', 0);
		r = 1;
	}

	position++;
	if (position == 2) position++;

	return r;
}

void backpack_writeDigitRaw(uint8_t d, uint8_t bitmask) {
	if (d > 4) return;
	displaybuffer[d] = bitmask;
}

void backpack_drawColon(bool state) {
	if (state)
	displaybuffer[2] = 0x2;
	else
	displaybuffer[2] = 0;
}

void backpack_writeColon(void) {
    i2c_start_external(i2c_addr);
	i2c_write_external((uint8_t)0x04); // start at address $02
	
	i2c_write_external(displaybuffer[2] & 0xFF);
	i2c_write_external(displaybuffer[2] >> 8);
	i2c_stop_external();

}

void backpack_writeDigitNum(uint8_t d, uint8_t num, bool dot) {
	if (d > 4) return;

	backpack_writeDigitRaw(d, numbertable[num] | (dot << 7));
}

void backpack_print(long n, int base)
{
	backpack_printNumber(n, base);
}

void backpack_printNumber(long n, uint8_t base)
{
	backpack_printFloat(n, 0, base);
}

void backpack_printFloat(double n, uint8_t fracDigits, uint8_t base)
{
	uint8_t numericDigits = 4;   // available digits on display
	bool isNegative = 0;  // true if the number is negative
	
	// is the number negative?
	if(n < 0) {
		isNegative = 1;  // need to draw sign later
		--numericDigits;    // the sign will take up one digit
		n *= -1;            // pretend the number is positive
	}
	
	// calculate the factor required to shift all fractional digits
	// into the integer part of the number
	double toIntFactor = 1.0;
	for(int i = 0; i < fracDigits; ++i) toIntFactor *= base;
	
	// create integer containing digits to display by applying
	// shifting factor and rounding adjustment
	uint32_t displayNumber = n * toIntFactor + 0.5;
	
	// calculate upper bound on displayNumber given
	// available digits on display
	uint32_t tooBig = 1;
	for(int i = 0; i < numericDigits; ++i) tooBig *= base;
	
	// if displayNumber is too large, try fewer fractional digits
	while(displayNumber >= tooBig) {
		--fracDigits;
		toIntFactor /= base;
		displayNumber = n * toIntFactor + 0.5;
	}
	
	// did toIntFactor shift the decimal off the display?
	if (toIntFactor < 1) {
		//printError();
		} else {
		// otherwise, display the number
		int8_t displayPos = 4;
		
		if (displayNumber)  //if displayNumber is not 0
		{
			for(uint8_t i = 0; displayNumber || i <= fracDigits; ++i) {
				bool displayDecimal = (fracDigits != 0 && i == fracDigits);
				backpack_writeDigitNum(displayPos--, displayNumber % base, displayDecimal);
				if(displayPos == 2) backpack_writeDigitRaw(displayPos--, 0x00);
				displayNumber /= base;
			}
		}
		else {
			backpack_writeDigitNum(displayPos--, 0, 0);
		}
		
		// display negative sign if negative
		if(isNegative) backpack_writeDigitRaw(displayPos--, 0x40);
		
		// clear remaining display positions
		while(displayPos >= 0) backpack_writeDigitRaw(displayPos--, 0x00);
	}
}