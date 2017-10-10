/**
   MappyDot Firmware

   Copyright (C) 2017 SensorDots.org

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   Features Not Currently Implemented

   - I2C Passthrough Mode
   - MappyDot Mode
   - Set as master for inter-device crosstalk. This allows you to group devices.
   - Add Initisation Error Codes

   ATmega328pb:
   Fuse E: 0xfd (2.7V BOD)
   Fuse H: 0xd4 (512 words bootloader)
   Fuse L: 0xc2 (8Mhz internal RC-Osz.)
   BOOTRST - starts at boot sector on boot.
   Bootloader LED Flashes every ~25ms
   Please note, the Debug release of this firmware will not fit with
   the bootloader. When developing and debugging you will need to 
   disable some functions or disable the bootloader and BOOTRST. 
   Some non critical functions can be disabled with the DEV_DISABLE 
   compiler flag.
   Compiled size needs to be under 7C00 bytes to work with bootloader 

 */

#include <atmel_start.h>
#include <avr/wdt.h>
#include <math.h>
#include <string.h>
#include <util/delay.h>
#include "vl53l0x.h"
#include "history_buffer.h"
#include "bwlpf.h"
#include "mappydot_reg.h"
#include "tc16.h"
#include "stackmon.h"
#include "vl53l0x_types.h"
#include "main.h"
#include "led_backpack.h"

#ifdef DEV_DISABLE
#warning Some functions are disabled with DEV_DISABLE
#endif


#define RANGE_TIMEOUT_VALUE           300000
#define FILTER_ORDER                  2  //Filter order
#define SAMPLING_FREQ                 30 //Samples per second
#define FILTER_FREQUENCY              6  //Hz
#define MIN_DIST                      30 //minimum distance from VL53L0X
#define MAX_DIST                      4000 //Max sanity distance from VL53L0X
#define ACCURACY_LIMIT                1200
#define T_BOOT_MS                     2 //tBOOT is 1.2ms max as per VL53L0X datasheet

/* Private methods */
#ifndef DEV_DISABLE
static uint16_t avg(uint16_t * array, uint8_t count);
#endif
static void flash_led(uint32_t timeout_ms, int8_t num_of_flashes, bool pwm);
static uint8_t translate_measurement_mode(uint8_t measurement_mode);
static uint8_t translate_ranging_mode(uint8_t ranging_mode);
static void delay_ms(uint16_t ms);


/* State Variables */
#ifndef DEV_DISABLE
circular_history_buffer history_buffer;
#endif
filter_state low_pass_filter_state;

/* We do this so the struct is allocated as pointers are not allocated */
VL53L0X_Dev_t device;
VL53L0X_Dev_t * pDevice;
VL53L0X_RangingMeasurementData_t measure;
VL53L0X_Error status;

uint16_t filtered_distance;
uint16_t real_distance;
uint8_t error_code;
uint16_t distance_error;
uint16_t current_millimeters  = 0;

volatile bool filtering_enabled  = 1;
volatile bool averaging_enabled  = 1;
volatile bool vl53l0x_powerstate = 0;
volatile bool passthrough_mode   = 0;

volatile uint8_t led_mode                      = LED_PWM_ENABLED;
volatile uint8_t gpio_mode                     = GPIO_MEASUREMENT_INTERRUPT;
volatile uint8_t current_ranging_mode          = SET_CONTINUOUS_RANGING_MODE;
volatile uint8_t current_measurement_mode      = LONG_RANGE;
volatile uint32_t led_threshold                = 1000;
volatile uint32_t gpio_threshold               = 300;
volatile uint8_t averaging_size                = 15;

/* SPAD Calibration */
uint32_t refSpadCount = 0;
uint8_t ApertureSpads = 0;

/* Distance Calibration */
int32_t offsetMicroMeter = 0;

/* Crosstalk (cover) Calibration */
FixPoint1616_t xTalkCompensationRateMegaCps = 0;

/* Temperature Calibrate */
uint8_t vhvSettings = 0;
uint8_t phaseCal = 0;

int8_t led_pulse = 0;
uint8_t led_pulse_dir = 0;
uint16_t led_pulse_timeout = 0;

static volatile bool measurement_interrupt_fired = false;

int main(void)
{	   
    /* Initializes MCU, drivers and middleware */
    atmel_start_init();

    /* Set XSHUT to high to turn on (Shutdown is active low). */
    XSHUT_set_level(true);
	//_delay_ms(T_BOOT_MS);

    /* Set sync to input and no pull mode as it's connected to MST. */
    SYNC_set_dir(PORT_DIR_IN);
    SYNC_set_pull_mode(PORT_PULL_OFF);

	/* Enable interrupts */
    sei();


	/* Setup Crosstalk reduction interrupt */
	/* PB6 - PCINT6 (PCMSK1) */
	PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
	PCMSK0 |= (1 << PCINT6);  // set PCINT6 to trigger an interrupt on state change
#ifndef DEV_DISABLE

    /* Initialise history buffer */
    hb_init(&history_buffer,averaging_size,sizeof(uint16_t));

    /* Initialise Filter */
    bwlpf_init(&low_pass_filter_state,FILTER_ORDER,SAMPLING_FREQ,FILTER_FREQUENCY);

#endif

    /* Ranging Interrupt setup */
    /* Interrupt is active low when fired (after ranging complete), rather than high. */
    /* Set GPIO1 (interrupt) pin to input */
    GPIO1_set_dir(PORT_DIR_IN);
    GPIO1_set_pull_mode(PORT_PULL_OFF);

    /* GPIO POUT Setup */
    SYNC_set_dir(PORT_DIR_OUT);

    /* Setup measurement interrupt */
    /* PC3 - PCINT11 (PCMSK1) */
    PCICR |= (1 << PCIE1);    // set PCIE1 to enable PCMSK1 scan
    PCMSK1 |= (1 << PCINT11);  // set PCINT11 to trigger an interrupt on state change


	#ifdef FILL_SRAM_DEBUG
	int16_t count = StackCount();
	#endif

	/* VL53L0X Init */

	/* Assign struct to pointer */
	pDevice = &device;

	if (!init_ranging(pDevice, &status, translate_ranging_mode(current_ranging_mode), translate_measurement_mode(current_measurement_mode),
					  refSpadCount,ApertureSpads,offsetMicroMeter,xTalkCompensationRateMegaCps,vhvSettings,phaseCal))
	{
		/* Ops we had an init failure */
		flash_led(5,-1, 1);
	}

	measurement_interrupt_fired = false;
	resetVl53l0xInterrupt(pDevice, &status);

	uint32_t rangeTimeout = RANGE_TIMEOUT_VALUE;

	/* Enable PWM timer if PWM enabled */
	if ( gpio_mode == GPIO_PWM_ENABLED) TIMER_0_init();

	if (led_mode == LED_PWM_ENABLED ) TIMER_1_init();

	static uint8_t led_duty_cycle = 0;
    static uint8_t gpio_duty_cycle = 0;

	/* Setup the display. */
	backpack_begin(0xE0);

    /* Main code */
    while (1)
    {	
		if (measurement_interrupt_fired)
		{
			/* We can do a fair bit of work here once the ranging has complete, 
			* because the VL53L0X is now busy getting another range ready. */

			measurement_interrupt_fired = false;

			rangeTimeout = RANGE_TIMEOUT_VALUE;

			/* If we are in measurement output modes */
			if (led_mode == LED_MEASUREMENT_OUTPUT) LED_set_level(false);

			if (gpio_mode == GPIO_MEASUREMENT_INTERRUPT) SYNC_set_level(true);

			/* Read current mm */
			readRange(pDevice, &status, &measure);

			error_code = measure.RangeStatus;
			/* If not phase error */
			if (measure.RangeStatus != 4)
			{
				current_millimeters = measure.RangeMilliMeter;
				distance_error = (uint32_t)measure.Sigma >> 16;
			} else {
				current_millimeters = 0;
				distance_error = 0;
			}			

			/* 0 is invalid measurement */

			/* Do some sanity checking (if measurement greater than 4 meters) */
			if (current_millimeters >= MAX_DIST) current_millimeters = 0;

			/* Valid measurements come out of the sensor as values > 30mm. */
			if (current_millimeters <= MIN_DIST && current_millimeters > 0) current_millimeters = MIN_DIST;

			real_distance = current_millimeters;
			filtered_distance = real_distance;

			if (filtered_distance != 0)
			{
                
#ifndef DEV_DISABLE

				if (filtering_enabled)
					filtered_distance = bwlpf(filtered_distance, &low_pass_filter_state);

				/* Averaging happens after filtering */
				if (averaging_enabled)
				{
					//Push current measurement into ring buffer for filtering
					hb_push_back(&history_buffer, &filtered_distance);
					filtered_distance = avg(history_buffer.buffer,averaging_size);
				}
#endif
			}
			
			/* Output modes */
			if (led_mode == LED_THRESHOLD_ENABLED)
			{
				if (filtered_distance <= led_threshold && filtered_distance != 0)
				{
					/* Turn on LED */
					LED_set_level(false);
				}

				else
				{
					/* Turn off LED */
					LED_set_level(true);
				}
			}

			if (gpio_mode == GPIO_THRESHOLD_ENABLED)
			{
				if (filtered_distance <= gpio_threshold && filtered_distance != 0)
				{
					/* Turn on GPIO */
					SYNC_set_level(true);
					
				}

				else
				{
					/* Turn off GPIO */
					SYNC_set_level(false);
				}
			}

			/* If we are in measurement output modes */
			/* Note that in some modes this will output very quickly (around 60Hz)
				when there is no valid measurement */
			if (led_mode == LED_MEASUREMENT_OUTPUT) LED_set_level(true);

			if (gpio_mode == GPIO_MEASUREMENT_INTERRUPT) SYNC_set_level(false);

			/* Calculate "Software" (Timer Based) PWM */
			/* This is pretty lightweight, it just uses software to calc the duty cycle.
				* Timers then work to fire the pins. */

			if (measure.RangeStatus != 3 && measure.RangeStatus != 4) //If min range failure hasn't occurred
			{ 
			    backpack_print(filtered_distance, 10);
			    backpack_writeDisplay();

				if (filtered_distance > led_threshold) led_duty_cycle = 0;
				else if (filtered_distance == 0) led_duty_cycle = 0; //Stop "bounce" when invalid measurement
				else led_duty_cycle = (led_threshold - filtered_distance)*100/(led_threshold - MIN_DIST);

				//if (led_duty_cycle > 99) led_duty_cycle = 99; // Done in set_duty

				if (filtered_distance > gpio_threshold) gpio_duty_cycle = 0;
				else if (filtered_distance == 0) gpio_duty_cycle = 0; //Stop "bounce" when invalid measurement
				else gpio_duty_cycle = (gpio_threshold - filtered_distance)*100/(gpio_threshold - MIN_DIST);

				//if (gpio_duty_cycle > 99) gpio_duty_cycle = 99; // Done in set_duty

			} else {
				led_duty_cycle = 0;
				gpio_duty_cycle = 0;
					
			}
			TIMER_1_set_duty(led_duty_cycle);
			TIMER_0_set_duty(gpio_duty_cycle);
		} 

		else
		{
			rangeTimeout--;


			/* Reset interrupt if we have a communication timeout */
			if (rangeTimeout == 0)
			{
				rangeTimeout = RANGE_TIMEOUT_VALUE;
				resetVl53l0xInterrupt(pDevice, &status);
			}
		}
	}
}

/**
 * \brief _delay_ms helper function
 * 
 * \param ms
 * 
 * \return void
 */
void delay_ms(uint16_t ms)
{
	while (0 < ms)
	{
		_delay_ms(1);
		--ms;
	}
}


/**
 * \brief Flash LED
 * 
 * \param flash_ms between state chances, double this value for the period
 * \param num_of_flashes (-1 for indefinite)
 * 
 * \return void
 */
static void flash_led(uint32_t timeout_ms, int8_t num_of_flashes, bool pwm)
{
    /* Turn off LED */
    LED_set_level(true);

	delay_ms(timeout_ms);

	//Can only breath in indefinite mode
    if (pwm && num_of_flashes == -1)
	{
		uint8_t i = 0;
		TIMER_1_init();

		while (1)
		{	
			if (i >= 99) i = 0;
			TIMER_1_set_duty(i);
			delay_ms(timeout_ms);
			i++;
		}
		
	}
	while (abs(num_of_flashes) > 0)
    {
	    delay_ms(timeout_ms);

		/* Turn on LED */
		LED_set_level(false);

		delay_ms(timeout_ms);

		/* Turn off LED */
		LED_set_level(true);

		/* Never decrement if -1 */
		if (num_of_flashes > 0) num_of_flashes--;
    }
}

/**
 * \brief Translates the human readable measurement mode to API mode.
 * 
 * \param measurement_mode
 * 
 * \return uint8_t
 */
static uint8_t translate_measurement_mode(uint8_t measurement_mode)
{
    if (measurement_mode == HIGHLY_ACCURATE) return 0;

    if (measurement_mode == LONG_RANGE) return 1;

    if (measurement_mode == HIGH_SPEED) return 2;

	//This gets set by default in this method. So don't bother checking.
    //if (measurement_mode == VL53L0X_DEFAULT) return 3;

	//We auto switch this range
    //if (measurement_mode == MAPPYDOT_MODE) return 3;

	return 3;
}

/**
 * \brief Translates the ranging mode for API mode
 * 
 * \param ranging_mode
 * 
 * \return uint8_t
 */
static uint8_t translate_ranging_mode(uint8_t ranging_mode)
{
    if (ranging_mode == SET_CONTINUOUS_RANGING_MODE) return 1;

	//if (ranging_mode == SET_SINGLE_RANGING_MODE) //Set by default
	return 0;
}


/**
 * \brief Average function
 * 
 * \param array
 * \param count
 * 
 * \return uint16_t
 */
uint16_t avg(uint16_t * array, uint8_t count)
{
	uint16_t sum = 0;

	for(uint8_t i = 0; i < count; i++)
	{
		sum += array[i];
	}

	return sum / count;
}


/* Measurement Interrupt Fired */
ISR (PCINT1_vect)
{
    if(GPIO1_get_level() == 0)
    {
        measurement_interrupt_fired = true;
    }
}

/* GPIO PWM Timer (TIMER_0) */
/* "Start" of the PWM signal. While COMPA resets after interrupt,
 * we can never get full off with this because there is always time.
 * until the next interrupt fire */
ISR(TIMER1_COMPA_vect)

{
    SYNC_set_level(true);
}

/* Duty Cycle point */
ISR(TIMER1_COMPB_vect)
{
    SYNC_set_level(false); 
}

/* LED PWM Timer (TIMER_1) */
ISR(TIMER3_COMPA_vect)
{
    /* Turn on LED */
    LED_set_level(false);

}

/* Duty Cycle point */
ISR(TIMER3_COMPB_vect)
{
    /* Turn off LED */
    LED_set_level(true);

}

/* Debugging purposes */
ISR(BADISR_vect)
{
    /* Turn on LED */
	//LED_set_level(true);
	asm("nop;");
}