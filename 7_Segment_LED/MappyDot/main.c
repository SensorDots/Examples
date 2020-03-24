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
   Compiled size needs to be under 7C00 bytes to work with bootloader.

   NO_INT compile flag is used to test the VL53L0X without the interupt
   pin. This should not be used under normal circumstances. This is for
   debugging purposes only.

 */

#include <atmel_start.h>
#include <avr/wdt.h>
#include <string.h>
#include "vl53l1x.h"
#include "addr.h"
#include "history_buffer.h"
#include "bwlpf.h"
#include "mappydot_reg.h"
#include "tc16.h"
#include "nvmctrl.h"
#include "crc8.h"
#include "stackmon.h"
#include "vl53l1_types.h"
#include "main.h"
#include "sleeping.h"
#include "helper.h" //Main helper function
#include "led_backpack.h"


#ifdef DEV_DISABLE
#warning Some functions are disabled with DEV_DISABLE
#endif

#define VERSION_STRING                "MDPFW_V1.2"

#define EEPROM_ADDRESS_BYTE           0x01
#define EEPROM_BOOTLOADER_BYTE        0x02
#define EEPROM_DEVICE_NAME            0x20
#define EEPROM_FACTORY_SETTINGS_START 0x40
#define EEPROM_USER_SETTINGS_START    0x60
#define EEPROM_FACTORY_CALIB_START    0x80 //Size 95 bytes
#define EEPROM_USER_CALIB_START       0xF0

#define FILTER_FREQUENCY              6  //Hz
#define SETTINGS_SIZE                 21
#define CALIB_SIZE                    95
#define MIN_DIST                      30 //minimum distance from VL53L1X
#define MAX_DIST                      5000 //Max sanity distance from VL53L1X
#define ACCURACY_LIMIT                1200
//#define T_BOOT_MS                     2



/* State Variables */
#ifndef DEV_DISABLE
circular_history_buffer history_buffer;
#endif
int16_t slave_address;
filter_state low_pass_filter_state;

/* We do this so the struct is allocated as pointers are not allocated */
VL53L1_Dev_t device;
VL53L1_Dev_t * pDevice;
VL53L1_RangingMeasurementData_t measure;
VL53L1_Error status;
VL53L1_UserRoi_t ROI;

#define HISTORY_SIZE 6

uint16_t filtered_distance         = 0;
uint16_t real_distance             = 0;
uint8_t error_code                 = 0;
uint16_t distance_error            = 0;
uint16_t current_millimeters       = 0;
uint16_t signal_rate_rtn_mega_cps  = 0;
uint16_t ambient_rate_rtn_mega_cps = 0;

bool filtering_enabled  = 1;
bool averaging_enabled  = 1;
bool factory_mode       = 0;
bool crosstalk_enabled  = 0;
bool vl53l1x_powerstate = 0;
bool is_master          = 0;

uint8_t led_mode                      = LED_PWM_ENABLED;
uint8_t gpio_mode                     = GPIO_LOW;
uint8_t current_ranging_mode          = SET_CONTINUOUS_RANGING_MODE; //SET_SINGLE_RANGING_MODE; 
uint8_t current_measurement_mode      = MED_RANGE;
uint32_t led_threshold                = 600;
uint32_t gpio_threshold               = 300;
uint8_t averaging_size                = HISTORY_SIZE;
uint8_t current_average_size          = 0;
uint8_t intersensor_crosstalk_delay   = 0;
uint8_t intersensor_crosstalk_timeout = 40; //40*ticks
uint16_t measurement_budget           = 50; //ms
uint8_t read_interrupt                = 0;
uint16_t signal_limit_check           = 1000;
uint8_t sigma_limit_check             = 15;
uint8_t intersensor_sync              = 0;

VL53L1_CalibrationData_t calibration_data;
bool got_calibration_data = false;

int8_t led_pulse           = 0;
uint8_t led_pulse_dir      = 0;
uint16_t led_pulse_timeout = 0;
uint8_t ignore_next_filter = 0;

int16_t temp_buffer[HISTORY_SIZE];



#define SETTLE_MEASUREMENTS 2
int8_t run_until_settle    = SETTLE_MEASUREMENTS;


/* Command handler */
bool command_to_handle;
uint8_t todo_command;
uint8_t todo_arg[16];
uint8_t todo_arg_length;

static volatile bool measurement_interrupt_fired = false;
static volatile bool interrupt_timeout_interrupt_fired = false;
static volatile bool crosstalk_sync_interrupt_fired = false;
static volatile bool calibrating = false;
uint8_t mappydot_name[16];
uint8_t settings_buffer[SETTINGS_SIZE + 2]; //(+2 is for code reuse)

int main(void)
{	   
    /* Initializes MCU, drivers and middleware */
    atmel_start_init();

	/* Disables analog comparator to save power */
    disable_analog();

    /* Set XSHUT to high to turn on (Shutdown is active low). */
	//This is now done in start_init
    //XSHUT_set_level(true);
	//_delay_ms(T_BOOT_MS); //no longer required

    /* Set sync to input and no pull mode as if it's connected to MST. */
    //This is now done in start_init
    //SYNC_set_dir(PORT_DIR_IN);
    //SYNC_set_pull_mode(PORT_PULL_OFF);

	/* Setup Crosstalk reduction interrupt */
	/* PB6 - PCINT6 (PCMSK1) */
	PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
	PCMSK0 |= (1 << PCINT6);  // set PCINT6 to trigger an interrupt on state change

	/* Ranging Interrupt setup */
	/* Interrupt is active low when fired (after ranging complete), rather than high. */
	/* Set GPIO1 (interrupt) pin to input */
	GPIO1_set_dir(PORT_DIR_IN);
	GPIO1_set_pull_mode(PORT_PULL_OFF);

	/* Setup measurement interrupt */
	/* PC3 - PCINT11 (PCMSK1) */
	PCICR |= (1 << PCIE1);    // set PCIE1 to enable PCMSK1 scan
	PCMSK1 |= (1 << PCINT11);  // set PCINT11 to trigger an interrupt on state change

	/* Init EEPROM */
    //FLASH_0_init();

	/* Enable interrupts */
    sei();

	/* Read calibration data */
	/*uint8_t * calib_ptr = (uint8_t *)&calibration_data;

	FLASH_0_read_eeprom_block(EEPROM_USER_CALIB_START, calib_ptr, CALIB_SIZE);
	uint8_t crc_calib = Crc8(calib_ptr, CALIB_SIZE);*/

	/* Sanity checks CRC when EEPROM is all zeros */
	/*if (crc_calib == 0x00 && calibration_data.customer.global_config__spad_enables_ref_0 == 0x00
		&& calibration_data.customer.global_config__spad_enables_ref_1 == 0x00
		&& calibration_data.customer.global_config__spad_enables_ref_2 == 0x00
		&& calibration_data.customer.global_config__spad_enables_ref_3 == 0x00) crc_calib = 0x01;*/

	got_calibration_data = false;
	

	#ifdef FILL_SRAM_DEBUG
	//int16_t count = StackCount();
	#endif

	/* VL53L1X Init */

	/* Assign struct to pointer */
	pDevice = &device;

	/* Run init in continous mode (0) until measurements settle */
	if (!init_ranging(pDevice, &status, 0, current_measurement_mode, measurement_budget, &ROI,
					  &calibration_data, got_calibration_data))
	{
		/* Ops we had an init failure */
		flash_led(5,-1, 1);
	}

	#ifndef DEV_DISABLE

	/* Initialise history buffer */
	hb_init(&history_buffer,averaging_size,sizeof(uint16_t));

	/* Initialise Filter */
	bwlpf_init(&low_pass_filter_state,1000/measurement_budget,FILTER_FREQUENCY);

	#endif

	measurement_interrupt_fired = false;

	/* Enable PWM timer if PWM enabled */

	if (led_mode == LED_PWM_ENABLED ) TIMER_1_init();

	static uint8_t led_duty_cycle = 0;

	run_until_settle    = SETTLE_MEASUREMENTS;

	resetVL53L1Interrupt(pDevice, &status);

	#ifdef NO_INT
	uint32_t no_interrupt_counter = 0;
	#else
	/* Start no interrupt timer */
	//if (current_ranging_mode == SET_CONTINUOUS_RANGING_MODE) TIMER_2_init(); //Now set in vl53l1x.c
	#endif

	/* Setup the display. */
	backpack_begin(0xE0);

	int16_t prev_measurement = 0;


    /* Main code */
    while (1)
    {
	    #ifdef NO_INT
	    no_interrupt_counter++;
		#endif
	    if (!calibrating)
		{	
		    /* Put the microcontroller to sleep
			   Everything after this is affected by interrupts */
		    #ifndef DEBUG
				sleep_avr();
			#endif

			#ifdef NO_INT
			#warning NO_INT set.
			if (no_interrupt_counter > 38) //runs at about 52Hz, this doesn't have to be accurate.
			{
				no_interrupt_counter = 0;
			#else 
			if (measurement_interrupt_fired)
			{
			#endif
				/* We can do a fair bit of work here once the ranging has complete, 
				* because the VL53L1X is now busy getting another range ready. */

				/* Reset the no interrupt timer */
				if (current_ranging_mode == SET_CONTINUOUS_RANGING_MODE) TIMER_2_reset();

				measurement_interrupt_fired = false;

				if (crosstalk_enabled || (intersensor_sync && is_master && current_ranging_mode == SET_CONTINUOUS_RANGING_MODE))
				{
					/* Create trigger after measurement finished.
					   For sync, we assume a new measurement is already underway on master */
					ADDR_OUT_set_level(false);
				}     

				/* If we are in measurement output modes */
				if (led_mode == LED_MEASUREMENT_OUTPUT && !factory_mode) LED_set_level(false);

				if (gpio_mode == GPIO_MEASUREMENT_INTERRUPT && !factory_mode) SYNC_set_level(true);

				/* Read current mm */
				readRange(pDevice, &status, &measure);   

				if (intersensor_sync || crosstalk_enabled)
				{
					if (is_master && intersensor_sync)
					{
						if (current_ranging_mode == SET_CONTINUOUS_RANGING_MODE) resetVL53L1Interrupt(pDevice, &status);
						else if (current_ranging_mode == SET_SINGLE_RANGING_MODE) stopContinuous(pDevice, &status);
					}
					// else do nothing

				} else {
					if (run_until_settle < 0)
					{
						if (current_ranging_mode == SET_CONTINUOUS_RANGING_MODE) resetVL53L1Interrupt(pDevice, &status);
						else if (current_ranging_mode == SET_SINGLE_RANGING_MODE) stopContinuous(pDevice, &status);
						} else {
						resetVL53L1Interrupt(pDevice, &status);
					}
				}


				error_code = measure.RangeStatus;
				
				/* If phase error */
				if (measure.RangeStatus == 4)
				{
				    //current_millimeters = 0;
				} else {
				    prev_measurement = current_millimeters;

					current_millimeters = measure.RangeMilliMeter;
					if (current_millimeters == 0) current_millimeters = prev_measurement;
				}			

				distance_error = measure.SigmaMilliMeter >> 16; 
				signal_rate_rtn_mega_cps = measure.SignalRateRtnMegaCps >> 16;
				ambient_rate_rtn_mega_cps = measure.AmbientRateRtnMegaCps >> 16;

				/* 0 is invalid measurement */

				/* Do some sanity checking */
				if (current_millimeters >= MAX_DIST) current_millimeters = prev_measurement; //0;

				if (distance_error >= 10) current_millimeters = prev_measurement;

				/* Valid measurements come out of the sensor as values > 30mm. */
				if (current_millimeters <= MIN_DIST && current_millimeters > 0) current_millimeters = MIN_DIST;

				real_distance = current_millimeters;
				filtered_distance = real_distance;

#ifndef DEV_DISABLE
				/* Push current measurement into ring buffer for averaging */
				hb_push_back(&history_buffer, &filtered_distance);

				if (filtered_distance != 0)
				{
					if (filtering_enabled)
					{
						filtered_distance = bwlpf(filtered_distance, &low_pass_filter_state);

						/* Ignore filtered distance if over max, this can happen when there is a rapid change 
						   from no measurement to large value. This is because the filter does take time to settle
						   but during this time the error code will generally be 7 */
						if (ignore_next_filter > 0)
						{
							filtered_distance = real_distance;
							ignore_next_filter--;
						}

						if ((filtered_distance > MAX_DIST || distance_error >= 7) && !ignore_next_filter)
						{
							filtered_distance = real_distance;
							ignore_next_filter = FILTER_ORDER * 2;
						}
					}

					/* Averaging happens after filtering */
					if (averaging_enabled)
					{
						/* Wait for history buffer to fill */
						if (current_average_size < averaging_size)
						{
							filtered_distance = real_distance;
							current_average_size++;
						}
						else
						{
							filtered_distance = avg(history_buffer.buffer,averaging_size);
							//memcpy(temp_buffer, (int16_t *)history_buffer.buffer, history_buffer.count);
							//filtered_distance = quick_select_median(temp_buffer, history_buffer.count);
						}
					}

				}
	#endif
				read_interrupt = 1;

				/* We run continuous measurements until the device settles after init. When in single shot mode
				   on startup, the interrupts don't fire after first few tries due to VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL */
				if (run_until_settle == 0)
				{
					setRangingMode(pDevice, &status, translate_ranging_mode(current_ranging_mode), current_measurement_mode, measurement_budget);
					run_until_settle--;
				}
				else if (run_until_settle > 0)
				{
					run_until_settle--;
				}
			
				/* Output modes */
				if (!factory_mode) 
				{
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

				

					} else {
						//led_duty_cycle = 0;
					
					}
					TIMER_1_set_duty(led_duty_cycle);

				} 
			}

			else if (!crosstalk_enabled && interrupt_timeout_interrupt_fired)
			{
				interrupt_timeout_interrupt_fired = 0;

				//Reset pullup on interrupt pin.
				GPIO1_set_pull_mode(PORT_PULL_UP);

				/* Reset interrupt if we have a communication timeout */
				resetVL53L1Interrupt(pDevice, &status);
				
			}
		}
	}
}



/**
 * \brief Resets the VL53L0X ranging system. We do this here, because we control the XSHUT pin
 * 
 * 
 * \return void
 */
static void reset_vl53l1x_ranging()
{
	stopContinuous(pDevice, &status);

	/* Set XSHUT to low to turn off (Shutdown is active low). */
	XSHUT_set_level(false);

	/* Wait for shutdown */
	delay_ms(100);

	/* Set XSHUT to high to turn on (Shutdown is active low). */
	XSHUT_set_level(true);

	/* Boot time */
	//delay_ms(T_BOOT_MS);
	waitDeviceReady(pDevice,&status);

	run_until_settle    = SETTLE_MEASUREMENTS;

    init_ranging(pDevice, &status, 0, current_measurement_mode, measurement_budget, &ROI,
			        &calibration_data, got_calibration_data);

}


#ifndef NO_INT
/* Measurement Interrupt Fired */
ISR (PCINT1_vect)
{
    if(GPIO1_get_level() == 0)
    {
        measurement_interrupt_fired = true;
    }
}
#endif

/* Crosstalk/sync interrupt fired */
ISR (PCINT0_vect)
{

	/* Pass on pulse straight away to next in chain */
	if (intersensor_sync)
	{
		ADDR_OUT_set_level(ADDR_IN_get_level());
	}

	if(ADDR_IN_get_level() == 0)
	{
		crosstalk_sync_interrupt_fired = true;
	}
}


/* No interrupt overflow (this will fire every ~2000ms if no interrupt arrived) */
ISR(TIMER4_OVF_vect)
{
    interrupt_timeout_interrupt_fired = true;
}

