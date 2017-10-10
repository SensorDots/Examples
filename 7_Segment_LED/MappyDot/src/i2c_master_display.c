//Based on https://github.com/g4lvanix/I2C-master-lib

#ifndef  F_CPU
#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <util/twi.h>
#include "twi.h"

#include "i2c_master_display.h"

#define F_SCL 400000UL // SCL frequency
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)

void i2c_init_display(void)
{
    I2C_1_init();
}

uint8_t i2c_start_display(uint8_t address)
{
    // reset TWI control register
    TWCR1 = 0;
    // transmit START condition
    TWCR1 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

    // wait for end of transmission
    while( !(TWCR1 & (1<<TWINT)) );

    // check if the start condition was successfully transmitted
    if((TWSR1 & 0xF8) != TW_START)
    {
        return 1;
    }

    // load slave address into data register
    TWDR1 = address;
    // start transmission of address
    TWCR1 = (1<<TWINT) | (1<<TWEN);

	uint32_t timeout = 0;
    // wait for end of transmission
    while( !(TWCR1 & (1<<TWINT)) && timeout < 100000) timeout++;

    // check if the device has acknowledged the READ / WRITE mode
    uint8_t twst = TW_STATUS1 & 0xF8;

    if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

    return 0;
}

uint8_t i2c_write_display(uint8_t data)
{
	uint32_t timeout = 0;
    // load data into data register
    TWDR1 = data;
    // start transmission of data
    TWCR1 = (1<<TWINT) | (1<<TWEN);

    // wait for end of transmission
    while( !(TWCR1 & (1<<TWINT)) && timeout < 100000) timeout++;

    if( (TWSR1 & 0xF8) != TW_MT_DATA_ACK )
    {
        return 1;
    }

    return 0;
}

uint8_t i2c_read_ack_display(void)
{
	uint32_t timeout = 0;
    // start TWI module and acknowledge data after reception
    TWCR1 = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);

    // wait for end of transmission
    while( !(TWCR1 & (1<<TWINT)) && timeout < 100000) timeout++;

    // return received data from TWDR1
    return TWDR1;
}

uint8_t i2c_read_nack_display(void)
{
    uint32_t timeout = 0;
    // start receiving without acknowledging reception
    TWCR1 = (1<<TWINT) | (1<<TWEN);

    // wait for end of transmission
    while( !(TWCR1 & (1<<TWINT)) && timeout < 100000) timeout++;

    // return received data from TWDR1
    return TWDR1;
}

uint8_t i2c_transmit_display(uint8_t address, uint8_t* data, uint16_t length)
{
    if (i2c_start_display(address | I2C_WRITE)) return 1;

    for (uint16_t i = 0; i < length; i++)
    {
        if (i2c_write_display(data[i])) return 1;
    }

    i2c_stop_display();
    return 0;
}

uint8_t i2c_receive_display(uint8_t address, uint8_t* data, uint16_t length)
{
    if (i2c_start_display(address | I2C_READ)) return 1;

    for (uint16_t i = 0; i < (length-1); i++)
    {
        data[i] = i2c_read_ack_display();
    }

    data[(length-1)] = i2c_read_nack_display();
    i2c_stop_display();
    return 0;
}

uint8_t i2c_writeReg_display(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length)
{
    if (i2c_start_display(devaddr | 0x00)) return 1;

    i2c_write_display(regaddr);

    for (uint16_t i = 0; i < length; i++)
    {
        if (i2c_write_display(data[i])) return 1;
    }

    i2c_stop_display();
    return 0;
}

uint8_t i2c_readReg_display(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length)
{
    if (i2c_start_display(devaddr)) return 1;

    i2c_write_display(regaddr);

    if (i2c_start_display(devaddr | 0x01)) return 1;

    for (uint16_t i = 0; i < (length-1); i++)
    {
        data[i] = i2c_read_ack_display();
    }

    data[(length-1)] = i2c_read_nack_display();
    i2c_stop_display();
    return 0;
}

void i2c_stop_display(void)
{
    // transmit STOP condition
    TWCR1 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}