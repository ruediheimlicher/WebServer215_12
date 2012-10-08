/*************************************************************************
* Title:    I2C master library using hardware TWI interface
* Author:   Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
* File:     $Id: twimaster.c,v 1.3 2005/07/02 11:14:21 Peter Exp $
* Software: AVR-GCC 3.4.3 / avr-libc 1.2.3
* Target:   any AVR device with hardware TWI 
* Usage:    API compatible with I2C Software Library i2cmaster.h
**************************************************************************/
#include <inttypes.h>
#include <compat/twi.h>
#include <avr/delay.h>
#include "i2cmaster.h"


/* define CPU frequency in Mhz here if not defined in Makefile */

#ifndef F_CPU
#define F_CPU 12000000UL
#endif

/* I2C clock in Hz */
#define SCL_CLOCK  100000L

uint8_t MAX_TRIES = 100;
volatile uint8_t	TWI_Flag=0;

//#define WHILEMAX 0x12FF // Wartezeit in while-Schleife : 5 ms

#define WHILEMAX 0xFFFF // Wartezeit in while-Schleife : 5 ms

#define STARTERR			0xB0
#define REPSTARTERR		0xB1
#define ADDRESSERR		0xB2
#define MAX_TRIESERR		0xB3
#define STOPERR			0xB4
#define WRITEERR			0xB5
#define READ_ACKERR		0xB6
#define READ_NACKERR		0xB7

extern void lcd_puts(const char *s);
extern void lcd_gotoxy(uint8_t x, uint8_t y);
extern void err_clr_line(uint8_t Linie);
extern void err_gotoxy(uint8_t x, uint8_t y);
extern void err_putc(const char c);
extern void err_puts(const char *s);
extern void delay_ms(uint16_t);
extern void err_puthex(uint8_t zahl);
void err_clr_part(uint8_t Linie, uint8_t von, uint8_t bis);

/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void i2c_init(void)
{
  /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
  
  // Fuer Atmega8 eventuell Prescaler 4* und TWBR Groesser
	TWSR |= (1<<0);                         /* prescaler 4*/
	TWSR |= (1<<1);                         /* prescaler 16*/
	//	beide Bits: prescaler 64

	
	//  TWSR = 0;								/* no prescaler */
	//	TWBR = ((F_CPU/SCL_CLOCK)-16)/2;		/* must be > 10 for stable operation */
	TWBR=0xFF;
}/* i2c_init */


/*************************************************************************	
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char i2c_start(unsigned char address)
{
    uint8_t   twst;
	uint8_t counter=0;
	uint16_t whilecounter = WHILEMAX; // 5 ms
//	lcd_cls();
//	lcd_puts("i2c_start\0");
//	delay_ms(400);


	restart:
	if (counter++ >= MAX_TRIES)
	{	
		//delay_ms(10);
		//err_clr_part(1,9,19);
		//err_puts(" max tr\0");
		
		//delay_ms(200);
		TWI_Flag=MAX_TRIESERR;
		return MAX_TRIESERR;
	}

	begin:

//  TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE)

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	//return 0;
//	lcd_gotoxy(0,0);
//	lcd_puts("i2c_start1\0");
	
	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)) && whilecounter)
	{
	whilecounter--;
	}
	
	if (whilecounter==0) // kein Erfolg
	{
		TWI_Flag=STARTERR;
		return STARTERR;
	}

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
//	lcd_gotoxy(0,0);
//	lcd_puts("i2c_start2 \0");
//	lcd_puthex(twst);
//	lcd_putc('*');
//	delay_ms(20);
	if (twst == TW_MT_ARB_LOST)
	{
		lcd_gotoxy(10,1);
		lcd_puts("> begin\0");
		goto begin;
	 }
	if ( (twst != TW_REP_START) && (twst != TW_START)) 
	{
		lcd_gotoxy(10,1);
		lcd_puts("return 2 \0");

		return REPSTARTERR;
	}

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait until transmission completed and ACK/NACK has been received
//	delay_ms(20);
	whilecounter=WHILEMAX; 
	while(!(TWCR & (1<<TWINT)) && whilecounter) 
	{
	whilecounter--;
	}
	
	if (whilecounter==0)	// Address-Fehler
	{
		lcd_gotoxy(10,0);
		lcd_puts("adr err \0");
		TWI_Flag=ADDRESSERR;
		return ADDRESSERR;
	}
	
//	lcd_puts("wh \0");

	twst = TW_STATUS & 0xF8;
	
	if (twst == TW_MT_SLA_NACK) goto restart;
	if (twst == TW_MT_ARB_LOST) goto begin;
	if ( (twst != TW_MT_SLA_ACK ) && (twst != TW_MR_SLA_ACK)) goto error;

	

	return 0;

//quit:
	// send stop condition
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	return 0;

error:
	// send stop condition
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	return 3;
}/* i2c_start */


/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready
 
 Input:   address and transfer direction of I2C device
*************************************************************************/
void i2c_start_wait(unsigned char address)
{
    uint8_t   twst;


    while ( 1 )
    {
	    // send START condition
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    	// wait until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
    
    	// send device address
    	TWDR = address;
    	TWCR = (1<<TWINT) | (1<<TWEN);
    
    	// wail until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ) 
    	{    	    
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	        
	        // wait until stop condition is executed and bus released
	        while(TWCR & (1<<TWSTO));
	        
    	    continue;
    	}
    	//if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     }

}/* i2c_start_wait */


/*************************************************************************
 Issues a repeated start condition and sends address and transfer direction 

 Input:   address and transfer direction of I2C device
 
 Return:  0 device accessible
          1 failed to access device
*************************************************************************/
unsigned char i2c_rep_start(unsigned char address)
{
    return i2c_start( address );

}/* i2c_rep_start */


/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void i2c_stop(void)
{
	uint16_t whilecounter=WHILEMAX; // 5 ms
    /* send stop condition */
//	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
//	29.10.08
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO)|(1<<TWEA)|(1<<TWIE);
	// wait until stop condition is executed and bus released
	
	while((TWCR & (1<<TWSTO))&& whilecounter)
	{
	whilecounter--;
	}

	if (whilecounter==0)	// Address-Fehler
	{
	TWI_Flag=STOPERR;
	//return STOPERR;
	}

}/* i2c_stop */


/*************************************************************************
  Send one byte to I2C device
  
  Input:    byte to be transfered
  Return:   0 write successful 
            1 write failed
*************************************************************************/
unsigned char i2c_write( unsigned char data )
{	
    uint8_t   twst;
    uint16_t whilecounter=WHILEMAX; // 5 ms
	
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)) && whilecounter)
	{
	whilecounter--;
	}
	
	if (whilecounter==0)	// Address-Fehler
	{
		TWI_Flag=WRITEERR;
		return WRITEERR;
	}

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;

	if( twst != TW_MT_DATA_ACK) return twst;
	return 0;

}/* i2c_write */


/*************************************************************************
 Read one byte from the I2C device, request more data from device 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readAck(void)
{
	uint16_t whilecounter=WHILEMAX; // 5 ms
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)) && whilecounter)
	{
		whilecounter--;
	 } 
	if (whilecounter==0)	// Read-Ack-Fehler
	{
		TWI_Flag=READ_ACKERR;
		
	}

    return TWDR;

}/* i2c_readAck */


/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readNak(void)
{
	uint16_t whilecounter=WHILEMAX; // 5 ms
		//lcd_gotoxy(16,0);
		//lcd_puts("NACK\0");
		//delay_ms(100);

	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)) && whilecounter)
	{
		whilecounter--;
	}

	if (whilecounter==0)	// Read-Nack-Fehler
	{
		TWI_Flag=READ_ACKERR;
		
	}
	
    return TWDR;

}/* i2c_readNak */
