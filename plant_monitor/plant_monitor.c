/***************************************************************************************************
 Title		:   Plant monitor
 Description:   A system monitoring soil moisture and lux receiving of a plant object. Automatically
				waters the object when it's dry. The dry value is adaptive.
 Author		:   Vasileios Bimpikas <v.bimpikas@gmail.com>
 Files		:	plant_monitor.c (this file)
				lcd.h
				lcd.c
 Date		:	March 14th, 2015
 Location	:	Uppsala, SE
 Purpose	:	Project for Microcontroller Programming 1TE663 at Uppsala Universitet, Uppsala, SE
 Hardware	:	- any AVR device, tested using ATmega328P
				- LCD 16x2
				- Soil moisture sensor
				- Photo resistor in voltage divider setup
				- Peristaltic pump connected to I/O pin using an NPN transistor and a relay for
				  switching
****************************************************************************************************
Setup:

					  Reset PC6|1   28|PC5
					  Pump  PD0|2   27|PC4		 Lux reading
							PD1|3   26|PC3       LCD_RW ( 6)
							PD2|4   25|PC2       LCD_E  ( 5)
							PD3|5   24|PC1       LCD_RS ( 3)
							PD4|6   23|PC0		 Soil reading
						Vcc	   |7   22|		GND
						GND	   |8   21|		Aref
							PB6|9   20|		AVcc
							PB7|10  19|PB5 SCK   LCD_D7 (13)
							PD5|11  18|PB4 MISO  LCD_D6 (14)
							PD6|12  17|PB3 MOSI  LCD_D5 (11)
							PD7|13  16|PB2       LCD_D4 (12)
							PB0|14  15|PB1       switch (17)
***************************************************************************************************/

#define F_CPU 1000000UL  // 1 MHz clock

// I/O pins
#define PC0			0
#define PC4			4

// Includes
#include <avr/io.h>
#include <stdlib.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "lcd.h"
#include <avr/eeprom.h>	// store the reference value of the soil moist
#include <math.h>		// convert the ADC values to lux

// Moist soil value
#define WATERED 700	// at this point the ADC reading corresponds to a voltage high enough for the soil to be considered watered

// Macros
#define CLEAR_DISP lcd_clrscr(); scr_FLG = 1;
#define EMPTY dry == 65535 || dry == 0
#define OPEN PORTD = 0b00000001;
#define CLOSE PORTD = 0b00000000;

// Timing values
#define SAMPLES 10
#define INTERVAL 10
#define DEBOUNCE 10

// convert to lux constants
#define a 0.0182
#define b 18.365
#define c 6334.6

// Global variables
char text[20];
int soil;
uint16_t dry;

// Flags
uint8_t scr_FLG = 1;
uint8_t water_FLG = 1;

// Function prototypes
uint16_t adc_read(uint8_t adcx);
void mem_check(void);
void mem_set(void);
uint8_t S1(void);
void sensors_disp(void);
void watering(void);

void init (void)         // collect hardware initializations here
{
	ADCSRA |= _BV(ADEN);	// enable the ADC
	PORTB = 0b00000010;    // pull-up on PB1
	DDRD = 0b00000001;		// PD0 output
	CLOSE					// immediately set the pulp pin LOW
	lcd_init(LCD_DISP_ON); // initialize LCD
	lcd_clrscr();		// clear the display

	// Check the stored dry point
	mem_check(); // get dry value from EEPROM
	if(EMPTY)
	{
		lcd_puts("EEPROM empty");
		_delay_ms(3000);
		mem_set();
	}
	else
	{
		CLEAR_DISP
		lcd_puts("Dry:");
		lcd_gotoxy(6,0);
		lcd_puts("         ");
		lcd_gotoxy(6,0);
		lcd_puts(utoa(dry, text, 10));
		_delay_ms(3000);
	}
}

int main (void)
{
	// Variables
	
	init();
	
	// Infinite loop
	while (1)
	{
		sensors_disp();
		if(soil<dry)	// if the soil value indicates that the plant is dried out
		{
			while(soil<WATERED)
			{
				OPEN	// pulp is powered on
				watering();	// continue updating and displaying the values
			}
			CLOSE	// when the soil reaches the WATERED value, power off the pulp
			water_FLG = 1;
		}
		
		
		if(S1())
			mem_set();

	}
}

void sensors_disp(void)
{
	int sun, i, sum;
	
	// Static part of this screen
	if(scr_FLG)	// Update this part only when the screen flag has been raised
	{
		lcd_clrscr();
		lcd_puts("Soil: ");
		lcd_gotoxy(0,1);
		lcd_puts("Sun: ");
		lcd_gotoxy(0,0);
		lcd_gotoxy(13,1);
		lcd_puts("lux");
		scr_FLG = 0;	// clear the screen flag
	}


	// Dynamic part of this screen

	sum = 0;
	for(i=0; i<SAMPLES; i++)
	{
		sum += adc_read(PC0);
		_delay_ms(INTERVAL);
	}
	soil = sum/i;
	lcd_gotoxy(6,0);
	lcd_puts("         ");
	lcd_gotoxy(6,0);
	lcd_puts(utoa(soil, text, 10));

	sum = 0;
	for(i=0; i<SAMPLES; i++)
	{
		sum += adc_read(PC4);
		_delay_ms(INTERVAL);
	}
	sun = sum/i;
	
	// convert to lux test
	float conv = 13311*pow(M_E, -0.007*sun); // y = 13311e-0.007x
	lcd_gotoxy(6,1);
	lcd_puts("       ");
	lcd_gotoxy(6,1);
	lcd_puts(utoa(conv, text, 7));
}

void mem_check(void)		// check if a reference value is stored in the EEPROM
{
	int eep_word_addr = 0;
	dry = eeprom_read_word((uint16_t*)eep_word_addr++);
}

void mem_set(void)	// set a reference value in the EEPROM
{
	int i, count = 5, n = 100;
	uint16_t val = 0;
	int eep_word_addr = 0;
	long sum = 0;
	
	CLEAR_DISP
	lcd_puts("Insert probe and");
	lcd_gotoxy(0,1);
	lcd_puts("press S1");
	_delay_ms(1000);
	
	while(!S1());

	CLEAR_DISP
	lcd_puts("Setting new dry");
	lcd_gotoxy(0,1);
	lcd_puts("point in: 5     ");
	for(i=0; i<500; i++)
	{
		sum += adc_read(PC0);
		if(i == n)
		{
			n += 100;
			count--;
			lcd_gotoxy(10,1);
			lcd_puts("     ");
			lcd_gotoxy(10,1);
			lcd_puts(utoa(count, text, 5));
		}
		_delay_ms(10);
	}
	
	val = sum/500;
	dry = (uint16_t)val;
	eeprom_write_word((uint16_t*)eep_word_addr++, dry);
	
	CLEAR_DISP
	lcd_puts("New dry point:");
	lcd_gotoxy(0,1);
	lcd_gotoxy(6,1);
	lcd_puts(utoa(val, text, 10));
	_delay_ms(3000);
}

void watering(void)
{
	int sum = 0, i;
	
	if (water_FLG)
	{
		CLEAR_DISP
		lcd_puts("**Watering now**");
		lcd_gotoxy(0,1);
		lcd_puts("Soil:");
		water_FLG = 0;
	}
	
	for(i=0; i<SAMPLES; i++)
	{
		sum += adc_read(PC0);
		_delay_ms(1);
	}
	soil = sum/i;
	lcd_gotoxy(6,1);
	lcd_puts("         ");
	lcd_gotoxy(6,1);
	lcd_puts(utoa(soil, text, 10));
}

uint8_t S1(void)
{
	uint8_t retval = 0;
	if (!(PINB & 0b00000010))          // button pressed?
	{
		retval = 1;
		while(!(PINB & 0b00000010))
			_delay_ms(DEBOUNCE);
	}
	
	return retval;
}


uint16_t adc_read(uint8_t adcx)
{
	/* adcx is the analog pin we want to use.  ADMUX's first few bits are
	 * the binary representations of the numbers of the pins so we can
	 * just 'OR' the pin's number with ADMUX to select that pin.
	 * We first zero the four bits by setting ADMUX equal to its higher
	 * four bits. */
	ADMUX	&=	0xf0;
	ADMUX	|=	adcx;
 
	/* This starts the conversion. */
	ADCSRA |= _BV(ADSC);
 
	/* This is an idle loop that just wait around until the conversion
	 * is finished.  It constantly checks ADCSRA's ADSC bit, which we just
	 * set above, to see if it is still set.  This bit is automatically
	 * reset (zeroed) when the conversion is ready so if we do this in
	 * a loop the loop will just go until the conversion is ready. */
	while ( (ADCSRA & _BV(ADSC)) );
 
	/* Finally, we return the converted value to the calling function. */
	return ADC;
}