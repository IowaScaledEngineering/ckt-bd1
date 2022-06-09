/*************************************************************************
Title:    CKT-BD1 Test Jig DCC Generator
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2021 Michael Petersen & Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <util/atomic.h>

static inline void dcc_one()
{
	PORTB ^= _BV(1) | _BV(0);
	_delay_us(58);
	PORTB ^= _BV(1) | _BV(0);
	_delay_us(58);
}

static inline void dcc_zero()
{
	PORTB ^= _BV(1) | _BV(0);
	_delay_us(100);
	PORTB ^= _BV(1) | _BV(0);
	_delay_us(100);
}

void sendPreamble()
{
	for(uint8_t i=0; i<16; i++)
		dcc_one();
}

void sendByte(uint8_t b)
{
	for(uint8_t i=0; i<8; i++)
	{
		if (b & 0x01)
			dcc_one();
		else
			dcc_zero();
		b >>= 1;
	}
}

int main(void)
{
	// Deal with watchdog first thing
	MCUSR = 0;					// Clear reset status

	// Initialization
	wdt_reset();
	wdt_enable(WDTO_250MS);
	wdt_reset();

	// Pin Assignments for PORTB/DDRB
	//  PB0 - Power/Setpoint LED (out)
	//  PB1 - Occupancy LED - Detect (out)
	//  PB2 - /Detect (out)
	//  PB3 - Setpoint button
	//  PB4 - Detector (in, analog, ADC2)
	//  PB5 - Not used
	//  PB6 - Not used
	//  PB7 - Not used
	DDRB  = 0b00111111;
	PORTB = 0b00000001;

	//sei();

	while(1)
	{
		wdt_reset();
		
		// Preamble - 14 ones
		sendPreamble();
		
		dcc_zero(); // Start bit
		
		sendByte(0xFF);

		dcc_zero(); // Start bit
		
		sendByte(0x00);
		
		dcc_zero(); // Start bit
		
		sendByte(0xFF);

		dcc_one(); // End bit
		
		// Send DCC idle packet
	}
}

