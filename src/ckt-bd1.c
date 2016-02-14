/*************************************************************************
Title:    CKT-BD1 Single Channel Block Detector
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2016 Michael Petersen & Nathan Holmes

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

#define CHANNEL0_ON_DELAY   0x00
#define CHANNEL0_OFF_DELAY  0x01

volatile uint8_t eventFlags = 0;
#define EVENT_DO_BD_READ 0x01
#define EVENT_DO_ADC_RUN 0x02
#define EVENT_1HZ_BLINK  0x04

uint8_t detectorOnDelayCount = 0;
uint8_t detectorOffDelayCount = 0;
uint8_t detectorOnDelay = 4;
uint8_t detectorOffDelay = 25;

volatile uint16_t adcValue[2];
#define ADC_CHANNEL_DETECTOR_0  0
#define ADC_CHANNEL_SETPOINT_0  1

void initializeDelays()
{
	detectorOnDelay = eeprom_read_byte((uint8_t*)(CHANNEL0_ON_DELAY));
	if (0xFF == detectorOnDelay || 0x00 == detectorOnDelay)
	{
		eeprom_write_byte((uint8_t*)(CHANNEL0_ON_DELAY), 4);
		detectorOnDelay = eeprom_read_byte((uint8_t*)(CHANNEL0_ON_DELAY));
	}	


	detectorOffDelay = eeprom_read_byte((uint8_t*)(CHANNEL0_OFF_DELAY));
	if (0xFF == detectorOffDelay || 0x00 == detectorOffDelay)
	{
		eeprom_write_byte((uint8_t*)(CHANNEL0_OFF_DELAY), 25);
		detectorOffDelay = eeprom_read_byte((uint8_t*)(CHANNEL0_OFF_DELAY));
	}	
}

ISR(ADC_vect)
{
	static uint8_t workingChannel = 0;
	static uint16_t accumulator = 0;
	static uint8_t count = 0;
	
	accumulator += ADC;
	if (++count >= 64)
	{
		adcValue[workingChannel] = accumulator / 64;
		accumulator = 0;
		count = 0;
		workingChannel++;
		
		ADMUX = (ADMUX & 0xFE) | (workingChannel & 0x01);
		
		if (2 == workingChannel)
		{
			workingChannel = 0;
			eventFlags |= EVENT_DO_BD_READ;
		}
	}

	if (0 == (eventFlags & EVENT_DO_BD_READ))
	{
		// Trigger the next conversion.  Not using auto-trigger so that we can safely change channels
		ADCSRA |= _BV(ADSC);
	}
}


void initialize10HzTimer(void)
{
	// Set up timer 0 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 78;  // 8MHz / 1024 / 78 = 100Hz
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);  // 1024 prescaler
	TIMSK |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	static uint8_t ticks = 0;
	static uint8_t decisecs = 0;
	
	if (++ticks >= 10)
	{
		ticks = 0;
		eventFlags |= EVENT_DO_ADC_RUN;

		if (++decisecs >=5)
		{
			eventFlags ^= EVENT_1HZ_BLINK;
			decisecs = 0;
		}
	}
}

void initializeADC()
{
	for(uint8_t i=0; i<sizeof(adcValue)/sizeof(adcValue[0]); i++)
		adcValue[i] = 0;

	// Setup ADC for bus voltage monitoring
	ADMUX  = 0x02;  // AVCC reference, ADC2 starting channel
	ADCSRA = _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 64 prescaler, ~8.3kconv / s
	ADCSRB = 0x00; // Free running mode
	DIDR0  = _BV(ADC2D) | _BV(ADC3D);  // Turn ADC2/ADC3 pins into analog inputs
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);
}


void setOccupancyOn()
{
	PORTB &= ~_BV(PB2);
	PORTB |= _BV(PB1);
}

void setOccupancyOff()
{
	PORTB &= ~_BV(PB1);
	PORTB |= _BV(PB2);
}

void setAuxLEDOn()
{
	PORTB |= _BV(PB0);
}

void setAuxLEDOff()
{
	PORTB &= ~_BV(PB0);
}

void init(void)
{
	MCUSR = 0;
	wdt_reset();
	wdt_enable(WDTO_250MS);
	wdt_reset();

	// Pin Assignments for PORTB/DDRB
	//  PB0 - Power/Alive LED (out)
	//  PB1 - Occupancy LED - Detect (out)
	//  PB2 - /Detect (out)
	//  PB3 - Setpoint (in, analog, ADC3)
	//  PB4 - Detector (in, analog, ADC2)
	//  PB5 - Not used
	//  PB6 - Not used
	//  PB7 - Not used
	DDRB  = 0b00000111;
	PORTB = 0b00000100;

	initializeDelays();
	initializeADC();

}

uint8_t detectorStatus = 0;

void processDetector(void)
{	
	uint16_t threshold = adcValue[ADC_CHANNEL_SETPOINT_0];

	// Introduce a bit of hysteresis. 
	// If the channel is currently "on", lower the threshold by 5 counts
	// If the channel is currently "off", raise the threshold by 5
	if ((0 != detectorStatus) && (threshold >= 5))
	{
		threshold -= 5;
	} 
	else if ((0 == detectorStatus) && (threshold <= (1023-5)) )
	{
		threshold += 5;
	}
	
	if (adcValue[ADC_CHANNEL_DETECTOR_0] > threshold)
	{
		// Current for the channel has exceeded threshold
		if (0 == detectorStatus)
		{
			// Channel is currently in a non-detecting state
			// Wait for the turnon delay before actually indicating on
			if (detectorOnDelayCount < detectorOnDelay)
				detectorOnDelayCount++;
			else
			{
				detectorStatus = 1;
				detectorOffDelayCount = 0;
			}
		} else {
			// Channel is currently in a detecting state
			detectorOffDelayCount = 0;
		}
	} else {
		// Current for the channel is under the threshold value
		if (0 != detectorStatus)
		{
			// Channel is currently in a non-detecting state
			if (detectorOffDelayCount < detectorOffDelay)
				detectorOffDelayCount++;
			else
			{
				detectorStatus = 0;
				detectorOnDelayCount = 0;
			}
		} else {
			// Channel is currently in a detecting state
			detectorOnDelayCount = 0;
		}
	}
}

int main(void)
{
	// Deal with watchdog first thing
	MCUSR = 0;					// Clear reset status
	init();


	setOccupancyOff();
	setAuxLEDOff();

	initialize10HzTimer();
	initializeADC();
	initializeDelays();

	sei();

	while(1)
	{
		wdt_reset();

		// If all the analog inputs have been read, the flag will be set and we
		// can then process the analog detector inputs
		if (eventFlags & EVENT_DO_BD_READ)
		{
			// Do all the analog magic
			processDetector();

			if (detectorStatus)
				setOccupancyOn();
			else
				setOccupancyOff();


			// Clear the flag and start the next chain of conversions
			eventFlags &= ~(EVENT_DO_BD_READ);
		}

		if (EVENT_DO_ADC_RUN == (eventFlags & (EVENT_DO_ADC_RUN | EVENT_DO_BD_READ)))
		{
			// If the ISR tells us it's time to run the ADC again and we've handled the last read, 
			// start the ADC again
			ADCSRA |= _BV(ADSC);
		}

		if (eventFlags & EVENT_1HZ_BLINK)
			setAuxLEDOn();
		else
			setAuxLEDOff();
	}
}

