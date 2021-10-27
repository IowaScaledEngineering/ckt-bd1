/*************************************************************************
Title:    CKT-BD1 Single Channel Block Detector v3
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2020 Michael Petersen & Nathan Holmes

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
#include <stdint.h>
#include <stdbool.h>

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

#include "ct-linearize.h"
#include "bd1-interrupts.h"
#include "bd1-hardware.h"

#define CHANNEL0_ON_DELAY       0x00
#define CHANNEL0_OFF_DELAY      0x01
#define CHANNEL0_THRESHOLD_ON   0x02
#define CHANNEL0_THRESHOLD_OFF  0x04
#define CHANNEL0_IDLE_CURRENT   0x06


uint8_t detectorOnDelayCount = 0;
uint8_t detectorOffDelayCount = 0;
uint8_t detectorOnDelay = 4;
uint8_t detectorOffDelay = 25;

void initializeDelays()
{
	while (0xFF == (detectorOnDelay = eeprom_read_byte((uint8_t*)(CHANNEL0_ON_DELAY))))
	{
		eeprom_write_byte((uint8_t*)(CHANNEL0_ON_DELAY), 4);
	}

	while (0xFF == (detectorOffDelay = eeprom_read_byte((uint8_t*)(CHANNEL0_OFF_DELAY))))
	{
		eeprom_write_byte((uint8_t*)(CHANNEL0_OFF_DELAY), 25);
	}
}

uint8_t detectorStatus = 0;

#define THRESHOLD_ON  0
#define THRESHOLD_OFF 1

#define THRESHOLD_ON_MICROAMPS  1000
#define THRESHOLD_OFF_MICROAMPS  500

uint16_t threshold[2];

void writeThresholdCalibration(uint16_t adcValue)
{
	// Threshold ON is idle + 1mA
	// Threshold OFF is idle + 0.5mA
	
	uint8_t dma = ctCountToDecimilliamps(adcValue);
	
	threshold[THRESHOLD_ON] = ctDecimilliampsToCount(dma + (THRESHOLD_ON_MICROAMPS / 100));
	threshold[THRESHOLD_OFF] = ctDecimilliampsToCount(dma + (THRESHOLD_OFF_MICROAMPS / 50));
	
	eeprom_write_word((uint16_t*)(CHANNEL0_IDLE_CURRENT), adcValue);
	eeprom_write_word((uint16_t*)(CHANNEL0_THRESHOLD_ON), threshold[THRESHOLD_ON]);
	eeprom_write_word((uint16_t*)(CHANNEL0_THRESHOLD_OFF), threshold[THRESHOLD_OFF]);
}

void readThresholdCalibration()
{
	threshold[THRESHOLD_ON] = eeprom_read_word((uint16_t*)CHANNEL0_THRESHOLD_ON);
	threshold[THRESHOLD_OFF] = eeprom_read_word((uint16_t*)CHANNEL0_THRESHOLD_OFF);
	if (0xFFFF == threshold[THRESHOLD_OFF])
	{
		writeThresholdCalibration(0);
	}
}


void processDetector(uint16_t adc)
{
	static uint16_t adcValueFiltered = 0;
	static bool firstLoop = true;

	// Do a little filtering
	if (firstLoop)
	{
		firstLoop = false;
		adcValueFiltered = adc;
	} else {
		adcValueFiltered += (adc - adcValueFiltered)/4;
	}
	
	// Set threshold here based on whether we're on or off
	if (detectorStatus)
	{
		// Detector is currently on

		// If we're below the turn-off threshold
		if (adcValue < threshold[THRESHOLD_OFF])
		{
			if (detectorOffDelayCount < detectorOffDelay)
				detectorOffDelayCount++;
			else
			{
				detectorStatus = 0;
				detectorOnDelayCount = 0;
			}
		} else {
			detectorOffDelayCount = 0;
		}
	} else {
		// Detector is currently off
		// If we're above the turn-off threshold
		if (adcValue >= threshold[THRESHOLD_ON])
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
			detectorOnDelayCount = 0;
		}
	}
}

#define max(a,b)  ((a>b)?(a):(b))

int main(void)
{
	// Deal with watchdog first thing
	MCUSR = 0;					// Clear reset status

	// Start Initialization
	wdt_reset();
	wdt_enable(WDTO_250MS);
	wdt_reset();

	CLKPR = _BV(CLKPCE);
	CLKPR = 0x00;

	// Pin Assignments for PORTB/DDRB
	//  PB0 - Power/Setpoint LED (out)
	//  PB1 - Occupancy LED - Detect (out)
	//  PB2 - /Detect (out)
	//  PB3 - Setpoint button
	//  PB4 - Detector (in, analog, ADC2)
	//  PB5 - Not used
	//  PB6 - Not used
	//  PB7 - Not used
	DDRB  = 0b00000111;
	PORTB = 0b00001100;

	setOccupancyOff();
	
	initializeDelays();
	initializeADC();
	initialize100HzTimer();
	initializeADC();
	// End Initialization

	readThresholdCalibration();

	sei();

	while(1)
	{
		wdt_reset();
		
		

		if (eventFlags & EVENT_DO_BD_READ) 
		{
			// If all the analog inputs have been read, the flag will be set and we
			// can then process the analog detector inputs
			// Do all the analog magic
			processDetector(adcValue);

			if (detectorStatus)
				setOccupancyOn();
			else
				setOccupancyOff();

			// Clear the flag and start the next chain of conversions
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				eventFlags &= ~(EVENT_DO_BD_READ);
			}
		}
	}
}

