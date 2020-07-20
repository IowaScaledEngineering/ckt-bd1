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

#define CHANNEL0_ON_DELAY     0x00
#define CHANNEL0_OFF_DELAY    0x01
#define CHANNEL0_THRESHOLD_H  0x02
#define CHANNEL0_THRESHOLD_L  0x03

#define CALIBRATION_CYCLES        16
#define CALIBRATION_BUTTON_DELAY  20

volatile uint8_t eventFlags = 0;
#define EVENT_DO_BD_READ        0x01
#define EVENT_DO_ADC_RUN        0x02
#define EVENT_1HZ_BLINK         0x04
#define EVENT_CALIBRATION_MODE  0x08


uint8_t detectorOnDelayCount = 0;
uint8_t detectorOffDelayCount = 0;
uint8_t detectorOnDelay = 4;
uint8_t detectorOffDelay = 25;

volatile uint16_t adcValue[2];
#define ADC_CHANNEL_DETECTOR_0  0
#define ADC_CHANNEL_SETPOINT_0  1


uint16_t readThresholdCalibration()
{
	return ((uint16_t)eeprom_read_byte((uint8_t*)(CHANNEL0_THRESHOLD_H)))<<8 | (uint16_t)(eeprom_read_byte((uint8_t*)(CHANNEL0_THRESHOLD_L)));
}

void writeThresholdCalibration(uint16_t threshold)
{
	eeprom_write_byte((uint8_t*)(CHANNEL0_THRESHOLD_H), 0x03 & (threshold>>8));
	eeprom_write_byte((uint8_t*)(CHANNEL0_THRESHOLD_L), 0xFF & threshold);
}


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

	while (0xFFFF == (adcValue[ADC_CHANNEL_SETPOINT_0] = readThresholdCalibration()))
	{
		writeThresholdCalibration(0x2f);
	}
}

ISR(ADC_vect)
{
	static uint16_t accumulator = 0;
	static uint8_t count = 0;
	
	accumulator += ADC;
	if (++count >= 64)
	{
		adcValue[0] = accumulator / 64;
		accumulator = 0;
		count = 0;
		eventFlags |= EVENT_DO_BD_READ;
	} else {
		// Trigger the next conversion.  Not using auto-trigger so that we can safely change channels
		ADCSRA |= _BV(ADSC);
	}
}

void initialize100HzTimer(void)
{
	// Set up timer 0 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 94;  // 9.6MHz / 1024 / 94 ~= 100Hz
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);  // 1024 prescaler
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIM0_COMPA_vect)
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
	// Setup ADC for bus voltage monitoring
	ADMUX  = 0x02;  // AVCC reference, ADC2 starting channel
	ADCSRA = _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 64 prescaler, ~8.3kconv / s
	ADCSRB = 0x00; // Free running mode
	DIDR0  = _BV(ADC2D);  // Turn ADC2/ADC3 pins into analog inputs
	ADCSRA |= _BV(ADEN) | _BV(ADIE) | _BV(ADIF);
}


inline void setOccupancyOn()
{
	PORTB &= ~_BV(PB2);
	PORTB |= _BV(PB1);
}

inline void setOccupancyOff()
{
	PORTB &= ~_BV(PB1);
	PORTB |= _BV(PB2);
}

inline void setAuxLEDOn()
{
	PORTB |= _BV(PB0);
}

inline void setAuxLEDOff()
{
	PORTB &= ~_BV(PB0);
}

uint8_t detectorStatus = 0;

inline void processDetector(void)
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
	uint8_t calibrationSwitchCountdown = CALIBRATION_BUTTON_DELAY;
	uint8_t calibrationCycles = CALIBRATION_CYCLES;
	uint32_t calibrationAccumulator = 0;
	
	// Deal with watchdog first thing
	MCUSR = 0;					// Clear reset status

	// Initialization
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

	initializeDelays();
	initializeADC();

	setOccupancyOff();

	initialize100HzTimer();
	initializeADC();
	initializeDelays();

	// End Initialization

	sei();

	while(1)
	{
		wdt_reset();

		// If we're in cal mode, do special stuff
		if (eventFlags & EVENT_CALIBRATION_MODE)
		{
			setOccupancyOff();
			
			if (calibrationCycles & 0x02)
			{
				setAuxLEDOff();
			} else {
				setAuxLEDOn();
			}
			
			if (eventFlags & EVENT_DO_BD_READ)
			{
				if (calibrationCycles)
				{
					calibrationCycles--;
					calibrationAccumulator += adcValue[ADC_CHANNEL_DETECTOR_0];
				} else {
					// last calibration cycle, store values
					uint16_t threshold = (uint16_t)((calibrationAccumulator) / (CALIBRATION_CYCLES));
					uint16_t ninetyPercentThreshold = (uint16_t)((calibrationAccumulator) / (10 * CALIBRATION_CYCLES / 9));
					if (threshold - ninetyPercentThreshold < 8 && threshold >= 9) // 5 is the hysteresis threshold
						threshold -= 8;
					else 
						threshold = ninetyPercentThreshold;

					writeThresholdCalibration(threshold);
					adcValue[ADC_CHANNEL_SETPOINT_0] = readThresholdCalibration();
					eventFlags &= ~(EVENT_CALIBRATION_MODE);
					setAuxLEDOff();
				}
				eventFlags &= ~(EVENT_DO_BD_READ);
			}

		} else if (eventFlags & EVENT_DO_BD_READ) {
			// Not calibration mode - do normal stuff
			
			// If all the analog inputs have been read, the flag will be set and we
			// can then process the analog detector inputs
			// Do all the analog magic
			processDetector();

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

		// Put this after the main loop, so that the ADC values don't get corrupted by 
		// starting another conversion
		if (EVENT_DO_ADC_RUN == (eventFlags & (EVENT_DO_ADC_RUN | EVENT_DO_BD_READ)))
		{
			// If the ISR tells us it's time to run the ADC again and we've handled the last read, 
			// start the ADC again

			ADCSRA |= _BV(ADSC);
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				eventFlags &= ~(EVENT_DO_ADC_RUN);
			}

			if (!(PINB & _BV(PB3)))
			{
				// If the calibration switch is down
				if (0 != calibrationSwitchCountdown)
				{
					setAuxLEDOff();
					calibrationSwitchCountdown--;
				} else {
					setAuxLEDOn();
				}
			} else if (!(eventFlags & EVENT_CALIBRATION_MODE)) {
				if (0 == calibrationSwitchCountdown) 
				{
					eventFlags |= EVENT_CALIBRATION_MODE;
				}
				
				calibrationSwitchCountdown = CALIBRATION_BUTTON_DELAY;
				calibrationCycles = CALIBRATION_CYCLES;
			}
		}
	}
}

