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
#include <util/atomic.h>

#include "bd1-interrupts.h"

volatile uint8_t eventFlags = 0;
volatile uint16_t adcValue = 0;

void initializeADC()
{
	// Setup ADC for bus voltage monitoring
	ADMUX  = 0x02;  // AVCC reference, ADC2 starting channel
	ADCSRA = _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 64 prescaler, ~8.3kconv / s
	ADCSRB = 0x00; // Free running mode
	DIDR0  = _BV(ADC2D);  // Turn ADC2/ADC3 pins into analog inputs
	ADCSRA |= _BV(ADEN) | _BV(ADIE) | _BV(ADIF);
}

ISR(ADC_vect)
{
  static int32_t accumulator = 0;
  static uint16_t count = 0;

  accumulator += ADC;
  if (++count >= 256)
  {
    adcValue = accumulator/4;
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
	TIMSK |= _BV(OCIE0A);
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
