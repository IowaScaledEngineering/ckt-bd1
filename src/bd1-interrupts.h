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

#ifndef _BD1_INTERRUPTS_H_
#define _BD1_INTERRUPTS_H_

#include <stdlib.h>

extern volatile uint16_t adcValue;
extern volatile uint8_t eventFlags;

#define EVENT_DO_BD_READ        0x01
#define EVENT_DO_ADC_RUN        0x02
#define EVENT_1HZ_BLINK         0x04
#define EVENT_CALIBRATION_MODE  0x08

void triggerADC();
void initializeADC();
void initialize100HzTimer();

#endif


