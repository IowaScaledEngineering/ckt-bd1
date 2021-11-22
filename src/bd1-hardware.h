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

#ifndef _BD1_HARDWARE_H_
#define _BD1_HARDWARE_H_

#include <stdlib.h>
#include <stdbool.h>
#include <avr/io.h>
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

inline void setCalLEDOn()
{
	PORTB |= _BV(PB0);
}

inline void setCalLEDOff()
{
	PORTB &= ~_BV(PB0);
}

inline bool getCalSwitchState()
{
	return (PINB & _BV(PB3))?false:true;
}

#endif

