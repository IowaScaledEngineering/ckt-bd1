/*************************************************************************
Title:    CKT-BD1 Single Channel Block Detector ADC to Milliamps
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     ct-linearize.h
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
#ifndef _CTLINEARIZE_H_
#define _CTLINEARIZE_H_

#include <stdlib.h>

uint8_t ctCountToDecimilliamps(uint16_t count64);
uint16_t ctDecimilliampsToCount(uint8_t dma);

#endif
