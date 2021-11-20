/*************************************************************************
Title:    CKT-BD1 Single Channel Block Detector ADC to Milliamps
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
#include <avr/pgmspace.h>

// These two magic tables are determined empirically through testing multiple
// BD1 analog sections to determine the transfer function when the CT, the burden
// load, the amp, and the filtering section are all combined

const uint8_t ctXlateDMA[] PROGMEM =
{
  0,   //     0 },
  5,   //   314 },
  10,  //   839 },
  15,  //  1965 },
  20,  //  3079 },
  25,  //  4365 },
  30,  //  5815 },
  35,  //  7407 },
  40,  //  9108 },
  45,  // 10997 },
  50,  // 12747 },
  55,  // 14861 },
  60,  // 16575 },
  65,  // 18008 },
  70,  // 19451 },
  75,  // 20622 },
  80,  // 21763 },
  85,  // 22803 },
  90,  // 23506 },
  95,  // 24264 },
  100, // 25050 },
  105, // 25668 },
  110, // 26277 },
  120, // 27385 },
  130, // 28062 },
  140, // 28456 },
  150, // 28839 },
  160  // 29137 }
};

const uint16_t ctXlateCount64[] PROGMEM = 
{
  0,     // 0
  314,   // 5
  839,   // 10
  1965,  // 15
  3079,  // 20
  4365,  // 25
  5815,  // 30
  7407,  // 35
  9108,  // 40
  10997, // 45
  12747, // 50
  14861, // 55
  16575, // 60
  18008, // 65
  19451, // 70
  20622, // 75
  21763, // 80
  22803, // 85
  23506, // 90
  24264, // 95
  25050, // 100
  25668, // 105
  26277, // 110
  27385, // 120
  28062, // 130
  28456, // 140
  28839, // 150
  29137 // 160
};

uint16_t ctDecimilliampsToCount(uint8_t dma)
{
  uint8_t i;
  for(i=0; i<sizeof(ctXlateDMA) / sizeof(uint8_t); i++)
  {
	uint8_t thisDma = pgm_read_byte(&ctXlateDMA[i]);
	if (thisDma == dma)
		return(pgm_read_word(&ctXlateCount64[i]));
	if (thisDma > dma)
		break;
  }

  if (i == sizeof(ctXlateDMA) / sizeof(uint8_t))
  {
    // Overrange, just return full scale
    return (0x7FFF);
  }

  uint16_t count_n = (uint16_t)pgm_read_word(&ctXlateCount64[i]);
  uint16_t count_n1 = (uint16_t)pgm_read_word(&ctXlateCount64[i-1]);
  uint8_t dma_n = (uint8_t)pgm_read_byte(&ctXlateDMA[i]);
  uint8_t dma_n1 = (uint8_t)pgm_read_byte(&ctXlateDMA[i-1]);

  return count_n1 + (dma - dma_n) * ((count_n - count_n1)/(dma_n - dma_n1));
}

uint8_t ctCountToDecimilliamps(uint16_t count64)
{
  uint8_t i;
  for(i=0; i<sizeof(ctXlateCount64) / sizeof(uint16_t); i++)
  {
	uint16_t thisCount = (uint16_t)pgm_read_word(&ctXlateCount64[i]);
    if (thisCount == count64)
      return (uint8_t)pgm_read_byte(&ctXlateDMA[i]);
 
    if (thisCount > count64)
      break;
  }

  if (i == sizeof(ctXlateCount64) / sizeof(uint16_t))
  {
    // Overrange, just return full scale
    return (255);
  }

  uint16_t count_n = (uint16_t)pgm_read_word(&ctXlateCount64[i]);
  uint16_t count_n1 = (uint16_t)pgm_read_word(&ctXlateCount64[i-1]);
  uint8_t dma_n = (uint8_t)pgm_read_byte(&ctXlateDMA[i]);
  uint8_t dma_n1 = (uint8_t)pgm_read_byte(&ctXlateDMA[i-1]);
  
  return (count64 - count_n1) / ((count_n - count_n1) / (dma_n - dma_n1)) + dma_n1;
}
