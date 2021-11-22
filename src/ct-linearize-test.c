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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

// These two magic tables are determined empirically through testing multiple
// BD1 analog sections to determine the transfer function when the CT, the burden
// load, the amp, and the filtering section are all combined

const uint8_t ctXlateDMA[] =
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

const uint16_t ctXlateCount64[] = 
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

uint8_t pgm_read_byte(const uint8_t* a)
{
	return (*a);
}

uint16_t pgm_read_word(const uint16_t* a)
{
	return (*a);
}


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

  return count_n1 + (dma - dma_n1) * ((count_n - count_n1)/(dma_n - dma_n1));
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

#define THRESHOLD_ON  0
#define THRESHOLD_OFF 1
#define THRESHOLD_ON_MICROAMPS  1000
#define THRESHOLD_OFF_MICROAMPS  500
uint16_t threshold[2];

void testfunc(uint16_t i)
{
	uint8_t dma = ctCountToDecimilliamps(i);
	uint16_t count = ctDecimilliampsToCount(dma);

	printf("Actual: %05d  %02d.%01d %05d\n\n", i, dma/10, dma%10, count);

	threshold[THRESHOLD_ON] = ctDecimilliampsToCount(dma + (THRESHOLD_ON_MICROAMPS / 100));
	threshold[THRESHOLD_OFF] = ctDecimilliampsToCount(dma + (THRESHOLD_OFF_MICROAMPS / 100));

	printf("Off threshold:  %05d\n\n", threshold[THRESHOLD_OFF]);
	printf("On threshold:  %05d\n\n", threshold[THRESHOLD_ON]);
}

#define CHANNEL0_ON_DELAY       0x00
#define CHANNEL0_OFF_DELAY      0x01
#define CHANNEL0_THRESHOLD_ON   0x02
#define CHANNEL0_THRESHOLD_OFF  0x04
#define CHANNEL0_IDLE_CURRENT   0x06
#define CHANNEL0_IDLE_CURRENT_2 0x08

#define CHANNEL0_IDLE_CURRENT_DMA   0x10
#define CHANNEL0_THRESHOLD_ON_DMA   0x11
#define CHANNEL0_THRESHOLD_OFF_DMA  0x12

void eeprom_write_byte(uint8_t* p, uint8_t v)
{
	printf("Addr 0x%02X:  [%02X] (%d)\n", (uint8_t)p, v, v);
}

void eeprom_write_word(uint16_t* p, uint16_t v)
{
	printf("Addr 0x%02X:  [%02X%02X] (%d)\n", (uint8_t)p, v & 0xFF, v>>8, v);
}


void writeThresholdCalibration(uint16_t adcValue)
{
	// Threshold ON is idle + 1mA
	// Threshold OFF is idle + 0.5mA
	
	uint8_t dma = ctCountToDecimilliamps(adcValue);
	
	threshold[THRESHOLD_ON] = ctDecimilliampsToCount(dma + (THRESHOLD_ON_MICROAMPS / 100));
	threshold[THRESHOLD_OFF] = ctDecimilliampsToCount(dma + (THRESHOLD_OFF_MICROAMPS / 100));
	
	eeprom_write_byte((uint8_t*)(CHANNEL0_IDLE_CURRENT_DMA), dma);
	eeprom_write_byte((uint8_t*)(CHANNEL0_THRESHOLD_ON_DMA), dma + (THRESHOLD_ON_MICROAMPS / 100));
	eeprom_write_byte((uint8_t*)(CHANNEL0_THRESHOLD_OFF_DMA), dma + (THRESHOLD_OFF_MICROAMPS / 100));
	
	eeprom_write_word((uint16_t*)(CHANNEL0_IDLE_CURRENT), adcValue);
	eeprom_write_word((uint16_t*)(CHANNEL0_IDLE_CURRENT_2), ctDecimilliampsToCount(dma));
	eeprom_write_word((uint16_t*)(CHANNEL0_THRESHOLD_ON), threshold[THRESHOLD_ON]);
	eeprom_write_word((uint16_t*)(CHANNEL0_THRESHOLD_OFF), threshold[THRESHOLD_OFF]);
}


int main(void)
{
	uint16_t i;
	writeThresholdCalibration(0x440);
}
