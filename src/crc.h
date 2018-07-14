/**********************************************************
 * Copyright:   Shanghai ReCAT Control Tech. Co., Ltd.
 * Author:      Jephy
 * Version:     0.0.1
 * Data:        2015.7.7
 * url:         http://www.recat.cc
 * @Description
 * crc.h define the parameter and function used for crc caculation.
 * The parameter is defined for Nikon at the current.
 **********************************************************/

#ifndef		__CRC__H__
#define 	__CRC__H__
#include "stdint.h"

#define WIDTH  (8 * sizeof(uint8_t))
#define TOPBIT (1 << (WIDTH - 1))

#define POLYNOMIAL  0x01 

uint8_t crcTable[256];
void crcInit(void);  
uint8_t crcFast(uint8_t const message[], int nBytes);


#endif



