/**********************************************************
 * Copyright:   Shanghai ReCAT Control Tech. Co., Ltd.
 * Author:      Jephy
 * Version:     0.0.1
 * Data:        2015.7.7
 * url:         http://www.recat.cc
 *
 * @Description
 * 	crc.c implement the parameter and function used for crc caculation.
 * The parameter is defined for Nikon at the current. 
 * 	To reduce the caculate time consume, the crcTable  is  constructed 
 * in an offline mode. Thus call the crcInit() before you  look up the 
 * crcTable[].
 **********************************************************/

#include "crc.h"

uint8_t crcTable[256] = {0};
void crcInit(void)
{
    uint8_t  remainder;
	int dividend, bit;

    /*
     * Compute the remainder of each possible dividend.
     */
    for (dividend = 0; dividend < 256; ++dividend)
    {
        /*
         * Start with the dividend followed by zeros.
         */
        remainder = dividend << (WIDTH - 8);

        /*
         * Perform modulo-2 division, a bit at a time.
         */
        for (bit = 8; bit > 0; --bit)
        {
            /*
             * Try to divide the current data bit.
             */			
            if (remainder & TOPBIT)
            {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }

        /*
         * Store the result into the table.
         */
        crcTable[dividend] = remainder;
    }

}   /* crcInit() */

uint8_t crcFast(uint8_t const message[], int nBytes)
{
    uint8_t data;
    uint8_t remainder = 0;
	  int byte;


    /*
     * Divide the message by the polynomial, a byte at a time.
     */
    for (byte = 0; byte < nBytes; ++byte)
    {
        data = message[byte] ^ (remainder >> (WIDTH - 8));
        remainder = crcTable[data] ^ (remainder << 8);
    }

    /*
     * The final remainder is the CRC.
     */
    return (remainder);

}   /* crcFast() */

