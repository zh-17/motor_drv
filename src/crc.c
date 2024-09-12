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
// 实现了CRC查找表的初始化。它通过对于所有可能的8位数据值（0到255）进行模2除法运算，并使用预定义的多项式生成CRC校验码，然后将这些校验码存储在crcTable数组中
void crcInit(void)
{
    uint8_t  remainder;//余数
	int dividend, bit;//被除数、位

    for (dividend = 0; dividend < 256; ++dividend)
    {
        // 被除数后跟零开始
        remainder = dividend << (WIDTH - 8);
        // 执行模2除法
        for (bit = 8; bit > 0; --bit)
        {
            // 如果余数最高位为1
            if (remainder & TOPBIT)
            {
                remainder = (remainder << 1) ^ POLYNOMIAL;//左移一位并与多项式异或
            }
            else//最高位为0
            {
                remainder = (remainder << 1);
            }
        }
        crcTable[dividend] = remainder;
    }

}

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
        // 获取当前字节，与余数的最高位进行异或操作
        data = message[byte] ^ (remainder >> (WIDTH - 8));
        // 使用查找表获取data的CRC值，并与余数左移8位进行异或操作，更新余数
        remainder = crcTable[data] ^ (remainder << 8);
    }


    return (remainder);

}

