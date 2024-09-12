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
// CRC计算中使用的位数，这里是8位
// 计算了WIDTH位中最高位的掩码。例如，对于8位，TOPBIT将是0x80（二进制表示为10000000）
#define WIDTH  (8 * sizeof(uint8_t)) 
#define TOPBIT (1 << (WIDTH - 1))
// 定义了用于CRC计算的多项式。CRC多项式决定了CRC校验的算法特性。在这个例子中，多项式是0x01，这是一个非常常见的CRC-8多项式。
#define POLYNOMIAL  0x01 
// CRC查找表。查找表是通过预计算每个可能的CRC值来构建的，这样可以在运行时快速计算CRC值，而不是实时计算。
uint8_t crcTable[256];
// 初始化CRC查找表  函数声明
// // 计算消息的CRC值
void crcInit(void);  
uint8_t crcFast(uint8_t const message[], int nBytes);


#endif



