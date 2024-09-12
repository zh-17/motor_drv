/**********************************************************
* Copyright:   Shanghai ECAT Tech. Co., Ltd.
* Author:      Volans Liao
* Version:     2.0.01
* Data:        2014.11.23
* url:         http://www.rect.cc
**********************************************************/
#ifndef __RTOS_PORT_H
#define __RTOS_PORT_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stddef.h>

	/* Size of each stack entry / stack alignment size (32 bits on ARMv7A) */
#define STACK_ALIGN_SIZE                sizeof(uint64_t)


	/* Uncomment to enable stack-checking */
	/* #define RTOS_STACK_CHECKING */
	/* Constants */
#define NS_PER_SEC 1000000000u  //表示每秒钟的纳秒数
#define U64MAX					0xFFFFFFFFFFFFFFFFu
#define TRUE                    1
#define FALSE                   0



	//#define	INT64_MAX	9223372036854775807LL
	//#define	UINT64_MAX	18446744073709551615ULL
#define EOF (-1)

#define VALUE_NOT_USEABLE (((int64_t)1)<<63)

#define ARRAY_SIZE(__arr) (sizeof((__arr)) / sizeof((__arr)[0]))
#define UNSAFE
#define ECATAPI

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)


	static inline uint32_t readl(void * addr)
	{
		return *((volatile uint32_t *)addr);
	}
	static inline uint16_t reads(void * addr)
	{
		return *((volatile uint16_t *)addr);
	}
	static inline uint8_t readb(void * addr)
	{
		return *((volatile uint8_t *)addr);
	}
	static inline void writel(uint32_t data, void * addr)
	{
		*((volatile uint32_t *)addr) = data;
	}
	static inline void writes(uint16_t data, void * addr)
	{
		*((volatile uint16_t *)addr) = data;
	}
	static inline void writeb(uint8_t data, void * addr)
	{
		*((volatile uint8_t *)addr) = data;
	}
	typedef volatile unsigned int * __REG;
#define HWREG(x)  ( *( (__REG)( x ) ) )
#define HWREGB(x)                                                             \
        (*((volatile unsigned char *)(x)))
#define HWREGH(x)                                                             \
        (*((volatile unsigned short *)(x)))

#ifdef __cplusplus
}
#endif
#endif /* __RTOS_PORT_H */
