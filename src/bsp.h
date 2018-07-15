#ifndef BSP_H__
#define BSP_H__
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "system_stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "fix16.h"
#include "token.h"

#define irq_disable()	__asm(" CPSID   I")
#define irq_enble()		__asm(" CPSIE   I")
void msleep(int16_t ms);
uint32_t system_time();
void board_init();
void board_test();
void power_on_init();

/*************encoder**************/
#define ABZ_ENCODER			1
#define AB_ENCODER			2
#define HALL_ENCODER		4
#define ABS_ENCODER			8
#define PANASONIC_QEP_ENCODER	 16
#define PANASONIC_ABS_ENCODER	 32

/************trigger type**********/
#define TRIGGER_RISING 1
#define TRIGGER_FALLING 2

//#define DEBUG 1
#define PPR_10000  0
#define RESOLUTION_17BITS 1
#define RESOLUTION_20BITS 2
#define RESOLUTION_23BITS 3

typedef char encoder_t;

#define PWM_FREQUENCY	16000
#define PWM_DEADTIME_NS	600
#define SYS_FREQUENCY	SystemCoreClock

/* the ia, ib feedback dead in A*/
#define driver_fbk_idead(ptr)	F16(0.1)
/* the pwm dead, 0.1 for 10% duty*/
#define driver_output_dead(ptr)	F16((PWM_DEADTIME_NS * 1.0 / (1000000000 / PWM_FREQUENCY)))
#define driver_output_dead2(ptr)	F16((PWM_DEADTIME_NS * 2.0 / (1000000000 / PWM_FREQUENCY)))
#define driver_frequency(ptr) (PWM_FREQUENCY * 2)

void driver_enable(void *ptr);
void driver_disable(void *ptr);
#define driver_output(ptr, a, b, c)	pwm_output(a, b, c)

int16_t driver_ia(void* self);
int16_t driver_ib(void* self);
int16_t driver_ibus(void* self);


int64_t encoder_position(encoder_t encoder);
int64_t encoder_offset(encoder_t encoder);
char encoder_has_offset(encoder_t encoder);
void encoder_clear_offset(encoder_t encoder);
int32_t encoder_ppr(encoder_t encoder);
void qep_speed();
int32_t encoder_speed(encoder_t encoder);

void pwm_output(fix16_t a, fix16_t b, fix16_t c);

//void usart_dma_send(DMA_Channel_TypeDef *dma_channel, uint8_t* buf, uint8_t size);
//void usart_receive_dma_reset(DMA_Channel_TypeDef * dma);
void flash_write_more_data(uint32_t startAddress, uint16_t *writeData, uint16_t countToWrite);
void pos_latch_init(int16_t trigger_type);

#include <stdio.h>
#include "control/protocol.h"
#define printk printf

extern uint8_t usart1_dma_rx_t[USART_BUFF_LEN];
extern uint8_t usart4_dma_rx_t[USART_BUFF_LEN];

#define  DMA_CCR_EN  1
#define  DWT_CR      *(volatile u32 *)0xE0001000
#define  DWT_CYCCNT  *(volatile u32 *)0xE0001004
#define  DEM_CR      *(volatile u32 *)0xE000EDFC
#define  DBGMCU_CR   *(volatile u32 *)0xE0042004
#define  DEM_CR_TRCENA                   (1 << 24)
#define  DWT_CR_CYCCNTENA                (1 <<  0)

#define  CORE_CLK_RESET()		do{ DEM_CR	|= DEM_CR_TRCENA;\
								 DWT_CR &= ~DWT_CR_CYCCNTENA;\
								DWT_CYCCNT = 0u;\
								 DWT_CR	|= DWT_CR_CYCCNTENA;} while(0)
#define	GET_CORE_CLK()			 DWT_CYCCNT;


#define MAX_PWM_DUTY 0.9
#endif