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

#define irq_disable() __asm(" CPSID   I") //禁用中断
#define irq_enble() __asm(" CPSIE   I")
void msleep(int16_t ms);
uint32_t system_time();
void board_init();
void board_test();
void power_on_init();

/*************encoder**************/
#define ABZ_ENCODER 1  // 定义了一种三相编码器
#define AB_ENCODER 2   // 不包含Z相的索引脉冲。
#define HALL_ENCODER 4 
#define ABS_ENCODER 8
#define PANASONIC_QEP_ENCODER 16 // 定义了一种特定于松下（Panasonic）公司的增量式编码器（QEP代表Quadrature Encoder Pulses），用于提供电机转子位置的增量信息。
#define PANASONIC_ABS_ENCODER 32 // 定义了一种特定于松下公司的绝对值编码器，能够提供电机转子的绝对位置信息。

/************trigger type**********/ // TRIGGER 触发，TRIGGER_FALLING 表示下降沿触发。这些可以用于设置编码器的中断触发方式。
#define TRIGGER_RISING 1
#define TRIGGER_FALLING 2

// #define DEBUG 1  resolution 分辨率
#define PPR_10000 0
#define RESOLUTION_17BITS 1
#define RESOLUTION_20BITS 2
#define RESOLUTION_23BITS 3

typedef char encoder_t;

#define PWM_FREQUENCY 16000 // 定义了PWM信号的频率为16000Hz，即每秒产生16000个脉冲。
#define PWM_DEADTIME_NS 600 // 定义了PWM的死区时间（Dead Time）为600纳秒。死区时间是为了防止上下桥臂的两个晶体管同时导通而设置的一个时间间隔。
#define SYS_FREQUENCY SystemCoreClock  //一般168MB

/* the ia, ib feedback dead in A*/
#define driver_fbk_idead(ptr) F16(0.1)
/* the pwm dead, 0.1 for 10% duty*/
#define driver_output_dead(ptr) F16((PWM_DEADTIME_NS * 1.0 / (1000000000 / PWM_FREQUENCY)))
#define driver_output_dead2(ptr) F16((PWM_DEADTIME_NS * 2.0 / (1000000000 / PWM_FREQUENCY)))
#define driver_frequency(ptr) (PWM_FREQUENCY * 2)

void driver_enable(void *ptr);
void driver_disable(void *ptr);
#define driver_output(ptr, a, b, c) pwm_output(a, b, c)

int16_t driver_ia(void *self);
int16_t driver_ib(void *self);
int16_t driver_ibus(void *self);

int64_t encoder_position(encoder_t encoder);
int64_t encoder_offset(encoder_t encoder);
char encoder_has_offset(encoder_t encoder);
void encoder_clear_offset(encoder_t encoder);
int32_t encoder_ppr(encoder_t encoder);
void qep_speed();
int32_t encoder_speed(encoder_t encoder);

void pwm_output(fix16_t a, fix16_t b, fix16_t c);

// void usart_dma_send(DMA_Channel_TypeDef *dma_channel, uint8_t* buf, uint8_t size);
// void usart_receive_dma_reset(DMA_Channel_TypeDef * dma);
void flash_write_more_data(uint32_t startAddress, uint16_t *writeData, uint16_t countToWrite);
void pos_latch_init(int16_t trigger_type);

#include <stdio.h>
#include "control/protocol.h"
#define printk printf

extern uint8_t usart1_dma_rx_t[USART_BUFF_LEN]; // 声明了两个外部数组，分别用于USART1和USART4的DMA接收缓冲区。
extern uint8_t usart4_dma_rx_t[USART_BUFF_LEN];

#define DMA_CCR_EN 1
#define DWT_CR *(volatile u32 *)0xE0001000
// 对应的是 Data Watchpoint and Trace (DWT) Comparator Register，用于设置和控制硬件断点、数据监视和性能计数器。
#define DWT_CYCCNT *(volatile u32 *)0xE0001004  //表示的是内存地址 0xE0001004 处的值。 刚上电值为0
#define DEM_CR *(volatile u32 *)0xE000EDFC
#define DBGMCU_CR *(volatile u32 *)0xE0042004
#define DEM_CR_TRCENA (1 << 24)  //取位操作
#define DWT_CR_CYCCNTENA (1 << 0) //取为操作

#define CORE_CLK_RESET()             \   //确保了每次调用 CORE_CLK_RESET() 时，循环计数器都从 0 开始计数，可以用于精确测量代码执行时间或其他需要高精度计时的操作。
	do                               \
	{                                \
		DEM_CR |= DEM_CR_TRCENA;     \   //使能 DWT（Data Watchpoint and Trace）模块
		DWT_CR &= ~DWT_CR_CYCCNTENA; \   //禁用循环计数器 清除了 DWT_CR 寄存器（位于地址 0xE0001000）的第 0 位
		DWT_CYCCNT = 0u;             \   //复位循环计数器
		DWT_CR |= DWT_CR_CYCCNTENA;  \   //重新启用循环计数器
	} while (0)
#define GET_CORE_CLK() DWT_CYCCNT;

#define MAX_PWM_DUTY 0.9
#endif