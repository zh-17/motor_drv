/******************interrupt *********************/

#include "bsp.h"
#include "crc.h"
#include "protocol.h"
#include "encoder.h"
#include "control/motor.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_flash.h"
#include "misc.h"
#include "stdlib.h"
#include "stdio.h"

static uint32_t crc_error = 0;
static uint32_t crc_ok = 0;
volatile uint16_t command[64];
volatile uint16_t feedback[64];

void HardFault_Handler(void)
{
	driver_disable(0);
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}


/* called by ADC DMA finished IRQ
* the main time sequence is PWM CC1 trigger the ADC, DMA collect the ADC sample, and trigger a IRQ, which call board_update, and update all the control loop.
*/
static void board_update();

/***********************SYS_TICK**************************/
static volatile uint32_t system_tick = 0;

void SysTick_Handler(void)
{
	system_tick += 1;
}
void msleep(int16_t ms)
{
	uint32_t wakeup = system_tick + ms;
	while (system_tick < wakeup);
}
uint32_t system_time()
{
	return system_tick;
}

/**********************PWM*************************/
#define PWM_COUNT		(SYS_FREQUENCY / PWM_FREQUENCY / 2 - 1) 
/* hardware config 
PA8		TIM1_CH1	PWM
PA9		TIM1_CH2	PWM	UART_RX
PA10	TIM1_CH3	PWM	UART_TX
PA7		TIM1_CH1N	PWM
PB0		TIM1_CH2N	PWM
PB1		TIM1_CH3N	PWM

PD2		ENGATE
PC4		OCFAULT
PB9		LED DEBUG
*/
void driver_enable(void* p)
{
	driver_output(ptr, F16(0.5), F16(0.5), F16(0.5));
	GPIOD->BSRRH = GPIO_Pin_2; 
} 
void driver_disable(void*p)
{ 
	GPIOD->BSRRL = GPIO_Pin_2; 
} 
void driver_ledon(void* p)
{
	GPIOB->BSRRL = GPIO_Pin_9;
}
void driver_ledoff(void* p)
{
	GPIOB->BSRRH = GPIO_Pin_9;
}
static void pwm_io_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	driver_disable(0);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

//	TO DO, we implement the OC by query
// 	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource4);
// 
// 	EXTI_InitTypeDef   EXTI_InitStructure;
// 	EXTI_InitStructure.EXTI_Line = EXTI_Line15;
// 	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
// 	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
// 	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
// 	EXTI_Init(&EXTI_InitStructure);
// 
// 	NVIC_InitTypeDef   NVIC_InitStructure;
// 	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
// 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
// 	NVIC_Init(&NVIC_InitStructure);
// 
// 	EXTI_ClearITPendingBit(EXTI_Line15);
}
static void pwm_tim1_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM1);

	TIM_DeInit(TIM1);

	/* Time Base configuration */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseStructure.TIM_Period = PWM_COUNT;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	/* Automatic Output enable, Break, dead time and lock configuration*/
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime = PWM_DEADTIME_NS * (SYS_FREQUENCY / 1000000) / 1000;
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = PWM_COUNT / 2;
	/* Main Output Enable */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
	TIM_UpdateRequestConfig(TIM1, TIM_UpdateSource_Global);
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
#if 0
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
#endif
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_Cmd(TIM1, ENABLE);
}
void pwm_output(fix16_t a, fix16_t b, fix16_t c)
{
	TIM1->CCR1 = fix16_to_int(fix16_mul(F16(PWM_COUNT), a));
	TIM1->CCR2 = fix16_to_int(fix16_mul(F16(PWM_COUNT), b));
	TIM1->CCR3 = fix16_to_int(fix16_mul(F16(PWM_COUNT), c));
}
/**********************QEP*************************/
static struct{
	volatile int64_t position;
	volatile int64_t offset;
	uint16_t last_cnt;

	//M/T function
	uint16_t last_delta_time;
	int64_t last_spd_pos;
	volatile int16_t T;
	int32_t	speed;
}qep;

/*
PA6	TIM3_CH1	FB0_A
PA7	TIM3_CH2	FB0_B
PB0 TIM3_CH3	FB0_I/LATCH
PB1 TIM3_CH4	PERIOD POSITION LATCH
*/
void qep_tim3_init()
{
	qep.position = 0;
	qep.offset = VALUE_NOT_USEABLE;
	qep.last_cnt = 0;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);

	/* Timer configuration in Encoder mode */
	TIM_DeInit(TIM3);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	//TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
	TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);


	TIM_ICInitTypeDef ch3;
	ch3.TIM_Channel = TIM_Channel_3;
	ch3.TIM_ICPolarity = TIM_ICPolarity_Rising;
	ch3.TIM_ICSelection = TIM_ICSelection_DirectTI;
	ch3.TIM_ICPrescaler = 0;
	ch3.TIM_ICFilter = 0;
	TIM_ICInit(TIM3, &ch3);
	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);

	/*This channel is configured to capture the signal from tim1_ch4 to latch time the signal comes*/
	TIM_ICInitTypeDef ch4;
	ch4.TIM_Channel = TIM_Channel_4;
	ch4.TIM_ICPolarity = TIM_ICPolarity_Rising;
	ch4.TIM_ICSelection = TIM_ICSelection_DirectTI;
	ch4.TIM_ICPrescaler = TIM_ICPSC_DIV1; //every edge
	ch4.TIM_ICFilter = 0;
	TIM_ICInit(TIM3, &ch4);

	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM3->CNT = 0;
	TIM_Cmd(TIM3, ENABLE);
}
void TIM3_IRQHandler() //index
{
	if (TIM_GetITStatus(TIM3, TIM_IT_CC3))
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
		TIM_ITConfig(TIM3, TIM_IT_CC3, DISABLE);
		irq_disable();
		int16_t delta = TIM3->CCR3 - qep.last_cnt;
		qep.offset = -qep.position - delta;
		fbk_t * pFeedback = (fbk_t*)feedback;
		pFeedback->status.index_found = 1;
		irq_enble();
	//	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	}
}
void qep_update()
{
	uint16_t new_cnt = TIM3->CNT;
	int16_t delta = new_cnt - qep.last_cnt;
	qep.last_cnt = new_cnt;
	qep.position += delta;
}

/*
PC6 TIM8_CH1	FB0_A
PC7 TIM8_CH2	FB0_B
PC8 TIM8_CH3	Reverse for hall mode to measure speed
PC9 TIM8_CH4	PERIOD SPEED LATCH
*/
void qep_spd_tim8_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Timer configuration in Encoder mode */
	TIM_DeInit(TIM8);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	
	TIM8->CR2 |= TIM_CR2_TI1S; //XOR mode
	TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Reset);
	TIM_SelectInputTrigger(TIM8, TIM_TS_TI1F_ED);

	/*This channel is configured to capture the signal from tim1_ch4 to latch time the signal comes*/
	TIM_ICInitTypeDef ch4;
	ch4.TIM_Channel = TIM_Channel_4;
	ch4.TIM_ICPolarity = TIM_ICPolarity_Rising;
	ch4.TIM_ICSelection = TIM_ICSelection_DirectTI;
	ch4.TIM_ICPrescaler = TIM_ICPSC_DIV1; //every edge
	ch4.TIM_ICFilter = 0;
	TIM_ICInit(TIM8, &ch4);



#if 0
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ClearFlag(TIM8, TIM_IT_CC4);
	TIM_ITConfig(TIM8, TIM_IT_CC4, ENABLE);
#endif

	TIM8->CNT = 0;
	TIM_Cmd(TIM8, ENABLE);
}
void qep_speed()
{
	if (!TIM_GetFlagStatus(TIM3, TIM_FLAG_CC4) || !TIM_GetFlagStatus(TIM8, TIM_FLAG_CC4))
		return;

	TIM_ClearFlag(TIM3, TIM_FLAG_CC4);
	TIM_ClearFlag(TIM8, TIM_FLAG_CC4);

	uint16_t new_pos = TIM3->CCR4;
	int16_t M = new_pos - qep.last_spd_pos;

	uint16_t time_cnt = TIM8->CCR4;
	int16_t delta_time = (qep.last_delta_time - time_cnt);
	qep.last_delta_time = time_cnt;
	qep.T += (int16_t)(NS_PER_SEC / PWM_FREQUENCY + delta_time * NS_PER_SEC / SYS_FREQUENCY);

	if (M > 10 || M < -10 || qep.T > NS_PER_SEC / 20)
	{
		qep.last_spd_pos = new_pos;
		qep.speed = (int64_t)M * NS_PER_SEC / qep.T;
		qep.T = 0;
	}
	
/*	if (cnt > 1 && (qep.speed > 100 || qep.speed < -100))
	{
		printf("M %d T %d cnt %d delta_time %d last_delta_time %d time_cnt %d\r\n ", qep.M, qep.T, cnt, delta_time, qep.last_delta_time,time_cnt);
	}*/
}

/*********************HALL*************************/
/*PA0 PA1 PA2 for hall U V W*/
void hall_io_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/********************RS485*************************/
/* PB10		UART3 TX
   PB11		UART3 RX
   PA5		RS485_DE
   PB9		RS485_DE */

uint8_t uart3_rx[11] = {1,2,3,4,5,6,7,8,9,10,0};
#define RS_485_RD		GPIOB->BSRRL = GPIO_Pin_9
#define RS_485_WR		GPIOB->BSRRL = GPIO_Pin_9
static struct{
	volatile int64_t position;
	uint32_t last_pos;
	int32_t speed;
	int32_t last_spd[3];

	uint32_t last_spd_pos;
	volatile int32_t T;
} abs_encoder;
void rs485_uart3_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 2500000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART3->CR1 |= USART_CR1_OVER8;
	USART_Init(USART3, &USART_InitStructure);

	//USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	//USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

// 	/* ##DMA1 CH2 for UART3 TX*/
 	DMA_InitTypeDef DMA_InitStructure;
// 	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->TDR); 
// //	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)SRC_Const_Buffer;
// 	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
// 	DMA_InitStructure.DMA_BufferSize = 0;
// 	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
// 	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
// 	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
// 	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
// 	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
// 	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
// 	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
// 	DMA_Init(DMA1_Channel2, &DMA_InitStructure);
// 	DMA_Cmd(DMA1_Channel2, DISABLE);

	/* ##DMA1 CH3 for UART3 RX*/
	DMA_DeInit(DMA1_Stream3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR); 
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uart3_rx;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; 
	DMA_InitStructure.DMA_BufferSize = ABS_RETURN_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	//DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);

	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(DMA1_Stream3, ENABLE);
	USART_Cmd(USART3, ENABLE);
}
void rs485_uart3_request()
{
	DMA1_Stream2->NDTR = 2;
	//DMA1_Stream2->MAR = 2;
	//DMA1_Stream2->CCR |= 1;
}
void restart_usart3(void)
{
	//DMA1_Stream3->CCR &= (uint32_t)(~DMA_CCR_EN); /* Disable DMA Channel 1 to write in CNDTR*/
	DMA1_Stream3->NDTR = ABS_RETURN_LEN; /* Reload the number of DMA tranfer to be performs on channel 1 */
	//DMA1_Stream3->CCR |= DMA_CCR_EN; /* Enable again DMA Channel 1 */
}
uint8_t error_cnt=0;

#define ABS(x) ((x) > 0 ? (x) : (-(x)))
void rs485_get_abs_data()
{
	static uint32_t tick = 1;
	tick++;
	char pos = motor.encoder.motor | motor.encoder.pos;
	if (tick % 4 == 0)
	{
		RS_485_WR; 
		restart_usart3();
		if ((pos & ABS_ENCODER) == ABS_ENCODER)
			USART3->DR = DATA_3;
		else if ((pos & PANASONIC_QEP_ENCODER) == PANASONIC_QEP_ENCODER)
			USART3->DR = ID_A;
		for (int i = 0; i < 36; ++i)
			__asm("nop");
		uint32_t clk = GET_CORE_CLK();
		abs_encoder.T += clk * ( NS_PER_SEC / 1000000) / (SYS_FREQUENCY / 1000000);
		CORE_CLK_RESET();
		RS_485_RD;
	}
	else if (tick % 4 == 2)
	{
		uint32_t abs_pos;
		uint32_t ppr;
		if ((pos & ABS_ENCODER) && (uart3_rx[10] == crcFast(uart3_rx, 10)))
		{
			if (motor.encoder.resolution == RESOLUTION_23BITS)
			{
				abs_pos = ((((uint32_t)uart3_rx[7] & 0xff) << 31) + (((uint32_t)uart3_rx[6] & 0xff) << 23) + ((((uint32_t)uart3_rx[4] & 0xff) << 16) + (((uint32_t)uart3_rx[3] & 0xff) << 8) + ((uint32_t)uart3_rx[2] & 0xff)));
				ppr = 8388608;
			}
			else if (motor.encoder.resolution == RESOLUTION_17BITS)
			{
				abs_pos = (((((uint32_t)uart3_rx[7] & 0xff) << 8) + (uart3_rx[6] & 0xff)) << 17) + (((((uint32_t)uart3_rx[4] & 0xff) << 16) + (((uint32_t)uart3_rx[3] & 0xff) << 8) + ((uint32_t)uart3_rx[2] & 0xff)));
				ppr = 131072;
			}
		}
		else if ((pos & PANASONIC_QEP_ENCODER) && (uart3_rx[8] == crcFast(uart3_rx, 8)))
		{
			if (motor.encoder.resolution == RESOLUTION_20BITS)
			{
				if ((uart3_rx[7] & 0x80) == 0)
					abs_pos = ((uint32_t)0x0 << 24) + (((((uint32_t)uart3_rx[7] & 0xff) << 16) + (((uint32_t)uart3_rx[6] & 0xff) << 8) + ((uint32_t)uart3_rx[5] & 0xff)));
				else
					abs_pos = ((uint32_t)0xff << 24) + (((((uint32_t)uart3_rx[7] & 0xff) << 16) + (((uint32_t)uart3_rx[6] & 0xff) << 8) + ((uint32_t)uart3_rx[5] & 0xff)));
				ppr = 1048576;
			}
		}
		int32_t delta_pos = abs_pos - abs_encoder.last_pos;

		char valid = 0;
		static int delta_tick = 0;
		if (ABS(delta_pos) > ppr / 100)
		{
			delta_tick += 1;
			if (delta_tick > 3)
			{
				delta_tick = 0;
				valid = 1;
			}
		}
		else {
			valid = 1;
			delta_tick = 0;
		}

		if (valid)
		{
			abs_encoder.last_pos = abs_pos;
			abs_encoder.position += delta_pos;

			int32_t M = abs_pos - abs_encoder.last_spd_pos;
			//M/T
			if (M > 10 || M < -10 || abs_encoder.T > NS_PER_SEC / 10)
			{
				abs_encoder.last_spd_pos = abs_pos;
				int32_t spd = (int64_t)M * NS_PER_SEC / abs_encoder.T;
				abs_encoder.last_spd[2] = abs_encoder.last_spd[1];
				abs_encoder.last_spd[1] = abs_encoder.last_spd[0];
				abs_encoder.last_spd[0] = spd;
				spd = abs_encoder.last_spd[0] + abs_encoder.last_spd[1] + abs_encoder.last_spd[2];
				abs_encoder.speed = spd / 3;

				abs_encoder.T = 0;
			}
			crc_ok++;
		}
	}
	else
	{
		crc_error++;
	}
}

int64_t encoder_position(encoder_t encoder)
{
	if (encoder == ABZ_ENCODER || encoder == AB_ENCODER)
		return  qep.position;
	else if (encoder == ABS_ENCODER || encoder == PANASONIC_QEP_ENCODER)
		return abs_encoder.position;
	else if (encoder == HALL_ENCODER)
		return hall_position(&(motor.hall_init.hall),1);

	return 0;
}
int64_t encoder_offset(encoder_t encoder)
{
	if (encoder == ABZ_ENCODER || encoder == AB_ENCODER)
		return qep.offset;
	else
		return 0;
}
char encoder_has_offset(encoder_t encoder)
{
	if (encoder == ABZ_ENCODER || encoder == AB_ENCODER)
		return qep.offset != VALUE_NOT_USEABLE;
	return 1;
}
void encoder_clear_offset(encoder_t encoder)
{
	if (encoder == ABZ_ENCODER || encoder == AB_ENCODER)
	{
		qep.offset = VALUE_NOT_USEABLE;
		TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	}
}
int32_t encoder_speed(encoder_t encoder)
{
	if (encoder == ABZ_ENCODER || encoder == AB_ENCODER)
		return qep.speed;
	else if (encoder == ABS_ENCODER)
		return abs_encoder.speed;
	return 0;
}
int32_t encoder_ppr(encoder_t encoder)
{
	return motor.parameter.ppr;
}

/**********************ADC*************************/
/*	PA0	A0
PA6 A6
PC1 A11	VBUS3
PC2 A12 TMP1
PC3	A13 TMP2 NOT support
PA2	A2	iv
PA3	A3	iu
*/
#define ADC_BUFF_LEN	1
struct adc_dma_t {
 	uint16_t vbus;;
 	uint16_t tmp;	
} adc_dma[1];
static int16_t IAB_OFFSET[2] = { 2048, 2120 };
static int16_t ia_A;
static int16_t ib_A;
static void adc_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	DMA_DeInit(DMA2_Stream0);
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;                   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)0x40012308; 
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adc_dma;   
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;       
	DMA_InitStructure.DMA_BufferSize = sizeof(adc_dma) / 2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;        
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;       
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);

// 	NVIC_InitTypeDef NVIC_InitStructure;
// 	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;     
// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
// 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
// 	NVIC_Init(&NVIC_InitStructure);
// 	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE);

	ADC_DeInit();
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_RegSimult;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1; 
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitTypeDef       ADC_InitStructure;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;    
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 
	ADC_InitStructure.ADC_NbrOfConversion = sizeof(adc_dma) / 4;

	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);

	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);


	ADC_InjectedSequencerLengthConfig(ADC1, ADC_BUFF_LEN);
	for (int i = 0; i < ADC_BUFF_LEN; ++i)
		ADC_InjectedChannelConfig(ADC1, ADC_Channel_2, i + 1, ADC_SampleTime_15Cycles);
	ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
	ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Rising);

	ADC_InjectedSequencerLengthConfig(ADC2, ADC_BUFF_LEN);
	for (int i = 0; i < ADC_BUFF_LEN; ++i)
		ADC_InjectedChannelConfig(ADC2, ADC_Channel_3, i + 1, ADC_SampleTime_15Cycles);
	ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T1_TRGO);
	ADC_ExternalTrigInjectedConvEdgeConfig(ADC2, ADC_ExternalTrigInjecConvEdge_Rising);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE); //interrupt

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);


	/* Enable DMA request after last transfer (Multi-ADC mode)  */
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);  
	//ADC_SoftwareStartConv(ADC1);
}

void ADC_IRQHandler(void)  //max: 1742 clock about 24.2us
{
	driver_ledon(0);

	if (ADC_GetITStatus(ADC1, ADC_IT_JEOC) == SET)
	{
		ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);

		uint16_t adc1[ADC_BUFF_LEN], adc2[ADC_BUFF_LEN];

#if ADC_BUFF_LEN >= 1
		adc1[0] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
		adc2[0] = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
#endif
#if ADC_BUFF_LEN >= 2
		adc1[1] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
		adc2[1] = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_2);
#endif
#if ADC_BUFF_LEN >= 3
		adc1[2] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);
		adc2[2] = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_3);
#endif
#if ADC_BUFF_LEN >= 4
		adc1[3] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_4);
		adc2[3] = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_4);
#endif
		static uint8_t adc_offset_cnt = 0;
		if (adc_offset_cnt < 100)
		{
			static int32_t offset_sum1 = 0, offset_sum2 = 0;
			adc_offset_cnt++;
			for (int j = 0; j < ADC_BUFF_LEN; ++j) {
				offset_sum1 += adc1[j];
				offset_sum2 += adc2[j];
			}
			if (adc_offset_cnt == 100)
			{
				adc_offset_cnt = 0xff;
				IAB_OFFSET[0] = offset_sum1 / ADC_BUFF_LEN / 100;
				IAB_OFFSET[1] = offset_sum2 / ADC_BUFF_LEN / 100;
			}
		}
		else
		{
			int32_t sum1 = 0, sum2 = 0;
			for (int i = 0; i < ADC_BUFF_LEN; ++i)
			{
				sum1 += adc1[i] - IAB_OFFSET[0];
				sum2 += adc2[i] - IAB_OFFSET[1];
			}
			ia_A = - sum1 * (3300 * 1000 / 4096 / 66 / ADC_BUFF_LEN);
			ib_A = - sum2 * (3300 * 1000 / 4096 / 66 / ADC_BUFF_LEN);
			//board_update();
		}
	}

	driver_ledoff(0);
}

int16_t driver_ia(void* self)
{
	return ia_A;
}
int16_t driver_ib(void* self)
{
	return ib_A;
}
int16_t driver_ibus(void* self)
{
	return 0;;// adc_dma->ibus;
}


/********************STATUS***********************/

uint8_t driver_enabled(void)
{
	/*0 enable 1 disable*/
	if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_12))
		return 0;
	else
		return 1;
}

/***********************SPI to A8************************/
/*PB3 PB4 PB5 for spi sck miso mosi*/
/*PA12 is sync pin from a8*/
/*##DMA2 CH1 for SPI3 RX*/
/*##DMA2 CH2 for SPI3 TX*/
volatile uint8_t pos_latched = 0;
uint16_t spi3_cr1, spi3_cr2;
uint32_t  dma1_cr, dma2_cr, dma1_cndtr, dma2_cndtr, dma1_cpar, dma2_cpar, dma1_cmar, dma2_cmar;

void spi3_init()
{
	DMA_InitTypeDef DMA_InitStructure_ch1;
	DMA_InitTypeDef DMA_InitStructure_ch2;
	SPI_InitTypeDef  SPI_InitStructure;

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* SPI  MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3);


	/* SPI NSS pin configuration */
	//GPIO_InitStructure.GPIO_Pin = SPIx_NSS_PIN;
	//GPIO_Init(SPIx_NSS_GPIO_PORT, &GPIO_InitStructure);

	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPI3);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 0x1021;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	SPI_Init(SPI3, &SPI_InitStructure);

	//SPI_RxFIFOThresholdConfig(SPI3, SPI_RxFIFOThreshold_HF);

	/* DMA Configuration -------------------------------------------------------*/

	DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure_ch1.DMA_PeripheralBaseAddr = (uint32_t)&SPI3->DR;
	DMA_InitStructure_ch1.DMA_Memory0BaseAddr = (uint32_t)command;
	DMA_InitStructure_ch1.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure_ch1.DMA_BufferSize = 128 /2;
	DMA_InitStructure_ch1.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure_ch1.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure_ch1.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure_ch1.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure_ch1.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure_ch1.DMA_Priority = DMA_Priority_VeryHigh;
	//DMA_InitStructure_ch1.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA2_Stream1, &DMA_InitStructure_ch1);

	DMA_DeInit(DMA2_Stream2);
	DMA_InitStructure_ch2.DMA_PeripheralBaseAddr = (uint32_t)&SPI3->DR;
	DMA_InitStructure_ch2.DMA_Memory0BaseAddr = (uint32_t)feedback;
	DMA_InitStructure_ch2.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure_ch2.DMA_BufferSize = 126/2;
	DMA_InitStructure_ch2.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure_ch2.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure_ch2.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure_ch2.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure_ch2.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure_ch2.DMA_Priority = DMA_Priority_VeryHigh;
	//DMA_InitStructure_ch2.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA2_Stream2, &DMA_InitStructure_ch2);

	//SPI_NSSPulseModeCmd(SPI3, ENABLE);
	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
	SPI_I2S_ClearFlag(SPI3, SPI_I2S_FLAG_TXE);
	SPI_I2S_ClearFlag(SPI3, SPI_I2S_FLAG_RXNE);
	//SPI_CRCLengthConfig(SPI3, SPI_CRCLength_16b);
	SPI_CalculateCRC(SPI3, DISABLE);
	SPI_CalculateCRC(SPI3, ENABLE);

// 	dma1_cr = DMA2_Stream1->CCR;
// 	dma1_cndtr = DMA2_Stream1->CNDTR;
// 	dma1_cpar = DMA2_Stream1->CPAR;
// 	dma1_cmar = DMA2_Stream1->CMAR;
// 	dma2_cr = DMA2_Stream2->CCR;
// 	dma2_cndtr = DMA2_Stream2->CNDTR;
// 	dma2_cpar = DMA2_Stream2->CPAR;
// 	dma2_cmar = DMA2_Stream2->CMAR;
	spi3_cr1 = SPI3->CR1;
	spi3_cr2 = SPI3->CR2;

	//SPI_NSSInternalSoftwareConfig(SPI3, SPI_NSSInternalSoft_Set);
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DMA_Cmd(DMA2_Stream2, ENABLE);
	
	SPI_Cmd(SPI3, ENABLE);
}
void sync_from_a8_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource12);

	EXTI_InitTypeDef   EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitTypeDef   NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	EXTI_ClearITPendingBit(EXTI_Line12);
}
void  EXTI15_10_IRQHandler(void) // max: 499 clock about 7us
{
	//sync from a8
	if (EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
		SPI3->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);
		SPI3->CR2 &= (uint16_t)~((uint16_t)(SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx));
		cmd_t* pCommand = (cmd_t*)command;
		if ((pCommand->crc == SPI3->RXCRCR) && (!motor.state.calibrating))
		{
				/*feedback update*/
				fbk_t * pFeedback = (fbk_t*)feedback;
				cmd_t * pCommand = (cmd_t*)command;
				if (pCommand->ctl_word.index_confirm == 1) // index confirmed by A8
					pFeedback->status.index_found = 0;
				pFeedback->torque = motor_torque(&motor);
				pFeedback->speed = (int32_t)encoder_speed(motor.encoder.motor);
				if ((pos_latched == 0) && (pFeedback->status.index_found == 0))
					pFeedback->position = (int32_t)encoder_position(motor.encoder.pos);
				else if (encoder_has_offset(motor.encoder.pos))
					pFeedback->position = (int32_t)encoder_offset(motor.encoder.pos);
				pFeedback->status.pos_latched_value = pos_latched;
				pos_latched = 0;
				pFeedback->status.enabled = motor.state.enabled;
				pFeedback->status.saturation = motor.state.saturation;
				pFeedback->error_code = motor_fault(&motor);

				motor_set_target(&motor, pCommand->torque);
				motor_set_vbus(&motor, pCommand->vbus);
				motor.direct_pwm = 0;
				if (pCommand->ctl_word.direct_pwm == 1)
					motor_direct_output(&motor, pCommand->pwm);
				if (pCommand->ctl_word.enable == 0)
					motor_disable(&motor);
				else
					motor_enable(&motor);

				
				motor.life_cnt = 0;
		}
		else
		{
			//++feedback.resvd;
			SPI_I2S_ClearFlag(SPI3, SPI_FLAG_CRCERR);
		}

		//DMA_DeInit(DMA2_Channel1);
		//DMA2_Channel1->CCR = 0;
		//DMA2->IFCR |= ((uint32_t)(DMA_ISR_GIF1 | DMA_ISR_TCIF1 | DMA_ISR_HTIF1 | DMA_ISR_TEIF1));

		//DMA2_Channel2->CCR = 0;
		//DMA2->IFCR |= ((uint32_t)(DMA_ISR_GIF2 | DMA_ISR_TCIF2 | DMA_ISR_HTIF2 | DMA_ISR_TEIF2));

		//DMA_DeInit(DMA2_Channel2);
		//SPI_I2S_DeInit(SPI3);
		//RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, ENABLE);
		RCC->APB1RSTR |= RCC_APB1Periph_SPI3;
		/* Release SPI3 from reset state */
		//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, DISABLE);
		RCC->APB1RSTR &= ~RCC_APB1Periph_SPI3;

		SPI3->CR1 = spi3_cr1;
		SPI3->CR2 = spi3_cr2;
		SPI3->I2SCFGR &= (uint16_t)~((uint16_t)SPI_I2SCFGR_I2SMOD);
		SPI3->CRCPR = 0x1021;
// 
// 		DMA2_Channel1->CCR = dma1_cr;
// 		DMA2_Channel1->CNDTR = dma1_cndtr;
// 		DMA2_Channel1->CPAR = dma1_cpar;
// 		DMA2_Channel1->CMAR = dma1_cmar;
// 		DMA2_Channel2->CCR = dma2_cr;
// 		DMA2_Channel2->CNDTR = dma2_cndtr;
// 		DMA2_Channel2->CPAR = dma2_cpar;
// 		DMA2_Channel2->CMAR = dma2_cmar;
// 		
// 
// 		DMA2_Channel1->CCR |= DMA_CCR_EN;
// 		DMA2_Channel2->CCR |= DMA_CCR_EN;
		SPI3->CR1 |= SPI_CR1_SPE;

		//EXTI_ClearITPendingBit(EXTI_Line12);
		*(__IO uint32_t *) (((uint32_t)&(EXTI->PR)) + ((EXTI_Line12) >> 5) * 0x20) = (1 << (EXTI_Line12 & 0x1F));
		return;
	}
	//IPU fault
	if (EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
		driver_disable(0);
		printk("hardware fault!!!\r\n");
		EXTI_ClearITPendingBit(EXTI_Line15);
		return;
	}
}
void parameter_sync(void)
{
// 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
// 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
// 
// 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
// 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
// 	spi3_init();
// 	GPIO_InitTypeDef GPIO_InitStructure;
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
// 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);
// 
// 	uint16_t receive_success = 0;
// 	uint32_t time_out = 0;
// 	while (1)
// 	{
// 		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12))
// 		{
// 			SPI3->CR1 &= (uint16_t)~((uint16_t)SPI_CR1_SPE);
// 			SPI3->CR2 &= (uint16_t)~((uint16_t)(SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx));
// 			motor_para_t* p = (motor_para_t*)command;
// 			if ((p->crc == SPI3->RXCRCR) && (p->crc !=  0))
// 			{
// 				if (p->function == CONFIG_PARAMETER)
// 				{
// 					motor_para_t * pback = (motor_para_t*)feedback;
// 
// 					//parameters
// 					motor.parameter.electric_offset = p->parameter.electric_offset;
// 					motor.parameter.invert = p->parameter.invert;
// 					motor.parameter.poles = p->parameter.poles;
// 					motor.parameter.inductance = p->parameter.inductance / 1000000.0;
// 					motor.parameter.resistance = p->parameter.resistance / 1000000.0;
// 
// 
// 					motor.parameter.ppr = p->encoder.motor_ppr;
// 					motor.parameter.bandwith = p->pid.bandwith;
// 					//encoder
// 					motor.encoder.init = p->encoder.encoder_init;
// 					motor.encoder.pos = p->encoder.encoder_pos;
// 					motor.encoder.spd = p->encoder.encoder_spd;
// 					motor.encoder.motor = p->encoder.encoder_spd;
// 
// 					char pos = motor.encoder.motor | motor.encoder.pos;
// 					if (pos & ABS_ENCODER)
// 					{
// 						if (p->encoder.motor_ppr == 131072 || p->encoder.position_ppr == 131072) //17bits
// 							motor.encoder.resolution = RESOLUTION_17BITS;
// 						if (p->encoder.motor_ppr == 8388608 || p->encoder.position_ppr == 8388608) //17bits
// 							motor.encoder.resolution = RESOLUTION_23BITS;
// 					}
// 					else if (pos & PANASONIC_QEP_ENCODER)
// 					{
// 						if (p->encoder.motor_ppr == 1048576 || p->encoder.position_ppr == 1048576) //17bits
// 							motor.encoder.resolution = RESOLUTION_20BITS;
// 					}
// 					if (p->pos_latch.pos_latch_function == 1)
// 						pos_latch_init(p->pos_latch.pos_latch_type); //trigger rising for initial
// 
// 					board_parameter_reset();
// 					receive_success = 1;
// 					memcpy(pback, p, 126);
// 				}
// 				else if (p->function == CALIBRATION)
// 				{
// 					motor.parameter.ppr = p->encoder.motor_ppr;
// 					motor.parameter.bandwith = p->pid.bandwith;
// 					//encoder
// 					motor.encoder.init = p->encoder.encoder_init;
// 					motor.encoder.pos = p->encoder.encoder_pos;
// 					motor.encoder.spd = p->encoder.encoder_spd;
// 					motor.encoder.motor = p->encoder.encoder_spd;
// 
// 					uint16_t pos = motor.encoder.motor | motor.encoder.pos;
// 					if (pos & ABS_ENCODER)
// 					{
// 						if (p->encoder.motor_ppr == 131072 || p->encoder.position_ppr == 131072) //17bits
// 							motor.encoder.resolution = RESOLUTION_17BITS;
// 						if (p->encoder.motor_ppr == 8388608 || p->encoder.position_ppr == 8388608) //17bits
// 							motor.encoder.resolution = RESOLUTION_23BITS;
// 					}
// 					else if (pos & PANASONIC_QEP_ENCODER)
// 					{
// 						if (p->encoder.motor_ppr == 1048576 || p->encoder.position_ppr == 1048576) //17bits
// 							motor.encoder.resolution = RESOLUTION_20BITS;
// 					}
// 
// 					if (!motor.calibrated)
// 					{
// 						RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
// 						RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
// 						pwm_tim1_init();
// 						pwm_io_init();
// 
// 						RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div1);
// 						RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
// 						adc_init();
// 
// 						RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
// 						rs485_uart3_init();
// 
// 						RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
// 						qep_tim3_init();
// 
// 						if (SysTick_Config((SystemCoreClock) / 1000))
// 						{
// 							/* Capture error */
// 							while (1);
// 						}
// 						NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
// 						NVIC_SetPriority(SysTick_IRQn, 0xff);
// 
// 						pmsm_calibrate(&motor, p->calibrate_ma);
// 					}
// 					//parameters
// 					motor_para_t* pback = (motor_para_t*)feedback;
// 					pback->ctrl_word = 5;
// 					pback->function = CALIBRATION;
// 					pback->parameter.electric_offset = motor.parameter.electric_offset;
// 					pback->parameter.invert = motor.parameter.invert;
// 					pback->parameter.poles = motor.parameter.poles;
// 					pback->parameter.inductance = motor.parameter.inductance * 1000000;
// 					pback->parameter.resistance = motor.parameter.resistance * 1000000;
// 					receive_success = 1;
// 				}
// 				else if (p->function == UPDATE_FIRMWARE)
// 				{
// 					FLASH_Unlock();
// 					FLASH_ErasePage(PROGRAM_UPDATE_FLAG_ADDRESS);
// 					FLASH_ProgramWord(PROGRAM_UPDATE_FLAG_ADDRESS+ 0x400, FLASH_UPDATE);
// 					FLASH_Lock();
// 
// 					motor_para_t* pback = (motor_para_t*)feedback;
// 					memcpy(pback, p, 126);
// 
// 					receive_success = 2;
// 
// 				}
// 			}
// 			else
// 			{
// 				SPI_I2S_ClearFlag(SPI3, SPI_FLAG_CRCERR);
// 			}
// 
// 			DMA2_Channel1->CCR = 0;
// 			DMA2->IFCR |= ((uint32_t)(DMA_ISR_GIF1 | DMA_ISR_TCIF1 | DMA_ISR_HTIF1 | DMA_ISR_TEIF1));
// 
// 			DMA2_Channel2->CCR = 0;
// 			DMA2->IFCR |= ((uint32_t)(DMA_ISR_GIF2 | DMA_ISR_TCIF2 | DMA_ISR_HTIF2 | DMA_ISR_TEIF2));
// 
// 			//DMA_DeInit(DMA2_Channel2);
// 			//SPI_I2S_DeInit(SPI3);
// 			//RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, ENABLE);
// 			RCC->APB1RSTR |= RCC_APB1Periph_SPI3;
// 			/* Release SPI3 from reset state */
// 			//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, DISABLE);
// 			RCC->APB1RSTR &= ~RCC_APB1Periph_SPI3;
// 
// 			SPI3->CR1 = spi3_cr1;
// 			SPI3->CR2 = spi3_cr2;
// 			SPI3->I2SCFGR &= (uint16_t)~((uint16_t)SPI_I2SCFGR_I2SMOD);
// 			SPI3->CRCPR = 0x1021;
// 
// 			DMA2_Channel1->CCR = dma1_cr;
// 			DMA2_Channel1->CNDTR = dma1_cndtr;
// 			DMA2_Channel1->CPAR = dma1_cpar;
// 			DMA2_Channel1->CMAR = dma1_cmar;
// 			DMA2_Channel2->CCR = dma2_cr;
// 			DMA2_Channel2->CNDTR = dma2_cndtr;
// 			DMA2_Channel2->CPAR = dma2_cpar;
// 			DMA2_Channel2->CMAR = dma2_cmar;
// 
// 
// 			DMA2_Channel1->CCR |= DMA_CCR_EN;
// 			DMA2_Channel2->CCR |= DMA_CCR_EN;
// 			SPI3->CR1 |= SPI_CR1_SPE;
// 			time_out = 0;
// 		}
// 		else if (receive_success == 1)
// 		{
// 			for (uint32_t i = 0; i < 100; i++); // about 10us
// 			time_out++;
// 			if (time_out > 10000)
// 			{
// 				dma1_cndtr = 8;
// 				dma2_cndtr = 7;
// 				break;
// 			}
// 		}
// 		else if (receive_success == 2)
// 		{
// 			for (uint32_t i = 0; i < 300; i++); // about 10us
// 			time_out++;
// 			if (time_out > 10000)
// 			{
// 				irq_disable();
// 				motor_disable(&motor);
// 				for (uint16_t i = 0; i < 1000; i++);
// 				__disable_fault_irq();
// 				NVIC_SystemReset();
// 				break;
// 			}
// 		}
// 	}
}

/*********************pos latch**************************/
void pos_latch_init(int16_t trigger_type)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); //PB8 is the switch control for latch and TIM3CH3

	GPIO_ResetBits(GPIOB, GPIO_Pin_8);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);

	EXTI_InitTypeDef   EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	if (trigger_type == TRIGGER_RISING)
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	else if (trigger_type == TRIGGER_FALLING)
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;

	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitTypeDef   NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	EXTI_ClearITPendingBit(EXTI_Line0);
}

void  EXTI0_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		qep_update();
		fbk_t * pFeedback = (fbk_t*)feedback;
		pos_latched = 1;
		if (motor.encoder.pos == ABZ_ENCODER)
			pFeedback->position = qep.position;
		else if (motor.encoder.pos == ABS_ENCODER)
			pFeedback->position = abs_encoder.position;
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}


/***************************USART********************/
uint8_t usart1_dma_rx_t[USART_BUFF_LEN];
uint8_t usart4_dma_rx_t[USART_BUFF_LEN];
DMA_InitTypeDef  DMA_InitStructure_CH4;
DMA_InitTypeDef  DMA_InitStructure_CH5;
USART_InitTypeDef USART_InitStructure;
void uart1_init()
{
	/*##DMA1 CH4 for UART1 TX*/
	/*##DMA1 CH5 for UART1 TX*/

	USART_Cmd(USART1, DISABLE);
	USART_DeInit(USART1);
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	DMA_InitTypeDef  DMA_InitStructure_CH4;
	DMA_InitTypeDef  DMA_InitStructure_CH5;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);


	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);

// 	/* DMA_Channel5 ---- Rx */
// 	DMA_DeInit(DMA1_Channel5);
// 	DMA_InitStructure_CH5.DMA_PeripheralBaseAddr = (uint32_t)&USART1->RDR;
// 	DMA_InitStructure_CH5.DMA_MemoryBaseAddr = (uint32_t)usart1_dma_rx_t;
// 	DMA_InitStructure_CH5.DMA_DIR = DMA_DIR_PeripheralSRC;
// 	DMA_InitStructure_CH5.DMA_BufferSize = USART_BUFF_LEN;
// 	DMA_InitStructure_CH5.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
// 	DMA_InitStructure_CH5.DMA_MemoryInc = DMA_MemoryInc_Enable;
// 	DMA_InitStructure_CH5.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
// 	DMA_InitStructure_CH5.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
// 	DMA_InitStructure_CH5.DMA_Mode = DMA_Mode_Normal;
// 	DMA_InitStructure_CH5.DMA_Priority = DMA_Priority_VeryHigh;
// 	DMA_InitStructure_CH5.DMA_M2M = DMA_M2M_Disable;
// 	DMA_Init(DMA1_Channel5, &DMA_InitStructure_CH5);
// 
// 	/* DMA_Channel4 ---- Tx */
// 	DMA_DeInit(DMA1_Channel4);
// 	DMA_InitStructure_CH4.DMA_PeripheralBaseAddr = (uint32_t)&USART1->TDR;
// 	DMA_InitStructure_CH4.DMA_DIR = DMA_DIR_PeripheralDST;
// 	DMA_InitStructure_CH4.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
// 	DMA_InitStructure_CH4.DMA_BufferSize = 0;
// 	DMA_InitStructure_CH4.DMA_MemoryInc = DMA_MemoryInc_Enable;
// 	DMA_InitStructure_CH4.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
// 	DMA_InitStructure_CH4.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
// 	DMA_InitStructure_CH4.DMA_Mode = DMA_Mode_Normal;
// 	DMA_InitStructure_CH4.DMA_Priority = DMA_Priority_VeryHigh;
// 	DMA_InitStructure_CH4.DMA_M2M = DMA_M2M_Disable;
// 
// 	DMA_Init(DMA1_Channel4, &DMA_InitStructure_CH4);
// 
// 	USART_DMACmd(USART1, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
// 
// 	DMA_Cmd(DMA1_Channel5, ENABLE);
// 
// 	DMA_Cmd(DMA1_Channel4, ENABLE);
// 
// 	USART_Cmd(USART1, ENABLE);
}

void uart4_init(void)
{
// 	GPIO_InitTypeDef GPIO_InitStructure;
// 	USART_InitTypeDef USART_InitStructure;
// 	DMA_InitTypeDef  DMA2_InitStructure_CH3;
// 	DMA_InitTypeDef  DMA2_InitStructure_CH5;
// 
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
// 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
// 	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_Init(GPIOC, &GPIO_InitStructure);
// 
// 	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_5);
// 	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_5);
// 
// 	USART_InitStructure.USART_BaudRate = 115200;
// 	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
// 	USART_InitStructure.USART_StopBits = USART_StopBits_1;
// 	USART_InitStructure.USART_Parity = USART_Parity_No;
// 	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
// 	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
// 	USART_Init(UART4, &USART_InitStructure);
// 
// 	/* DMA2_Channel3 ---- Rx */
// 	DMA_DeInit(DMA2_Channel3);
// 	DMA2_InitStructure_CH3.DMA_PeripheralBaseAddr = (uint32_t)&UART4->RDR;
// 	DMA2_InitStructure_CH3.DMA_MemoryBaseAddr = (uint32_t)(usart4_dma_rx_t);
// 	DMA2_InitStructure_CH3.DMA_DIR = DMA_DIR_PeripheralSRC;
// 	DMA2_InitStructure_CH3.DMA_BufferSize = USART_BUFF_LEN;
// 	DMA2_InitStructure_CH3.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
// 	DMA2_InitStructure_CH3.DMA_MemoryInc = DMA_MemoryInc_Enable;
// 	DMA2_InitStructure_CH3.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
// 	DMA2_InitStructure_CH3.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
// 	DMA2_InitStructure_CH3.DMA_Mode = DMA_Mode_Circular;
// 	DMA2_InitStructure_CH3.DMA_Priority = DMA_Priority_High;
// 	DMA2_InitStructure_CH3.DMA_M2M = DMA_M2M_Disable;
// 	DMA_Init(DMA2_Channel3, &DMA2_InitStructure_CH3);
// 
// 	/* DMA2_Channel5 ---- Tx */
// 	DMA_DeInit(DMA2_Channel5);
// 	DMA2_InitStructure_CH5.DMA_PeripheralBaseAddr = (uint32_t)&UART4->TDR;
// 	DMA2_InitStructure_CH5.DMA_DIR = DMA_DIR_PeripheralDST;
// 	DMA2_InitStructure_CH5.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
// 	DMA2_InitStructure_CH5.DMA_BufferSize = 0;
// 	DMA2_InitStructure_CH5.DMA_MemoryInc = DMA_MemoryInc_Enable;
// 	DMA2_InitStructure_CH5.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
// 	DMA2_InitStructure_CH5.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
// 	DMA2_InitStructure_CH5.DMA_Mode = DMA_Mode_Normal;
// 	DMA2_InitStructure_CH5.DMA_Priority = DMA_Priority_High;
// 	DMA2_InitStructure_CH5.DMA_M2M = DMA_M2M_Disable;
// 	DMA_Init(DMA2_Channel5, &DMA2_InitStructure_CH5);
// 
// 
// 	USART_DMACmd(UART4, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
// 	DMA_Cmd(DMA2_Channel3, ENABLE);
// 	DMA_Cmd(DMA2_Channel5, ENABLE);
// 	USART_Cmd(UART4, ENABLE);
}

uint8_t dma_send_buf[2][USART_BUFF_LEN];
void usart_dma_send(DMA_Stream_TypeDef *dma_channel, uint8_t* buf, uint8_t size)
{
// 	while ((dma_channel->CNDTR) != 0); //wait for the last transfer end
// 
// 	dma_channel->CCR &= (uint16_t)(~DMA_CCR_EN); // Disable
// 
// 	if (dma_channel == USART1_DMA_SEND){
// 		memcpy(dma_send_buf[0], buf, size);
// 		dma_channel->CMAR = (uint32_t)(dma_send_buf[0]);
// 	}
// 	else if (dma_channel == USART4_DMA_SEND){
// 		memcpy(dma_send_buf[1], buf, size);
// 		dma_channel->CMAR = (uint32_t)(dma_send_buf[1]);
// 	}
// 
// 	dma_channel->CNDTR = size;
// 	dma_channel->CCR |= DMA_CCR_EN; // Enable
}

void usart_receive_dma_reset(DMA_Stream_TypeDef * dma)
{
// 	dma->CCR &= (uint32_t)(~DMA_CCR_EN); /* Disable DMA Channel 1 to write in CNDTR*/
// 	dma->CNDTR = USART_BUFF_LEN; /* Reload the number of DMA tranfer to be performs on channel 1 */
// 	dma->CCR |= DMA_CCR_EN; /* Enable again DMA Channel 1 */
}

#if 0
#ifdef __GNUC__

uint8_t dma_send_buf1[256];
int _write(int fd, char * str, int len)
{
// 	while ((DMA1_Channel4->CNDTR) != 0); //wait for the last transfer end
// 	//irq_disable();
// 	DMA1_Channel4->CCR &= (uint16_t)(~DMA_CCR_EN); // Disable
// 	memcpy(dma_send_buf1, str, len);
// 	DMA1_Channel4->CMAR = (uint32_t)dma_send_buf1;
// 
// 	DMA1_Channel4->CNDTR = len;
// 	DMA1_Channel4->CCR |= DMA_CCR_EN; // Enable
	//irq_enble();
	return len;
}
int _read(int fd, char * str, int len)
{
// 	if (usart1_dma_rx_t[0] != 0xff)
// 	{
// 		((uint8_t*)str)[0] = usart1_dma_rx_t[0] - '0';
// 		usart1_dma_rx_t[0] = 0xff;
// 		return 1;
// 	}
	return 0;
}

int _isatty(int _FileHandle)
{
	return 1;
}
#endif
#else
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t)ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	return ch;
}
#endif

/**********************FLASH**************************/
#define FLASH_SIZE 256 
#define SECTOR_SIZE 2048    
void flash_write_more_data(uint32_t startAddress, uint16_t *writeData, uint16_t countToWrite)
{
	if (startAddress < FLASH_BASE || ((startAddress + countToWrite * 2) >= (FLASH_BASE + 1024 * FLASH_SIZE)))
	{
		return;
	}
	FLASH_Unlock();         
	uint32_t offsetAddress = startAddress - FLASH_BASE;               
	uint32_t sectorPosition = offsetAddress / SECTOR_SIZE;            

	uint32_t sectorStartAddress = sectorPosition*SECTOR_SIZE + FLASH_BASE;   

	FLASH_ErasePage(sectorStartAddress);

	uint16_t dataIndex;
	for (dataIndex = 0; dataIndex < countToWrite; dataIndex++)
	{
		FLASH_ProgramHalfWord(startAddress + dataIndex * 2, writeData[dataIndex]);
	}

	FLASH_Lock();
}

/**********************BOARD*************************/
void board_parameter_reset()
{
	fbk_t * pFeedback = (fbk_t*)feedback;
	pFeedback->status.enabled = 0;
	pFeedback->status.fault = 0;
	pFeedback->status.index_found = 0;
	pFeedback->status.pos_latched_value = 0;
	pFeedback->status.saturation = 0;

	pFeedback->error_code = 0;
	pFeedback->position = 0;
}
void board_init()
{
	/* software init crc table */
	crcInit();

	/* power on the port and dma */
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2D, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	//uart4_init();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	//uart1_init();

#ifndef DEBUG
	parameter_sync();
#endif // !DEBUG

	if (!motor.calibrated)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		pwm_tim1_init();
		pwm_io_init();

		adc_init();

		driver_enable(0);
		while (1);
// 


		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
		qep_spd_tim8_init();

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		rs485_uart3_init();

		//RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		//uart4_init();

		/*RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
		spi3_init();*/

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		qep_tim3_init();

		if (SysTick_Config((SystemCoreClock) / 1000))
		{
			/* Capture error */
			while (1);
		}
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
		NVIC_SetPriority(SysTick_IRQn, 0xff);
	}

	sync_from_a8_init();
}

void power_on_init()
{
// 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
// 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
// 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
// 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
	spi3_init();
	sync_from_a8_init();
}
void usart_token(motor_t* self);
static void board_update()
{
	char pos = motor.encoder.motor | motor.encoder.pos;
#ifndef DEBUG
	if ((pos & ABS_ENCODER) || (pos & PANASONIC_QEP_ENCODER))
		rs485_get_abs_data();
	if (pos & ABZ_ENCODER)
		qep_update();
#else
	rs485_get_abs_data();
	qep_update();
#endif
	switch (motor.encoder.spd){
	case ABZ_ENCODER: qep_speed(); break;
	case ABS_ENCODER: break;
	}
	if (!motor.direct_pwm)
		motor_update(&motor);
}
