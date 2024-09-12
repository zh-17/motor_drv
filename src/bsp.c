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

/* called by ADC DMA finished IRQ
* the main time sequence is PWM CC1 trigger the ADC, DMA collect the ADC sample,
and trigger a IRQ, which call board_update, and update all the control loop.
*/

static uint32_t crc_error = 0;
static uint32_t crc_ok = 0;
volatile uint16_t command[64];//spi
volatile uint16_t feedback[64];//spi tim3
void HardFault_Handler(void)
{
	driver_disable(0);
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}
static void board_update();

/***********************SYS_TICK**************************/
static volatile uint32_t system_tick = 0; // 可以全局调用
void SysTick_Handler(void)
{
	system_tick += 1;
}
void msleep(int16_t ms)
{
	uint32_t wakeup = system_tick + ms;
	while (system_tick < wakeup)
		;
}
uint32_t system_time()
{
	return system_tick;
}






/**********************PWM*************************/
/* hardware config
PA8		TIM1_CH1	PWM
PA9		TIM1_CH2	PWM	UART_RX
PA10	TIM1_CH3	PWM	UART_TX
PA7		TIM1_CH1N	PWM
PB0		TIM1_CH2N	PWM
PB1		TIM1_CH3N	PWM

PD2		ENGATE 启用或禁用
PC4		OCFAULT 故障状态
PB9		LED DEBUG 
*/
#define PWM_COUNT (SYS_FREQUENCY / PWM_FREQUENCY / 2 - 1)  //除以2是因为计数器中心对称
void driver_enable(void *p)
{
	driver_output(ptr, F16(0.5), F16(0.5), F16(0.5));
	GPIOD->BSRRH = GPIO_Pin_2; // PD2		
}
void driver_disable(void *p)
{
	GPIOD->BSRRL = GPIO_Pin_2;// PD2		
}
void driver_ledon(void *p) 
{
	GPIOB->BSRRL = GPIO_Pin_9;//  PB9
}
void driver_ledoff(void *p)
{
	GPIOB->BSRRH = GPIO_Pin_9;
}
static void pwm_io_init()
{
	GPIO_InitTypeDef GPIO_InitStructure; // PD2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;// 引脚输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	driver_disable(0);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // PB9 引脚输出模式
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; // PC4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //引脚输入模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}
static void pwm_tim1_init() 
{
	//6个引脚复用
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //引脚复用
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出(Push-Pull)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 上拉模式
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

	/* Tim1 Base configuration */
	TIM_DeInit(TIM1);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
	TIM_TimeBaseStructure.TIM_Prescaler = 0;								
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1; // 中心对齐计数模式
	TIM_TimeBaseStructure.TIM_Period = PWM_COUNT;	//自动重装载寄存器（ARR，Auto-Reload Register）					
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM_OCInitTypeDef TIM_OCInitStructure; // 输出比较
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                  
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;     // 使能输出比较的输出状态
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;   // 使能输出比较的互补输出状态
	TIM_OCInitStructure.TIM_Pulse = 0;							      //设置捕获比较寄存器（CCR）的值，决定 PWM 占空比
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_High;			
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;			
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;		
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;			
	TIM_OC1Init(TIM1, &TIM_OCInitStructure); 
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	/* Automatic Output enable, Break, dead time and lock configuration*/ //定时器的高级特性
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;							 
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime = PWM_DEADTIME_NS * (SYS_FREQUENCY / 1000000) / 1000;			// 配置死区时间（Dead-Time）
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;                                                // 配置刹车功能为禁用
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;									
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;								// 使能自动输出功能
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	TIM1->CCR1 = 0;//初始化每个通道的占空比
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = PWM_COUNT / 2;

	/* Main Output Enable */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);   //使能tim1输出PWM信号。
	TIM_UpdateRequestConfig(TIM1, TIM_UpdateSource_Global);  //配置TIM1的更新请求源为全局更新。 选择全局更新请求源确保从模式控制器的触发也能生成更新事件，保持所有定时器的同步。
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable); //将定时器1设置为主从模式
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);      //CNT计数到ARR时产生触发事件
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
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); //在更新事件发生后，将预装载寄存器的值加载到比较寄存器，以避免输出比较值的即时更新，从而防止输出信号的突变
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_Cmd(TIM1, ENABLE); //启动定时器，开始计数

}
void pwm_output(fix16_t a, fix16_t b, fix16_t c) 
{
	TIM1->CCR1 = fix16_to_int(fix16_mul(F16(PWM_COUNT), a));
	TIM1->CCR2 = fix16_to_int(fix16_mul(F16(PWM_COUNT), b));
	TIM1->CCR3 = fix16_to_int(fix16_mul(F16(PWM_COUNT), c));
}





/**********************QEP*************************///正交编码器（Quadrature Encoder Pulse, QEP）
/*
PA6	TIM3_CH1	FB0_A
PA7	TIM3_CH2	FB0_B
PB0 TIM3_CH3	FB0_I/LATCH（就是Z）
PB1 TIM3_CH4	PERIOD POSITION LATCH
*/
static struct
{
	volatile int64_t position;
	volatile int64_t offset;
	uint16_t last_cnt;
	// M/T function
	uint16_t last_delta_time;
	int64_t last_spd_pos;
	volatile int16_t T;
	int32_t speed;
} qep;
void qep_tim3_init() 
{
	qep.position = 0;
	qep.offset = VALUE_NOT_USEABLE;
	qep.last_cnt = 0;

	//4个引脚复用到tim3
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);				   // 初始化为默认值
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

	
	TIM_DeInit(TIM3);//重置
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	//Encoder模式 （这时 tim3的CNT加一减一由 A B通道的脉冲决定）
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	TIM_ICInitTypeDef ch3; 
	ch3.TIM_Channel = TIM_Channel_3;
	ch3.TIM_ICPolarity = TIM_ICPolarity_Rising;    //Z有一个脉冲时，代表转了一圈 这个时候捕获CNT的值放到CCR3
	ch3.TIM_ICSelection = TIM_ICSelection_DirectTI;
	ch3.TIM_ICPrescaler = 0;
	ch3.TIM_ICFilter = 0;
	TIM_ICInit(TIM3, &ch3);
	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE); //使能 TIM3 的捕获比较通道 3（CC3）中断

	TIM_ICInitTypeDef ch4; 
	ch4.TIM_Channel = TIM_Channel_4;
	ch4.TIM_ICPolarity = TIM_ICPolarity_Rising;
	ch4.TIM_ICSelection = TIM_ICSelection_DirectTI;
	ch4.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
	ch4.TIM_ICFilter = 0;
	TIM_ICInit(TIM3, &ch4);

	NVIC_InitTypeDef NVIC_InitStructure; 
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM3->CNT = 0;
	TIM_Cmd(TIM3, ENABLE);
}
void TIM3_IRQHandler() 
{
	if (TIM_GetITStatus(TIM3, TIM_IT_CC3))
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
		TIM_ITConfig(TIM3, TIM_IT_CC3, DISABLE);
		irq_disable();
		int16_t delta = TIM3->CCR3 - qep.last_cnt;
		qep.offset = -qep.position - delta;
		fbk_t *pFeedback = (fbk_t *)feedback;
		pFeedback->status.index_found = 1;
		irq_enble();
	}
}
void qep_update() //更新位置
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
void qep_spd_tim8_init() // 测量速度。
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);

	/* Timer configuration in Encoder mode */
	TIM_DeInit(TIM8);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	TIM8->CR2 |= TIM_CR2_TI1S; // XOR mode
	TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Reset);
	TIM_SelectInputTrigger(TIM8, TIM_TS_TI1F_ED);

	TIM_ICInitTypeDef ch4;
	ch4.TIM_Channel = TIM_Channel_4;
	ch4.TIM_ICPolarity = TIM_ICPolarity_Rising;
	ch4.TIM_ICSelection = TIM_ICSelection_DirectTI;
	ch4.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
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
void qep_speed() //用tim3和tim8的channel4计算速度
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
	qep.T += (int16_t)(NS_PER_SEC / PWM_FREQUENCY + delta_time * NS_PER_SEC / SYS_FREQUENCY); // 更新累积时间 qep.T 单位 ns

	if (M > 10 || M < -10 || qep.T > NS_PER_SEC / 20) // 如果位置变化量 M 较大，或者累积时间 qep.T 超过一定阈值，则认为有足够的数据来更新速度。
	{
		qep.last_spd_pos = new_pos;					 // 更新最后位置为当前位置
		qep.speed = (int64_t)M * NS_PER_SEC / qep.T; // 速度单位 s
		qep.T = 0;									 // 重置累积时间，以便开始下一次速度计算
	}
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
/*	PB6		UART1 TX（Transmit）发送
	PB7		UART1 RX（Receive） 接收
	PC7		RS485_DE */
//STM32 微控制器通常有多个 DMA 控制器，每个控制器有多个 Stream 和 Channel。例如，STM32F4 系列有两个 DMA 控制器（DMA1 和 DMA2），每个控制器有 8 个 Stream 和 16 个 Channel。
// BSRR（Bit Set/Reset Register）寄存器是一个特殊的寄存器，用于控制 GPIO 引脚的输出电平。BSRR 寄存器分为两个部分：BSRRL（低16位）和 BSRRH（高16位）。
// 当 BSRRL 的某一位被设置为 1 时，对应的 GPIO 引脚会被设置为高电平  //当 BSRRH 的某一位被设置为 1 时，对应的 GPIO 引脚会被置为低电平 写入 0 则不会改变该引脚的当前状态
uint8_t uart1_rx[11] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0};
#define RS_485_WR GPIOC->BSRRL = GPIO_Pin_7 //PC7
#define RS_485_RD GPIOC->BSRRH = GPIO_Pin_7
static struct
{
	volatile int64_t position; 
	uint32_t last_pos; 
	int32_t speed;  
	int32_t last_spd[3]; 
	uint32_t last_spd_pos; 
	volatile int32_t T; 
} abs_encoder;
void rs485_uart1_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	// USART1
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 2500000;  //比特每秒 (bps)。
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No; 
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //配置为无硬件流控制。
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //使能接收 (Rx) 和发送 (Tx) 模式。
	USART1->CR1 |= USART_CR1_OVER8; //8 倍过采样模式。这种模式可以在高波特率下提供更好的性能。
	USART_Init(USART1, &USART_InitStructure);

	//DMA2_Stream2 Channel_4   UART1 RX
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA2_Stream2);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4; 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);//外设基地址为 USART1 数据寄存器的地址。这是数据传输的源地址。
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uart1_rx;//内存基地址为 uart1_rx。这是数据传输的目标地址。
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//DMA 传输方向为外设到内存。
	DMA_InitStructure.DMA_BufferSize = ABS_RETURN_LEN; // 一次传输的数据长度。 11
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址不递增。每次传输完成后，外设地址保持不变。
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //内存地址递增。每次传输完成后，内存地址递增，以便存储连续的数据。
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //设置外设数据大小为字节。
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //设置内存数据大小为字节。
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //设置 DMA 为循环模式  数据传输完成后，DMA 会自动重新开始
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  //设置 DMA 优先级为非常高
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; //禁用 DMA FIFO 模式。
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; //设置 FIFO 阈值为半满（虽然 FIFO 模式已禁用，此配置无效）。
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; //设置内存突发传输为单次传输模式。每次只传输一个数据单位。
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; //设置外设突发传输为单次传输模式。每次只传输一个数据单位。
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);

	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE); //使能USART1的DMA请求  当USART1接收数据后，会触发DMA传输。
	DMA_Cmd(DMA2_Stream2, ENABLE);
	USART_Cmd(USART1, ENABLE);
}
void rs485_uart1_request() 
{
	DMA2_Stream2->NDTR = 2;//NDTR（Number of Data to Transfer Register）是一个寄存器，用于定义每次DMA传输过程中需要传输的数据单元数
	//	DMA1_Stream2->CMAR = 2;
	//	DMA1_Stream2->CCR |= 1;
}
void restart_usart1(void) 
{
	DMA2_Stream2->CR &= (uint32_t)(~DMA_CCR_EN); //将控制寄存器的第 0 位清零（即将 EN 位清零）。~1 的结果是 0xFFFFFFFE，与 DMA2_Stream2->CR 进行按位与操作，清除了 EN 位。
	DMA2_Stream2->NDTR = ABS_RETURN_LEN;		 
	DMA2_Stream2->CR |= (uint32_t)DMA_CCR_EN;	 //重新使能DMA通道1，DMA传输将按新的设置重新开始。 
}

uint8_t error_cnt = 0;
#define ABS(x) ((x) > 0 ? (x) : (-(x)))
/*每次数据处理前需要两个 tick 才开始数据处理，这是由于 RS485 通信的特性和流程所决定的。
 RS485 通信是一种半双工通信方式，即在同一时间只能有一个设备发送或接收数据
 一般来说，如果没有显式地初始化，静态变量的初始值会被默认为零或者空*/
void rs485_get_abs_data() 
{
	static uint32_t tick = 1;  //用于记录函数调用次数
	tick++; 
	if (tick % 4 == 0) 
	{
		RS_485_WR; //发送数据请求 
		restart_usart1();  //重启USART1，确保通信正常
		USART1->DR = DATA_3; //将 DATA_3 的值写入到 USART1 的数据寄存器（Data Register, DR），将它发送
		for (int i = 0; i < 80; ++i)  //延时一段时间，让数据有足够时间传输。
			__asm("nop");
		uint32_t clk = GET_CORE_CLK(); 
		abs_encoder.T += clk * (NS_PER_SEC / 1000000) / (SYS_FREQUENCY / 1000000); 
		CORE_CLK_RESET(); //重置计时器。
		RS_485_RD; //准备接收数据
	}
	else if (tick % 4 == 2) //处理接收到的数据
	{
		uint32_t abs_pos;
		uint32_t ppr;
		if (uart1_rx[10] == crcFast(uart1_rx, 10)) // 校验接收到的数据是否有效。
		{
			abs_pos = (((((uint32_t)uart1_rx[7] & 0xff) << 8) + (uart1_rx[6] & 0xff)) << 17) + (((((uint32_t)uart1_rx[4] & 0xff) << 16) + (((uint32_t)uart1_rx[3] & 0xff) << 8) + ((uint32_t)uart1_rx[2] & 0xff)));
			ppr = 131072;
		}

		int32_t delta_pos = abs_pos - abs_encoder.last_pos;  //计算位置变化 delta_pos。
		char valid = 0;   //确认位置变化的有效性
		static int delta_tick = 0;
		if (ABS(delta_pos) > ppr / 100)  //如果位置变化超过设定阈值，则需要多次确认变化有效性。
		{
			delta_tick += 1;
			if (delta_tick > 3)
			{
				delta_tick = 0;
				valid = 1;
			}
		}
		else
		{
			valid = 1;
			delta_tick = 0;
		}

		if (valid)
		{
			abs_encoder.last_pos = abs_pos;
			abs_encoder.position += delta_pos;

			int32_t M = abs_pos - abs_encoder.last_spd_pos;
			// spd = M/T
			if (M > 10 || M < -10 || abs_encoder.T > NS_PER_SEC / 10)
			{
				abs_encoder.last_spd_pos = abs_pos;
				int32_t spd = (int64_t)M * NS_PER_SEC / abs_encoder.T;  //速度等于位置变化除以时间
				abs_encoder.last_spd[2] = abs_encoder.last_spd[1];  //更新速度历史数据last_spd
				abs_encoder.last_spd[1] = abs_encoder.last_spd[0];
				abs_encoder.last_spd[0] = spd;
				spd = abs_encoder.last_spd[0] + abs_encoder.last_spd[1] + abs_encoder.last_spd[2];  //计算当前速度speed。
				abs_encoder.speed = spd / 3;

				abs_encoder.T = 0;  //重置计时器
			}
			crc_ok++; //增加校验成功计数
		}
	}
	else
	{
		crc_error++;
	}
}
// 使用 #define 创建的宏实际上是简单的文本替换 所以这里encoder == ABS_ENCODER为true（不用考虑char int）
int64_t encoder_position(encoder_t encoder) 
{
	if (encoder == ABZ_ENCODER || encoder == AB_ENCODER)
		return qep.position;
	else if (encoder == ABS_ENCODER || encoder == PANASONIC_QEP_ENCODER)  //执行这里
		return abs_encoder.position;
	else if (encoder == HALL_ENCODER)
		return hall_position(&(motor.hall_init.hall), 1);

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
/*	
PA0	A0
PA6 A6
PC1 ADC1_Channel11	VBUS3
PC2 ADC2_Channel12 TMP1
PC3	A13 TMP2 NOT support
PA2	A2	iv
PA3	A3	iu CH1
for phase current, flow to motor is positive. 对于相电流，流向电机的流量为正。
*/
#define ADC_BUFF_LEN 1 
struct adc_dma_t
{ 
	uint16_t vbus;// 电压
	uint16_t tmp; // 温度
} adc_dma[1];
static int16_t IAB_OFFSET[2] = {2048, 2120}; // 静态数组，存储了两个偏移量，用于电流测量的校准。
static int16_t ia_A, ib_A;					 // 存储计算后的电流值。
static void adc_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; //模拟输入模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOA, &GPIO_InitStructure); //PA2 PA3 iv iu
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_Init(GPIOC, &GPIO_InitStructure); // PC1 PC2 VBUS TEMP

	//DMA2_Stream0 Channel_0
	DMA_InitTypeDef DMA_InitStructure; 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	DMA_DeInit(DMA2_Stream0);
	DMA_StructInit(&DMA_InitStructure); 
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;					
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)0x40012308; // 配置外设基地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adc_dma;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; // 设置DMA传输方向为从外设（Peripheral）到内存（Memory）。
	DMA_InitStructure.DMA_BufferSize = sizeof(adc_dma) / 2; //半字（16 位）
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //禁用外设地址增量，即外设地址在每次传输后保持不变。
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //使能内存地址增量，即内存地址在每次传输后递增。
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //设置外设和内存的数据大小为半字（16 位）。
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //DMA 模式为循环模式，即当传输完成后，自动重新开始。
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //设置 DMA 优先级为高
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; //禁用 FIFO 模式，
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;  
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;  
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);  

	//ADC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE); 
	ADC_DeInit();	
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_RegSimult; //设置 ADC 模式为双模式，同时进行常规转换。
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; //设置两个采样之间的延迟为 5 个周期。
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1; //设置 DMA 访问模式为模式 1。
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; //设置 ADC 预分频器为 4，控制 ADC 时钟频率
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;      //关闭扫描模式，因为只采集一个通道。
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  //关闭连续模式，只进行一次转换。
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;               //选择外部事件作为触发源。这里选择tim1 的捕获比较 1 事件。
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;       //在上升沿时触发转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  
	ADC_InitStructure.ADC_NbrOfConversion = sizeof(adc_dma) / 4;  //设置需要转换的通道数量。这里设置为 1，表示只转换一个通道。


/	//注入通道：具有更高的优先级，当外部触发事件（如 TIM1 的 TRGO 事件）发生时，注入通道的转换会中断常规通道的转换，立即进行。
	//常规通道：在没有注入通道触发时，常规通道按照配置的顺序进行转换。

	//常规通道
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_28Cycles); 
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_12, 1, ADC_SampleTime_28Cycles);

	//注入通道
	ADC_InjectedSequencerLengthConfig(ADC1, ADC_BUFF_LEN); //设置ADC1的注入通道序列长度
	for (int i = 0; i < ADC_BUFF_LEN; ++i)				   
		ADC_InjectedChannelConfig(ADC1, ADC_Channel_3, i + 1, ADC_SampleTime_28Cycles);
	ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);       // 设置ADC1的外部触发源为tmi1的TRGO事件
	ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Rising);// 设置触发沿为上升沿

	ADC_InjectedSequencerLengthConfig(ADC2, ADC_BUFF_LEN); 
	for (int i = 0; i < ADC_BUFF_LEN; ++i)
		ADC_InjectedChannelConfig(ADC2, ADC_Channel_2, i + 1, ADC_SampleTime_28Cycles);
	ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T1_TRGO);
	ADC_ExternalTrigInjectedConvEdgeConfig(ADC2, ADC_ExternalTrigInjecConvEdge_Rising);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);  //NVIC_PriorityGroup_1 将优先级分为 2 位抢占优先级和 2 位子优先级
	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure);
	ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE); // 使能 ADC1 的注入通道转换完成中断

	/* Enable ADC DMA */
	ADC_DMACmd(ADC1, ENABLE); // 使能ADC1的DMA请求  当ADC1完成转换后，会触发DMA传输。
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE); //启用了多ADC模式。在多ADC模式下，主ADC通常处理DMA传输，而从ADC的数据会合并到主ADC的数据流中。
	ADC_Cmd(ADC1, ENABLE); //使能ADC1模块，开始ADC转换。
	ADC_Cmd(ADC2, ENABLE);
}
void ADC_IRQHandler(void) 
{
	if (ADC_GetITStatus(ADC1, ADC_IT_JEOC) == SET)  //ADC1 的注入通道转换完成中断发生
	{
		driver_ledon(0); //点个灯
		ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC); //清除中断标志

		uint16_t adc1[ADC_BUFF_LEN], adc2[ADC_BUFF_LEN];

		#if ADC_BUFF_LEN >= 1   
			adc1[0] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);  //ADC_InjectedChannel_1对应ADC_Channel_3的（i + 1）就是1，
			adc2[0] = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);	//ADC_InjectedChannel_1对应ADC_Channel_2的（i + 1）就是1，
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
		if (adc_offset_cnt < 100) //滤波 再操作电机之前算出基准的ADC值，通电后ADC本身有一个值
		{
			static int32_t offset_sum1 = 0, offset_sum2 = 0; 
			adc_offset_cnt++;
			for (int j = 0; j < ADC_BUFF_LEN; ++j) 
			{
				offset_sum1 += adc1[j];
				offset_sum2 += adc2[j];
			}
			if (adc_offset_cnt == 100)
			{
				adc_offset_cnt = 0xff; //255
				IAB_OFFSET[0] = offset_sum1 / ADC_BUFF_LEN / 100;  //计算出平均值，存储在IAB_OFFSET中。这个偏移量是为了消除ADC采样中的直流偏移。
				IAB_OFFSET[1] = offset_sum2 / ADC_BUFF_LEN / 100;  
			}
		}
		else //执行adc_offset_cnt = 0xff;后下一次中断调用来到这里 之后都不会执行if (adc_offset_cnt < 100)里面的语句
		{
			int32_t sum1 = 0, sum2 = 0;
			for (int i = 0; i < ADC_BUFF_LEN; ++i)
			{
				sum1 += adc1[i] - IAB_OFFSET[0]; //这里的adc1[i]是第101次ADC注入的数据，之后是102 103
				sum2 += adc2[i] - IAB_OFFSET[1];
			}
			ia_A = -sum1 * (3300 * 1000 / 4096 / 55 / ADC_BUFF_LEN);
			ib_A = -sum2 * (3300 * 1000 / 4096 / 55 / ADC_BUFF_LEN); /* 3.3 50A, 4096, A = AD * 1000 * 300 / 4096 / XXX , XXX for mV/A*/
			board_update(); 
		}
		driver_ledoff(0); //关灯
	}
}

int16_t driver_ia(void *self)
{
	return ia_A;
}
int16_t driver_ib(void *self)
{
	return ib_A;
}
int16_t driver_ibus(void *self)
{
	return 0;
}





/***********************SPI to A8************************/
/*PB3 PB4 PB5 for spi sck miso mosi*/
/*PA12 is sync pin from a8*/
/*##DMA2 CH1 for SPI3 RX*/
/*##DMA2 CH2 for SPI3 TX*/
volatile uint8_t pos_latched = 0;
uint16_t spi3_cr1, spi3_cr2;
uint32_t dma1_cr, dma2_cr, dma1_cndtr, dma2_cndtr, dma1_cpar, dma2_cpar, dma1_cmar, dma2_cmar;
void spi3_init()
{
	DMA_InitTypeDef DMA_InitStructure_ch1;
	DMA_InitTypeDef DMA_InitStructure_ch2;
	SPI_InitTypeDef SPI_InitStructure;

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
	// GPIO_InitStructure.GPIO_Pin = SPIx_NSS_PIN;
	// GPIO_Init(SPIx_NSS_GPIO_PORT, &GPIO_InitStructure);

	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPI3);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 0x1021;//CRC（循环冗余校验）多项式
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	SPI_Init(SPI3, &SPI_InitStructure);

	// SPI_RxFIFOThresholdConfig(SPI3, SPI_RxFIFOThreshold_HF);

	/* DMA Configuration -------------------------------------------------------*/

	DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure_ch1.DMA_PeripheralBaseAddr = (uint32_t)&SPI3->DR;
	DMA_InitStructure_ch1.DMA_Memory0BaseAddr = (uint32_t)command;
	DMA_InitStructure_ch1.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure_ch1.DMA_BufferSize = 128 / 2;
	DMA_InitStructure_ch1.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure_ch1.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure_ch1.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure_ch1.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure_ch1.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure_ch1.DMA_Priority = DMA_Priority_VeryHigh;
	// DMA_InitStructure_ch1.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA2_Stream1, &DMA_InitStructure_ch1);

	DMA_DeInit(DMA2_Stream2);
	DMA_InitStructure_ch2.DMA_PeripheralBaseAddr = (uint32_t)&SPI3->DR;
	DMA_InitStructure_ch2.DMA_Memory0BaseAddr = (uint32_t)feedback;
	DMA_InitStructure_ch2.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure_ch2.DMA_BufferSize = 126 / 2;
	DMA_InitStructure_ch2.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure_ch2.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure_ch2.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure_ch2.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure_ch2.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure_ch2.DMA_Priority = DMA_Priority_VeryHigh;
	// DMA_InitStructure_ch2.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA2_Stream2, &DMA_InitStructure_ch2);

	// SPI_NSSPulseModeCmd(SPI3, ENABLE);
	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
	SPI_I2S_ClearFlag(SPI3, SPI_I2S_FLAG_TXE);
	SPI_I2S_ClearFlag(SPI3, SPI_I2S_FLAG_RXNE);
	// SPI_CRCLengthConfig(SPI3, SPI_CRCLength_16b);
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

	// SPI_NSSInternalSoftwareConfig(SPI3, SPI_NSSInternalSoft_Set);
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DMA_Cmd(DMA2_Stream2, ENABLE);

	SPI_Cmd(SPI3, ENABLE);
}
void sync_from_a8_init() // 初始化与外部设备 这里假设为A8 同步的硬件和中断
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource12);

	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//它表示外部中断线 10 到 15 的中断请求。
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	EXTI_ClearITPendingBit(EXTI_Line12);
}
void EXTI15_10_IRQHandler(void) // max: 499 clock about 7us 这是一个中断处理函数，用于响应EXTI线12的中断请求。当外部设备（A8）触发中断时，此函数将被调用。
{
	// sync from a8
	if (EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
		SPI3->CR1 &= (uint16_t) ~((uint16_t)SPI_CR1_SPE);
		SPI3->CR2 &= (uint16_t) ~((uint16_t)(SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx));
		cmd_t *pCommand = (cmd_t *)command;
		if ((pCommand->crc == SPI3->RXCRCR) && (!motor.state.calibrating))
		{
			/*feedback update*/
			fbk_t *pFeedback = (fbk_t *)feedback;
			cmd_t *pCommand = (cmd_t *)command;
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

		// DMA_DeInit(DMA2_Channel1);
		// DMA2_Channel1->CCR = 0;
		// DMA2->IFCR |= ((uint32_t)(DMA_ISR_GIF1 | DMA_ISR_TCIF1 | DMA_ISR_HTIF1 | DMA_ISR_TEIF1));

		// DMA2_Channel2->CCR = 0;
		// DMA2->IFCR |= ((uint32_t)(DMA_ISR_GIF2 | DMA_ISR_TCIF2 | DMA_ISR_HTIF2 | DMA_ISR_TEIF2));

		// DMA_DeInit(DMA2_Channel2);
		// SPI_I2S_DeInit(SPI3);
		// RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, ENABLE);
		RCC->APB1RSTR |= RCC_APB1Periph_SPI3;
		/* Release SPI3 from reset state */
		//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, DISABLE);
		RCC->APB1RSTR &= ~RCC_APB1Periph_SPI3;

		SPI3->CR1 = spi3_cr1;
		SPI3->CR2 = spi3_cr2;
		SPI3->I2SCFGR &= (uint16_t) ~((uint16_t)SPI_I2SCFGR_I2SMOD);
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

		// EXTI_ClearITPendingBit(EXTI_Line12);
		*(__IO uint32_t *)(((uint32_t) & (EXTI->PR)) + ((EXTI_Line12) >> 5) * 0x20) = (1 << (EXTI_Line12 & 0x1F));
		return;
	}
	// IPU fault
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
}





/**********************FLASH**************************/
// 实现了向FLASH存储写入数据的功能
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

	uint32_t sectorStartAddress = sectorPosition * SECTOR_SIZE + FLASH_BASE;
	FLASH_ErasePage(sectorStartAddress);
	uint16_t dataIndex;
	for (dataIndex = 0; dataIndex < countToWrite; dataIndex++)
	{
		FLASH_ProgramHalfWord(startAddress + dataIndex * 2, writeData[dataIndex]);
	}
	FLASH_Lock();
}






/**********************BOARD*************************/
void board_load_parameter()
{
	motor.parameter.electric_offset = 19586;  //电气偏移量
	motor.parameter.invert = 0;   //电机是否反转
	motor.parameter.poles = 4;   //极数
	motor.parameter.inductance = F16(0.999363); //电感
	motor.parameter.resistance = F16(1.139802); //电阻
	motor.parameter.bandwith = F16(1000);  //电机带宽1000HZ

	motor.parameter.ppr = 1 << 17;      //编码器每转脉冲数为 131072
	motor.encoder.init = ABS_ENCODER;  //这里用的是绝对值编码器，阔以自己选择
	motor.encoder.pos = ABS_ENCODER;
	motor.encoder.spd = ABS_ENCODER;
	motor.encoder.motor = ABS_ENCODER;
	motor_set_vbus(&motor, 25);//  设置电动机母线电压为 25 伏特
}
void board_init() 
{
	board_load_parameter();
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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	pwm_tim1_init();
	pwm_io_init();	 

	adc_init();
	driver_enable(0);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	rs485_uart1_init();

	if (SysTick_Config((SystemCoreClock) / 1000))
	{
		/* Capture error */ 
		while (1)
			;
	}
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_SetPriority(SysTick_IRQn, 0xff);

}
static void board_update()
{
	char pos = motor.encoder.motor | motor.encoder.pos; //pos=8
#ifndef DEBUG
	if ((pos & ABS_ENCODER) || (pos & PANASONIC_QEP_ENCODER))
		rs485_get_abs_data();
	if (pos & ABZ_ENCODER)  
		qep_update();
#else
	rs485_get_abs_data();
	qep_update();
#endif
	switch (motor.encoder.spd)
	{
	case ABZ_ENCODER:
		qep_speed();
		break;
	case ABS_ENCODER:  //执行
		break;
	}
	if (!motor.direct_pwm)
		motor_update(&motor);
}
