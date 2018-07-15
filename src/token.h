#ifndef __TOKEN_H__
#define __TOKEN_H__

#define USART1_DMA_SEND		 DMA1_Stream4
#define USART1_DMA_RECEIVE	 DMA1_Stream5
#define USART4_DMA_SEND		 DMA2_Stream5
#define USART4_DMA_RECEIVE	 DMA2_Stream3
#define USART_BUFF_LEN 300

uint8_t get_devide_id();
void set_device_id(uint8_t id);

typedef struct{
	uint8_t device_id;
}power_on_t;
#endif