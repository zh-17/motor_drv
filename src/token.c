#include "bsp.h"
#include "motor.h"
#include "token.h"
#if 1

static int token = 1;
static uint8_t device_id = 0;
int get_token()
{
	return token;
}
void set_token()
{
	token = 1;
}

uint8_t get_devide_id()
{
	return device_id;
}
void set_device_id(uint8_t id)
{
	device_id = id;
}

uint8_t usart_receive_data(DMA_Stream_TypeDef * dma, uint8_t* receive_buffer, uint8_t* opPacket, int* token_now)
{
	static uint8_t data_len;
	data_len = USART_BUFF_LEN - dma->NDTR;

	if (data_len != 0)
	{
		memcpy(opPacket, receive_buffer, data_len);
		*token_now = get_token();
		usart_receive_dma_reset(dma);
		memset(receive_buffer, ' ', data_len);
	}
	return data_len;
}

const char* cmdcmp(const char* buff, const char* cmd)
{
	for (; *buff == *cmd && *buff != 0 && *cmd != 0; cmd++, buff++);
	if (*cmd == 0)
	{
		while (*buff == ' ') buff++;
		return buff;
	}
	return NULL;
}

void board_cmd(motor_t* self, uint8_t* rx_buffer_t);
void usart1_receive_data_process(motor_t* self, uint8_t* opPacket, uint8_t len, uint8_t token_now)
{
	const char* argv;
	if (token_now == 1)
	{
		board_cmd(self, opPacket);
	}
	else
	{
		usart_dma_send(USART4_DMA_SEND, opPacket, len);
	}
}

void usart4_receive_data_process(uint8_t* opPacket, uint8_t len)
{
	usart_dma_send(USART1_DMA_SEND, opPacket, len);
}

void usart_token(motor_t* self)
{
	int token_now;
	uint8_t receive_data1[USART_BUFF_LEN], receive_data2[USART_BUFF_LEN];
	uint8_t usart1_len = usart_receive_data(USART1_DMA_RECEIVE, usart1_dma_rx_t, receive_data1, &token_now);
	if (usart1_len)
	{
		usart1_receive_data_process(self, receive_data1, usart1_len, token_now);
	}
	uint8_t usart4_len = usart_receive_data(USART4_DMA_RECEIVE, usart4_dma_rx_t, receive_data2, &token_now);
	if (usart4_len)
	{
		usart4_receive_data_process(receive_data2, usart4_len);
	}
}

void board_cmd(motor_t* self, uint8_t* rx_buffer_t)
{
	const char* argv;

	if ((argv = cmdcmp(rx_buffer_t, "dis")))
	{
		motor_disable(self);
		printf("driver disable\r\n");
	}
	if ((argv = cmdcmp(rx_buffer_t, "who")))
	{
		printf("Driver: %d\r\n",device_id);
	}
	else if ((argv = cmdcmp(rx_buffer_t, "en")))
	{
		motor_enable(self);
		printf("driver enable\r\n");
	}
	else if ((argv = cmdcmp(rx_buffer_t, "cali")))
	{
		printf("start calibrate\r\n");
		pmsm_calibrate(self, 1000);
	}
	else if ((argv = cmdcmp(rx_buffer_t, "speed")))
	{
		printf("speed %d\r\n", encoder_speed(self->encoder.motor));
	}
	else if ((argv = cmdcmp(rx_buffer_t, "pos")))
	{
		printf("pos %d \r\n", (int32_t)encoder_position(self->encoder.motor));
	}
	else if (argv = cmdcmp(rx_buffer_t, "pid"))
	{		
		if (*argv != 0)
		{
			if (*argv != 'j')
				printf("%s \r\n", argv);
			else
				printf("ok \r\n");
		}
	}
	else if ((argv = cmdcmp(rx_buffer_t, "phase")))
	{
		driver_output(0, 0.5, 0.5, 0.5);
		msleep(100);
		printf("555:%d, %d\r\n", (int)driver_ia(0), (int)driver_ib(0));
		driver_output(0, 0.7, 0.5, 0.5);
		msleep(400);
		printf("755:%d, %d\r\n", (int)driver_ia(0), (int)driver_ib(0));
		driver_output(0, 0.5, 0.7, 0.5);
		msleep(400);
		printf("575:%d, %d\r\n", (int)driver_ia(0), (int)driver_ib(0));
		driver_output(0, 0.5, 0.5, 0.7);
		msleep(400);
		printf("557:%d, %d\r\n", (int)driver_ia(0), (int)driver_ib(0));
		driver_output(0, 0.6, 0.7, 0.5);
		msleep(400);
		printf("675:%d, %d\r\n", (int)driver_ia(0), (int)driver_ib(0));
		driver_output(0, 0.7, 0.6, 0.5);
		msleep(400);
		printf("765:%d, %d\r\n", (int)driver_ia(0), (int)driver_ib(0));
		driver_output(0, 0.7, 0.5, 0.6);
		msleep(400);
		printf("756:%d, %d\r\n", (int)driver_ia(0), (int)driver_ib(0));
		driver_output(0, 0.5, 0.5, 0.5);
		msleep(100);
		printf("555:%d, %d\r\n", (int)driver_ia(0), (int)driver_ib(0));
		driver_disable(0);
	}
	/*else if ((argv = cmdcmp(rx_buffer_t, "1")))
	{
		self->pid._target = 1;
		printf("set target torque %d \r\n", (int)self->pid._target);
	}
	else if ((argv = cmdcmp(rx_buffer_t, "2")))
	{
		self->pid._target = 2;
		printf("set target torque %d \r\n", (int)self->pid._target);
	}
	else if ((argv = cmdcmp(rx_buffer_t, "3")))
	{
		self->pid._target = 3;
		printf("set target torque %d \r\n", (int)self->pid._target);
	}
	else if ((argv = cmdcmp(rx_buffer_t, "4")))
	{
		self->pid._target = 4;
		printf("set target torque %d \r\n", (int)self->pid._target);
	}
	else if ((argv = cmdcmp(rx_buffer_t, "5")))
	{
		self->pid._target = 5;
		printf("set target torque %d \r\n", (int)self->pid._target);
	}
	else if ((argv = cmdcmp(rx_buffer_t, "6")))
	{
		self->pid._target = 6;
		printf("set target torque %d \r\n", (int)self->pid._target);
	}
	else if ((argv = cmdcmp(rx_buffer_t, "7")))
	{
		self->pid._target = 7;
		printf("set target torque %d \r\n", (int)self->pid._target);
	}
	else if ((argv = cmdcmp(rx_buffer_t, "8")))
	{
		self->pid._target = 8;
		printf("set target torque %d \r\n", (int)self->pid._target);
	}
	else if ((argv = cmdcmp(rx_buffer_t, "9")))
	{
		self->pid._target = 9;
		printf("set target torque %d \r\n", (int)self->pid._target);
	}
	else if ((argv = cmdcmp(rx_buffer_t, "010")))
	{
		self->pid._target = 10;
		printf("set target torque %d \r\n", (int)self->pid._target);
	}
	else if ((argv = cmdcmp(rx_buffer_t, "015")))
	{
		self->pid._target = 15;
		printf("set target torque %d \r\n", (int)self->pid._target);
	}
	else if ((argv = cmdcmp(rx_buffer_t, "030")))
	{
		self->pid._target = 30;
		printf("set target torque %d \r\n", (int)self->pid._target);
	}*/
}
#endif