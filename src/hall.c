#if 1
#include "stm32f4xx_gpio.h"
#include "types.h"
#include "hall.h"
#include "stdio.h"

/************HALL*************/
/*PA0 PA1 PA2 HALL U V W*/
#define HALL_W GPIO_Pin_2
#define HALL_V GPIO_Pin_1
#define HALL_U GPIO_Pin_0

static int __inv_gray_code[] =
{
	0, // 0
	1, // 1
	3, // 2
	2, // 3
	5, // 4
	6, // 5
	4, // 6
	7, // 7
};

static inline uint32_t hall_section(hall_t* self)
{
	uint32_t sec = ((GPIOA->IDR) & HALL_W);
	sec |= ((GPIOA->IDR) & HALL_V);
	sec |= ((GPIOA->IDR) & HALL_U);
	return sec;
}

int64_t hall_position(hall_t* self, uint32_t abs_pos)
{
	uint32_t sec = hall_section(self);

	if (sec > 6 || sec <= 0)
		return VALUE_NOT_USEABLE;

	return __inv_gray_code[sec] - 1;
}
int hall_init(hall_t* self, uint32_t hall_table)
{
	int i = 0;
	self->_hall_table = hall_table;
	for (i = 1; i < 7; ++i)
		__inv_gray_code[i] = (self->_hall_table >> (i * 4)) & 0x00F;

	return 0;
}
int hall_calibrate_start(hall_t* self)
{
	uint32_t sec = hall_section(self);
	if (sec > 6 || sec <= 0)
		return -1;
	self->_last_sect = 0;
	self->_index = 1;
	self->_hall_table = 0x70000000;
	self->_sect_cnt = 0;
	self->_sect = sec;
	return 0;
}
int hall_calibrate(hall_t* self)
{
	uint32_t sec = hall_section(self);
	if (sec > 6 || sec <= 0)
	{
		return -1;
	}
	if (self->_sect == sec) {
		if (self->_sect_cnt < 10)
			self->_sect_cnt++;
	}
	else
		self->_sect_cnt = 0;

	self->_sect = sec;

	if (self->_sect_cnt < 5)
		return 1;

	if (sec == self->_last_sect)
		return 1;

	if (self->_index < 7)
	{
		self->_hall_table |= (self->_index++) << (sec * 4);
		hall_test(self, 1);
	}
	else
		return 0;
	self->_last_sect = sec;
	return 1;
}
#define DBG_INFO printf
void hall_invert_table(hall_t* self)
{
	int i;
	DBG_INFO("orgin: ");
	for (i = 1; i < 7; ++i)
	{
		__inv_gray_code[i] = (self->_hall_table >> (i * 4)) & 0x00F;
		DBG_INFO("%d ", __inv_gray_code[i]);
	}
	DBG_INFO("\r\n");

	int seq[8];
	for (i = 1; i < 7; ++i)
		seq[__inv_gray_code[i]] = i;

	DBG_INFO("HALL origin: ");
	for (i = 1; i < 7; ++i)
		DBG_INFO("%d ", seq[i]);
	DBG_INFO("\r\n");

	int tmp = seq[2];
	seq[2] = seq[6];
	seq[6] = tmp;
	tmp = seq[3];
	seq[3] = seq[5];
	seq[5] = tmp;

	DBG_INFO("HALL invert: ");
	for (i = 1; i < 7; ++i)
		DBG_INFO("%d ", seq[i]);
	DBG_INFO("\r\n");


	self->_hall_table = 0x70000000;
	for (i = 1; i < 7; ++i)
		self->_hall_table |= (i) << (seq[i] * 4);

}


int hall_calibrate_zero(hall_t* self)
{
	int pos = hall_position(self, 1);
	if (pos == 0)
		return 0;
	if (pos < 0 || pos > 6)
		return -1;
	int i;
	DBG_INFO("orgin: ");
	for (i = 1; i < 7; ++i)
	{
		__inv_gray_code[i] = (self->_hall_table >> (i * 4)) & 0x00F;
		DBG_INFO("%d ", __inv_gray_code[i]);
	}
	DBG_INFO("\r\n");

	int seq[8];
	for (i = 1; i < 7; ++i)
		seq[__inv_gray_code[i]] = i;

	DBG_INFO("HALL origin: ");
	for (i = 1; i < 7; ++i)
		DBG_INFO("%d ", seq[i]);
	DBG_INFO("\r\n");

	int tmp = seq[1];
	for (i = 1; i < pos + 1; ++i)
		seq[i] = seq[i + 1];
	for (i = pos + 1; i < 7; ++i)
		seq[i] = seq[i + 1];
	seq[6] = tmp;


	DBG_INFO("HALL zero: ");
	for (i = 1; i < 7; ++i)
		DBG_INFO("%d ", seq[i]);
	DBG_INFO("\r\n");


	self->_hall_table = 0x70000000;
	for (i = 1; i < 7; ++i)
		self->_hall_table |= (i) << (seq[i] * 4);
	return 0;
}
int hall_test(hall_t* self, int cycle)
{
	while ((cycle--) > 0)
	{
		uint32_t u = GPIOA->ODR & (HALL_U);
		uint32_t v = GPIOA->ODR & (HALL_V);
		uint32_t w = GPIOA->ODR & (HALL_W);
		//printf("%d %d %d=%d\r\n", u, v, w, (int)hall_position(self));
		//msleep(10);
	}
	return 0;
}

#endif