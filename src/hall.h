#ifndef __HALL_H__
#define __HALL_H__
#include "encoder.h"
// 它定义了一个结构体 hall_t 和一些与霍尔效应传感器（Hall sensors）相关的函数。霍尔效应传感器通常用于电机控制系统中，用于检测磁场的变化，从而确定电机轴的位置。
typedef struct
{
	//encoder_t __encoder;
	uint8_t _last_sect;
	uint8_t _sect_cnt;
	uint8_t _sect;
	uint8_t _index;
	uint32_t _hall_table;
} hall_t;
int hall_init(hall_t* self, uint32_t hall_table);
static inline void hall_set_table(hall_t* self, uint32_t hall_table)
{
	hall_init(self , hall_table);
}
static inline uint32_t hall_table(hall_t* self)
{
	return self->_hall_table;
}
int64_t hall_position(hall_t* self, uint32_t abs_pos);
void hall_invert_table(hall_t* self);
int hall_calibrate(hall_t* self);
int hall_calibrate_start(hall_t* self);
int hall_calibrate_zero(hall_t* self);
int hall_test(hall_t* self, int cycle);

#endif