#ifndef __HALL_H__
#define __HALL_H__
#include "encoder.h"
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