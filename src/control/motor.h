#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__
#include "types.h"
#include <fix16.h>
#include "bsp.h"
#include "hall.h"
#define HALL_ERROR -1
#define HW_CONFIG_ERROR -2
#define MOTOR_CALIBRATE_NO_CURRENT -3
#define MEMORY_ERROR -4
#define FILTER_CONFIG_ERROR -5
typedef struct motor
{
	struct
	{
		char enabled;
		int error_code;
		char calibrating; //校准
		char saturation; //饱和
	} state;
	struct
	{
		fix16_t *ia_buffer;
		uint16_t sample_size;
		volatile uint16_t index;
	} calibrate;

	struct
	{
		encoder_t motor;
		encoder_t init;
		encoder_t spd;
		encoder_t pos;
		uint8_t id;
		int resolution;
	} encoder;
	struct
	{
		/** here is in A */
		fix16_t ia; //A相电流反馈 就是测A相电流
		fix16_t ib;

		fix16_t id;
		fix16_t iq;
		fix16_t vbus;
		fix16_t history_ia[4];
		fix16_t history_ib[4];
	} fbk;
	struct
	{
		fix16_t value;
		fix16_t cosine;
		fix16_t sine;
	} theta;
	struct
	{
		fix16_t rs_now; //当前线路电阻值
		int32_t rs_error_1;
		int32_t rs_error_2;
	} line;

// _target：目标值，通常是目标电流或速度。
// kp：比例增益，用于PI控制器的P部分。
// ki：积分增益，用于PI控制器的I部分。
// 积分系数 ki 是用于调节积分项的权重。而 _ui_iq 和 _ui_id 是 PID 控制器内部状态，用于存储积分项的累加值
// _umax：控制器输出的最大值，用于限制控制器输出，防止过饱和。
	struct
	{
		fix16_t _target;
		fix16_t kp;
		fix16_t ki;
		fix16_t _ui_id;
		fix16_t _ui_iq;
		fix16_t _umax;
	} pid;
	struct
	{
		int64_t offset;
		double motor_initial_theta;
		int64_t initial_pos;
		int32_t error_cnt;
		hall_t hall;
	} hall_init;

	struct
	{
		int32_t electric_offset;
		int32_t ppr;
		int32_t poles;
		char invert;

		fix16_t resistance; /* phase, ohm */
		fix16_t inductance; /* phase, mH */
		fix16_t bandwith;
	} parameter;
	uint16_t life_cnt;
	int8_t calibrated;
	int8_t direct_pwm;
} motor_t;

void motor_init(motor_t *self);
void motor_disable(motor_t *self);
void motor_enable(motor_t *self);
void motor_update(motor_t *self);
int32_t motor_ppr(motor_t *self);
int pmsm_calibrate(motor_t *self, int32_t mA);
void motor_direct_output(motor_t *self, uint16_t pwm);


 
extern motor_t motor;
// 内联函数在编译时会被插入到每个调用该函数的地方
// static 关键字限定了函数的作用域，使得函数只能在定义它的源文件内部被调用
static inline int16_t motor_torque(motor_t *self)
{
	return self->fbk.iq * 1000;
}
static inline void motor_set_target(motor_t *self, int16_t target_ma) //将毫安（mA）转换为安培（A）
{
	self->pid._target = fix16_div(fix16_from_int(target_ma), F16(1000)); 
}

static inline void motor_set_vbus(motor_t *self, uint16_t vbus)
{
	self->fbk.vbus = fix16_from_int(vbus);
}
static inline void motor_clear_error(motor_t *self)
{
	self->state.error_code = 0;
}

static inline char motor_enabled(motor_t *self)
{
	return self->state.enabled;
}
static inline char motor_fault(motor_t *self)
{
	return self->state.error_code;
}
static inline char motor_saturation(motor_t *self)
{
	return self->state.saturation;
}


#endif
