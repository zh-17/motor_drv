#include <bsp.h>
#include "motor.h"
#include <stdlib.h>
#include <math.h>
#define M_PI 3.141592653
#define ABS(x) ((x) > 0 ? (x) : (-(x)))
const float fir_coff[] = {
	0.256201154036449941,
	0.345470348027566820,
	0.345470348027566820,
	0.052858149908416349,
};
static inline void motor_error(motor_t* self, int16_t code)
{
	motor_disable(self);
	self->state.error_code = code;
}
static inline void motor_get_feedback(motor_t* self)
{
	static fix16_t scale = F16(0.001);
	self->fbk.ia = fix16_from_int(driver_ia(self));
	self->fbk.ia = fix16_mul(self->fbk.ia, scale);
	self->fbk.ib = fix16_from_int(driver_ib(self));
	self->fbk.ib = fix16_mul(self->fbk.ib, scale);
#if 0
	self->fbk.history_ia[0] = self->fbk.ia;
	self->fbk.history_ib[0] = self->fbk.ib;


	self->fbk.ia = self->fbk.history_ia[0] * fir_coff[0] + self->fbk.history_ia[1] * fir_coff[1] + self->fbk.history_ia[2] * fir_coff[2] + self->fbk.history_ia[3] * fir_coff[3];
	self->fbk.history_ia[1] = self->fbk.history_ia[0];
	self->fbk.history_ia[2] = self->fbk.history_ia[1];
	self->fbk.history_ia[3] = self->fbk.history_ia[2];


	self->fbk.ib = self->fbk.history_ib[0] * fir_coff[0] + self->fbk.history_ib[1] * fir_coff[1] + self->fbk.history_ib[2] * fir_coff[2] + self->fbk.history_ib[3] * fir_coff[3];
	self->fbk.history_ib[1] = self->fbk.history_ib[0];
	self->fbk.history_ib[2] = self->fbk.history_ib[1];
	self->fbk.history_ib[3] = self->fbk.history_ib[2];
#endif
	//self->fbk.vbus = fix16_from_int(300);
}


static char pmsm_init_find(motor_t* self);
static void pmsm_output(motor_t* self, fix16_t uq, fix16_t ud, char correct_dead)
{
	fix16_t a = fix16_add(fix16_mul(uq, uq), fix16_mul(ud, ud));
	if (a > F16(0.75 * MAX_PWM_DUTY * MAX_PWM_DUTY))
	{
		a = fix16_div(F16(1.7320508075688772935274463415059 / 2 * MAX_PWM_DUTY), fix16_sqrt(a));
		uq = fix16_mul(uq, a);
		ud = fix16_mul(ud, a);
		self->state.saturation = 1;
	}
	else
		self->state.saturation = 0;
	/** inverse park transformation */
	fix16_t ualpha = fix16_sub(fix16_mul(ud, self->theta.cosine), fix16_mul(uq, self->theta.sine));
	fix16_t ubeta = fix16_add(fix16_mul(ud, self->theta.sine), fix16_mul(uq, self->theta.cosine));

	/** in the frame of the two phase, we inverse the U V W */

	/* 60 degree Sector determination */
	/* http://wenku.baidu.com/view/9d420682bceb19e8b8f6ba9d.html?re=view page 11*/
#define SQRT3 (1.7320508075688772935274463415059)
	fix16_t u1 = fix16_mul(ubeta, F16(2.0 / SQRT3));
	fix16_t tmp = fix16_div(u1, F16(2));
	fix16_t u2 = fix16_sub(ualpha, tmp);
	fix16_t u3 = fix16_add(ualpha, tmp);
	uint16_t sector = 0;
	if (u1 > 0)
		sector = 1;
	if (u2 > 0)
		sector += 2;
	if (u3 < 0)
		sector += 4;

	fix16_t ta = F16(0.5), tb = F16(0.5), tc = F16(0.5);
	switch (sector) {
	case 3:
		tc = fix16_sub(F16(0.5), fix16_mul(F16(0.5), u3));
		ta = fix16_add(tc, u3);
		tb = fix16_add(tc, u1);
		break;
	case 1:
		tc = fix16_sub(F16(0.5), fix16_mul(F16(0.5), u1));
		ta = fix16_add(tc, u3);
		tb = fix16_add(tc, u1);
		break;
	case 5:
		ta = fix16_add(F16(0.5), fix16_mul(F16(0.5), u2));
		tc = fix16_sub(ta, u3);
		tb = fix16_sub(ta, u2);
		break;
	case 4:
		ta = fix16_add(F16(0.5), fix16_mul(F16(0.5), u3));
		tc = fix16_sub(ta, u3);
		tb = fix16_sub(ta, u2);
		break;
	case 6:
		tb = fix16_add(F16(0.5), fix16_mul(F16(0.5), u1));
		ta = fix16_add(tb, u2);
		tc = fix16_sub(tb, u1);
		break;
	case 2:
		tb = fix16_sub(F16(0.5), fix16_mul(F16(0.5), u2));
		ta = fix16_add(tb, u2);
		tc = fix16_sub(tb, u1);
		break;
	}
	if (correct_dead)
	{
// 		float idead = driver_fbk_idead() * 6;
// 		float dead = driver_output_dead(self);
// 		float ic = -self->fbk.ia - self->fbk.ib;
// 		if (self->fbk.ia > idead)
// 			ta -= dead;
// 		else if (self->fbk.ia < -idead)
// 			ta += dead;
// 		else
// 			ta -= dead * self->fbk.ia / idead;
// 
// 		if (self->fbk.ib > idead)
// 			tb -= dead;
// 		else if (self->fbk.ib < -idead)
// 			tb += dead;
// 		else
// 			tb -= dead * self->fbk.ib / idead;
// 
// 		if (ic > idead)
// 			tc -= dead;
// 		else if (ic < -idead)
// 			tc += dead;
// 		else
// 			tc -= dead * ic / idead;
	}
	driver_output(self, ta, tb, tc);
}
static char pmsm_electric(motor_t* self)
{
	/** get the output from the sensor */
	encoder_t motor_encoder = self->encoder.motor;
	encoder_t hall_encoder = self->encoder.init;

	int64_t pos = encoder_position(motor_encoder);
	int64_t offset = encoder_offset(motor_encoder);
	if (offset != VALUE_NOT_USEABLE)
	{
		pos += offset;
		pos -= self->parameter.electric_offset;
		if (self->parameter.invert) pos = -pos;
		pos = pos % self->parameter.ppr;
		self->theta.value = fix16_from_int(pos) / self->parameter.ppr;
		self->theta.value = fix16_mul(fix16_from_int(self->parameter.poles), self->theta.value);
		self->theta.value =  fix16_mul(self->theta.value, F16(2 * M_PI));
		return 1;
	}
// 	else if (hall_encoder)
// 	{
// 		int64_t hall_pos = encoder_position(hall_encoder);
// 		if (hall_pos == VALUE_NOT_USEABLE)
// 		{
// 			if (self->hall_init.error_cnt > 10)
// 				motor_error(self, HALL_ERROR);
// 			else
// 				self->hall_init.error_cnt++;
// 			return 0;
// 		}
// //  		else if (self->hall_init.offset == VALUE_NOT_USEABLE)
// //  		{
// //  			self->hall_init.offset = encoder_ppr(motor_encoder) * ((hall_pos + 0.5) * encoder_scale(hall_encoder) / 2 / M_PI);
// //  			self->hall_init.offset += driver_initial_motor_offset(self->parent._driver);
// //  		}
// //  		self->hall_init.error_cnt = 0;
// //  		pos = encoder_position(motor_encoder, 0) + self->motor_initial_offset;
// //  
// //  		pos -= self->parameter.electric_offset;
// //  		if (self->parameter.invert) pos = -pos;
// //  		self->theta.value = (pos % self->parameter.electric_ppr) * (2 * M_PI) / self->parameter.electric_ppr;
// 	}
	else if (hall_encoder == 0)
	{
// 		*found = pmsm_init_find(self);
// 		pos = encoder_position(motor_encoder, 0);
// 		int64_t pos_delta = pos - self->initial_pos;
// 		//if (self->parent.cia402->motor_data.invert) pos_delta = -pos_delta;
// 		self->theta.value = pos_delta % encoder_ppr(motor_encoder) * poles * encoder_scale(motor_encoder) + self->motor_initial_theta;
// 		if (self->parameter.invert)
// 			self->theta.value -= M_PI / 2;
// 		else
// 			self->theta.value += M_PI / 2;
// 		return 1;
	}
	else
		motor_error(self, HW_CONFIG_ERROR);

	return 0;
}
static void pmsm_current(motor_t* self)
{
	fix16_t clarke_alpha = fix16_sub(0, self->fbk.ia);
	fix16_t clarke_beta = fix16_add(fix16_mul(self->fbk.ib, F16(-2)), clarke_alpha);
	clarke_beta = fix16_mul(clarke_beta, F16(0.57735026918962576450914878050196));
	
	/** park transformation
	*      d = alpha * cos(t) + beta * sin(t)
	*      q =-alpha * sin(t) + beta * cos(t) */
	self->fbk.id = fix16_add(fix16_mul(clarke_alpha, self->theta.cosine), fix16_mul(clarke_beta, self->theta.sine));
	self->fbk.iq = fix16_sub(fix16_mul(clarke_beta, self->theta.cosine), fix16_mul(clarke_alpha, self->theta.sine));
}
static void pmsm_update(motor_t* self)
{
	if (!pmsm_electric(self))
	{
		self->line.rs_now = self->parameter.resistance;
		return;
	}
	self->theta.cosine = fix16_cos(self->theta.value);
	self->theta.sine = fix16_sin(self->theta.value);

	fix16_t rated_vbus = self->fbk.vbus;

	pmsm_current(self);
#ifndef OPENLOOP
	fix16_t error = fix16_sub(self->pid._target, self->fbk.iq);
	if (error < driver_fbk_idead(self) && error > -driver_fbk_idead(self))
		error = F16(0);
	self->pid._ui_iq = fix16_add(self->pid._ui_iq, fix16_mul(error, self->pid.ki));
	fix16_t qs = fix16_add(fix16_mul(error, self->pid.kp), self->pid._ui_iq);

	if (qs > rated_vbus)
	{
		self->pid._ui_iq = fix16_sub(self->pid._ui_iq, fix16_sub(qs, rated_vbus));
		if (self->pid._ui_iq < 0)
			self->pid._ui_iq = 0;
		qs = rated_vbus;
	}
	else if (qs < -rated_vbus)
	{
		self->pid._ui_iq += (-rated_vbus - qs);
		if (self->pid._ui_iq > 0)
			self->pid._ui_iq = 0;
		qs = -rated_vbus;
	}


	error = fix16_sub(0, self->fbk.id);
	if (error < driver_fbk_idead(self) && error > -driver_fbk_idead(self))
		error = F16(0);
	self->pid._ui_id = fix16_add(self->pid._ui_id, fix16_mul(error, self->pid.ki));
	fix16_t ds = fix16_add(fix16_mul(error, self->pid.kp), self->pid._ui_id);
	if (ds > rated_vbus)
	{
		self->pid._ui_id = fix16_sub(self->pid._ui_id, fix16_sub(ds, rated_vbus));
		if (self->pid._ui_id < 0)
			self->pid._ui_id = 0;
		ds = rated_vbus;
	}
	else if (ds < -rated_vbus)
	{
		self->pid._ui_id += (-rated_vbus - ds);
		if (self->pid._ui_id > 0)
			self->pid._ui_id = 0;
		ds = -rated_vbus;
	}

#else
	fix16_t ds = 0;
	fix16_t qs = self->pid._target;
#endif
	pmsm_output(self, fix16_div(qs, rated_vbus), fix16_div(ds, rated_vbus), 0);

	/* motor line*/
 	fix16_t i = fix16_abs(self->fbk.iq);
 	fix16_t u = fix16_abs(qs);
 	if (u > F16(0.2) || i > F16(0.2))
 		self->line.rs_now = fix16_div(u, i);
 	else
 		self->line.rs_now = 0;
}

static void __pmsm_move(motor_t* self, fix16_t spd, fix16_t duty)
{
	self->theta.cosine = fix16_cos(self->theta.value);
	self->theta.sine = fix16_sin(self->theta.value);
	pmsm_output(self, duty, 0, 0);
	msleep(1);
	self->theta.value = fix16_add(self->theta.value, spd);
	while (self->theta.value > F16(M_PI)) 
		self->theta.value = fix16_sub(self->theta.value, F16(2 * M_PI));
	while (self->theta.value < F16(-M_PI)) 
		self->theta.value = fix16_add(self->theta.value, F16(2 * M_PI));
}
int pmsm_self_test(motor_t* self, fix16_t duty)
{
	self->theta.value = F16(0);
	self->theta.cosine = fix16_cos(self->theta.value);
	self->theta.sine = fix16_sin(self->theta.value);
	motor_enable(self);
	pmsm_output(self, duty, 0, 0);
	msleep(1000);
	
	motor_get_feedback(self);
	pmsm_current(self);

	int a = driver_ia(self);
	int b = driver_ib(self);
	motor_disable(self);

	printk("%f, %f, %d, %d\n", 
		fix16_to_float(self->fbk.iq), 
		fix16_to_float(self->fbk.id), a, b);
	
}
static int parameter_identify_LR(fix16_t *i_sample, fix16_t v_sample, double step, size_t sample_size, double* R, double* T)
{
	int i;
	double I0 = 0;
	for (i = 0; i < sample_size; ++i)
	{
		I0 += fix16_to_float(i_sample[i]);
		if (I0 > fix16_to_float(i_sample[i]) * (i + 1) * 0.90 && i > 100)
			break;
	}

	double Is = 0;
	double Vs = 0;
	int cnt = 0;
	for (; i < sample_size; ++i)
	{
		Is += fix16_to_float(i_sample[i]);
		Vs += fix16_to_float(v_sample);
		cnt++;
	}

	*R = Vs / Is;
	Vs = Vs / cnt;
	double sum_y = 0;
	double sum_i = 0;
	for (i = 0; i < sample_size; ++i)
	{
		if ((1 - (*R) * fix16_to_float(i_sample[i]) / Vs) < 0.1)
			break;
		sum_i += i * step;
		sum_y += log(1 - (*R) * fix16_to_float(i_sample[i]) / Vs);
	}
	*T = -sum_i / sum_y;

	return 0;
}
int pmsm_calibrate(motor_t* self, int32_t mA)
{
	self->calibrated = 1;
	motor_enable(self);
	self->state.calibrating = 1;
	self->theta.value = F16(2 * M_PI / 3);
	fix16_t torque = 0;
	/* get the duty for the current */
#if 0
	while (1)
	{
		driver_enable(1);
		driver_output(self, 0.4, 0.5, 0.5);
		msleep(500);
	}
#endif
#if 1
	if (1)
	{
		while (self->fbk.iq < fix16_mul(F16(mA), F16(0.001)))
		{
			__pmsm_move(self, 0, torque);
			torque = fix16_add(torque, F16(0.001));
			msleep(20);
			pmsm_current(self);

			if (torque > F16(0.7))
			{
				self->parameter.electric_offset = 12345678;
				self->parameter.resistance = self->fbk.iq;
				self->parameter.poles = mA;
				motor_error(self, MOTOR_CALIBRATE_NO_CURRENT);
				return -1;
			}
		}
		printk("torque=%f\r\n", fix16_to_float(torque));
	}

	/** get the Rs */
	if (1)
	{
		driver_output(self, F16(0.5), F16(0.5), F16(0.5));
		msleep(200);
		driver_output(self, 
			fix16_sub(F16(0.5), fix16_div(torque, F16(2.0))),
			F16(0.5), 
			fix16_add(F16(0.5), fix16_div(torque, 2.0)));

		/* allocate the calibrate buffer */
		fix16_t* ptr = malloc(sizeof(float) * self->calibrate.sample_size);
		if (!ptr)
			motor_error(self, MEMORY_ERROR);
		irq_disable();
		self->calibrate.index = 0;
		self->calibrate.sample_size = driver_frequency(self) / 10;
		self->calibrate.ia_buffer = ptr;
		irq_enble();
		
		fix16_t v_sample = F16(48);
		v_sample = fix16_mul(v_sample, fix16_sub(torque, driver_output_dead2(self)));
		
		while (self->calibrate.index < self->calibrate.sample_size)
			msleep(10);
		driver_output(self, F16(0.5), F16(0.5), F16(0.5));

		double T, R;
		parameter_identify_LR(self->calibrate.ia_buffer, v_sample,
			1.0 / driver_frequency(self), self->calibrate.sample_size,
			&R, &T);
		free(self->calibrate.ia_buffer);

		/* free the calibrate buffer */
		irq_disable();
		self->calibrate.sample_size = 0;
		self->calibrate.ia_buffer = 0;
		self->calibrate.index = 0;
		irq_enble();

		self->parameter.resistance = fix16_from_float(R / 2);
		self->parameter.inductance = fix16_from_float(T * R / 2 * 1000);

		printk("R=%f Ohm, L=%f mH\r\n",
			fix16_to_float(self->parameter.resistance),
			fix16_to_float(self->parameter.inductance));
	}


	/* get the PPR of QEP */
	if (1)
	{
		if (self->encoder.motor == ABZ_ENCODER)
		{
			double theta = 0;
			encoder_clear_offset(self->encoder.motor);
			while (1)
			{
				if (encoder_has_offset(self->encoder.motor))
					break;
				__pmsm_move(self, F16(0.01), torque);
			}
			int64_t offset1 = encoder_offset(self->encoder.motor);

			encoder_clear_offset(self->encoder.motor);
			while (1)
			{
				if (encoder_has_offset(self->encoder.motor))
					break;
				__pmsm_move(self, F16(0.01), torque);
				theta += 0.01;
			}
			int64_t offset2 = encoder_offset(self->encoder.motor);

			int32_t ppr = offset1 - offset2;
			int32_t poles = theta / (M_PI * 2) + 0.5;
			if (ppr < 0)
			{
				self->parameter.invert = 1;
				ppr = -ppr;
			}
			else
				self->parameter.invert = 0;
			self->parameter.ppr = ppr;
			self->parameter.poles = poles;
			printk("ppr = %d, poles = %d, invert = %d\n", ppr, poles, self->parameter.invert);
		}
		else if (self->encoder.motor == ABS_ENCODER || self->encoder.motor == PANASONIC_QEP_ENCODER)
		{
			double theta = 0;
			int64_t start_pos = encoder_position(self->encoder.motor);
			while (1)
			{
				int64_t delta = encoder_position(self->encoder.motor) - start_pos;
				if (delta < (int64_t)0)
					delta = -delta;
				if (delta > motor_ppr(self))
				{
					break;
				}
				__pmsm_move(self, F16(0.01), torque);
				theta += 0.01;
			}
			int32_t poles = theta / (M_PI * 2) + 0.5;
			if (encoder_position(self->encoder.motor) < start_pos)
				self->parameter.invert = 1;
			else
				self->parameter.invert = 0;
			self->parameter.ppr = motor_ppr(self);
			self->parameter.poles = poles;
			printk("ppr = %d, poles = %d, invert = %d\n", motor_ppr(self), poles, self->parameter.invert);
		}
	}

	/** if hall used calibrate it */
	if (self->encoder.motor == HALL_ENCODER)
	{//only has hall encoder
			hall_t* hall = (hall_t *)&(self->hall_init.hall);
			if (hall_calibrate_start(hall))
			{
				motor_error(self, HALL_ERROR);
				goto CALIBRATION_END;
			}

			while (hall_calibrate(hall)) {
				__pmsm_move(self, F16(0.01), torque);
			}
	}
	else if (self->encoder.init == HALL_ENCODER)
	{//hall encoder as initial encoder
		/*hall*/
		hall_t* hall = (hall_t*)&(self->hall_init.hall);

		/* first calibrate the sequence of the hall encoder*/
		int64_t start_pos = encoder_position(self->encoder.motor);

		if (hall_calibrate_start(hall))
		{
			motor_error(self, HALL_ERROR);
			goto CALIBRATION_END;
		}

		int ret;
		while ((ret = hall_calibrate(hall))) {
			__pmsm_move(self, F16(0.01), torque);
			if (ret < 0)
			{
				motor_error(self, HALL_ERROR);
				goto CALIBRATION_END;
			}
		}

		/* hall sequence is incremental, if encoder is decremenal, then invert the hall*/
		int hall_invert = 0;
		if (encoder_position(self->encoder.motor) < start_pos)
		{
			hall_invert_table(hall);
			printk("hall invert\r\n");
			hall_invert = 1;
		}

		/* then align the hall to the motor encoder */
		int64_t last_hall = hall_position(hall, 1);
		while (1)
		{
			start_pos = encoder_position(self->encoder.motor);

			if (hall_invert == 0 && hall_position(hall, 1) == 0 && last_hall == 5)
			{
				start_pos = start_pos % motor_ppr(self);
				printk("%d\r\n", (int32_t)start_pos);
				if (hall_calibrate_zero(hall) != 0)
					printk("HALL Zero setting Error\r\n");
				break;
			}
			else if (hall_invert == 1 && hall_position(hall, 1) == 0 && last_hall == 1)
			{
				start_pos = start_pos % motor_ppr(self);
				printk("%d\r\n", (int32_t)start_pos);
				if (hall_calibrate_zero(hall) != 0)
					printk("HALL Zero setting Error\r\n");
				break;
			}
			else
			{
				last_hall = hall_position(hall, 1);
				__pmsm_move(self, F16(0.01), torque);
				continue;
			}
		}

		printk("hall_table = 0x%x\r\n",self->hall_init.hall._hall_table);
	}

	/* get the electric offset*/
	if (1)
	{
		torque = fix16_mul(torque, F16(1.5));
		fix16_t theta = self->theta.value;
		int64_t offset = 0;
		for (int i = 0; i < 3; ++i)
		{
			self->theta.value = fix16_add(self->theta.value, F16(0.02));
			__pmsm_move(self, 0, torque);
			msleep(50);

			self->theta.value = theta;
			__pmsm_move(self, 0, torque);
			msleep(100);
			int64_t offset1 = encoder_position(self->encoder.motor);

			self->theta.value = fix16_sub(self->theta.value, F16(0.02));
			__pmsm_move(self, 0, torque);
			msleep(50);

			self->theta.value = theta;
			__pmsm_move(self, 0, torque);
			msleep(100);
			int64_t offset2 = encoder_position(self->encoder.motor);
			offset += (offset1 + offset2) / 2;
		}
		offset = offset / 3;
		offset += encoder_offset(self->encoder.motor);

		int32_t electric_ppr = self->parameter.ppr / self->parameter.poles;
		if (self->parameter.invert)
		{
			int64_t delta = -electric_ppr * fix16_to_float(fix16_div(theta, F16(2 * M_PI)));
			offset -= delta;
			offset -= electric_ppr / 4;
		}
		else
		{
			int64_t delta = electric_ppr * fix16_to_float(fix16_div(theta, F16(2 * M_PI)));
			offset -= delta;
			offset -= electric_ppr / 4;
		}
		int64_t mul = (self->parameter.poles * offset) / self->parameter.ppr;
		self->parameter.electric_offset = offset - (mul * self->parameter.ppr) / self->parameter.poles;
		printk("%f, %d\n", fix16_to_float(theta), (int32_t)self->parameter.electric_offset);
	}
#else
	self->parameter.resistance = 1.291417;
	self->parameter.inductance = 8.729873;
 	self->parameter.invert = 0;
	self->parameter.poles = 5;
	self->parameter.electric_offset = 14605;
	self->parameter.ppr = 131072;
#endif
	
CALIBRATION_END:

	motor_disable(self);

 	motor_enable(self);
 	self->pid._target = F16(0);
	printf("Target torque is 0\r\n");
	return 0;
}

void motor_init(motor_t* self)
{
	self->calibrate.sample_size = 0;
	self->calibrate.ia_buffer = 0;
	self->calibrate.index = 0;

	//self->encoder.motor = ABS_ENCODER;
	self->state.error_code = 0;


	motor_disable(self);
}
void motor_enable(motor_t* self)
{
	if (self->state.error_code)
		return;
	if (self->state.enabled)
		return;
	fix16_t R = self->parameter.resistance;
	fix16_t L = fix16_div(self->parameter.inductance, F16(1000));
	fix16_t kscale = F16(1.0);
	fix16_t iscale = F16(1.0);
	fix16_t band = self->parameter.bandwith;
	self->pid.kp = fix16_mul(fix16_mul(fix16_mul(kscale, band), L), F16(1.5 * 2 * 3.141592653)); // 1.5
	self->pid.ki = fix16_mul(fix16_mul(iscale, self->pid.kp), fix16_div(R, L));
	self->pid.ki = fix16_div(self->pid.ki, fix16_from_int(driver_frequency(self)));

	self->pid._ui_iq = 0;
	self->pid._ui_id = 0;
	//self->pid.kp /= 3;
	//self->pid.ki /= 3;
	self->pid._target = F16(0);
	driver_enable(self);
	self->state.enabled = 1;
	self->state.saturation = 0;
}
void motor_disable(motor_t* self)
{
	if (!self->state.enabled)
		return;
	self->state.calibrating = 0;
	self->state.enabled = 0;
	self->state.saturation = 0;
	self->pid._target = F16(0);
	self->pid._ui_id = 0;
	self->pid._ui_iq = 0;
	driver_disable(self);
}

void motor_update(motor_t* self)
{
	motor_get_feedback(self);

	if (self->state.calibrating)
	{
		if (self->calibrate.index < self->calibrate.sample_size && self->calibrate.ia_buffer)
			self->calibrate.ia_buffer[self->calibrate.index++] = self->fbk.ia;
		return;
	}

#ifndef DEBUG
	self->life_cnt++;
	if (self->life_cnt > 3200)
	{
		irq_disable();
		motor_disable(self);
		for (uint16_t i = 0; i < 1000; i++);
		__disable_fault_irq();
		NVIC_SystemReset();
	}
#endif 

	if (!self->state.enabled)
		return;
	pmsm_update(self);
}

int32_t motor_ppr(motor_t* self)
{
	return self->parameter.ppr;
}

void motor_direct_output(motor_t* self, uint16_t pwm)
{
	self->direct_pwm = 1;
	uint8_t u = (pwm & 0x000f);
	uint8_t v = (pwm >> 4) & 0x000f;
	uint8_t w = (pwm >> 8) & 0x000f;

	driver_output(self, u / 10.0, v / 10.0, w / 10.0);
}
