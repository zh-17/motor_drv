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
static inline void motor_get_feedback(motor_t* self)//计算fbk成员变量 获得三相电流
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
}






static char pmsm_init_find(motor_t* self);
static void pmsm_output(motor_t* self, fix16_t uq, fix16_t ud, char correct_dead)//逆Park变换、SVPWM
{
	fix16_t a = fix16_add(fix16_mul(uq, uq), fix16_mul(ud, ud));//uq 和 ud 的平方和 a。 这个计算目的是确定电压向量的大小。
	if (a > F16(0.75 * MAX_PWM_DUTY * MAX_PWM_DUTY))//饱和检查：
	{
		a = fix16_div(F16(1.7320508075688772935274463415059 / 2 * MAX_PWM_DUTY), fix16_sqrt(a));
		uq = fix16_mul(uq, a);
		ud = fix16_mul(ud, a);
		self->state.saturation = 1;
	}
	else
		self->state.saturation = 0;
	/** inverse park transformation */
	fix16_t ualpha = fix16_sub(fix16_mul(ud, self->theta.cosine), fix16_mul(uq, self->theta.sine));//逆Park变换 将 uq 和 ud 转换为α-β坐标系下的电压分量 ualpha 和 ubeta。
	fix16_t ubeta = fix16_add(fix16_mul(ud, self->theta.sine), fix16_mul(uq, self->theta.cosine));

	/** in the frame of the two phase, we inverse the U V W */
	/* 60 degree Sector determination */
	/* http://wenku.baidu.com/view/9d420682bceb19e8b8f6ba9d.html?re=view page 11*/
#define SQRT3 (1.7320508075688772935274463415059)//N=4C+2B+A  即 √3 的近似值
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

	fix16_t ta = F16(0.5), tb = F16(0.5), tc = F16(0.5);//ta、tb、tc 代表电机三相的电压或者PWM占空比
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
static char pmsm_electric(motor_t* self)//根据pos计算电角度
{
	encoder_t motor_encoder = self->encoder.motor;//ABS_ENCODER=8    encoder_t 定义的是 char  
	encoder_t hall_encoder = self->encoder.init;  //ABS_ENCODER=8

	int64_t pos = encoder_position(motor_encoder);
	int64_t offset = encoder_offset(motor_encoder);  //0
	if (offset != VALUE_NOT_USEABLE)  //VALUE_NOT_USEABLE (((int64_t)1)<<63)
	{
		pos += offset;//将编码器位置 pos 与偏移量 offset 相加
		pos -= self->parameter.electric_offset;//减去电机参数中的电角度偏移
		if (self->parameter.invert) pos = -pos;//invert 标志为真，则取反 pos 值，用于处理电机旋转方向的反转。
		pos = pos % self->parameter.ppr;//将位置值对每圈脉冲数取模，以确保角度在0到一圈的范围内。
		self->theta.value = fix16_from_int(pos) / self->parameter.ppr;//获得每脉冲的角度增量。
		self->theta.value = fix16_mul(fix16_from_int(self->parameter.poles), self->theta.value);//将每脉冲的角度增量乘以电机的极对数,转换为电机的电气角度。
		self->theta.value =  fix16_mul(self->theta.value, F16(2 * M_PI));//电气角度转换为弧度，通过乘以 2 * M_PI（π的两倍，即一圈的弧度数）。
		return 1;
	}
	else if (hall_encoder == 0)
	{

	}
	else
		motor_error(self, HW_CONFIG_ERROR);

	return 0;
}
static void pmsm_current(motor_t* self)// Clarke变换和Park变换
{
	//clarke 变换
	fix16_t clarke_alpha = fix16_sub(0, self->fbk.ia);
	fix16_t clarke_beta = fix16_add(fix16_mul(self->fbk.ib, F16(-2)), clarke_alpha);
	clarke_beta = fix16_mul(clarke_beta, F16(0.57735026918962576450914878050196));
	
	/** park transformation
	*      d = alpha * cos(t) + beta * sin(t)
	*      q = beta * cos(t) - alpha * sin(t)*/
	self->fbk.id = fix16_add(fix16_mul(clarke_alpha, self->theta.cosine), fix16_mul(clarke_beta, self->theta.sine));
	self->fbk.iq = fix16_sub(fix16_mul(clarke_beta, self->theta.cosine), fix16_mul(clarke_alpha, self->theta.sine));
}
static void pmsm_update(motor_t* self)//PID控制
{
	if (!pmsm_electric(self)) 
	{
		self->line.rs_now = self->parameter.resistance;//出错执行
		return;
	}
	self->theta.cosine = fix16_cos(self->theta.value);//cosθ
	self->theta.sine = fix16_sin(self->theta.value);//sinθ

	fix16_t rated_vbus = self->fbk.vbus;// 25
	pmsm_current(self);

//#ifndef 是 "if not defined" 的缩写
#ifndef OPENLOOP
	fix16_t error = fix16_sub(self->pid._target, self->fbk.iq);//目标电流与反馈电流之差
	if (error < driver_fbk_idead(self) && error > -driver_fbk_idead(self))//如果误差在死区内，则将误差设置为0  -0.1---0.1
		error = F16(0);
	self->pid._ui_iq = fix16_add(self->pid._ui_iq, fix16_mul(error, self->pid.ki));//累计偏差 只用了PI控制  pid._ui_iq =pid._ui_iq + e*ki
	fix16_t qs = fix16_add(fix16_mul(error, self->pid.kp), self->pid._ui_iq);//PI控制公式 u = Kp * e + pid._ui_iq 
	if (qs > rated_vbus)//进行饱和检查，确保不会超过额定电压。 解决积分饱和问题
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
	if (error < driver_fbk_idead(self) && error > -driver_fbk_idead(self)) //-0.1---0.1
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
	/* 计算实时电阻*/
 	fix16_t i = fix16_abs(self->fbk.iq);
 	fix16_t u = fix16_abs(qs);
 	if (u > F16(0.2) || i > F16(0.2))
 		self->line.rs_now = fix16_div(u, i); // u/i=电阻
 	else
 		self->line.rs_now = 0;
}








static void __pmsm_move(motor_t* self, fix16_t spd, fix16_t duty)//通过改变角度来移动电机
{
	self->theta.cosine = fix16_cos(self->theta.value);
	self->theta.sine = fix16_sin(self->theta.value);
	pmsm_output(self, duty, 0, 0);
	msleep(1);
	self->theta.value = fix16_add(self->theta.value, spd);//更新角度
	while (self->theta.value > F16(M_PI)) //角度归一化
		self->theta.value = fix16_sub(self->theta.value, F16(2 * M_PI));
	while (self->theta.value < F16(-M_PI)) 
		self->theta.value = fix16_add(self->theta.value, F16(2 * M_PI));
}
int pmsm_self_test(motor_t* self, fix16_t duty)//应用一个占空比来测试电机，并读取反馈。
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
//代码通过施加已知电压并测量电流，利用稳态电流计算电阻R，利用瞬态响应计算电感L。
static int parameter_identify_LR(fix16_t *i_sample, fix16_t v_sample, double step, size_t sample_size, double* R, double* T)
{
	//计算电流稳态值 I0 
	int i;
	double I0 = 0;
	for (i = 0; i < sample_size; ++i)
	{
		I0 += fix16_to_float(i_sample[i]);
		if (I0 > fix16_to_float(i_sample[i]) * (i + 1) * 0.90 && i > 100)
			break;
	}
	//和电压稳态值 Vs
	double Is = 0;
	double Vs = 0;
	int cnt = 0;
	for (; i < sample_size; ++i)
	{
		Is += fix16_to_float(i_sample[i]);
		Vs += fix16_to_float(v_sample);
		cnt++;
	}

	// 计算电感 T
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
int pmsm_calibrate(motor_t* self, int32_t mA)//校准电机的参数和编码器。
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
	//逐步增加扭矩来达到设定的电流目标。如果达到了电流目标，则校准成功并输出使用的扭矩值；如果增加扭矩超过了某个阈值（0.7）仍未达到电流目标，则认为校准失败，并执行相应的错误处理。
	if (1)
	{
		while (self->fbk.iq < fix16_mul(F16(mA), F16(0.001)))
		{
			__pmsm_move(self, 0, torque);
			torque = fix16_add(torque, F16(0.001)); //1A
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

	/** get the Rs */ //用于测量电机的电阻（Rs）和电感（Lr）
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

	/* get the PPR of QEP */ //用于确定电机每圈脉冲数（PPR）和极对数 PPR是编码器每旋转一圈产生的脉冲数量，而极对数是电机内部磁场极数的一半。
	if (1)
	{
		if (self->encoder.motor == ABZ_ENCODER)//针对 ABZ 编码器的情况
		{
			double theta = 0;
			encoder_clear_offset(self->encoder.motor);
			while (1)//使用 __pmsm_move 函数以小角度步进移动电机，并等待编码器产生偏移。
			{
				if (encoder_has_offset(self->encoder.motor))
					break;
				__pmsm_move(self, F16(0.01), torque);
			}
			int64_t offset1 = encoder_offset(self->encoder.motor);

			encoder_clear_offset(self->encoder.motor);
			while (1)//再次旋转电机直到产生偏移
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
		else if (self->encoder.motor == ABS_ENCODER || self->encoder.motor == PANASONIC_QEP_ENCODER)//针对 ABS 编码器或 Panasonic QEP 编码器的情况
		{
			double theta = 0;//初始化角度 theta 为 0。
			int64_t start_pos = encoder_position(self->encoder.motor);
			while (1)//旋转电机直到编码器覆盖至少一圈：
			{
				int64_t delta = encoder_position(self->encoder.motor) - start_pos;
				if (delta < (int64_t)0)
					delta = -delta;
				if (delta > motor_ppr(self))//如果变化量超过当前 PPR，则认为电机已旋转一圈。
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

	/** if hall used calibrate it */ //霍尔传感器通常用于检测电机转子的位置
	if (self->encoder.motor == HALL_ENCODER)//当霍尔传感器作为主要编码器时
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

		/* hall sequence is incremental 增量, if encoder is decremenal 递减, then invert the hall*/
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
		fix16_t theta = self->theta.value;//初始化 theta 为当前电机的角度值
		int64_t offset = 0;
		for (int i = 0; i < 3; ++i)//进行 3 次循环，每次增加或减少一个小的角度（0.02 弧度），然后移动电机并测量编码器的位置变化。
		{
			self->theta.value = fix16_add(self->theta.value, F16(0.02));//旋转一个较小的角度（正方向）。
			__pmsm_move(self, 0, torque);//使电机以给定的扭矩 torque 移动到这个新的角度。
			msleep(50);//确保电机有足够的时间达到新的位置并稳定下来。

			self->theta.value = theta;//回到初始位置。
			__pmsm_move(self, 0, torque);
			msleep(100);
			int64_t offset1 = encoder_position(self->encoder.motor);

			self->theta.value = fix16_sub(self->theta.value, F16(0.02));//旋转相同大小的角度（负方向）。
			__pmsm_move(self, 0, torque);
			msleep(50);

			self->theta.value = theta;//又回去
			__pmsm_move(self, 0, torque);
			msleep(100);
			int64_t offset2 = encoder_position(self->encoder.motor);
			offset += (offset1 + offset2) / 2;//这个平均值 offset 反映了电机在正负方向上旋转相同角度时编码器读数的变化。
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
		int64_t mul = (self->parameter.poles * offset) / self->parameter.ppr;//计算电气偏移量
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
	printf("Target torque is 0\r\n");//目标扭矩为0
	return 0;
}









void motor_init(motor_t* self)
{
	self->calibrate.sample_size = 0;
	self->calibrate.ia_buffer = 0;
	self->calibrate.index = 0;
	self->state.error_code = 0;
	motor_disable(self);
}
void motor_enable(motor_t* self)//设置PID参数并启用驱动器。
{
	if (self->state.error_code)
		return;
	if (self->state.enabled)
		return;
	fix16_t R = self->parameter.resistance;//inductance 电感  resistance 电阻 
	fix16_t L = fix16_div(self->parameter.inductance, F16(1000));
	fix16_t kscale = F16(1.0);
	fix16_t iscale = F16(1.0);
	fix16_t band = self->parameter.bandwith;
	self->pid.kp = fix16_mul(fix16_mul(fix16_mul(kscale, band), L), F16(1.5 * 2 * 3.141592653)); // `kp`（比例系数） kp=kscale * band * L * 1.5 * 2 * π
	self->pid.ki = fix16_mul(fix16_mul(iscale, self->pid.kp), fix16_div(R, L));
	self->pid.ki = fix16_div(self->pid.ki, fix16_from_int(driver_frequency(self))); //`ki`（积分系数） iscale * kp * R / L / driver_frequency

	self->pid._ui_iq = 0; //积分累加值
	self->pid._ui_id = 0;
	self->pid._target = F16(0);
	driver_enable(self);//启用电机驱动器。
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
void motor_update(motor_t* self)//电机的主循环，用于更新反馈和控制。pmsm_update(self);
{
	motor_get_feedback(self); //获取A B相电流
	if (self->state.calibrating)
	{
		if (self->calibrate.index < self->calibrate.sample_size && self->calibrate.ia_buffer)
			self->calibrate.ia_buffer[self->calibrate.index++] = self->fbk.ia;
		return;
	}

#ifndef DEBUG
	self->life_cnt++;//增加一个生命周期计数器。
	if (self->life_cnt > 3200)//如果计数器超过某个阈值（这里为 3200），则执行一系列操作来禁用中断、禁用电机、延迟、禁用故障中断，并最终通过 NVIC_SystemReset() 重置系统。
	{
		irq_disable(); //禁用中断
		motor_disable(self);
		for (uint16_t i = 0; i < 1000; i++);
		__disable_fault_irq(); //用于禁用所有的故障中断
		NVIC_SystemReset();
	}
#endif 

	if (!self->state.enabled)
		return;
	pmsm_update(self);


}
int32_t motor_ppr(motor_t* self)//获取电机的每圈脉冲数（PPR）表示电机转一圈时编码器产生的脉冲数量
{
	return self->parameter.ppr;
}
void motor_direct_output(motor_t* self, uint16_t pwm)
{
	self->direct_pwm = 1;//从pwm参数中提取三个8位的值，分别代表三相电机的U、V、W三相的PWM占空比。这是通过位操作实现的
	uint8_t u = (pwm & 0x000f);//获取pwm的最低4位。
	uint8_t v = (pwm >> 4) & 0x000f;//获取pwm的次低4位。
	uint8_t w = (pwm >> 8) & 0x000f;//获取pwm的最高4位。

	driver_output(self, u / 10.0, v / 10.0, w / 10.0);
}
