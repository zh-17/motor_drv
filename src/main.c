#include "bsp.h"
#include "control/motor.h"
#include "stdio.h"
/* for fix math add macro MATH_FIX16 FIXMATH_NO_OVERFLOW FIXMATH_NO_ROUNDING */

motor_t motor;
#define BSRR_VAL 0xC000
int main()
{
	//NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3800);
	//power_on_init();
//	while (!get_devide_id());

 	board_init();
	motor_init(&motor);
	printf("mcu start\r\n");
	//pmsm_calibrate(&motor, 3000);
	motor_enable(&motor);
	motor_set_target(&motor, 500);
	while (1);
}