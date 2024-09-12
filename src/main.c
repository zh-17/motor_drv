#include "bsp.h"
#include "control/motor.h"
#include "stdio.h"

motor_t motor;
#define BSRR_VAL 0xC000   //设置或清除特定引脚的状态
int main()
{
	board_init();		
	motor_init(&motor); 
	printf("mcu start\r\n");
	// pmsm_calibrate(&motor, 3000);
	motor_enable(&motor); 
	motor_set_target(&motor, 500);
	while (1)
		;
}

// 定点数运算在嵌入式系统中广泛使用，因为它们提供了一种在没有硬件浮点单元的情况下进行精确数学运算的方法。
// 定点数的使用确保了算法的精确性和效率，这对于实时电机控制系统来说至关重要。
