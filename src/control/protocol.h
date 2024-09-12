#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define REBOOT_MAGIC_WORD 0xFEFFFF
#define CONFIG_PARAMETERS 1
#define UPDATE_FIRMWARE 2

#define PROGRAM_UPDATE_FLAG_ADDRESS 0x08003000 // 12k-14k
#define FLASH_UPDATE  0x5A5A5A5A
#define FLASH_UPDATE_COMPLETE 0xFFFFFFFF

// pragma pack(1)：确保结构体成员按照1字节对齐，避免因编译器优化导致的内存对齐问题。
#pragma pack(1)
// cmd_t：命令结构体，包含控制字、扭矩、电压、PWM、保留字段和CRC校验码。
typedef struct {
	struct
	{
		uint16_t cmd : 8;
		uint16_t enable : 1;
		uint16_t index_confirm : 1;
		uint16_t fault_clear : 1;
		uint16_t direct_pwm : 1;
	}  ctl_word;
	uint16_t torque;
	uint16_t vbus;
	uint16_t pwm;

	uint16_t resvd[3];
#ifndef HW_CRC
	uint16_t crc;
#endif

}cmd_t;
// fbk_t：反馈结构体，包含状态位字段、电流、扭矩、速度、位置、错误代码和CRC校验码
typedef struct 
{
	struct
	{
		uint8_t enabled : 1;
		uint8_t index_found : 1;/* when index_found bit set, the speed is overwritten by position offset*/
		uint8_t saturation : 1;
		uint8_t fault : 1;
		uint8_t pos_latched_value : 1;
	} status;
	uint8_t ibus;
	uint16_t torque;
	int32_t speed;
	int32_t position;
	int16_t error_code;
#ifndef HW_CRC
	uint16_t crc;
#endif
} fbk_t;
#pragma pack()
/* 14byte * 7 = 98 */


/*motor parameters*/
/*spi send when power on*/

#define CONFIG_PARAMETER 1
#define UPDATE_FIRMWARE 2
#define CALIBRATION 3
typedef struct{
	uint16_t function; // 1: config motor parameters 2: update firmware 3: calibration
	uint16_t ctrl_word; // 1: read 2: write

	struct{
		uint32_t encoder_init;
		uint32_t encoder_pos;
		uint32_t encoder_spd;
		

		uint32_t hall_table;
		uint32_t resvd;
		uint32_t motor_ppr;
		uint32_t position_ppr;
	}encoder;
	
	struct
	{
		uint16_t kp; //*1000
		uint16_t ki; //*1000
		uint32_t bandwith;
	}pid;

	struct
	{
		uint32_t invert;
		uint32_t poles;
		uint32_t back_emf_constant;
		int32_t  electric_offset;
		uint32_t resistance; /* phase, ohm */ //*1000000
		uint32_t inductance; /* phase, mH *///*1000000
	}parameter;

	struct 
	{
		int16_t pos_latch_function;
		int16_t pos_latch_type;
	}pos_latch;
	
	uint16_t calibrate_ma;
	uint16_t resvd[28];
	uint16_t crc;
}motor_para_t;

// 固件更新结构体
typedef struct{

	uint16_t function; // 1: config motor parameters 2: update firmware
	int16_t sequence; // 1: read 2: write

	uint16_t data[61];
	uint16_t crc;
}firmware_update_t;


#endif