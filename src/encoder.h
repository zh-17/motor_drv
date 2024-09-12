#ifndef __ENCODER_H_
#define __ENCODER_H_

//TAMAGAWA
#define		ABS_RETURN_LEN		11
#define 	DATA_0	0x02   	/* ABS0, ABS1, ABS2 */
#define 	DATA_1 	0x8a 	/* ABM0, ABM1, ABM2 */
#define 	DATA_2  0x92	/* 		 ENID		*/
#define 	DATA_3  0x1a	/* ABS0, ABS1, ABS2, ENID, ABM0, ABM1, ABM2, ALMC */  //每位的含义
#define  	DATA_6  0x32	/* Writing to EEPROM   */
#define  	DATA_D  0xea	/* Readout from EEPROM */
#define  	DATA_7  0xba	/* Reset   */
#define  	DATA_8  0xc2	/* Reset   */
#define  	DATA_C  0x62	/* Reset   */
#define 	TAMAGAWA_CRC	0x101

//Panasonic
#define  PANASONIC_QEP_RETURN_LEN 9
#define  ID_5	0X2A  /* CF SF ABSA0 ABSA1 ABSA2 ABSS0 ABSS1 ABSS2 CRC*/
#define  ID_A	0x52  /* CF SF ABSA0 ABSA1 ABSA2 ENID1 ENDI2 ALMC CRC*/
#define  ID_B	0xDA  /* CF SF ABSA0 ABSA1 ABSA2 ABSS0 ABSS1 ABSS2 CRC*/
#define  ID_E	0xF2  /* CF SF ABSA0 ABSA1 ABSA2 ENID1 ENDI2 ALMC CRC*/


#endif