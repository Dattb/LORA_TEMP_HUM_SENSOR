/*
 * rd_ccs811.h
 *
 *  Created on: Sep 17, 2021
 *      Author: Dat_UTC
 */

//#ifndef __CJMCU811

#include "main.h"
extern I2C_HandleTypeDef hi2c1;

typedef struct {
	uint16_t	co2;
	uint16_t	tvoc;
	uint8_t		ss_current;
}rd_ccs811_data_t;

//#define MEASURE_MODE 0x10u // Mode 1 measure every second without interrupt
#define MEASURE_MODE 0x20u // Mode 2 measure every 10 seconds without interrupt
//#define MEASURE_MODE 0x30u // Mode 3 measure every 60 seconds without interrupt
//Interrupt connects to INT pin help you to recognize if INT is HIGH means NO DATA otherwise there is DATA (INT is active-low)

#define CJMCU811_ADDRESS (0x5Au) << 1u //Default Address when ADD pin pull down to LOW
//#define CJMCU811_ADDRESS (0x5Bu) << 1u //Address when ADD pin pull up to HIGH
#define CJMCU811_STATUS 0x00u
#define CJMCU811_SW_RESET 0xFFu  //Enter BOOT mode
#define CJMCU811_MEAS_MODE 0x01u
#define CJMCU811_ALG_RESULT_DATA 0x02u
#define CJMCU811_ENV_DATA 0x05u  //Temperature and Humidity of ENV
#define CJMCU811_NTC 0x06u
#define CJMCU811_START	0xF4u

//Check Firmware and Init the MEASURE MODE

HAL_StatusTypeDef rd_ccs811_init(I2C_HandleTypeDef *hi2c);

rd_ccs811_data_t rd_ccs811_read_data (I2C_HandleTypeDef *hi2c) ;


//#endif
