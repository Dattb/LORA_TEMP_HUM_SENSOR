/*
 * rd_ccs811.c
 *
 *  Created on: Sep 17, 2021
 *      Author: Dat_UTC
 */

#include "rd_ccs811.h"



HAL_StatusTypeDef rd_ccs811_init(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef CHECK_ALIVE = HAL_ERROR;
    uint8_t START_data = CJMCU811_START;
    uint8_t CHECK_FIRMWARE;
    //Check firmware if not available then PULL DOWN RST pin to reset the sensor
    while(CHECK_ALIVE != HAL_OK) {
        CHECK_ALIVE = HAL_I2C_IsDeviceReady(&hi2c1,CJMCU811_ADDRESS,3,50);
        if (CHECK_ALIVE== HAL_BUSY || CHECK_ALIVE == HAL_ERROR) {
        	return HAL_ERROR;
        }
    }
    //Check ERROR bit
    HAL_I2C_Mem_Read(hi2c, CJMCU811_ADDRESS, CJMCU811_STATUS, 1, &CHECK_FIRMWARE, 1, 50);
    if((CHECK_FIRMWARE & 0x01u) == 0 ) { // check erro
        HAL_I2C_Master_Transmit(hi2c, CJMCU811_ADDRESS, &START_data, 1, 50);
        uint8_t MEAS_mode = MEASURE_MODE;
        HAL_I2C_Mem_Write(hi2c, CJMCU811_ADDRESS, CJMCU811_MEAS_MODE, 1, &MEAS_mode, 1,50);
        return HAL_OK;
    }
    else return HAL_ERROR;
}

rd_ccs811_data_t ccs811 = {0};
uint8_t Received_Data[8];
uint8_t check_have_data = 0;
rd_ccs811_data_t rd_ccs811_read_data (I2C_HandleTypeDef *hi2c) {

    uint8_t Status_Register = 0x00;
    HAL_I2C_Mem_Read(hi2c, CJMCU811_ADDRESS,CJMCU811_STATUS,1, &Status_Register,1,50);
    if ((Status_Register >> 3u & 1u) == 1) //check if new data is push in register in "DATA_READY" bit
		{
    	check_have_data = 1;
    	HAL_I2C_Mem_Read(hi2c, CJMCU811_ADDRESS,0x02,1,Received_Data,8,50);
        ccs811.co2 = (uint16_t)((Received_Data[0] << 8)| Received_Data[1]);
        ccs811.tvoc = (uint16_t)((Received_Data[2] << 8) | Received_Data[3]);
        ccs811.ss_current = Received_Data[6]>>2;
        return ccs811;
    }
    else{
    	check_have_data = 0;
    	return ccs811;
    }
}

