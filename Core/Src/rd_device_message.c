/*
 * rd_flash.c
 *
 *  Created on: Oc 06, 2021
 *      Author: Dat_UTC
 */


#include "rd_device_message.h"
#include "rd_flash.h"
#include "rd_gw_message.h"
#include "max44009_sensor.h"
#include "sht30_sensor.h"
#include "rd_ccs811.h"

//extern unsigned char net_key[16];
//extern unsigned int unicast_ID;
//extern unsigned char join_accept_flag;
//extern unsigned int Light_sensor_data;
//extern sht30_data_tdef Temp_hum_sensor_data;
//extern rd_ccs811_data_t Co2_sensor_data;
//extern unsigned int Ground_sensor_data;
//uint8_t device_addr[8] = {0};



#if(RD_LORA_MODEL_SEL != RD_LORA_MODEL_GW)


void device_get_mac_id (unsigned char *mac){
	uint32_t mac_id[2];
	mac_id[0] = HAL_GetUIDw1();
	mac_id[1] = HAL_GetUIDw2();
	mac = (unsigned char *)mac_id;
}

	#if (RD_LORA_MODEL_SEL == RD_LORA_MODEL_LIGHT_SENSOR)
			void test_control (){
				
			}
	#elif (RD_LORA_MODEL_SEL == RD_LORA_MODEL_TEMP_HUM_SENSOR)
			
	#elif (RD_LORA_MODEL_SEL == RD_LORA_MODEL_CO2_SENSOR)
			
	#elif (RD_LORA_MODEL_SEL == RD_LORA_MODEL_GROUND_SENSOR)
			
	#endif
#endif

