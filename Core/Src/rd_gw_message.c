/*
 * rd_flash.c
 *
 *  Created on: Sep 23, 2021
 *      Author: Dat_UTC
 */



#include "rd_flash.h"
#include "rd_gw_message.h"
#include "max44009_sensor.h"
#include "sht30_sensor.h"
#include "rd_ccs811.h"

extern unsigned char net_key[16];
extern unsigned int unicast_ID;
extern unsigned char join_accept_flag;
extern unsigned int Light_sensor_data;
extern sht30_data_tdef Temp_hum_sensor_data;
extern rd_ccs811_data_t Co2_sensor_data;
extern unsigned int Ground_sensor_data;
uint8_t device_addr[8] = {0};

#if(RD_LORA_MODEL_SEL == RD_LORA_MODEL_GW)
uint8_t *gw_Rx_join_request_message(uint8_t *par){
	uint16_t	opcode = par[1]<<8|par[0];
	if(opcode == OP_JOIN_REQUEST){
		gw_Tx_join_accept_message (par);
	}
	//gw_Tx_join_accept();
	return device_addr;
}

unsigned char send_message[32] = {0};

uint8_t * gw_Tx_join_accept_message (uint8_t *par){
	unsigned int des_addr = par[11]<<8|par[10];
	if(des_addr == GW_ADDR){
		for (char i=0;i<8;i++) device_addr[i] = par[2+i];
		send_message[0] = OP_JOIN_ACCEPT;
		send_message[1] = OP_JOIN_ACCEPT>>8;
		for (int i=0;i<8;i++){
			send_message[i+2] = device_addr[i];
		}
		send_message[2] = GW_ADDR;
		send_message[3] = GW_ADDR>>8;
		for (int i =0;i<8;i++){
				send_message[i+4] = device_addr[i];
		}
		for (int i =0;i<16;i++){
				send_message[i+12] = net_key[i];
		}
		send_message[28] = unicast_ID;
		send_message[29] = unicast_ID>>8;
	}
	//	*par = net_key;
	//	unicast_ID = 
	return 0;	
}

uint8_t *gw_Rx_join_accept_ACK_message(uint8_t *par){
	uint16_t	opcode = par[1]<<8|par[0];
	
	if(opcode == OP_JOIN_ACCEPT_ACK){
		join_accept_flag = 1;
	}
	
	return device_addr;
}

unsigned char Rx_sensor_data(unsigned char *par){
	uint16_t	opcode = par[1]<<par[0];
	if(opcode == OP_LIGHT_TO_GW_MESSAGE){
		Light_sensor_data = par[13]<<8|par[12];
	}
	else if(opcode == OP_TEMP_HUM_TO_GW_MESSAGE){
		Temp_hum_sensor_data.temperature = par[13]<<8|par[12];
		Temp_hum_sensor_data.humitidy    = par[15]<<8|par[14];
	}
	else if(opcode == OP_CO2_TO_GW_MESSAGE){
		Co2_sensor_data.co2 = par[13]<<8|par[12];
		Co2_sensor_data.tvoc = par[15]<<8|par[14];
		Co2_sensor_data.ss_current = par[16];
	}
	else if(opcode == OP_GROUND_TO_GW_MESSAGE){
		Ground_sensor_data = par[13]<<8|par[12];
	}
	return 0;
}

#endif

