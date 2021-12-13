/**
  ******************************************************************************
  * @file    subghz_phy_app.h
  * @author  MCD Application Team
  * @brief   Header of application of the SubGHz_Phy Middleware
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SUBGHZ_PHY_APP_H__
#define __SUBGHZ_PHY_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* MODEM type: one shall be 1 the other shall be 0 */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   0

#define REGION_EU868


#define RD_LORA_MODEL_SEL	  						RD_LORA_MODEL_TEMP_HUM_SENSOR




#define RD_LORA_MODEL_GW										0
#define RD_LORA_MODEL_LIGHT_SENSOR					1
#define RD_LORA_MODEL_TEMP_HUM_SENSOR				2
#define RD_LORA_MODEL_CO2_SENSOR				 		3
#define RD_LORA_MODEL_GROUND_SENSOR		 			4
#define RD_LORA_MODEL_NORMAL_LIGHT					5

#define RD_UP_LINK_FREQUENCY						921400000
#define RD_DOWN_LINK_FREQUENCY						922800000






#if (RD_LORA_MODEL_SEL == RD_LORA_MODEL_LIGHT_SENSOR)
	#define RD_OPCODE_SEND 			OP_LIGHT_TO_GW_MESSAGE	
	#define RD_TYPE_DEVICE			TYPE_DEVICE_LIGHT_SENSOR
#elif(RD_LORA_MODEL_SEL == RD_LORA_MODEL_TEMP_HUM_SENSOR)
	#define RD_OPCODE_SEND 			OP_TEMP_HUM_TO_GW_MESSAGE	
	#define RD_TYPE_DEVICE			TYPE_DEVICE_TEMP_HUM_SENSOR
#elif(RD_LORA_MODEL_SEL == RD_LORA_MODEL_CO2_SENSOR)
	#define RD_OPCODE_SEND 			OP_CO2_TO_GW_MESSAGE
	#define RD_TYPE_DEVICE	    TYPE_DEVICE_CO2_SENSOR	
#elif(RD_LORA_MODEL_SEL == RD_LORA_MODEL_GROUND_SENSOR)
	#define RD_OPCODE_SEND 			OP_GROUND_TO_GW_MESSAGE	
	#define RD_TYPE_DEVICE      TYPE_DEVICE_GROUND_HUM_SENSOR
#elif(RD_LORA_MODEL_SEL == RD_LORA_MODEL_NORMAL_LIGHT)
	#define RD_OPCODE_SEND   
	#define RD_TYPE_DEVICE			TYPE_DEVICE_NORMAL_NIGHT  
#endif




#define NUMBER_OF_OPCODE													20

#define GW_ADDR																				0x0001
#define OP_JOIN_REQUEST																0x0001	//join request,
#define OP_JOIN_ACCEPT																0x0002	//join accept
#define OP_JOIN_ACCEPT_ACK														0x0003	//join accept ACK
#define OP_DELETE_DEVICE															0x0004	// opcode xoa device

#define OP_LIGHT_TO_GW_MESSAGE												0x0100	//opcode //cam bien anh sang gui data ve gw
#define OP_GW_TO_LIGHT_MESSAGE												0x0101	//opcode //gw set tham so xuong cam bien anh sang
#define OP_TEMP_HUM_TO_GW_MESSAGE											0x0200	//opcode //cam bien temp hum gui data ve gw
#define OP_GW_TO_TEMP_HUM_MESSAGE											0x0201	//opcode //gw set tham so xuong cam bien temp hum
#define OP_CO2_TO_GW_MESSAGE													0x0300	//opcode //cam bien do am dat gui data ve gw
#define OP_GW_TO_CO2_MESSAGE													0x0301	//opcode //gw set tham so xuong cam bien do am dat
#define OP_GROUND_TO_GW_MESSAGE												0x0400	//opcode //cam bien CO2 gui 
#define OP_GW_TO_GROUND_MESSAGE												0x0401	//opcode //gw set tham so xuong cam bien CO2


#define OP_UART_SEND_LIGHT_ON												0x0500
#define OP_UART_SEND_LIGHT_OFF											0x0501




#define TYPE_DEVICE_LIGHT_SENSOR										  	0x0100
#define TYPE_DEVICE_TEMP_HUM_SENSOR											0x0200
#define TYPE_DEVICE_CO2_SENSOR											  	0x0300
#define TYPE_DEVICE_GROUND_HUM_SENSOR										0x0400

#define TYPE_DEVICE_NORMAL_NIGHT												0x0501


enum {
	NOT_JOINT = 0,
	JOINED = 1
};
typedef enum {
	gw_to_device = 0,
	device_to_gw = 1
}rd_direction_t;

typedef struct {
	unsigned int opcode ;
	int(*cb_cmd_function_t)(unsigned char *par,rd_direction_t Direction);
}rd_message_t;

int op_join_request_handle(unsigned char *par,rd_direction_t direction_handle);
int op_join_accept_handle(unsigned char *par,rd_direction_t direction_handle);
int op_join_accept_ack_handle(unsigned char *par,rd_direction_t direction_handle);
int op_delete_device_handle(unsigned char *par,rd_direction_t direction_handle);

int op_light_to_gw_handle(unsigned char *par,rd_direction_t direction_handle);
int op_gw_to_light_handle(unsigned char *par,rd_direction_t direction_handle);
int op_temp_hum_to_gw_handle(unsigned char *par,rd_direction_t direction_handle);
int op_gw_to_temp_hum_handle(unsigned char *par,rd_direction_t direction_handle);
int op_co2_to_gw_handle(unsigned char *par,rd_direction_t direction_handle);
int op_gw_to_co2_handle(unsigned char *par,rd_direction_t direction_handle);
int op_ground_to_gw_handle(unsigned char *par,rd_direction_t direction_handle);
int op_gw_to_ground_handle(unsigned char *par,rd_direction_t direction_handle);
int op_uart_send_light_off_handle(unsigned char *par,rd_direction_t direction_handle);
int op_uart_send_light_on_handle(unsigned char *par,rd_direction_t direction_handle);


#if defined( REGION_AS923 )

#define RF_FREQUENCY                                923000000 /* Hz */
#elif defined( REGION_AU915 )

#define RF_FREQUENCY                                915000000 /* Hz */

#elif defined( REGION_CN470 )

#define RF_FREQUENCY                                470000000 /* Hz */

#elif defined( REGION_CN779 )

#define RF_FREQUENCY                                779000000 /* Hz */

#elif defined( REGION_EU433 )

#define RF_FREQUENCY                                433000000 /* Hz */

#elif defined( REGION_EU868 )

#define RF_FREQUENCY                                868000000 /* Hz */

#elif defined( REGION_KR920 )

#define RF_FREQUENCY                                920000000 /* Hz */

#elif defined( REGION_IN865 )

#define RF_FREQUENCY                                865000000 /* Hz */

#elif defined( REGION_US915 )

#define RF_FREQUENCY                                915000000 /* Hz */

#elif defined( REGION_RU864 )

#define RF_FREQUENCY                                864000000 /* Hz */

#else
#error "Please define a frequency band in the compiler options."
#endif /* REGION_XXxxx */

#define TX_OUTPUT_POWER                             0        /* dBm */

#if (( USE_MODEM_LORA == 1 ) && ( USE_MODEM_FSK == 0 ))
#define LORA_BANDWIDTH                              1         /* [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved] */
#define LORA_SPREADING_FACTOR                       8         /* [SF7..SF12] */
#define LORA_CODINGRATE                             1         /* [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8] */
#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#elif (( USE_MODEM_LORA == 0 ) && ( USE_MODEM_FSK == 1 ))

#define FSK_FDEV                                    25000     /* Hz */
#define FSK_DATARATE                                50000     /* bps */
#define FSK_BANDWIDTH                               50000     /* Hz */
#define FSK_AFC_BANDWIDTH                           83333     /* Hz */
#define FSK_PREAMBLE_LENGTH                         5         /* Same for Tx and Rx */
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#else
#error "Please define a modem in the compiler subghz_phy_app.h."
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */

#define RX_TIMEOUT_VALUE                            500
#define TX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 32  /* Define the payload size here */
#define LED_PERIOD_MS                               200

#define TCXO_WORKAROUND_TIME_MARGIN                 10  /* 10ms margin */

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Init Subghz Application
  * @param None
  * @retval None
  */
	enum {
	ENCRYPT_DATA =0,
	DECRYPT_DATA =1,
};
void SubghzApp_Init(void);
void RD_AES_Init(void);
int rd_message_hande(unsigned char *par,rd_direction_t direction);
unsigned char *device_Tx_joinrequest_message();
void rd_error_call_back();
void rd_encrypt_aes_key_call_back();
void rd_set_aes_key (unsigned int *key);
unsigned int *netkey8_to_32(unsigned char *key_8);
void led_toggle(unsigned char mode);
void aes_encrypt_decrypt (unsigned int *key, unsigned int *data_in,unsigned int *data_out,unsigned char direct);
void device_flag_set();
void device_ack_message();
void led_blink(unsigned char time, unsigned char blink_flag);
void write_led(unsigned char mode);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /*__SUBGHZ_PHY_APP_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
