/*!
 * \file      subghz_phy_app.c
 *
 * \brief     Ping-Pong implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
/**
  ******************************************************************************
  *
  *          Portions COPYRIGHT 2020 STMicroelectronics
  *
  * @file    subghz_phy_app.c
  * @author  MCD Application Team
  * @brief   Application of the SubGHz_Phy Middleware
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "stm32_timer.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "app_version.h"
#include "../../Core/Src/rd_flash.h"
//#include "App"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

extern unsigned char device_send_message_cnt;
extern unsigned char device_send_flag;
extern int32_t dev_key_32;
extern uint32_t random_time;
extern unsigned char radio_free_flag;
unsigned char add_device_flag = 1;
unsigned char network_status  = NOT_JOINT;
uint8_t device_send_buff[32] = {0};
unsigned char rd_mac_id[8] = {8};	
unsigned char rd_netkey_char[16] = {0};
uint32_t rd_net_key_32bit[4] = {0};
unsigned short rd_device_id = 0;
uint32_t mac_id[2];

extern unsigned char save_flash_flag;
unsigned char dev_key [4] = {0};


rd_message_t rd_message_func_handle[NUMBER_OF_OPCODE] = { {OP_JOIN_REQUEST,op_join_request_handle},
																													{OP_JOIN_ACCEPT,op_join_accept_handle},
																													{OP_JOIN_ACCEPT_ACK,op_join_accept_ack_handle},
																													{OP_DELETE_DEVICE,op_delete_device_handle}, 
																													{OP_LIGHT_TO_GW_MESSAGE,op_light_to_gw_handle},
																													{OP_GW_TO_LIGHT_MESSAGE,op_gw_to_light_handle},
																													{OP_TEMP_HUM_TO_GW_MESSAGE,op_temp_hum_to_gw_handle},
																													{OP_GW_TO_TEMP_HUM_MESSAGE,op_gw_to_temp_hum_handle},
																													{OP_CO2_TO_GW_MESSAGE,op_co2_to_gw_handle},
																													{OP_GW_TO_CO2_MESSAGE,op_gw_to_co2_handle},
																													{OP_GROUND_TO_GW_MESSAGE,op_ground_to_gw_handle},
																													{OP_GW_TO_GROUND_MESSAGE,op_gw_to_ground_handle},
																													{OP_UART_SEND_LIGHT_ON,op_uart_send_light_on_handle},
																													{OP_UART_SEND_LIGHT_OFF,op_uart_send_light_off_handle}};

unsigned int rd_tx_opcode = OP_JOIN_REQUEST;
unsigned int rd_rx_opcode = 0;
unsigned char message_cnt = 0;

unsigned char radio_rx_flag = 1,radio_tx_flag = 1;
unsigned char radio_rx_cnt = 0;
uint8_t tx_cnt = 0;
extern unsigned char i2c_read_done_flag;
extern unsigned int sht30_temp;
extern unsigned int sht30_hum;
extern 	uint32_t data_max44009;
extern uint32_t ground_hum_data;

extern unsigned int ccs811_co2;
extern unsigned int ccs811_tvoc;
extern unsigned char ccs811_current;
extern unsigned char delete_device_flag;
/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  LOWPOWER,
  RX,
  RX_TIMEOUT,
  RX_ERROR,
  TX,
  TX_TIMEOUT,
} States_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";
uint8_t RD_header [] = "DATPR";
uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];
CRYP_HandleTypeDef hcryp;
uint32_t rd_aes_key[4]  = {0x1F352C07,0x3B6108D7,0x2D9810A3,0x0914DFF4};
uint32_t test_netkey[4]  = {0x01020304,0x05060708,0x0a0b0c0d,0x0e0e0f0f};
__ALIGN_BEGIN uint32_t pKeyAES[4] __ALIGN_END = {0};

uint32_t input_data[4]= {0x6bc1bee3,0x2e409f96,0xe93d7e11,0x7393172a};
uint32_t encrypted_data[8] = {0};
uint32_t decrypted_data[8] = {0};
extern unsigned int trick_time;
States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

/* Led Timers objects*/
static  UTIL_TIMER_Object_t timerLed;

bool isMaster = true;

/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief  Function executed on when led timer elapses
 * @param  LED context
 * @retval none
 */
static void OnledEvent(void *context);

/*!
 * @brief Function to be executed on Radio Tx Done event
 * @param  none
 * @retval none
 */
static void OnTxDone(void);

/*!
 * @brief Function to be executed on Radio Rx Done event
 * @param  payload sent
 * @param  payload size
 * @param  rssi
 * @param  snr
 * @retval none
 */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

/*!
 * @brief Function executed on Radio Tx Timeout event
 * @param  none
 * @retval none
 */
static void OnTxTimeout(void);

/*!
 * @brief Function executed on Radio Rx Timeout event
 * @param  none
 * @retval none
 */
static void OnRxTimeout(void);

/*!
 * @brief Function executed on Radio Rx Error event
 * @param  none
 * @retval none
 */
static void OnRxError(void);

/*!
 * @brief PingPong state machine implementation
 * @param  none
 * @retval none
 */
static void PingPong_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */

  /* USER CODE END SubghzApp_Init_1 */
  /* Print APP version*/
  /*APP_LOG(TS_OFF, VLEVEL_M, "APP_VERSION= V%X.%X.%X\r\n", (uint8_t)(__APP_VERSION >> __APP_VERSION_MAIN_SHIFT),
          (uint8_t)(__APP_VERSION >> __APP_VERSION_SUB1_SHIFT), (uint8_t)(__APP_VERSION >> __APP_VERSION_SUB2_SHIFT));*/

  /* Led Timers*/
  //UTIL_TIMER_Create(&timerLed, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnledEvent, NULL);
  //UTIL_TIMER_SetPeriod(&timerLed, LED_PERIOD_MS);

  //UTIL_TIMER_Start(&timerLed);

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);

#if (( USE_MODEM_LORA == 1 ) && ( USE_MODEM_FSK == 0 ))
  // RD_EDIT cong suat phat
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  Radio.SetMaxPayloadLength(MODEM_LORA, BUFFER_SIZE);

#elif (( USE_MODEM_LORA == 0 ) && ( USE_MODEM_FSK == 1 ))

  Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                    FSK_DATARATE, 0,
                    FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, 0, TX_TIMEOUT_VALUE);

  Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                    0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                    0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                    0, 0, false, true);

  Radio.SetMaxPayloadLength(MODEM_FSK, BUFFER_SIZE);

#else
#error "Please define a frequency band in the sys_conf.h file."
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */

  Radio.SetChannel(RF_FREQUENCY);

#if defined(USE_BSP_DRIVER)
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
#elif defined(MX_BOARD_PSEUDODRIVER)
  SYS_LED_Init(SYS_LED_GREEN);
  SYS_LED_Init(SYS_LED_RED);
#else
#error user to provide its board code or to call his board driver functions
#endif  /* USE_BSP_DRIVER || MX_NUCLEO_WL55JC*/

  Radio.Rx(RX_TIMEOUT_VALUE);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_PingPong_Process), UTIL_SEQ_RFU, PingPong_Process);

  /* USER CODE BEGIN SubghzApp_Init_2 */

  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void PingPong_Process(void)//RD_EDIT: ham xu ly ping pong
{


	if(device_send_flag){
		device_send_flag = 0;
		trick_time = HAL_GetTick();
		radio_free_flag = 1;
	}
	if(radio_free_flag == 1){
		if(HAL_GetTick() - trick_time >= random_time){
			radio_rx_flag = 0;
			radio_tx_flag = 1;
			radio_free_flag = 0;
		}
	}
	if(radio_rx_flag){
			Radio.SetChannel(RD_DOWN_LINK_FREQUENCY);
			//Radio.SetChannel(RF_FREQUENCY);
			Radio.Rx(RX_TIMEOUT_VALUE);
			aes_encrypt_decrypt(rd_aes_key,(uint32_t *)Buffer,decrypted_data,DECRYPT_DATA);
			rd_rx_opcode = Buffer[1]<<8|Buffer[0];
			if(rd_message_hande(Buffer,gw_to_device) == 0) device_flag_set();
			else{
				if(add_device_flag == 0){
					aes_encrypt_decrypt(netkey8_to_32(rd_netkey_char),(uint32_t *)Buffer,decrypted_data,DECRYPT_DATA);	
					rd_rx_opcode = Buffer[1]<<8|Buffer[0];
					if(rd_message_hande(Buffer,gw_to_device) == 0){
							trick_time = HAL_GetTick();
					}
				}
			}
	}
	if(radio_tx_flag){
				unsigned char *opcode_convert = (unsigned char *)&rd_tx_opcode;
				rd_message_hande(opcode_convert,device_to_gw);
				for(char i=0;i<32;i++) Buffer[i] = device_send_buff[i];
				if(add_device_flag == 0){
					aes_encrypt_decrypt(netkey8_to_32(rd_netkey_char),(uint32_t *)Buffer,encrypted_data,ENCRYPT_DATA);
				}
				aes_encrypt_decrypt(rd_aes_key,(uint32_t *)Buffer,encrypted_data,ENCRYPT_DATA);
				
				Radio.SetChannel(RD_UP_LINK_FREQUENCY);
				//Radio.SetChannel(RF_FREQUENCY);
				HAL_Delay(Radio.GetWakeupTime() + TCXO_WORKAROUND_TIME_MARGIN);
				Radio.Send(Buffer, BufferSize);
				radio_tx_flag = 0;
				radio_rx_flag = 1;
	}
	for(uint8_t i = 0; i < BufferSize; i++) Buffer[i] = 0;
	
}


static void OnTxDone(void)
{
  Radio.Sleep();
  State = TX;
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_PingPong_Process), CFG_SEQ_Prio_0);
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  Radio.Sleep();
  BufferSize = size;
  memcpy(Buffer, payload, BufferSize);
  RssiValue = rssi;
  SnrValue = snr;
  State = RX;
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_PingPong_Process), CFG_SEQ_Prio_0);
}

static void OnTxTimeout(void)
{
  Radio.Sleep();
  State = TX_TIMEOUT;
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_PingPong_Process), CFG_SEQ_Prio_0);

}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout_1 */

  /* USER CODE END OnRxTimeout_1 */
  //APP_LOG(TS_ON, VLEVEL_L,  "OnRxTimeout\n\r");

  Radio.Sleep();
  State = RX_TIMEOUT;
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_PingPong_Process), CFG_SEQ_Prio_0);
  /* USER CODE BEGIN OnRxTimeout_2 */

  /* USER CODE END OnRxTimeout_2 */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError_1 */

  /* USER CODE END OnRxError_1 */
  //APP_LOG(TS_ON, VLEVEL_L, "OnRxError\n\r");

  Radio.Sleep();
  State = RX_ERROR;
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_PingPong_Process), CFG_SEQ_Prio_0);
  /* USER CODE BEGIN OnRxError_2 */

  /* USER CODE END OnRxError_2 */
}

static void OnledEvent(void *context)
{
  /* USER CODE BEGIN OnledEvent_1 */

  /* USER CODE END OnledEvent_1 */
#if defined(USE_BSP_DRIVER)
  BSP_LED_Toggle(LED_GREEN) ;
  BSP_LED_Toggle(LED_RED) ;
#elif defined(MX_BOARD_PSEUDODRIVER)
  SYS_LED_Toggle(SYS_LED_RED) ;
  SYS_LED_Toggle(SYS_LED_GREEN) ;
#endif /* USE_BSP_DRIVER || MX_BOARD_PSEUDODRIVER */

  UTIL_TIMER_Start(&timerLed);
  /* USER CODE BEGIN OnledEvent_2 */

  /* USER CODE END OnledEvent_2 */
}

/* USER CODE BEGIN PrFD */
 void RD_AES_Init(void)
{

  /* USER CODE BEGIN AES_Init 0 */

  /* USER CODE END AES_Init 0 */

  /* USER CODE BEGIN AES_Init 1 */

  /* USER CODE END AES_Init 1 */
  hcryp.Instance = AES;
  hcryp.Init.DataType = CRYP_DATATYPE_32B;
  hcryp.Init.KeySize = CRYP_KEYSIZE_128B;
  hcryp.Init.pKey = (uint32_t *)pKeyAES;
  hcryp.Init.Algorithm = CRYP_AES_ECB;
  hcryp.Init.DataWidthUnit = CRYP_DATAWIDTHUNIT_WORD;
  hcryp.Init.HeaderWidthUnit = CRYP_HEADERWIDTHUNIT_WORD;
  hcryp.Init.KeyIVConfigSkip = CRYP_KEYIVCONFIG_ALWAYS;
  if (HAL_CRYP_Init(&hcryp) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN AES_Init 2 */

  /* USER CODE END AES_Init 2 */

}

void HAL_CRYP_MspInit(CRYP_HandleTypeDef *hcryp){
	if(hcryp->Instance==AES){
		__HAL_RCC_AES_CLK_ENABLE();
	}
}



unsigned char *device_Tx_joinrequest_message(){


	return device_send_buff;
}


int op_join_request_handle(unsigned char *par,rd_direction_t direction_handle){
	if(direction_handle == device_to_gw){
		#if RD_LORA_MODEL_SEL == RD_LORA_MODEL_GW
		unsigned int opcode = par[1]<<8|par[0];
		if(opcode == OP_JOIN_REQUEST){
			for(char i=0;i<8;i++) rd_mac_id[i] = par[2+i];
			APP_LOG(TS_ON, VLEVEL_L,"**********************************************************************\n");
			APP_LOG(TS_ON, VLEVEL_L, "op_join_request_handle: \n device mac id:%x-%x-%x-%x-%x-%x-%x-%x\n",rd_mac_id[0],
			rd_mac_id[1],rd_mac_id[2],rd_mac_id[3],rd_mac_id[4],rd_mac_id[5],rd_mac_id[6],rd_mac_id[7]);
		}
		radio_tx_flag = 1;
		radio_rx_flag = 0;
		rd_tx_opcode = OP_JOIN_ACCEPT;
		#endif
		#if RD_LORA_MODEL_SEL != RD_LORA_MODEL_GW
		APP_LOG(TS_ON, VLEVEL_L, "op_join_request_handle\n");
			for(char i=0;i<32;i++) device_send_buff[i] = 0;
			mac_id[0] = HAL_GetUIDw0();
			mac_id[1] = HAL_GetUIDw1();
			device_send_buff[0] = (OP_JOIN_REQUEST>>0);
			device_send_buff[1] = (OP_JOIN_REQUEST>>8);
			device_send_buff[2] = (mac_id[0]>>24);
			device_send_buff[3] = (mac_id[0]>>16);
			device_send_buff[4] = (mac_id[0]>>8);
			device_send_buff[5] = (mac_id[0]>>0);
			device_send_buff[6] = (mac_id[1]>>24);
			device_send_buff[7] = (mac_id[1]>>16);
			device_send_buff[8] = (mac_id[1]>>8);
			device_send_buff[9] = (mac_id[1]>>0);
			device_send_buff[10] = (GW_ADDR>>0);
			device_send_buff[11] = (GW_ADDR>>8);
			device_send_buff[12] = message_cnt++;
			for(char i=0;i<8;i++) rd_mac_id[i] = device_send_buff[2+i];
		#endif
	}
	else if (direction_handle == gw_to_device){
		
	}
	return 0;
}


int op_join_accept_handle(unsigned char *par,rd_direction_t direction_handle){
	if(direction_handle == device_to_gw){
	}
	else if (direction_handle == gw_to_device){
		#if RD_LORA_MODEL_SEL != RD_LORA_MODEL_GW
		unsigned int opcode = par[1]<<8|par[0];
		unsigned int rx_gw_addr = par[3]<<8|par[2];
		unsigned char rx_mac_id[8] = {0};
		for(char i=0;i<8;i++) rx_mac_id[i] = par[4+i];
		if((opcode == OP_JOIN_ACCEPT) && (rx_gw_addr == GW_ADDR)){
			for(char i=0;i<8;i++){
				if (rx_mac_id[i] != rd_mac_id[i]){
					APP_LOG(TS_ON, VLEVEL_L,"mac addr is wrong!\n");
					return -1;
				}
				else{
					network_status = JOINED;
				} 
			}
		}
		if (network_status ==JOINED){
			for(char i=0;i<16;i++) rd_netkey_char[i] = par[12+i];
			rd_device_id = par[29]<<8|par[28];
			unsigned char mes_cnt = par[30];
			APP_LOG(TS_ON, VLEVEL_L,"**********************************************************************\n");
			APP_LOG(TS_ON, VLEVEL_L, "op_join_accept_handle:\n Rx mac id:%x-%x-%x-%x-%x-%x-%x-%x\n",rx_mac_id[0],
			rx_mac_id[1],rx_mac_id[2],rx_mac_id[3],rx_mac_id[4],rx_mac_id[5],rx_mac_id[6],rx_mac_id[7]);
			APP_LOG(TS_ON, VLEVEL_L, "Rx net key id:-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x\n",rd_netkey_char[0],
			rd_netkey_char[1],rd_netkey_char[2],rd_netkey_char[3],rd_netkey_char[4],rd_netkey_char[5],rd_netkey_char[6],rd_netkey_char[7],
			rd_netkey_char[8],rd_netkey_char[9],rd_netkey_char[10],rd_netkey_char[11],rd_netkey_char[12],rd_netkey_char[13],rd_netkey_char[14],
			rd_netkey_char[15]);
			uint32_t *conver_key_p = (uint32_t *)rd_netkey_char;
			for(char i=0;i<4;i++)rd_net_key_32bit[i] = conver_key_p[i];
			APP_LOG(TS_ON, VLEVEL_L, "Rx device id:-%d---mes_cnt: %d\n",rd_device_id,mes_cnt);	
			save_flash_flag = 1;
			radio_tx_flag = 1;
			radio_rx_flag = 0;
			rd_tx_opcode = OP_JOIN_ACCEPT_ACK;
			led_toggle(1);
		}
		//rd_tx_opcode = OP_JOIN_ACCEPT;
		#endif
	}
	return 0;
}


int  op_join_accept_ack_handle(unsigned char *par,rd_direction_t direction_handle){
	APP_LOG(TS_ON, VLEVEL_L, "op_join_accept_ack_handle \n");
	if(direction_handle == device_to_gw){
		#if RD_LORA_MODEL_SEL != RD_LORA_MODEL_GW
		if(network_status == JOINED) {
			for(char i=0;i<32;i++) device_send_buff[i] = 0;
			device_send_buff[0] = OP_JOIN_ACCEPT_ACK;
			device_send_buff[1] = OP_JOIN_ACCEPT_ACK>>8;
			device_send_buff[2] = rd_device_id;
			device_send_buff[3] = rd_device_id>>8;
			device_send_buff[4] = GW_ADDR;
			device_send_buff[5] = GW_ADDR>>8;
			device_send_buff[6] = (RD_TYPE_DEVICE)&0xff;
			device_send_buff[7] = (RD_TYPE_DEVICE>>8)&0xff;
			device_send_buff[8] = (mac_id[0]>>24);
			device_send_buff[9] = (mac_id[0]>>16);
			device_send_buff[10] = (mac_id[0]>>8);
			device_send_buff[11] = (mac_id[0]>>0);
			device_send_buff[12] = (mac_id[1]>>24);
			device_send_buff[13] = (mac_id[1]>>16);
			device_send_buff[14] = (mac_id[1]>>8);
			device_send_buff[15] = (mac_id[1]>>0);
			device_send_buff[16] = (dev_key_32>>24);
			device_send_buff[17] = (dev_key_32>>16);
			device_send_buff[18] = (dev_key_32>>8);
			device_send_buff[19] = (dev_key_32>>0);
			APP_LOG(TS_ON, VLEVEL_L, "key send is: %x\n",dev_key_32);
			//device_send_buff[6] =
			rd_tx_opcode = RD_OPCODE_SEND;
			radio_tx_flag = 1;
			radio_rx_flag = 0;
			APP_LOG(TS_ON, VLEVEL_L, "op_join_accept_ack_handle  rd tx opcode is %d \n",rd_tx_opcode);
		}
		#endif
	}
	else if (direction_handle == gw_to_device){
		#if RD_LORA_MODEL_SEL == RD_LORA_MODEL_GW
		#endif
		return 0;
	}
	return 0;
}


int  op_light_to_gw_handle(unsigned char *par,rd_direction_t direction_handle){
	#if RD_LORA_MODEL_SEL == RD_LORA_MODEL_LIGHT_SENSOR
			data_max44009 = 1234;
			add_device_flag = 0;
			APP_LOG(TS_ON, VLEVEL_L, "op_temp_hum_to_gw_handle\n");
			for(char i=0;i<32;i++) device_send_buff[i] = 0;
			device_send_buff[0] = OP_LIGHT_TO_GW_MESSAGE&0xff;
			device_send_buff[1] = (OP_LIGHT_TO_GW_MESSAGE>>8&0xff);
			device_send_buff[2] = rd_device_id;
			device_send_buff[3] = rd_device_id>>8;
			device_send_buff[4] = GW_ADDR;
			device_send_buff[5] = GW_ADDR>>8;
			device_send_buff[6] = data_max44009;
			device_send_buff[7] = data_max44009>>8;
			radio_tx_flag = 0;
			radio_rx_flag = 1;
			rd_tx_opcode = RD_OPCODE_SEND;
	#endif
	return 0;
}


int  op_gw_to_light_handle(unsigned char *par,rd_direction_t direction_handle){
	#if RD_LORA_MODEL_SEL == RD_LORA_MODEL_GW
	APP_LOG(TS_ON, VLEVEL_L, "op_gw_to_light_handle \n");
	#endif
	return 0;
}

int  op_temp_hum_to_gw_handle(unsigned char *par,rd_direction_t direction_handle){
	#if RD_LORA_MODEL_SEL == RD_LORA_MODEL_TEMP_HUM_SENSOR
//			sht30_temp ;
//			sht30_hum  ;
			add_device_flag = 0;
	APP_LOG(TS_ON, VLEVEL_L, "op_temp_hum_to_gw_handle--------- device id:\n");
			for(char i=0;i<32;i++) device_send_buff[i] = 0;
			device_send_buff[0] = OP_TEMP_HUM_TO_GW_MESSAGE&0xff;
			device_send_buff[1] = (OP_TEMP_HUM_TO_GW_MESSAGE>>8&0xff);
			device_send_buff[2] = rd_device_id;
			device_send_buff[3] = rd_device_id>>8;
			device_send_buff[4] = GW_ADDR;
			device_send_buff[5] = GW_ADDR>>8;
			device_send_buff[6] = sht30_temp;
			device_send_buff[7] = sht30_temp>>8;
			device_send_buff[8] = sht30_hum;
			device_send_buff[9] = sht30_hum>>8;
//			radio_tx_flag = 0;
//			radio_rx_flag = 1;
			rd_tx_opcode = OP_TEMP_HUM_TO_GW_MESSAGE;
	#endif
	return 0;
}

int  op_gw_to_temp_hum_handle(unsigned char *par,rd_direction_t direction_handle){
	#if RD_LORA_MODEL_SEL == RD_LORA_MODEL_GW
	APP_LOG(TS_ON, VLEVEL_L, "op_gw_to_temp_hum_handle \n");
	#endif
	return 0;
}

int  op_co2_to_gw_handle(unsigned char *par,rd_direction_t direction_handle){
	#if RD_LORA_MODEL_SEL == RD_LORA_MODEL_GW
	APP_LOG(TS_ON, VLEVEL_L, "op_co2_to_gw_handle \n");
	#endif
	return 0;
}
int  op_gw_to_co2_handle(unsigned char *par,rd_direction_t direction_handle){
	#if RD_LORA_MODEL_SEL == RD_LORA_MODEL_GW
	APP_LOG(TS_ON, VLEVEL_L, "op_gw_to_co2_handle \n");
	#endif
	return 0;
}
int  op_ground_to_gw_handle(unsigned char *par,rd_direction_t direction_handle){
	#if RD_LORA_MODEL_SEL == RD_LORA_MODEL_GW
	APP_LOG(TS_ON, VLEVEL_L, "op_ground_to_gw_handle \n");
	#endif
	return 0;
}
int  op_gw_to_ground_handle(unsigned char *par,rd_direction_t direction_handle){
	#if RD_LORA_MODEL_SEL == RD_LORA_MODEL_GW
	APP_LOG(TS_ON, VLEVEL_L, "op_gw_to_ground_handle \n");
	#endif
	return 0;
}

int op_uart_send_light_on_handle(unsigned char *par,rd_direction_t direction_handle){
	

	#if RD_LORA_MODEL_SEL == RD_LORA_MODEL_GW
				for(unsigned char i=0;i<32;i++) rd_device_send_buff[i] = 0;
				for(unsigned char i=0;i<32;i++) rd_device_send_buff[i] = par[i];
	#elif RD_LORA_MODEL_SEL != RD_LORA_MODEL_GW
		unsigned int opcode = par[1]<<8|par[0];
		unsigned int RX_device_id = par[3]<<8|par[2];
		unsigned char mode = par[5];
		if((opcode == OP_UART_SEND_LIGHT_ON) && (RX_device_id == rd_device_id)){
					led_toggle (1);
			return 0;
		}
			//OP_UART_SEND_LIGHT_OFF	
		rd_tx_opcode = RD_OPCODE_SEND;
	#endif
		return 0;
}

int op_uart_send_light_off_handle(unsigned char *par,rd_direction_t direction_handle){
	#if RD_LORA_MODEL_SEL == RD_LORA_MODEL_GW
				for(unsigned char i=0;i<32;i++) rd_device_send_buff[i] = 0;
				for(unsigned char i=0;i<32;i++) rd_device_send_buff[i] = par[i];
	#endif
	return 0;
}

int op_delete_device_handle(unsigned char *par,rd_direction_t direction_handle){
		
	unsigned int opcode = par[1]<<8|par[0];
	unsigned int RX_device_id = par[3]<<8|par[2];
	unsigned char mode = par[5];
	if((opcode == OP_DELETE_DEVICE) && (RX_device_id == rd_device_id)){
		delete_device_infor();
		for(unsigned char j=0;j<4;j++){
			rd_net_key_32bit[j]  = 0;
		}
		led_blink(1,1);
		return 0;
	}
	rd_tx_opcode = RD_OPCODE_SEND;
	return 0;
}

int rd_message_hande(unsigned char *par,rd_direction_t mes_direction){
	unsigned int handle_opcode = par[1]<<8|par[0];
	APP_LOG(TS_ON, VLEVEL_L, "  rd tx opcode1 is %d \n",rd_tx_opcode);
	for(uint8_t i=0;i<NUMBER_OF_OPCODE;i++){
		if(rd_message_func_handle[i].opcode == handle_opcode){
			APP_LOG(TS_ON, VLEVEL_L, "  rd tx opcode2 is %d \n",handle_opcode);
			rd_message_func_handle[i].cb_cmd_function_t(par,mes_direction);
			return 0;
		}
	}
	rd_rx_opcode = 0xff;
	return -1;
}



 void led_toggle(unsigned char mode)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
 // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
}

void write_led(unsigned char mode){
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	if(mode){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
	}
	else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
	}
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
	
}

void led_blink(unsigned char time, unsigned char blink_flag){
	write_led(1);
	HAL_Delay(200);
	write_led(0);
	HAL_Delay(200);
	write_led(1);
}

void rd_error_call_back(){
	APP_LOG(TS_ON, VLEVEL_L, "GW encrypt AES netkey\nmessage structure is wrong\n");
}
void rd_encrypt_aes_key_call_back(){
	APP_LOG(TS_ON, VLEVEL_L, " GW encrypt RD AES key\nmessage structure is wrong\n");
}
void rd_set_aes_key (unsigned int *key) // uint32_t --> unsigned int 
{
	for(char i=0;i<4;i++) pKeyAES[i] = key[i];
}


void message_checksum(unsigned char * data){
	unsigned char message_checksum;
	for(unsigned char i=0;i<32;i++){
		
	}
}





unsigned int *netkey8_to_32(unsigned char *key_8){
	unsigned int *key_32 = (unsigned int *)key_8;
	return key_32;
}



void aes_encrypt_decrypt (unsigned int *key, unsigned int *data_in,unsigned int *data_out,unsigned char direct){
	rd_set_aes_key (key);
	RD_AES_Init();
	HAL_CRYP_MspInit(&hcryp);
	if(direct == DECRYPT_DATA){
		HAL_CRYP_Decrypt(&hcryp, (uint32_t *)data_in, 8, data_out, 1000);
	}else if (direct  == ENCRYPT_DATA){
		HAL_CRYP_Encrypt(&hcryp, (uint32_t *)data_in, 8, data_out, 1000);
	}
		for(unsigned char i=0;i<8;i++)
		data_in[i] = data_out[i];
}


void device_flag_set(){
	if((rd_rx_opcode == OP_JOIN_REQUEST)||
	(rd_rx_opcode == OP_JOIN_ACCEPT)||
	(rd_rx_opcode == OP_JOIN_ACCEPT_ACK)){
	add_device_flag = 1;
	}
}

//void device_ack_message(unsigned char *par,unsigned char respone_num){
//stactic Respone_num = respone_num;
//	while(--Respone_num){
//	
//	}
//}


/* USER CODE END PrFD */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
