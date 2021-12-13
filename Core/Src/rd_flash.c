/*
 * rd_flash.c
 *
 *  Created on: Sep 23, 2021
 *      Author: Dat_UTC
 */



#include "rd_flash.h"
#include "sys_app.h"
uint64_t flash_buff[5] = {0};
uint64_t Flash_Unicast = 0;
uint32_t rd_random[4] = {0};
uint32_t dev_key_32 = 0;
uint32_t flash_word[4] = {0};
uint32_t random_time = 0;
RNG_HandleTypeDef hrng;
extern unsigned char rd_netkey_char[16];
extern unsigned short rd_device_id;
unsigned char delete_device_flag = 0;



unsigned int device_listen_befor_talk(unsigned char flag){
//	//static unsigned int trick_time;
//	if(flag){
//	//	 trick_time = HAL_GetTick();
//		flag = 0;
//	}
//	//return clock_time_exceed(trick_time,random_time);
	return 0;
}

unsigned int clock_time_exceed(unsigned int ref, unsigned int ms)
{
	return ((unsigned int)(HAL_GetTick() - ref) > ms );
}


void device_gen_random_time(unsigned int *time){
	RD_RNG_Init();
	HAL_RNG_MspInit(&hrng);
	HAL_Delay(50);
	HAL_RNG_GenerateRandomNumber(&hrng,time);
	APP_LOG(TS_ON,VLEVEL_L,"key gen is: %x\n",(*time));
}

unsigned int convert_random_time(unsigned int input){
	input = input/10000000;
	input = input*5000/4295;
	return input;
}
void device_gen_dev_key(unsigned int * dev_key){
	RD_RNG_Init();
	HAL_RNG_MspInit(&hrng);
	HAL_Delay(50);
	HAL_RNG_GenerateRandomNumber(&hrng,dev_key);
	APP_LOG(TS_ON, VLEVEL_L, "key gen is: %x\n",dev_key[0]);
}

int _save_netkey_device_id (unsigned char *netkey_8,unsigned short *deviceid,unsigned int *dev_key,unsigned int *rx_random_time){
//	SystemCoreClock
//	volatile uint64_t *Rd_flash_read_ptr = RD_FLASH_ADDR;
//	flash_buff[0] = *(Rd_flash_read_ptr);
//	flash_buff[1] = *(Rd_flash_read_ptr+1);
	
	*rx_random_time = convert_random_time(*rx_random_time);
	//u8_to_u32_key(key_u8,key_32);
	uint64_t *TEMP = (	uint64_t *)netkey_8;
	flash_buff[0] = TEMP[0] ;
	flash_buff[1] = TEMP[1];
	flash_buff[2] = *dev_key;
	flash_buff[3] = *deviceid ;
	flash_buff[4] = *rx_random_time;

	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;
	HAL_FLASH_Unlock();
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page 		= 127;
	EraseInitStruct.NbPages     = 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK){
		return HAL_FLASH_GetError ();
	}
	for(unsigned char i=0;i<5;i++){
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, RD_FLASH_ADDR + i*8, flash_buff[i]) == HAL_OK);
		else return HAL_FLASH_GetError ();
	}
	HAL_FLASH_Lock();
	extern unsigned short rd_device_id;
	read_key_device_id(netkey_8,&rd_device_id,&dev_key_32,&random_time);
	return 0;
}

int delete_device_infor() {
	
	flash_buff[0] = 0;
	flash_buff[1] = 0;
	flash_buff[2] = 0;
	flash_buff[3] = 0 ;
	flash_buff[4] = 0;

	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;
	HAL_FLASH_Unlock();
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page 		= 127;
	EraseInitStruct.NbPages     = 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK){
		return HAL_FLASH_GetError ();
	}
	for(unsigned char i=0;i<5;i++){
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, RD_FLASH_ADDR + i*8, flash_buff[i]) == HAL_OK);
		else return HAL_FLASH_GetError ();
	}
	HAL_FLASH_Lock();
	delete_device_flag = 1;
	read_key_device_id(rd_netkey_char,&rd_device_id,&dev_key_32,&random_time);
	return 0;
}

void u8_to_u32_key (uint8_t *key_u8,uint32_t *key_u32){
	for(unsigned char i=0;i<4;i++){
		key_u32[i] = key_u8[4*i]<<24|key_u8[4*i+1]<<16|key_u8[4*i+2]<<8|key_u8[4*i+3];
	}
}

unsigned char * dev_key_32_to_8 (unsigned int *data_32){
	extern unsigned char dev_key[];
	unsigned char *u8_ptr_ = (unsigned char *)data_32;
	for(unsigned char i=0;i<4;i++) dev_key[i] = u8_ptr_[i];
	return u8_ptr_;
}

void read_key_device_id (unsigned char *netkey__8,unsigned short *_device_id,unsigned int *_dev_key_32,unsigned int *rx_random_time){
	uint64_t *read_flash_p = (uint64_t *)RD_FLASH_ADDR;
	unsigned char *u8_temp = (unsigned char *)read_flash_p;
	for(unsigned char i=0;i<16;i++) netkey__8[i] = u8_temp[i];
	*_dev_key_32 =  read_flash_p[2];
	*_device_id =  read_flash_p[3];
	*rx_random_time = read_flash_p[4];
	//APP_LOG(TS_ON, VLEVEL_L, "key read is: %x\n",_dev_key_32[0]);
}








unsigned char  *rd_flash_to_key(uint64_t *data_hi,uint64_t *data_low,unsigned char *key_gen){
	unsigned char *key_buff = key_gen;
	unsigned char *Flash_char_p;
	Flash_char_p =(unsigned char *)data_hi+7;
	for (char i=0;i<8;i++){
		*key_buff = *Flash_char_p;
		key_buff++;
		Flash_char_p--;
	}
	Flash_char_p =(unsigned char *)data_low+7;
	for (char i=0;i<8;i++){
		*key_buff = *Flash_char_p;
		key_buff++;
		Flash_char_p--;
	}
	return (key_buff-16);
}
// check debug flash

/*#define RD_FLASH_DEBUG	*/
int rd_gw_gen_net_key (unsigned char *key_p){
	
			uint64_t *Rd_flash_read_ptr = (uint64_t *)RD_FLASH_ADDR;
			flash_buff[0] = *(Rd_flash_read_ptr);
			flash_buff[1] = *(Rd_flash_read_ptr+1);
			Flash_Unicast = *(Rd_flash_read_ptr+2);
			rd_flash_to_key(&flash_buff[0],&flash_buff[1],key_p);
	
			#ifdef RD_FLASH_DEBUG
				flash_word[0] = flash_buff[0]>>32;
				flash_word[1] = flash_buff[0];
				flash_word[2] = flash_buff[1]>>32;
				flash_word[3] = flash_buff[1];
				APP_LOG(TS_ON, VLEVEL_L, "\n1:%x-%x-%x-%x  %x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x\n"
				,flash_word[0],flash_word[1],flash_word[2],flash_word[3],key_p[0],key_p[1],key_p[2],key_p[3]
				,key_p[4],key_p[5],key_p[6],key_p[7],key_p[8],key_p[9],key_p[10],key_p[11],key_p[12],key_p[13]
				,key_p[14],key_p[15]);
			#endif
	
			if((flash_buff[0] == 0xffffffffffffffff) && (flash_buff[1] == 0xffffffffffffffff)){
				RD_RNG_Init();
				HAL_RNG_MspInit(&hrng);
				for(char i = 0;i<4;i++){
					HAL_Delay(50);
					HAL_RNG_GenerateRandomNumber(&hrng, &rd_random[i]);
				}
				flash_buff[0] = rd_random[0]&0x00000000ffffffff;
				flash_buff[0] = flash_buff[0]<<32|rd_random[1];
				flash_buff[1] = rd_random[2]&0x00000000ffffffff;
				flash_buff[1] = flash_buff[1]<<32|rd_random[3];
				rd_flash_to_key(&flash_buff[0],&flash_buff[1],key_p);
				
				FLASH_EraseInitTypeDef EraseInitStruct;
				uint32_t PAGEError;
				HAL_FLASH_Unlock();
				
				EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
				EraseInitStruct.Page = 127;
				EraseInitStruct.NbPages     = 1;
				
				if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK){
					return HAL_FLASH_GetError ();
				}
				if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, RD_FLASH_ADDR, flash_buff[0]) == HAL_OK);
				else return HAL_FLASH_GetError ();
				
				if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, RD_FLASH_ADDR+8, flash_buff[1]) == HAL_OK);
				else return HAL_FLASH_GetError ();
				HAL_FLASH_Lock();

				flash_buff[0] = *(Rd_flash_read_ptr);
				flash_buff[1] = *(Rd_flash_read_ptr+1);
				
				#ifdef RD_FLASH_DEBUG
				flash_word[0] = flash_buff[0]>>32;
				flash_word[1] = flash_buff[0];
				flash_word[2] = flash_buff[1]>>32;
				flash_word[3] = flash_buff[1];
				
				APP_LOG(TS_ON, VLEVEL_L, "\n2: %x-%x-%x-%x  %x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x\n"
				,flash_word[0],flash_word[1],flash_word[2],flash_word[3],key_p[0],key_p[1],key_p[2],key_p[3]
				,key_p[4],key_p[5],key_p[6],key_p[7],key_p[8],key_p[9],key_p[10],key_p[11],key_p[12],key_p[13]
				,key_p[14],key_p[15]);
				#endif
			}
			return 0;
}


static void RD_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

void HAL_RNG_MspInit(RNG_HandleTypeDef* hrng)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hrng->Instance==RNG)
  {
  /* USER CODE BEGIN RNG_MspInit 0 */


  /* USER CODE END RNG_MspInit 0 */
  /** Initializes the peripherals clocks
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RNG;
    PeriphClkInitStruct.RngClockSelection = RCC_RNGCLKSOURCE_MSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_RNG_CLK_ENABLE();
  /* USER CODE BEGIN RNG_MspInit 1 */

  /* USER CODE END RNG_MspInit 1 */
  }

}






