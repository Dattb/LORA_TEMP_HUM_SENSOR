



#include "main.h"



#define RD_FLASH_ADDR		0x0803F800
static void RD_RNG_Init(void);
void HAL_RNG_MspInit(RNG_HandleTypeDef* hrng);
unsigned char  *rd_flash_to_key(uint64_t *data_hi,uint64_t *data_low,unsigned char *key_gen);
int rd_gw_gen_net_key (unsigned char *key_p);
void u8_to_u32_key (uint8_t *key_u8,uint32_t *key_u32);
void read_key_device_id (unsigned char *netkey__8,unsigned short *_device_id,unsigned int *_dev_key_32,unsigned int *rx_random_time);
unsigned char * dev_key_32_to_8 (unsigned int *data_32);
int _save_netkey_device_id (unsigned char *netkey_8,unsigned short *deviceid,unsigned int *dev_key,unsigned int *rx_random_time);
void device_gen_dev_key(unsigned int * dev_key);
void device_gen_random_time(unsigned int *time);
unsigned int convert_random_time(unsigned int input);
  unsigned int clock_time_exceed(unsigned int ref, unsigned int ms);
unsigned int device_listen_befor_talk(unsigned char flag);
int delete_device_infor();