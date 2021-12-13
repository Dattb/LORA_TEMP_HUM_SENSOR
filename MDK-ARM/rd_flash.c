/*
 * rd_flash.c
 *
 *  Created on: Sep 23, 2021
 *      Author: Dat_UTC
 */



#include"rd_flash.h"


unsigned char  *rd_flash_to_key(uint64_t *data_hi,uint64_t *data_low,unsigned char *key_gen){
	unsigned char *key_buff = key_gen;
	unsigned char *key_p;
	key_p =(unsigned char *)data_hi;
	for (char i=0;i<8;i++){
		*key_buff = *key_p;
		key_buff++;
		key_p--;
	}
	key_p =(unsigned char *)data_low;
	for (char i=0;i<8;i++){
		*key_buff = *key_p;
		key_buff++;
		key_p--;
	}
	return (key_buff-16);
}



