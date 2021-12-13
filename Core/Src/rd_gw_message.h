



#include "main.h"
#include "rd_lora_model.h"


#if(RD_LORA_MODEL_SEL == RD_LORA_MODEL_GW)
void join_request_message(unsigned char *par);
uint8_t * gw_Tx_join_accept_message  (uint8_t *par);
unsigned char Rx_sensor_data(unsigned char *par);
#elif(RD_LORA_MODEL_SEL == RD_LORA_MODEL_LIGHT_SENSOR)
void light_sensor_send_data();
#elif(RD_LORA_MODEL_SEL == RD_LORA_MODEL_TEMP_HUM_SENSOR)
#elif(RD_LORA_MODEL_SEL == RD_LORA_MODEL_CO2_SENSOR)
#elif(RD_LORA_MODEL_SEL == RD_LORA_MODEL_GROUND_SENSOR)
#endif