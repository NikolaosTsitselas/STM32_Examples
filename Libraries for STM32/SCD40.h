

#define SCD40_H_

#include "stm32g0xx_hal.h"
#include "math.h"
#include "stdio.h"



void start_periodic_measurement(I2C_HandleTypeDef *hi2c){
    uint8_t pData[2]={0x21,0xb1};
    HAL_I2C_Master_Transmit(hi2c,0x62 <<1 , pData,2,1);
}

void read_measurement(I2C_HandleTypeDef *hi2c,float *temperature,uint16_t *co2,float  *humidity){
   uint8_t pData[2]={0xec,0x05};
   uint8_t rx[9];
   HAL_I2C_Master_Transmit(hi2c,0x62 <<1,pData,2,10);
   HAL_Delay(1);
   HAL_I2C_Master_Receive(hi2c,0x62 << 1,rx,9,100);
   uint16_t co2_raw= (rx[0]<<8) | rx[1];
   uint16_t temp_raw = (rx[3]<<8) | rx[4];
   uint16_t humidity_raw = (rx[6]<<8) | rx[7];


   *co2=co2_raw;
   *temperature = -45 + 175 * (temp_raw/(pow(2,16)-1));
   *humidity = 100 * (humidity_raw/(pow(2,16)-1));
   

    
}

