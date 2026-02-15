#include "sensirion_i2c_hal.h"
#include "stm32g0xx_hal.h"

extern I2C_HandleTypeDef hi2c1; // Make sure hi2c1 is initialized via CubeMX

#define NOT_IMPLEMENTED_ERROR -1

// Single-bus setup: do nothing
int16_t sensirion_i2c_hal_select_bus(uint8_t bus_idx) {
    return 0;
}

// I2C peripheral is initialized in CubeMX, so nothing needed here
void sensirion_i2c_hal_init(void) {
}

// Nothing to free for STM32 HAL
void sensirion_i2c_hal_free(void) {
}

// I2C read using HAL
int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint8_t count) {
    if(HAL_I2C_Master_Receive(&hi2c1, (address << 1), data, count, HAL_MAX_DELAY) == HAL_OK)
        return 0;
    else
        return -1;
}

// I2C write using HAL
int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data, uint8_t count) {
    if(HAL_I2C_Master_Transmit(&hi2c1, (address << 1), (uint8_t*)data, count, HAL_MAX_DELAY) == HAL_OK)
        return 0;
    else
        return -1;
}

// Sleep in microseconds (approximate using HAL_Delay)
void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
    HAL_Delay((useconds + 999) / 1000);  // round up to milliseconds
}
