#ifndef DFRobot_LIS2DH12_H
#define DFRobot_LIS2DH12_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Change this to match your STM32 family */
#include "stm32g0xx_hal.h"

#define LIS2DH12_RANGE_2GA  0x00
#define LIS2DH12_RANGE_4GA  0x10
#define LIS2DH12_RANGE_8GA  0x20
#define LIS2DH12_RANGE_16GA 0x30

/* Device context struct replacing the C++ class */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t sensorAddress;
    uint8_t mgScaleVel;
    uint8_t mgPerDigit; 
} DFRobot_LIS2DH12_t;

/* Function Prototypes */
int8_t DFRobot_LIS2DH12_Init(DFRobot_LIS2DH12_t *dev, I2C_HandleTypeDef *hi2c, uint8_t range, uint8_t sensorAddress);
void DFRobot_LIS2DH12_ReadXYZ(DFRobot_LIS2DH12_t *dev, int16_t *x, int16_t *y, int16_t *z);
void DFRobot_LIS2DH12_MgScale(DFRobot_LIS2DH12_t *dev, int16_t *x, int16_t *y, int16_t *z);

uint8_t DFRobot_LIS2DH12_ReadReg(DFRobot_LIS2DH12_t *dev, uint8_t regAddress);
void DFRobot_LIS2DH12_ReadRegs(DFRobot_LIS2DH12_t *dev, uint8_t regAddress, uint8_t *regValue, uint8_t quantity, bool autoIncrement);

uint8_t DFRobot_LIS2DH12_WriteReg(DFRobot_LIS2DH12_t *dev, uint8_t regAddress, uint8_t regValue);
uint8_t DFRobot_LIS2DH12_WriteRegs(DFRobot_LIS2DH12_t *dev, uint8_t regAddress, uint8_t *regValue, size_t quantity, bool autoIncrement);

void DFRobot_LIS2DH12_SetRange(DFRobot_LIS2DH12_t *dev, uint8_t range);

#endif /* DFRobot_LIS2DH12_H */
