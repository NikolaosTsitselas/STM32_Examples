#include "DFRobot_LIS2DH12.h"

int8_t DFRobot_LIS2DH12_Init(DFRobot_LIS2DH12_t *dev, I2C_HandleTypeDef *hi2c, uint8_t range, uint8_t sensorAddress)
{
    dev->hi2c = hi2c;
    /* STM32 HAL expects the 7-bit address shifted left by 1 */
    dev->sensorAddress = sensorAddress << 1; 
    
    DFRobot_LIS2DH12_SetRange(dev, range);
    
    /* Check if device is ready on the I2C bus */
    if (HAL_I2C_IsDeviceReady(dev->hi2c, dev->sensorAddress, 3, 100) != HAL_OK) {
        return -1;
    }

    uint8_t ctrl_reg_values[] = {0x2F, 0x00, 0x00, range, 0x00, 0x00};
    
    /* Write configuration array to registers starting at 0x20 (using 0xA0 bitmask) */
    return (int8_t)DFRobot_LIS2DH12_WriteRegs(dev, 0xA0, ctrl_reg_values, sizeof(ctrl_reg_values), true);
}

void DFRobot_LIS2DH12_ReadXYZ(DFRobot_LIS2DH12_t *dev, int16_t *x, int16_t *y, int16_t *z) 
{
    uint8_t sensorData[6];
    DFRobot_LIS2DH12_ReadRegs(dev, 0xA8, sensorData, 6, true); 
    
    *x = ((int8_t)sensorData[1])*256+sensorData[0]; 
    *y = ((int8_t)sensorData[3])*256+sensorData[2];
    *z = ((int8_t)sensorData[5])*256+sensorData[4];
}

void DFRobot_LIS2DH12_MgScale(DFRobot_LIS2DH12_t *dev, int16_t *x, int16_t *y, int16_t *z)
{
    *x = (int32_t)(*x)*1000/(1024 * dev->mgScaleVel); 
    *y = (int32_t)(*y)*1000/(1024 * dev->mgScaleVel); 
    *z = (int32_t)(*z)*1000/(1024 * dev->mgScaleVel); 
}

uint8_t DFRobot_LIS2DH12_ReadReg(DFRobot_LIS2DH12_t *dev, uint8_t regAddress)
{
    uint8_t regValue = 0;
    HAL_I2C_Mem_Read(dev->hi2c, dev->sensorAddress, regAddress, I2C_MEMADD_SIZE_8BIT, &regValue, 1, 100);
    return regValue;
}

void DFRobot_LIS2DH12_ReadRegs(DFRobot_LIS2DH12_t *dev, uint8_t regAddress, uint8_t *regValue, uint8_t quantity, bool autoIncrement)
{   
    uint8_t reg = regAddress;
    if(autoIncrement){
        reg = 0x80 | regAddress; /* LIS2DH12 requires MSB set for auto-increment */
        HAL_I2C_Mem_Read(dev->hi2c, dev->sensorAddress, reg, I2C_MEMADD_SIZE_8BIT, regValue, quantity, 100);
    } else {
        for(uint8_t i = 0; i < quantity; i++){
            HAL_I2C_Mem_Read(dev->hi2c, dev->sensorAddress, regAddress + i, I2C_MEMADD_SIZE_8BIT, &regValue[i], 1, 100);
        }
    }
}

uint8_t DFRobot_LIS2DH12_WriteReg(DFRobot_LIS2DH12_t *dev, uint8_t regAddress, uint8_t regValue)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(dev->hi2c, dev->sensorAddress, regAddress, I2C_MEMADD_SIZE_8BIT, &regValue, 1, 100);
    return (status == HAL_OK) ? 0 : 1; 
}

uint8_t DFRobot_LIS2DH12_WriteRegs(DFRobot_LIS2DH12_t *dev, uint8_t regAddress, uint8_t *regValue, size_t quantity, bool autoIncrement)
{   
    HAL_StatusTypeDef status = HAL_OK;
    if(autoIncrement) {
        status = HAL_I2C_Mem_Write(dev->hi2c, dev->sensorAddress, regAddress, I2C_MEMADD_SIZE_8BIT, regValue, quantity, 100);
    } else {
        for(size_t i = 0; i < quantity; i++){
            status = HAL_I2C_Mem_Write(dev->hi2c, dev->sensorAddress, regAddress + i, I2C_MEMADD_SIZE_8BIT, &regValue[i], 1, 100);
            if(status != HAL_OK) break;
        }
    }
    return (status == HAL_OK) ? 0 : 1;
}

void DFRobot_LIS2DH12_SetRange(DFRobot_LIS2DH12_t *dev, uint8_t range)
{
    switch(range)
    {
    case LIS2DH12_RANGE_2GA:
        dev->mgScaleVel = 16;
        break;
    case LIS2DH12_RANGE_4GA:
        dev->mgScaleVel = 8;
        break;
    case LIS2DH12_RANGE_8GA:
        dev->mgScaleVel = 4;
        break;
    case LIS2DH12_RANGE_16GA:
        dev->mgScaleVel = 2;
        break;
    default:
        dev->mgScaleVel = 16;
        break;
    }
}