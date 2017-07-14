#ifndef __I2C_H
#define __I2C_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


uint8_t I2C_Read(uint8_t addr, uint8_t *pData, uint8_t len);

uint8_t I2C_Write(uint8_t addr, uint8_t *pData, uint8_t len);


#endif
