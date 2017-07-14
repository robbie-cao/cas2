#include "i2c.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2C_TIMEOUT     10000

extern I2C_HandleTypeDef hi2c1;

uint8_t I2C_Read(uint8_t addr, uint8_t *pData, uint8_t len)
{
  return (uint8_t)HAL_I2C_Master_Receive(&hi2c1, addr, pData, len, I2C_TIMEOUT);
}

uint8_t I2C_Write(uint8_t addr, uint8_t *pData, uint8_t len)
{
  return (uint8_t)HAL_I2C_Master_Transmit(&hi2c1, addr, pData, len, I2C_TIMEOUT);
}
