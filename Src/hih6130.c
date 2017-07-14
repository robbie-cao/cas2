#include <string.h>
#include "i2c.h"
#include "hih6130.h"

void HIH6130_Init(void)
{
}

/**
 * @brief  Read H/T sensor raw data and do some simple processing
 * @param  1: pointer to 16-bit humidity data
 *         2: pointer to 16-bit temp data
 * @retval Operation SUCCESS or ERROR
 */
ErrorStatus HIH6130_ReadHumiTemp(uint16_t* pDataH, uint16_t* pDataT)
{
  uint8_t res = 0;
  uint8_t buf[4];

  memset(buf, 0, sizeof(buf));
  res = I2C_Write(HIH6130_I2C_ADDRESS, NULL, 0);
//  printf("I2C W - %d\r\n", res);
  if (res != HAL_OK) {
    return ERROR;
  }
  /* Wait some time before read operation */
  HAL_Delay(60);
  res = I2C_Read(HIH6130_I2C_ADDRESS, buf, sizeof(buf));
//  printf("H/T R - %d\r\n", res);
//  printf("Data: ");
//  for (int i = 0; i < sizeof(buf); i++) {
//    printf("%02x ", buf[i]);
//  }
//  printf("\r\n");
  if (res != HAL_OK) {
    return ERROR;
  }
  /* Check status */
  if (buf[0] & 0xC0) {
    return ERROR;
  }
  /* Get Humidity and Temperature */
  *pDataH = ((buf[0] & 0x3F) << 8) | buf[1];
  *pDataT = ((buf[2] << 8) | buf[3]) >> 2;

  /* Return SUCCESS if no ERROR occurs */
  return SUCCESS;
}

/**
 * @brief  Change raw data to human readable.
 * @param  1: pointer to float relative humidity data
 *         2: pointer to float temperature in Celcius
 * @retval None
 */
void Get_HumiTemp(float* rh, float* tc)
{
  uint16_t humi, temp;

  if (HIH6130_ReadHumiTemp(&humi, &temp)) {
    *rh = (float)humi * 6.10e-3;
    *tc = (float)temp * 1.007e-2 - 40.0;
  } else {
    /* Render certain error values to display */
    *rh = -1.0;
    *tc = -1.0;
  }
}
