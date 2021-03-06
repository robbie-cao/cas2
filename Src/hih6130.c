#include <string.h>
#include <math.h>
#include "config.h"
#include "i2c.h"
#include "hih6130.h"

// Store the correct data reading from sensor
// and use it if there's error when reading next data from sensor
static float s_hum_old = 70.0, s_temp_old = 25.0;

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
#if SENSOR_DATA_DEBUG
  printf("I2C W - %d\r\n", res);
#endif
  if (res != HAL_OK) {
    return ERROR;
  }
  /* Wait some time before read operation */
  HAL_Delay(60);
  res = I2C_Read(HIH6130_I2C_ADDRESS, buf, sizeof(buf));
#if SENSOR_DATA_DEBUG
  printf("H/T R - %d\r\n", res);
  printf("Data: ");
  for (int i = 0; i < sizeof(buf); i++) {
    printf("%02x ", buf[i]);
  }
  printf("\r\n");
#endif
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
  float rt = 0.0;

  if (HIH6130_ReadHumiTemp(&humi, &temp)) {
    rt = round((temp * 1.007e-2 - 40.0) * 10.0) / 10.0;     // -> xx.x
    *rh = (float)humi * 6.10e-3;
    *tc = (float)rt;
    s_hum_old = *rh;
    s_temp_old = *tc;
  } else {
    /* Render certain error values to display */
    *rh = s_hum_old;
    *tc = s_temp_old;
  }
}
