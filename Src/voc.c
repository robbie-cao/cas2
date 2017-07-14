#include <string.h>
#include "voc.h"
#include "i2c.h"

#define IAQ_CORE_I2C_ADDRESS    (0x5A << 1)

void Voc_Init(void)
{
}

ErrorStatus Read_VocData(uint16_t* pDataCO2 ,uint16_t* pDataVOC)
{
  uint8_t res = 0;
  uint8_t buf[9];

  memset(buf, 0, sizeof(buf));
  res = I2C_Read(IAQ_CORE_I2C_ADDRESS, buf, sizeof(buf));
//  printf("IAQ R - %d\r\n", res);
//  printf("Data: ");
//  for (int i = 0; i < sizeof(buf); i++) {
//    printf("%02x ", buf[i]);
//  }
//  printf("\r\n");

  if (res != HAL_OK) {
    return ERROR;
  }
  /**
   * Check status:
   *   0x01 - Busy;
   *   0x80 - ERROR;
   *   0x00 - OK
   */
  if (buf[2]) {
    return ERROR;
  }
  /* Store data for processing */
  *pDataCO2 = (((uint16_t)buf[0]) << 8) | buf[1];
  *pDataVOC = (((uint16_t)buf[7]) << 8) | buf[8];

  /* Return SUCCESS if no ERROR occurs */
  return SUCCESS;
}

/**
 * @brief  Change raw data to human readable.
 * @param  1: pointer to uint16_t VOC DATA
 * @retval None
 */
void Get_VocData(uint16_t* pDataCO2 ,uint16_t* pDataVOC)
{
  uint16_t co2, voc;

  if (Read_VocData(&co2, &voc)) {
    *pDataCO2 = (uint16_t)co2;
    *pDataVOC = (uint16_t)voc;
  } else {
    /* Render certain error values to display */
    *pDataCO2 = 0;
    *pDataVOC = 0;
  }
}
