#include <string.h>
#include "config.h"
#include "sensair.h"

#define TIMEOUT         500

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

// Store the correct data reading from sensor
// and use it if there's error when reading next data from sensor
static uint16_t s_co2_old = 500;

uint8_t S8_Read(uint16_t *c)
{
  uint8_t cmd[8] = {0xFE, 0x04, 0x00, 0x03, 0x00, 0x01, 0xD5, 0xC5};
  uint8_t rcv[8];

  memset(rcv, 0, sizeof(rcv));
  HAL_UART_Transmit(&CO2_S8_UART, cmd, 8, TIMEOUT);
  HAL_UART_Receive(&CO2_S8_UART, rcv, 7, TIMEOUT);
#if SENSOR_DATA_DEBUG
  for (int i = 0; i < 7; i++) {
    printf("0x%02x ", rcv[i]);
  }
  printf("\r\n");
#endif
  if (rcv[1] == 0x04 && rcv[2] == 0x02) {
    uint16_t co2 = rcv[3] << 8 | rcv[4];
    if (co2 > 9999) {
      co2 = 9999;
    }
    *c = co2;
    s_co2_old = *c;
  } else {
    *c = s_co2_old;
    return ERROR;
  }

  return SUCCESS;
}
