#include <string.h>
#include "sensair.h"

#define TIMEOUT         500

#define PORT    huart2

extern UART_HandleTypeDef huart2;

uint8_t S8_Read(uint16_t *c)
{
  uint8_t cmd[8] = {0xFE, 0x04, 0x00, 0x03, 0x00, 0x01, 0xD5, 0xC5};
  uint8_t rcv[8];

  memset(rcv, 0, sizeof(rcv));
  HAL_UART_Transmit(&PORT, cmd, 8, TIMEOUT);
  HAL_UART_Receive(&PORT, rcv, 7, TIMEOUT);
//  for (int i = 0; i < 7; i++) {
//    printf("0x%02x ", rcv[i]);
//  }
//  printf("\r\n");
  if (rcv[1] == 0x04 && rcv[2] == 0x02) {
    uint16_t co2 = rcv[3] << 8 | rcv[4];
    printf("CO2: %d\r\n", co2);
    *c = co2;
  } else {
    *c = 0;
    return ERROR;
  }

  return SUCCESS;
}
