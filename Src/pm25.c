#include <string.h>
#include "pm25.h"

#define TIMEOUT         500
#define PORT    huart2

extern UART_HandleTypeDef huart2;

uint8_t PM25_EnableAutoSend(void)
{
  uint8_t cmd[] = {0x68, 0x01, 0x40, 0x57};
  uint8_t rcv[8];

  memset(rcv, 0, sizeof(rcv));
  HAL_UART_Transmit(&PORT, cmd, 4, TIMEOUT);
  HAL_UART_Receive(&PORT, rcv, 2, TIMEOUT);
//  for (int i = 0; i < 2; i++) {
//    printf("0x%02x ", rcv[i]);
//  }
//  printf("\r\n");
  if (rcv[0] != 0xA5 || rcv[1] != 0xA5) {
    return ERROR;
  }

  return SUCCESS;
}

uint8_t PM25_StopAutoSend(void)
{
  uint8_t cmd[] = {0x68, 0x01, 0x20, 0x77};
  uint8_t rcv[8];

  memset(rcv, 0, sizeof(rcv));
  HAL_UART_Transmit(&PORT, cmd, 4, TIMEOUT);
  HAL_UART_Receive(&PORT, rcv, 2, TIMEOUT);
//  for (int i = 0; i < 2; i++) {
//    printf("0x%02x ", rcv[i]);
//  }
//  printf("\r\n");
  if (rcv[0] != 0xA5 || rcv[1] != 0xA5) {
    return ERROR;
  }

  return SUCCESS;
}

uint8_t PM25_StartMeasurement(void)
{
  uint8_t cmd[] = {0x68, 0x01, 0x01, 0x96};
  uint8_t rcv[8];

  memset(rcv, 0, sizeof(rcv));
  HAL_UART_Transmit(&PORT, cmd, 4, TIMEOUT);
  HAL_UART_Receive(&PORT, rcv, 2, TIMEOUT);
//  for (int i = 0; i < 2; i++) {
//    printf("0x%02x ", rcv[i]);
//  }
//  printf("\r\n");
  if (rcv[0] != 0xA5 || rcv[1] != 0xA5) {
    return ERROR;
  }

  return SUCCESS;
}

uint8_t PM25_StopMeasurement(void)
{
  uint8_t cmd[] = {0x68, 0x01, 0x02, 0x95};
  uint8_t rcv[8];

  memset(rcv, 0, sizeof(rcv));
  HAL_UART_Transmit(&PORT, cmd, 4, TIMEOUT);
  HAL_UART_Receive(&PORT, rcv, 2, TIMEOUT);
//  for (int i = 0; i < 2; i++) {
//    printf("0x%02x ", rcv[i]);
//  }
//  printf("\r\n");
  if (rcv[0] != 0xA5 || rcv[1] != 0xA5) {
    return ERROR;
  }

  return SUCCESS;
}

uint8_t PM25_Read(uint16_t *pm25, uint16_t *pm10)
{
  uint8_t cmd[] = {0x68, 0x01, 0x04, 0x93};
  uint8_t rcv[8];

  memset(rcv, 0, sizeof(rcv));
  HAL_UART_Transmit(&PORT, cmd, 4, TIMEOUT);
  HAL_UART_Receive(&PORT, rcv, 8, TIMEOUT);
//  for (int i = 0; i < 8; i++) {
//    printf("0x%02x ", rcv[i]);
//  }
//  printf("\r\n");
  if (!(rcv[0] == 0x40 && rcv[1] == 0x05 && rcv[2] == 0x04)) {
    *pm25 = 0;
    *pm10 = 0;
    return ERROR;
  }

  *pm25 = (rcv[3] << 8) | rcv[4];
  *pm10 = (rcv[5] << 8) | rcv[6];

  return SUCCESS;
}

