#include <string.h>
#include "pm25.h"
#include "main.h"

#define DEBUG   1

#define TIMEOUT         500

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

uint16_t g_pm25_old = 50, g_pm10_old = 50;

uint8_t PM25_EnableAutoSend(void)
{
  uint8_t cmd[] = {0x68, 0x01, 0x40, 0x57};
  uint8_t rcv[8];

  memset(rcv, 0, sizeof(rcv));
  HAL_UART_Transmit(&PM25_UART, cmd, 4, TIMEOUT);
  HAL_UART_Receive(&PM25_UART, rcv, 2, TIMEOUT);
#if DEBUG
  for (int i = 0; i < 2; i++) {
    printf("0x%02x ", rcv[i]);
  }
  printf("\r\n");
#endif
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
  HAL_UART_Transmit(&PM25_UART, cmd, 4, TIMEOUT);
  HAL_UART_Receive(&PM25_UART, rcv, 2, TIMEOUT);
#if DEBUG
  for (int i = 0; i < 2; i++) {
    printf("0x%02x ", rcv[i]);
  }
  printf("\r\n");
#endif
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
  HAL_UART_Transmit(&PM25_UART, cmd, 4, TIMEOUT);
  HAL_UART_Receive(&PM25_UART, rcv, 2, TIMEOUT);
#if DEBUG
  for (int i = 0; i < 2; i++) {
    printf("0x%02x ", rcv[i]);
  }
  printf("\r\n");
#endif
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
  HAL_UART_Transmit(&PM25_UART, cmd, 4, TIMEOUT);
  HAL_UART_Receive(&PM25_UART, rcv, 2, TIMEOUT);
#if DEBUG
  for (int i = 0; i < 2; i++) {
    printf("0x%02x ", rcv[i]);
  }
  printf("\r\n");
#endif
  if (rcv[0] != 0xA5 || rcv[1] != 0xA5) {
    return ERROR;
  }

  return SUCCESS;
}

uint8_t PM25_Read(uint16_t *pm25, uint16_t *pm10)
{
#define RECV_SIZE       10
  uint8_t cmd[] = {0x68, 0x01, 0x04, 0x93};
  uint8_t rcv[RECV_SIZE];
  int i;

  memset(rcv, 0, sizeof(rcv));
  HAL_UART_Transmit(&PM25_UART, cmd, 4, TIMEOUT);
  HAL_UART_Receive(&PM25_UART, rcv, 10, TIMEOUT);
#if DEBUG
  for (i = 0; i < RECV_SIZE; i++) {
    printf("0x%02x ", rcv[i]);
  }
  printf("\r\n");
#endif

  for (i = 0; i < RECV_SIZE - 8; i++) {
    if (rcv[i] == 0x40 && rcv[i + 1] == 0x05 && rcv[i + 2] == 0x04) {
      break;
    }
  }
  if (!(rcv[i] == 0x40 && rcv[i + 1] == 0x05 && rcv[i + 2] == 0x04)) {
    *pm25 = g_pm25_old;
    *pm10 = g_pm10_old;
    return ERROR;
  }

  *pm25 = (rcv[i + 3] << 8) | rcv[i + 4];
  *pm10 = (rcv[i + 5] << 8) | rcv[i + 6];

  g_pm25_old = *pm25;
  g_pm10_old = *pm10;

  return SUCCESS;
}

