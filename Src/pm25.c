#include <string.h>
#include "config.h"
#include "pm25.h"

#define TIMEOUT         500

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

// Store the correct data reading from sensor
// and use it if there's error when reading next data from sensor
static uint16_t s_pm25_old = 50, s_pm10_old = 50;

uint8_t PM25_EnableAutoSend(void)
{
  uint8_t cmd[] = {0x68, 0x01, 0x40, 0x57};
  uint8_t rcv[8];

  memset(rcv, 0, sizeof(rcv));
  HAL_UART_Transmit(&PM25_UART, cmd, 4, TIMEOUT);
  HAL_UART_Receive(&PM25_UART, rcv, 2, TIMEOUT);
#if SENSOR_DATA_DEBUG
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
#if SENSOR_DATA_DEBUG
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
#if SENSOR_DATA_DEBUG
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
#if SENSOR_DATA_DEBUG
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
  uint8_t cs = 0;
  int i;

  memset(rcv, 0, sizeof(rcv));
  HAL_UART_Transmit(&PM25_UART, cmd, 4, TIMEOUT);
  HAL_UART_Receive(&PM25_UART, rcv, 10, TIMEOUT);
#if SENSOR_DATA_DEBUG
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
    *pm25 = s_pm25_old;
    *pm10 = s_pm10_old;
    return ERROR;
  }

  // checksum
  cs = (65536 - rcv[i] - rcv[i + 1] - rcv[i + 2] - rcv[i + 3] - rcv[i + 4] - rcv[i + 5] - rcv[i + 6]) % 256;
  if (rcv[i+7] == 0 || cs != rcv[i+7]) {
    *pm25 = s_pm25_old;
    *pm10 = s_pm10_old;
    return ERROR;
  }

  *pm25 = (rcv[i + 3] << 8) | rcv[i + 4];
  *pm10 = (rcv[i + 5] << 8) | rcv[i + 6];

  s_pm25_old = *pm25;
  s_pm10_old = *pm10;

  return SUCCESS;
}

