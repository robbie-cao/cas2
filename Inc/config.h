/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONFIG_H
#define __CONFIG_H
/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define SENSOR_DATA_DEBUG       0

#define TEMPERATURE_NO_DOT      1
#define USE_BMP_FONT            1

#define TEMPERATURE_COMPENSATE  1
#define FAHRENHEIT_DEGREE       1

#define COMM_RECV_BUF_MAX       16
#define COMM_SEND_BUF_MAX       256
#define REC_TIM_DELAY           50

#define WIFI_COMM_UART          huart6  // -> uart6
#define CO2_S8_UART             huart3  // -> uart3
#define PM25_UART               huart2  // -> uart2

#define PM25_THRESHOLD          100
#define TVOC_THRESHOLD          375
#define CO2_THRESHOLD           750

#define PM25_THRESHOLD_MAX      1000
#define TVOC_THRESHOLD_MAX      1000
#define CO2_THRESHOLD_MAX       2000

#define PM25_THRESHOLD_MIN      0
#define TVOC_THRESHOLD_MIN      0
#define CO2_THRESHOLD_MIN       200

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */

/**
  * @}
*/

#endif /* __CONFIG_H */
