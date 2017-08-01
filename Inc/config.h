/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONFIG_H
#define __CONFIG_H
/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define SENSOR_DATA_DEBUG       0

#define DISPLAY_INVERSE         0

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

#define PM25_LEVEL_0            0
#define PM25_LEVEL_1            35
#define PM25_LEVEL_2            75
#define PM25_LEVEL_3            150
#define PM25_LEVEL_4            250

#define TVOC_LEVEL_0            0
#define TVOC_LEVEL_1            160
#define TVOC_LEVEL_2            300
#define TVOC_LEVEL_3            500
#define TVOC_LEVEL_4            2000

#define CO2_LEVEL_0             400
#define CO2_LEVEL_1             600
#define CO2_LEVEL_2             800
#define CO2_LEVEL_3             1000
#define CO2_LEVEL_4             1500

#define PM25_THRESHOLD          PM25_LEVEL_3
#define TVOC_THRESHOLD          TVOC_LEVEL_3
#define CO2_THRESHOLD           CO2_LEVEL_3

#define PM25_THRESHOLD_MIN      PM25_LEVEL_0
#define TVOC_THRESHOLD_MIN      TVOC_LEVEL_0
#define CO2_THRESHOLD_MIN       CO2_LEVEL_0

#define PM25_THRESHOLD_MAX      750
#define TVOC_THRESHOLD_MAX      5000
#define CO2_THRESHOLD_MAX       2000


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */

/**
  * @}
*/

#endif /* __CONFIG_H */
