#ifndef __HIH6130_H
#define __HIH6130_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#define HIH6130_I2C_ADDRESS     ((uint8_t)(0x27)<<1)     /* HIH6130 address */
#define HIH6130_REG_TEMP        0x00    /* Temperature Register of HIH6130 */
#define HIH6130_REG_CONF        0x01    /* Configuration Register of HIH6130 */
#define HIH6130_REG_THYS        0x02    /* Temperature Register of HIH6130 */
#define HIH6130_REG_TOS         0x03    /* Over-temp Shutdown Threshold Register of HIH6130 */
#define TH_DELAY_TIME           60      /* in microsecond */
#define TH_REFRESH_RATE         2000    /* in second */


#define ADJ_DURATION            1800000 //1800s
#define ADJ_VAL                 1.5

void HIH6130_Init(void);
ErrorStatus HIH6130_ReadHumiTemp(uint16_t* , uint16_t* );
void Get_HumiTemp(float *, float *);

#endif
