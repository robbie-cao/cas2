#ifndef __VOC_H
#define __VOC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

void Voc_Init(void);

ErrorStatus Read_VocData(uint16_t* pDataCO2 ,uint16_t* pDataVOC);

void Get_VocData(uint16_t* pDataCO2 ,uint16_t* pDataVOC);

#endif
