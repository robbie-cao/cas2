#ifndef __FONT_H
#define __FONT_H

#include "stm32f4xx_hal.h"

typedef struct {
	uint8_t width;    /*!< Font width in pixels */
	uint8_t height;   /*!< Font height in pixels */
	const unsigned char *data; /*!< Pointer to data font data array */
} Font_t;


//常用ASCII表
//偏移量32
//ASCII字符集: !"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmnopqrstuvwxyz{|}~
//PC2LCD2002取模方式设置：阴码+逐列式+顺向+C51格式
//总共：3个字符集（12*12、16*16、24*24和32*32），用户可以自行新增其他分辨率的字符集。
//每个字符所占用的字节数为:(size/8+((size%8)?1:0))*(size/2),其中size:是字库生成时的点阵大小(12/16/24/32...)

//12*12 ASCII字符集点阵
extern const unsigned char asc2_1206[95][12];//16*16 ASCII字符集点阵
extern const unsigned char asc2_1608[95][16];//24*24 ASICII字符集点阵
extern const unsigned char asc2_2412[95][36];
//32*32 ASCII字符集点阵
extern const unsigned char asc2_3216[95][128];
//64*42 数字字符集点阵
extern const unsigned char asc2_6432[10][336];

//96*72 数字字符集点阵
extern const unsigned char asc2_96x72[10][16*83];
//128*83 数字字符集点阵
extern const unsigned char asc2_128[10][16*83];
extern const unsigned char digit[10][2304];
extern const unsigned char honeydigit[10][2304];

// Honeywell Sans TT Bold 144*112
extern const unsigned char honeydigit144_bold[10][2016];

// Honeywell Sans TT Book 144*108
extern const unsigned char honeydigit144_book[10][1944];

// Honeywell Sans TT Light 144*106
extern const unsigned char honeydigit144_light[10][1908];



extern Font_t font_honey_light;


#endif
