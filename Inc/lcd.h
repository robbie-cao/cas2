#ifndef __LCD_H
#define __LCD_H

#include <stdlib.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"
//#include "lcd_font.h"
#include "config.h"



/* Pin definitions */
#define LCD_RST_PORT      GPIOF
#define LCD_RST_PIN       GPIO_PIN_10
#define LCD_RST_CLK_ENABLE()           __HAL_RCC_GPIOF_CLK_ENABLE()
#define LCD_RST_CLK_DISABLE()          __HAL_RCC_GPIOF_CLK_DISABLE()

#define LCD_RST_SET			HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET)
#define LCD_RST_RESET			HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET)


#define LCD_BKL_PORT      GPIOF
#define LCD_BKL_PIN       GPIO_PIN_9
#define LCD_BKL_CLK_ENABLE()           __HAL_RCC_GPIOF_CLK_ENABLE()
#define LCD_BKL_CLK_DISABLE()          __HAL_RCC_GPIOF_CLK_DISABLE()

#define LCD_BKL_SET			HAL_GPIO_WritePin(LCD_BKL_PORT, LCD_BKL_PIN, GPIO_PIN_SET)
#define LCD_BKL_RESET			HAL_GPIO_WritePin(LCD_BKL_PORT, LCD_BKL_PIN, GPIO_PIN_RESET)


typedef uint32_t  u32;
typedef uint16_t  u16;
typedef uint8_t   u8;


typedef volatile uint32_t vu32;
typedef volatile uint16_t vu16;
typedef volatile uint8_t  vu8;

typedef struct
{
  u16 width;      // LCD 宽度
  u16 height;     // LCD 高度
  u16 id;         // LCD ID
  u8  dir;        // 横屏还是竖屏控制：0，竖屏；1，横屏。
  u16 wramcmd;    // 开始写gram指令
  u16 setxcmd;    // 设置x坐标指令
  u16 setycmd;    // 设置y坐标指令
}_lcd_dev;


extern _lcd_dev   lcddev;      // 管理LCD重要参数
extern u32        POINT_COLOR; // 默认红色
extern u32        BACK_COLOR;  // 背景颜色.默认void LCD_ShowHonDigit(u16 x, u16 y, u8 num, Font_t *font, uint16_t color)


typedef struct {
	uint8_t width;    
	uint8_t height;   
	const unsigned char *data; 
} Font_t;




#define SOFT_RESET    0x1
#define LEFT          0
#define RIGHT         1

enum clear_mode
{
    CLASSIC =0,
    FASION  =1
};

enum slide_index
{
    INDEX_0 =0,
    INDEX_1 =1,
    INDEX_2 =2,
    INDEX_3 =3,
    INDEX_4 =4
};

#define ICON_SENSOR_WIDTH  200
#define ICON_SENSOR_HEIGHT  40
#define ICON_SENSOR_XPOS   140
#define ICON_SENSOR_YPOS   50

#define ICON_DOT_WIDTH  16
#define ICON_DOT_HEIGHT 16
#define ICON_DOT_XPOS   180
#define ICON_DOT_YPOS   14
#define ICON_DOT_GAP    26


#define ICON_SYMBOL_XPOS  40
#define ICON_SYMBOL_YPOS  290
#define ICON_SYMBOL_GAP   16
#define ICON_SYMBOL_WIDTH  16
#define ICON_SYMBOL_HEIGHT 16


#define DIGIT_DOT_XPOS  280
#define DIGIT_DOT_YPOS  230

#define DIGIT_XPOS    48
#define DIGIT_YPOS    118
#define DOT_XPOS_ADJ  24

#define DIG_DOT_WIDTH   20
#define DIG_DOT_HEIGHT  20

#define DIGIT_WIDTH       96
#define DIGIT_HEIGHT     192

#define DIGIT_WIDTH2      112
#define DIGIT_HEIGHT2     144

#define LOGO_XPOS        30
#define LOGO_YPOS       124
#define LOGO_WIDTH      420
#define LOGO_HEIGHT     72

#define BMP_WIDTH      114
#define BMP_HEIGHT     160
#define BMP_YPOS       110

typedef struct
{
  vu16 LCD_REG;
  vu16 LCD_RAM;
} LCD_TypeDef;


#define LCD_BASE        ((u32)(0x60000000 | 0x0007E))
#define LCD             ((LCD_TypeDef *) LCD_BASE)
//////////////////////////////////////////////////////////////////////////////////

// 扫描方向定义
#define L2R_U2D  0 		// 从左到右,从上到下
#define L2R_D2U  1 		// 从左到右,从下到上
#define R2L_U2D  2 		// 从右到左,从上到下
#define R2L_D2U  3 		// 从右到左,从下到上

#define U2D_L2R  4 		// 从上到下,从左到右
#define U2D_R2L  5 		// 从上到下,从右到左
#define D2U_L2R  6 		// 从下到上,从左到右
#define D2U_R2L  7		// 从下到上,从右到左

#define DFT_SCAN_DIR  5  // 默认的扫描方向

// 画笔颜色
#define WHITE         0xFFFF
#define BLACK         0x0000
#define BLUE          0x001F
#define BRED          0xF81F
#define GRED          0xFFE0
#define GBLUE         0x07FF
#define RED           0xF800
#define HON_RED       0xE984
#define MAGENTA       0xF81F
#define GREEN         0x07E0
#define CYAN          0x7FFF
#define YELLOW        0xFFE0
#define BROWN         0xBC40  // 棕色
#define BRRED         0xFC07  // 棕红色
#define GRAY          0x8430  // 灰色

// GUI颜色
#define DARKBLUE      0x01CF  // 深蓝色
#define LIGHTBLUE     0x7D7C	// 浅蓝色
#define GRAYBLUE      0x5458  // 灰蓝色


#define LIGHTGREEN    0x841F  // 浅绿色
#define LGRAY         0xC618  // 浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE     0xA651  // 浅灰蓝色(中间层颜色)
#define LBBLUE        0x2B12  // 浅棕蓝色(选择条目的反色)

void LCD_Init(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);
void LCD_Switch_Off(void);
void LCD_Switch_On(void);
void LCD_Scroll_On(uint8_t mode);
void LCD_Clear(u32 Color);
void LCD_SetCursor(u16 x1, u16 y1, u16 x2, u16 y2);
void LCD_DrawPoint(u16 x, u16 y);
void LCD_Fast_DrawPoint(u16 x, u16 y, u32 color);
u32  LCD_ReadPoint(u16 x, u16 y);
void LCD_Draw_Circle(u16 x0, u16 y0, u8 r);
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2);
void LCD_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u32 color);
void LCD_Color_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 *color);
void LCD_ShowChar(u16 x, u16 y, u8 num, u8 size, u8 mode);
void LCD_ShowNum(u16 x, u16 y, u32 num, u8 len, u8 size);
void LCD_ShowxNum(u16 x, u16 y, u32 num, u8 len, u8 size, u8 mode);
void LCD_ShowString(u16 x, u16 y, u16 width, u16 height, u8 size, u8 *p);

void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue);
u16 LCD_ReadReg(u16 LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(u16 RGB_Code);
void LCD_SSD_BackLightSet(u8 pwm);
void LCD_Scan_Dir(u8 dir);
void LCD_Display_Dir(u8 dir);
void LCD_Set_Window(u16 sx, u16 sy, u16 width, u16 height);	//设置窗口

void LCD_Delay(volatile unsigned int delay);
void LCD_WR_REG(volatile uint16_t regval);
void LCD_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint16_t *pBmp);
void LCD_ShowImage(uint16_t Xpos, uint16_t Ypos, uint16_t width, uint16_t height, uint8_t *pBmp);
void LCD_MaskImage(uint16_t Xpos, uint16_t Ypos, uint16_t width, uint16_t height, uint16_t color);
void LCD_ShowSlide(uint8_t index);
void LCD_ShowDot(void);
void LCD_ShowDigtStr(u8 *p, uint8_t dot_flag, uint8_t bit_width);
void LCD_ShowSymbol(uint8_t index);
void LCD_ShowBMPDigit(u16 x, u16 y, u8 num, Font_t *font, uint16_t color);

void LCD_ShowNumCenterAlign(uint16_t num, Font_t *font, uint16_t color);
void LCD_ShowDotNumCenterAlign(float num, Font_t *font, uint16_t color);
void LCD_UpdateNumPartialCenterAlign(uint16_t num, uint16_t num_old, Font_t *font, uint16_t color);
void LCD_UpdateDotNumPartialCenterAlign(float num, float num_old, Font_t *font, uint16_t color);

#endif




