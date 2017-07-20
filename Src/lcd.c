#include "lcd.h"
#include "stdlib.h"
#include "lcd_font.h"
#include "slide.h"



u32 POINT_COLOR=0xFF000000;		//画笔颜色
u32 BACK_COLOR =0xFFFFFFFF;  	//背景色

#define PIXEL_OFF  0x23
#define NORMAL_DISP_ON  0x13

_lcd_dev lcddev;

void LCD_Delay(volatile unsigned int delay) {
	for (; delay != 0; delay--);
}


void LCD_WR_REG(volatile uint16_t regval)
{
	regval=regval;		//使用-O2优化的时候,必须插入的延时
	LCD->LCD_REG=regval & 0xFF;    //写入要写的寄存器序号
 }

void LCD_WR_DATA(volatile uint16_t data)
{
	data=data;			//使用-O2优化的时候,必须插入的延时
	LCD->LCD_RAM=data;
 }

u16 LCD_RD_DATA(void)
{
	vu16 ram;			//防止被优化
	ram=LCD->LCD_RAM;
	return ram;
}

void LCD_WriteReg(u16 LCD_Reg,u16 LCD_RegValue)
{
	LCD->LCD_REG = LCD_Reg;		//写入要写的寄存器序号
	LCD->LCD_RAM = LCD_RegValue;    //写入数据
}

u16 LCD_ReadReg(u16 LCD_Reg)
{
	LCD_WR_REG(LCD_Reg);		//写入要读的寄存器序号
	LCD_Delay(50);
	return LCD_RD_DATA();		//返回读到的值
}

void LCD_WriteRAM_Prepare(void)
{
 	LCD->LCD_REG=lcddev.wramcmd;
}

void LCD_WriteRAM(u16 RGB_Code)
{
	LCD->LCD_RAM = RGB_Code;//写十六位GRAM
}

u16 LCD_BGR2RGB(u16 c)
{
	u16  r,g,b,rgb;
	b=(c>>0)&0x1f;
	g=(c>>5)&0x3f;
	r=(c>>11)&0x1f;
	rgb=(b<<11)+(g<<5)+(r<<0);
	return(rgb);
}

void opt_delay(u8 i)
{
	while(i--);
}

u32 LCD_ReadPoint(u16 x,u16 y)
{
 	u16 r=0,g=0,b=0;
	if(x>=lcddev.width||y>=lcddev.height)return 0;	//超过了范围,直接返回
	LCD_SetCursor(x,y,x,y);
	LCD_WR_REG(0X2E); // 发送读GRAM指令
 	r=LCD_RD_DATA();								//dummy Read
	opt_delay(2);
 	r=LCD_RD_DATA();  		  						//实际坐标颜色
	//9341要分2次读出
	opt_delay(2);
	b=LCD_RD_DATA();
	g=r&0XFF;		//对于9341,第一次读取的是RG的值,R在前,G在后,各占8位
	g<<=8;
	return (((r>>11)<<11)|((g>>10)<<5)|(b>>11));
}

void LCD_DisplayOn(void)
{
	LCD_WR_REG(0X29);	//开启显示
}

void LCD_DisplayOff(void)
{
	LCD_WR_REG(0X28);	//关闭显示
}

void LCD_Switch_Off(void)
{
        LCD_WR_REG(PIXEL_OFF);
        LCD_WR_DATA(0);
}

void LCD_Scroll_On(uint8_t mode)
{
        uint16_t index;
        LCD_WR_REG(0x33);
        LCD_WR_DATA(0);
        LCD_WR_DATA(0);
        LCD_WR_DATA(0x1);
        LCD_WR_DATA(0xDF);
        LCD_WR_DATA(0);
        LCD_WR_DATA(0);

        if(mode==LEFT)
        {
           for(index=0; index<lcddev.width;index++)
          {
           LCD_WR_REG(0x37);
           LCD_WR_DATA((index>>8)&0xFF);
           LCD_WR_DATA(index & 0xFF);
           LCD_Delay(10000);
          }
        }
        else
        {
          for(index=lcddev.width; index>0;index--)
          {
           LCD_WR_REG(0x37);
           LCD_WR_DATA((index>>8)&0xFF);
           LCD_WR_DATA(index & 0xFF);
           LCD_Delay(10000);
          }

        }
}

void LCD_Switch_On(void)
{
        LCD_WR_REG(NORMAL_DISP_ON);
        LCD_WR_DATA(0);
}

void LCD_SetCursor(u16 x1, u16 y1, u16 x2, u16 y2)
{
		LCD_WR_REG(lcddev.setxcmd);
		LCD_WR_DATA(x1>>8);LCD_WR_DATA(x1&0xFF);
                LCD_WR_DATA(x2>>8);LCD_WR_DATA(x2&0xFF);
		LCD_WR_REG(lcddev.setycmd);
		LCD_WR_DATA(y1>>8);LCD_WR_DATA(y1&0xFF);
                LCD_WR_DATA(y2>>8);LCD_WR_DATA(y2&0xFF);
}

void LCD_Scan_Dir(u8 dir)
{
	u16 regval=0;
	u16 dirreg=0;
	u16 temp;

	switch(dir)
	{
		case L2R_U2D://从左到右,从上到下
			regval|=(0<<7)|(0<<6)|(0<<5);
			break;
		case L2R_D2U://从左到右,从下到上
			regval|=(1<<7)|(0<<6)|(0<<5);
			break;
		case R2L_U2D://从右到左,从上到下
			regval|=(0<<7)|(1<<6)|(0<<5);
			break;
		case R2L_D2U://从右到左,从下到上
			regval|=(1<<7)|(1<<6)|(0<<5);
			break;
		case U2D_L2R://从上到下,从左到右
			regval|=(0<<7)|(0<<6)|(1<<5);
			break;
		case U2D_R2L://从上到下,从右到左
			regval|=(0<<7)|(1<<6)|(1<<5);
			break;
		case D2U_L2R://从下到上,从左到右
			regval|=(1<<7)|(0<<6)|(1<<5);
			break;
		case D2U_R2L://从下到上,从右到左
			regval|=(1<<7)|(1<<6)|(1<<5);
			break;
		}

		dirreg=0X36;
                regval|=0X08;

 		LCD_WriteReg(dirreg,regval);

			if(regval&0X20)
			{
				if(lcddev.width<lcddev.height)//交换X,Y
				{
					temp=lcddev.width;
					lcddev.width=lcddev.height;
					lcddev.height=temp;
				}
			}
                        else
			{
				if(lcddev.width>lcddev.height)//交换X,Y
				{
					temp=lcddev.width;
					lcddev.width=lcddev.height;
					lcddev.height=temp;
				}
			}


			LCD_WR_REG(lcddev.setxcmd);
			LCD_WR_DATA(0);LCD_WR_DATA(0);
			LCD_WR_DATA((lcddev.width-1)>>8);LCD_WR_DATA((lcddev.width-1)&0XFF);
			LCD_WR_REG(lcddev.setycmd);
			LCD_WR_DATA(0);LCD_WR_DATA(0);
			LCD_WR_DATA((lcddev.height-1)>>8);LCD_WR_DATA((lcddev.height-1)&0XFF);

}

void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_SetCursor(x,y,x,y);		//设置光标位置
	LCD_WriteRAM_Prepare();	        //开始写入GRAM
	LCD->LCD_RAM=POINT_COLOR;
}

void LCD_Fast_DrawPoint(u16 x,u16 y,u32 color)
{

	LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(x>>8);LCD_WR_DATA(x&0XFF);   //set SC
        LCD_WR_DATA(x>>8);LCD_WR_DATA(x&0XFF);   //set EC

	LCD_WR_REG(lcddev.setycmd);
	LCD_WR_DATA(y>>8);LCD_WR_DATA(y&0XFF); 	 //set SP
        LCD_WR_DATA(y>>8);LCD_WR_DATA(y&0XFF);   //set EP

	LCD->LCD_REG=lcddev.wramcmd;
	LCD->LCD_RAM=color;
}


void LCD_Display_Dir(u8 dir)
{
	lcddev.dir=dir;		//竖屏/横屏
	if(dir==0)			//竖屏
	{
		lcddev.width=320;
		lcddev.height=480;
        }
	else 				//横屏
	{
		lcddev.width=480;
		lcddev.height=320;

	}
        lcddev.wramcmd=0X2C;
	lcddev.setxcmd=0X2A;
	lcddev.setycmd=0X2B;
	LCD_Scan_Dir(DFT_SCAN_DIR);	//默认扫描方向
}

void LCD_Set_Window(u16 sx,u16 sy,u16 width,u16 height)
{
	u16 twidth,theight;
	twidth=sx+width-1;
	theight=sy+height-1;

		LCD_WR_REG(lcddev.setxcmd);
		LCD_WR_DATA(sx>>8);
		LCD_WR_DATA(sx&0XFF);
		LCD_WR_DATA(twidth>>8);
		LCD_WR_DATA(twidth&0XFF);

		LCD_WR_REG(lcddev.setycmd);
		LCD_WR_DATA(sy>>8);
		LCD_WR_DATA(sy&0XFF);
		LCD_WR_DATA(theight>>8);
		LCD_WR_DATA(theight&0XFF);

}


void LCD_Init(void)
{
  /* Force reset */
  LCD_RST_RESET;
  LCD_Delay(20000);
  LCD_RST_SET;

  /* Delay for RST response */
  LCD_Delay(20000);

  /* Software reset */
  LCD_WR_REG(SOFT_RESET);
  LCD_Delay(50000);

	LCD_WR_REG(0xE0);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x03);
	LCD_WR_DATA(0x0C);
	LCD_WR_DATA(0x09);
	LCD_WR_DATA(0x17);
	LCD_WR_DATA(0x09);
	LCD_WR_DATA(0x3E);
	LCD_WR_DATA(0x89);
	LCD_WR_DATA(0x49);
	LCD_WR_DATA(0x08);
	LCD_WR_DATA(0x0D);
	LCD_WR_DATA(0x0A);
	LCD_WR_DATA(0x13);
	LCD_WR_DATA(0x15);
	LCD_WR_DATA(0x0f);

	LCD_WR_REG(0xE1);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x11);
	LCD_WR_DATA(0x15);
	LCD_WR_DATA(0x03);
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x05);
	LCD_WR_DATA(0x2D);
	LCD_WR_DATA(0x34);
	LCD_WR_DATA(0x41);
	LCD_WR_DATA(0x02);
	LCD_WR_DATA(0x0B);
	LCD_WR_DATA(0x0A);
	LCD_WR_DATA(0x33);
	LCD_WR_DATA(0x37);
	LCD_WR_DATA(0x0f);

	LCD_WR_REG(0xC0);
	LCD_WR_DATA(0x17);
	LCD_WR_DATA(0x15);

	LCD_WR_REG(0xC1);
	LCD_WR_DATA(0x41);

	LCD_WR_REG(0xC5);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x12); // VCOM
	LCD_WR_DATA(0x80);

	LCD_WR_REG(0x36);
	LCD_WR_DATA(0x08);

	LCD_WR_REG(0x3A);  //Interface Mode Control
	LCD_WR_DATA(0x55);  //5-6-5

	LCD_WR_REG(0XB0);  //Interface Mode Control
	LCD_WR_DATA(0x00);

	LCD_WR_REG(0xB4);
	LCD_WR_DATA(0x02);

	LCD_WR_REG(0xB6);  //mcu-interface
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x22);
	LCD_WR_DATA(0x3B);


	LCD_WR_REG(0xE9);
	LCD_WR_DATA(0x00);

	LCD_WR_REG(0XF7);
	LCD_WR_DATA(0xA9);
	LCD_WR_DATA(0x51);
	LCD_WR_DATA(0x2C);
	LCD_WR_DATA(0x82);

	LCD_WR_REG(0x11); //Exit Sleep
	LCD_Delay(15);
	LCD_WR_REG(0x29); //Display on
	LCD_Delay(10);

	LCD_Display_Dir(1);		//默认为竖屏
	LCD_Clear(BLACK);
}

void LCD_Clear(u32 color)
{
	u32 index=0;
	u32 totalpoint=lcddev.width;
	totalpoint*=lcddev.height; 			//得到总点数
	LCD_SetCursor(0, 0,lcddev.width-1,lcddev.height-1 );			//设置光标位置
	LCD_WriteRAM_Prepare();     		//开始写入GRAM
	for(index=0;index<totalpoint;index++)
	{
		LCD->LCD_RAM=color;
	}
}

void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u32 color)
{
	u16 i,j;
	u16 xlen=0;
	xlen=ex-sx+1;
	LCD_SetCursor(sx,sy,ex,ey);      				//设置光标位置
	LCD_WriteRAM_Prepare();     			//开始写入GRAM
	for(i=sy;i<=ey;i++)
	{
		for(j=0;j<xlen;j++)LCD->LCD_RAM=color;	//显示颜色
	}
}

void LCD_Color_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 *color)
{
	u16 height,width;
	u16 i,j;
	width=ex-sx+1; 			//得到填充的宽度
	height=ey-sy+1;			//高度
	for(i=0;i<height;i++)
	{
		LCD_SetCursor(sx,sy+i,sx,sy+i);   	//设置光标位置
		LCD_WriteRAM_Prepare();     //开始写入GRAM
		for(j=0;j<width;j++)LCD->LCD_RAM=color[i*width+j];//写入数据
	}
}

void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
	u16 t;
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //计算坐标增量
	delta_y=y2-y1;
	uRow=x1;
	uCol=y1;
	if(delta_x>0)incx=1; //设置单步方向
	else if(delta_x==0)incx=0;//垂直线
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if(delta_y==0)incy=0;//水平线
	else{incy=-1;delta_y=-delta_y;}
	if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴
	else distance=delta_y;
	for(t=0;t<=distance+1;t++ )//画线输出
	{
		LCD_DrawPoint(uRow,uCol);//画点
		xerr+=delta_x ;
		yerr+=delta_y ;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}

void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	LCD_DrawLine(x1,y1,x2,y1);
	LCD_DrawLine(x1,y1,x1,y2);
	LCD_DrawLine(x1,y2,x2,y2);
	LCD_DrawLine(x2,y1,x2,y2);
}

void LCD_Draw_Circle(u16 x0,u16 y0,u8 r)
{
	int a,b;
	int di;
	a=0;b=r;
	di=3-(r<<1);             //判断下个点位置的标志
	while(a<=b)
	{
		LCD_DrawPoint(x0+a,y0-b);             //5
 		LCD_DrawPoint(x0+b,y0-a);             //0
		LCD_DrawPoint(x0+b,y0+a);             //4
		LCD_DrawPoint(x0+a,y0+b);             //6
		LCD_DrawPoint(x0-a,y0+b);             //1
 		LCD_DrawPoint(x0-b,y0+a);
		LCD_DrawPoint(x0-a,y0-b);             //2
  		LCD_DrawPoint(x0-b,y0-a);             //7
		a++;
		//使用Bresenham算法画圆
		if(di<0)di +=4*a+6;
		else
		{
			di+=10+4*(a-b);
			b--;
		}
	}
}

void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode)
{
    u16 temp,t1,t;
	u16 y0=y;
	u16 csize;
        if (size == 64) {
          num = num - '0';
          csize=(size/8+((size%8)?1:0))*42;
          LCD_Fill(x,y,x+42-1,y+size-1,BLACK);
        } else if (size == 96) {
          num = num - '0';
          csize=(size/8+((size%8)?1:0))*72;
          LCD_Fill(x,y,x+72-1,y+size-1,BLACK);
        } else if (size == 128) {
          num = num - '0';
          csize=(size/8+((size%8)?1:0))*83;
          LCD_Fill(x,y,x+83-1,y+size-1,BLACK);
        } else {
 	num=num-' ';
        csize=(size/8+((size%8)?1:0))*(size/2);		//得到字体一个字符对应点阵集所占的字节数
          LCD_Fill(x,y,x+size/2-1,y+size-1,BLACK);
        }
	for(t=0;t<csize;t++)
	{
		if(size==12)temp=asc2_1206[num][t]; 	 	//调用1206字体
		else if(size==16)temp=asc2_1608[num][t];	//调用1608字体
		else if(size==24)temp=asc2_2412[num][t];	//调用2412字体
		else if(size==32)temp=asc2_3216[num][t];	//调用3216字体
		else if(size==64)temp=asc2_6432[num][t];	//调用3216字体
		else if(size==96)temp=asc2_96x72[num][t];	//调用3216字体
		else if(size==128)temp=asc2_128[num][t];	//调用3216字体
		else return;								//没有的字库
		for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)LCD_Fast_DrawPoint(x,y,POINT_COLOR);
			else if(mode==0)LCD_Fast_DrawPoint(x,y,BACK_COLOR);
			temp<<=1;
			y++;
			if(y>=lcddev.height)return;		//超区域了
			if((y-y0)==size)
			{
				y=y0;
				x++;
				if(x>=lcddev.width)return;	//超区域了
				break;
			}
		}
	}
}
void LCD_ShowDigit(u16 x,u16 y,u8 num,u16 size,u8 mode)
{
        u16 temp,t1,t;
	u16 y0=y;
	u16 csize=(size/8+((size%8)?1:0))*(size/2);		//得到字体一个字符对应点阵集所占的字节数

 	num=num-0x30;

	for(t=0;t<csize;t++)
	{
	       temp=honeydigit[num][t];

		for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)LCD_Fast_DrawPoint(x,y,POINT_COLOR);
			else if(mode==0)LCD_Fast_DrawPoint(x,y,BACK_COLOR);
			temp<<=1;
			y++;
			if(y>=lcddev.height)return;		//超区域了
			if((y-y0)==size)
			{
				y=y0;
				x++;
				if(x>=lcddev.width)return;	//超区域了
				break;
			}
		}
	}
}


void LCD_ShowDigtStr(u8 *p, uint8_t dot_flag, uint8_t bit_width)
{
    uint16_t cur_xpos, cur_ypos;
    cur_xpos=DIGIT_XPOS;
    cur_ypos=DIGIT_YPOS;

    switch(bit_width)
    {
       case 1:  cur_xpos=2*DIGIT_XPOS+ DIGIT_WIDTH;
                 break;
       case 2:  cur_xpos=DIGIT_XPOS+ DIGIT_WIDTH;
                 break;
       case 3:  cur_xpos=DIGIT_WIDTH;
                 break;
        default: cur_xpos=DIGIT_XPOS;
                 break;
     }

    if(dot_flag == 1)
   {

          cur_xpos-=DOT_XPOS_ADJ;
   }

    while(*p!=NULL)
    {
         if((*p>='0') && (*p<='9'))
         {
             LCD_ShowDigit(cur_xpos, cur_ypos, *p, DIGIT_HEIGHT,1);
         }
         else if(*p =='.')
         {
             LCD_ShowDot();
             cur_xpos-=DOT_XPOS_ADJ*2;
         }
         cur_xpos+=DIGIT_WIDTH;
         p++;
    }
}

u32 LCD_Pow(u8 m,u8 n)
{
	u32 result=1;
	while(n--)result*=m;
	return result;
}

void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len,u8 size)
{
	u8 t,temp;
	u8 enshow=0;
	for(t=0;t<len;t++)
	{
		temp=(num/LCD_Pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+(size/2)*t,y,' ',size,0);
				continue;
			}else enshow=1;

		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,0);
	}
}

void LCD_ShowxNum(u16 x,u16 y,u32 num,u8 len,u8 size,u8 mode)
{
	u8 t,temp;
	u8 enshow=0;
	for(t=0;t<len;t++)
	{
		temp=(num/LCD_Pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				if(mode&0X80)LCD_ShowChar(x+(size/2)*t,y,'0',size,mode&0X01);
				else LCD_ShowChar(x+(size/2)*t,y,' ',size,mode&0X01);
 				continue;
			}else enshow=1;

		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,mode&0X01);
	}
}

void LCD_ShowString(u16 x,u16 y, u16 width, u16 height, u8 size,u8 *p)
{
	u8 x0=x;
	width+=x;
	height+=y;
    while((*p<='~')&&(*p>=' '))//判断是不是非法字符!
    {
        if(x>=width){x=x0;y+=size;}
        if(y>=height)break;//退出
        LCD_ShowChar(x,y,*p,size,1);
        if (size == 64) {
          x += 42;
        } else if (size == 96) {
          x += 72;
        } else if (size == 128) {
          x += 83;
        } else {
        x+=size/2;
        }
        p++;
    }
}


/**
  * @brief  Draws a bitmap picture loaded in the STM32 MCU internal memory.
  * @param  Xpos: Bmp X position in the LCD
  * @param  Ypos: Bmp Y position in the LCD
  * @param  pBmp: Pointer to Bmp picture address
  * @retval None
  */
void LCD_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint16_t *pBmp)
{
  uint32_t height = 160, width  =240;

#if 0  //for standard BMP file, we need analyze header as below
  /* Read bitmap width */
  width = *(pBmp + 9);
  width |= (*(pBmp + 10)) << 16;

  /* Read bitmap height */
  height = *(pBmp + 11);
  height |= (*(pBmp + 12)) << 16;

  printf("w: %d, h: %d\r\n", width, height);

  uint32_t index = 0, size = 0;

  /* Read bitmap size */
  size = *(pBmp + 1);
  size |= (*(pBmp + 2)) << 16;

  /* Get bitmap data address offset */
  index = *(pBmp + 5);
  index |= (*(pBmp + 6)) << 16;
  printf("size: %d, index: %d\r\n", size, index);
  size = (size - index) / 2;
  pBmp += index;
#endif


  LCD_SetCursor(Xpos, Ypos, Xpos + width - 1, Ypos + height - 1);

//  LCD_WR_REG(0x36);
//  LCD_WR_DATA(0x20);

  LCD_WR_REG(0x2C);

  uint16_t *p = (uint16_t *)pBmp;
  for (int i = 0; i < height * width; i++)
  {
#if 0
	LCD_WR_DATA(((*p >> 11) & 0x1F) << 2);
	LCD_WR_DATA(((*p >> 5) & 0x3F) << 2);
	LCD_WR_DATA((*p & 0x1F) << 2);
#endif
        LCD_WR_DATA(*p++);
   }

  LCD_SetCursor(0, 0, lcddev.width-1, lcddev.height-1);

}

void LCD_ShowImage(uint16_t Xpos, uint16_t Ypos, uint16_t width, uint16_t height, uint8_t *pBmp)
{
  uint8_t *p = pBmp;
  uint16_t data;
  LCD_Set_Window(Xpos, Ypos, width, height);

   LCD_WR_REG(0x2C);


  for (int i = 0; i < height; i++)
  {
     for(int k=0; k < width; k++)
     {

         data =(*p)|((*(p+1))<<8);
         LCD_Fast_DrawPoint(Xpos+k,Ypos+i,data);
         p+=2;
     }
   }

}

void LCD_MaskImage(uint16_t Xpos, uint16_t Ypos, uint16_t width, uint16_t height, uint16_t color)
{
  LCD_Set_Window(Xpos, Ypos, width, height);
  LCD_WR_REG(0x2C);


  for (int i = 0; i < width/2; i++)
  {
     for(int k=0; k < height; k++)
     {
         LCD_Fast_DrawPoint(Xpos+i,Ypos+k,color);
         LCD_Fast_DrawPoint(Xpos+width-i-1,Ypos+k,color);
         //LCD_Delay(100);
     }
   }

}

void LCD_ShowSlide(uint8_t index)
{
    uint8_t* cur_icon;
    uint16_t cur_xpos;
    uint8_t k;

    cur_xpos=ICON_DOT_XPOS;
    if(index>=0 && index<5)
    {
       for(k=0;k<5;k++)
       {
           if(index==k)
          {
             cur_icon=(uint8_t*)icon_sdot;
           }
           else
          {
             cur_icon=(uint8_t*)icon_dot;
           }

           LCD_ShowImage(cur_xpos, ICON_DOT_YPOS,
                         ICON_DOT_WIDTH, ICON_DOT_HEIGHT, cur_icon);
           cur_xpos+=ICON_DOT_GAP;
       }
    }
}

void LCD_ShowDot(void)
{
      uint8_t* cur_icon;
      cur_icon=(uint8_t*)icon_bigdot;
      LCD_ShowImage(DIGIT_DOT_XPOS, DIGIT_DOT_YPOS, DIG_DOT_WIDTH, DIG_DOT_HEIGHT, cur_icon);
}
















