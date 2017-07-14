#include <stdlib.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "comm.h"

uint8_t temp_id_str[2];

#define UNIQUE_ID_ADDR_BASE 0x1FFF7A10
//unsigned char UNIQUE_ID_ADDR_BASE[] = "002E00510747353238353434";
#define STM32_UUID ((uint32_t *)0x1FFF7A10)
#define UNIQUE_ID_LEN 12

#define B2L(x)  ((((x) & 0xFF000000) >> 24) | (((x) & 0x00FF0000) >> 8) | (((x) & 0x0000FF00) << 8) | (((x) & 0x000000FF) << 24))

unsigned char g_ucaSeriNo[UNIQUE_ID_LEN];
uint8_t comm_send_len = 0;


extern uint8_t recv_comm_buf[];
extern uint8_t send_comm_buf[];
extern uint8_t recv_comm_idx;
extern uint8_t start_rcv_timer;
extern uint8_t rcv_tim_delay;
extern uint8_t comm_rcv_flag;

extern UART_HandleTypeDef huart3;


extern float g_humidity, g_temperature;
extern uint16_t g_co2, g_voc, g_pm25;


uint8_t To_ASCII_ver2(int16_t data, uint8_t* string)
{
  uint8_t i;
  uint8_t len = 4;
  uint8_t str[4];
  if(data < 0)
  {
    data = -data;
  }
  for(i=0; i<4; i++)
  {
    str[3-i] = '0' + data%10;
    data /= 10;
  }
  /* Remove the initial zero's but the last digit */
  for(i=0; (str[i]=='0')&&(i<3); i++)
  {
    len--;
  }
  for(i=0; i<len; i++)
  {
    string[i] = str[i+4-len];
  }
  return len;
}

uint8_t To_ASCII_two_deci(float data, uint8_t* string)
{
  uint8_t len_t, t2;
  uint8_t len;

  uint16_t t1 = (uint16_t)(data*100);
  uint8_t itgr = (uint8_t)(t1/100);
  uint8_t deci = (uint8_t)(t1%100);

  len_t = To_ASCII_ver2((int16_t)itgr, string);

  string[len_t] = '.';
  string[len_t+1] = '0' + deci/10;
  len = len_t + 2;
  t2 = deci%10;
  if(t2 > 0)
  {
    string[len_t+2] = '0' + t2;
    len++;
  }

  return len;
}

uint16_t pow_ver2(uint16_t base, uint8_t exp)
{
  uint8_t i;
  uint16_t result = 1;

  for(i=0; i<exp; i++)
    result *= base;

  return result;
}

uint16_t To_Digits_IAQ(uint8_t* string, uint8_t len)
{
  uint8_t* str = string;
  uint8_t i = len;
  uint32_t data = 0;
  while(i)
  {
    if((*str >= '0')&&(*str <= '9'))
    {
      data += (uint32_t)(*str - '0')*(uint32_t)pow_ver2(10, (i-1));
    }
    else //if(i != 1)
      return (data/(uint32_t)pow_ver2(10, i));
    /*
              else
              return data;
              */
    str++; i--;
  }
  return data;
}

uint16_t To_Digits(uint8_t* string, uint8_t len)
{
  uint8_t* str = string;
  uint8_t i = len;
  uint16_t data = 0;
  while(i)
  {
    if((*str >= '0')&&(*str <= '9'))
    {
      data += (uint16_t)(*str - '0')*(uint16_t)pow_ver2(10, (i-1));
    }
    else
      return data;
    str++; i--;
  }
  return data;
}

float To_Digits_float(uint8_t* string, uint8_t len)
{
  uint8_t itgr_len, deci_len;
  uint8_t* str = string;
  uint8_t i = 0;
  float data = 0;
  uint16_t itgr = 0;      uint16_t deci = 0;

  while(i<len)
  {
    if(str[i] == '.')
    {
      itgr_len = i;
      deci_len = len - i - 1;
      break;
    }
    i++;
  }

  i = itgr_len;
  while(i)
  {
    if((*str >= '0')&&(*str <= '9'))
    {
      itgr += (uint16_t)(*str - '0')*(uint16_t)pow_ver2(10, (i-1));
    }
    else
      return data;
    str++; i--;
  }
  /* Jump over the deci point */
  str++;
  i = deci_len;
  while(i)
  {
    if((*str >= '0')&&(*str <= '9'))
    {
      deci += (uint16_t)(*str - '0')*(uint16_t)pow_ver2(10, (i-1));
    }
    else
      return data;
    str++; i--;
  }
  data = itgr + deci/(float)pow_ver2(10, deci_len);

  return data;
}

void HEX_2_ASCII(uint8_t hex_value)
{
  uint8_t* str;
  uint8_t high4, low4;

  str = &temp_id_str[0];

  high4 = (uint8_t)(hex_value >> 4);
  low4 =  (uint8_t)(hex_value&0x0F);

  if(high4 > 9)
  {
    str[0] = high4 - 0x0A + 'A';
  }
  else
  {
    str[0] = '0' + high4;
  }

  if(low4 > 9)
  {
    str[1] = low4 - 0x0A + 'A';
  }
  else
  {
    str[1] = '0' + low4;
  }

}

uint8_t New_Line(uint8_t* buf)
{
  *buf = '\x0a';
  return 1;
}

bool Find_Pattern(uint8_t* string, uint8_t* pattern, uint8_t length)
{
  uint8_t* str = string;
  uint8_t* a = pattern;

  while(length--)
  {
    if((*str != *a)&&(*str != *(a+1)))
      return FALSE;
    else
    {
      str++;
      a += 2;
    }
  }
  return TRUE;
}


uint8_t Get_ID(uint8_t* buf)
{
  uint8_t i, j;

  buf[0] = '"';
  buf[1] = 'D';
  buf[2] = '"';
  buf[3] = ':';
  buf[4] = '"';

  for(i = 0, j = 0; i<UNIQUE_ID_LEN; i++)
  {
    HEX_2_ASCII(g_ucaSeriNo[i]);

    buf[5+j++] = temp_id_str[0];
    buf[5+j++] = temp_id_str[1];
  }

  buf[5+2*UNIQUE_ID_LEN] = '"';

  return (5+2*UNIQUE_ID_LEN+1);
}

uint8_t Get_RH(uint8_t* buf)
{
  uint16_t temp = (uint16_t)(g_humidity * 10);

  buf[0] = '"';
  buf[1] = 'H';
  buf[2] = '"';
  buf[3] = ':';
  buf[7] = '0' + (uint8_t)(temp%10);       /* deci */
  buf[6] = '.';
  temp /= 10;
  buf[5] = '0' + (uint8_t)(temp%10);       /* ones */
  temp /= 10;
  buf[4] = '0' + (uint8_t)(temp%10);       /* tens */
  buf[8] = ',';
  return 9;
}

uint8_t Get_TP(uint8_t* buf)
{
  uint8_t i;
  uint16_t temp;

  if(g_temperature < 0)
  {
    buf[4] = '-';
    i = 5;
    temp = (uint16_t)(-g_temperature*10);
  }
  else
  {
    i = 4;
    temp = (uint16_t)(g_temperature*10);
  }
  buf[0] = '"';
  buf[1] = 'T';
  buf[2] = '"';
  buf[3] = ':';

  buf[i+3] = '0' + (uint8_t)(temp%10);       /* deci */
  buf[i+2] = '.';
  temp /= 10;
  buf[i+1] = '0' + (uint8_t)(temp%10);       /* ones */
  temp /= 10;
  buf[i] = '0' + (uint8_t)(temp%10);       /* tens */
  buf[i+4] = ',';
  return (i+5);
}

uint8_t Get_TVOC(uint8_t* buf)
{
  uint8_t i;
  buf[0] = '"';
  buf[1] = 'V';
  buf[2] = '"';
  buf[3] = ':';
  if( -1 == g_voc )
  {
    buf[4] = '-';
    buf[5] = '1';
    buf[6] = ',';
    return 7;
  }
  i = To_ASCII_ver2((int16_t)g_voc, &buf[4]);
  buf[i+4] = ',';
  return (i+5);
}


uint8_t Get_CO2T(uint8_t* buf)
{
  uint8_t i;

  buf[0] = '"';
  buf[1] = 'C';
  buf[2] = '"';
  buf[3] = ':';
  i = To_ASCII_ver2((int16_t)g_co2, &buf[4]);
  buf[i+4] = ',';
  return (i+5);
}


uint8_t Get_PM25_P(uint8_t* buf)
{
  uint8_t i;

  buf[0] = '"';
  buf[1] = 'P';
  buf[2] = '"';
  buf[3] = ':';
  if( -1 == g_co2 )
  {
    buf[4] = '-';
    buf[5] = '1';
    buf[6] = ' ';
    return 7;
  }
  i = To_ASCII_ver2((int16_t)g_pm25, &buf[4]);
  buf[i+4] = ',';
  return (i+5);
}

/* Functions -----------------------------------------------------------------*/
void Clear_Rcv_Buf(void)
{
    uint8_t i;
    for(i=0;i<COMM_RECV_BUF_MAX;i++)
    {
        recv_comm_buf[i] = 0;
    }
    recv_comm_idx = 0;
}

void Comm_Init(void)
{
  unsigned char i;
//  for(i = 0; i<UNIQUE_ID_LEN; i++)
//  {
//    g_ucaSeriNo[i] = *((unsigned char*)(UNIQUE_ID_ADDR_BASE + i));
//  }
  uint32_t *p = (uint32_t *)g_ucaSeriNo;
  for (i = 0; i < 3; i++) {
    *p++ = B2L(STM32_UUID[i]);
  }
}

/* Process the incoming command, one piece at a time */
void Comm_Process(void)
{
  uint8_t* recv_ptr = recv_comm_buf;
  uint8_t* send_ptr = send_comm_buf;
  uint8_t recv_len = recv_comm_idx;
  uint8_t len = 0;
  uint8_t send_len = 0;

  static unsigned long s_ulSendPtrBackup;

  s_ulSendPtrBackup = (unsigned long)send_ptr;

  printf("%s\r\n", recv_comm_buf);

  while(recv_len)//???
  {
    if(Find_Pattern(recv_ptr, "GgTt", 2))
    {
      printf("Find GT\r\n");
      recv_ptr += 2;
      recv_len -= 2;
      if(Find_Pattern(recv_ptr, "AaLl", 2))
      {
        printf("Find AL\r\n");
        *send_ptr = '{';
        send_ptr++;
        send_len++;
        len = Get_TP(send_ptr);
        if((unsigned long)send_ptr < s_ulSendPtrBackup + COMM_SEND_BUF_MAX)
        {
          send_ptr += len;
          send_len += len;
        }
        recv_ptr += 2;
        recv_len -= 2;

        len = Get_RH(send_ptr);
        if((unsigned long)send_ptr < s_ulSendPtrBackup + COMM_SEND_BUF_MAX)
        {
          send_ptr += len;
          send_len += len;
        }

        len = Get_TVOC(send_ptr);
        if((unsigned long)send_ptr < s_ulSendPtrBackup + COMM_SEND_BUF_MAX)
        {
          send_ptr += len;
          send_len += len;
        }

        len = Get_CO2T(send_ptr);
        if((unsigned long)send_ptr < s_ulSendPtrBackup + COMM_SEND_BUF_MAX)
        {
          send_ptr += len;
          send_len += len;
        }

        len = Get_PM25_P(send_ptr);
        if((unsigned long)send_ptr < s_ulSendPtrBackup + COMM_SEND_BUF_MAX)
        {
          send_ptr += len;
          send_len += len;
        }

        len = Get_ID(send_ptr);
        if((unsigned long)send_ptr < s_ulSendPtrBackup + COMM_SEND_BUF_MAX)
        {
          send_ptr += len;
          send_len += len;
          *send_ptr = '}';
          send_ptr++;
          send_len++;
        }

//        printf("%s\r\n", send_comm_buf);

        //USART_ITConfig(COM_PORT, USART_IT_RXNE, ENABLE);
      }
    }
    else
    {
      recv_ptr++;
      recv_len--;
    }
  }
  /* Start a new line */
  len = New_Line(send_ptr);
  if((unsigned long)send_ptr < s_ulSendPtrBackup + COMM_SEND_BUF_MAX)
  {
    send_ptr += len;
    send_len += len;
  }

  if(send_len>COMM_SEND_BUF_MAX)
  {
    send_len = 0;
  }
  if(0 == send_len)
  {
//    USART_ITConfig(COM_PORT, USART_IT_RXNE, ENABLE);
//    g_ucComRxEnableFlag = 1;
  }
  comm_send_len = send_len;

  Clear_Rcv_Buf();
}

void Comm_Response(void)
{
  printf("Resp\r\n");
//  HAL_UART_Transmit(&huart3, "ACK\r\n", 5, 1000);
  uint8_t res = HAL_UART_Transmit(&huart3, send_comm_buf, comm_send_len, 1000);
  printf("%d - %d: %s\r\n", res, comm_send_len, send_comm_buf);
  comm_send_len = 0;
}

