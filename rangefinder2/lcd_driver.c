#include "init_periph.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "main.h"
#include "lcd_driver.h"

//uint8_t lcd_data[8] = {1,1,4,8,16,32,64,128};
//uint8_t lcd_numbers_data[8] = {1,2,3,4,10,6,7,8};
uint8_t lcd_numbers_data[8] = {10,10,10,10,10,10,10,10};//all symbols are empty
const uint8_t conv_table[12]={SYMB_0,SYMB_1,SYMB_2,SYMB_3,SYMB_4,SYMB_5,SYMB_6,SYMB_7,SYMB_8,SYMB_9,SYMB_EMPTY,SYMB_MINUS};


void init_lcd(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);


  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  
  GPIO_InitStructure.GPIO_Pin = LCD_CLK_PIN;
  GPIO_Init(LCD_CLK_PORT,&GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = LCD_DATA_PIN;
  GPIO_Init(LCD_DATA_PORT,&GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = LCD_RES_PIN;
  GPIO_Init(LCD_RES_PORT,&GPIO_InitStructure);
}

void update_lcd(void)
{
  uint8_t i,j,k;
  uint8_t data_to_send;
  static uint8_t modulation = 0;
  static uint8_t modulation_cnt = 0;
  
  GPIO_ResetBits(LCD_DATA_PORT, LCD_DATA_PIN);
  GPIO_ResetBits(LCD_CLK_PORT, LCD_CLK_PIN);
  
  modulation_cnt++;
  if (modulation_cnt == 10) 
  {
    modulation = 2;
    modulation_cnt = 0;
  }
  else {modulation = 0;}
  
  
  //reset pulse
  GPIO_SetBits(LCD_RES_PORT, LCD_RES_PIN);
  for (i=0;i<255;i++) {asm("nop");} //delay
  GPIO_ResetBits(LCD_RES_PORT, LCD_RES_PIN);
  
  for (k=0;k<8;k++)
  {
    data_to_send = conv_table[lcd_numbers_data[7-k]];
    if (k == 3) {data_to_send|=2;}
    
    if (modulation != 0) 
    {
      data_to_send^=255;
      if (k == 7) {
        data_to_send&=2;}
    }
    
    
    for (j=0;j<8;j++)
    {
      if (data_to_send & 1){GPIO_SetBits(LCD_DATA_PORT, LCD_DATA_PIN);} else {GPIO_ResetBits(LCD_DATA_PORT, LCD_DATA_PIN);}
      for (i=0;i<20;i++) {asm("nop");} //delay
      //clk pulse
      GPIO_SetBits(LCD_CLK_PORT, LCD_CLK_PIN);
      for (i=0;i<20;i++) {asm("nop");} //delay
      GPIO_ResetBits(LCD_CLK_PORT, LCD_CLK_PIN);
      data_to_send = data_to_send>>1;
    }
  }//end for k
}

//draw "number" as "pos"
void draw_number(int16_t number, uint8_t pos)
{
  snumber_to_buffer(number,&lcd_numbers_data[pos]);
}



//place "number" to "buf"
void snumber_to_buffer(int16_t number, uint8_t* buf)
{
  uint8_t  a=0;
  uint8_t  b=0;
  uint8_t  c=0;
  uint8_t  d=0;
  
  if (number < 0) {
    number = 0-number;
    a = 11;
  }
  else {a=  number / 1000;}

  if (number > 9999) {number = 9999;}
  b=  number % 1000 /100;
  c = number %100 / 10;
  d=  number % 10;
  
  
  if (a == 0) 
  {
    a = 10;
    if (b == 0) {b = 10;}
  }
  
  

  *buf++ = a;
  *buf++ = b;
  *buf++ = c;
  *buf++ = d;
}