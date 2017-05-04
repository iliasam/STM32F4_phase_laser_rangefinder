
#include "main.h"
#include "init_periph.h"
#include "lcd_driver.h"
#include "stm32f4xx_tim.h"
#include <math.h>

#define M_PI  (double)3.141592654

__IO uint8_t UserButtonPressed = 0;
__IO uint32_t TimingDelay;

volatile uint16_t adc_buf[POINTS_NUMBER];
volatile int16_t result_buf[DATA_POINTS_NUMBER];

volatile uint8_t i;
volatile uint16_t brightness;
volatile long double angle;
volatile long double angle2;//average
volatile long double angle3;//average + correction
volatile long double zero_angle;//for phase coffection on big distance
volatile long double tmp_angle;
volatile long double distance;
volatile long double dist_offset = 0.0;//zero calibration

uint8_t ready_to_capture = 0;
//1 - flag to copy data from adc_buf[] to result_buf[]
//0 - wait for flag set

uint16_t pwm_value = 400;
volatile uint16_t brightness_level = 400;


int main(void)
{
  uint8_t i;
  
  RCC_ClocksTypeDef RCC_Clocks;
  
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO); 
  
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);

  RCC_GetClocksFreq(&RCC_Clocks);
  
  init_all_periph();
  
  STM_EVAL_LEDOff(LED6);
  delay_ms(100);
  
  while(1)
  {
    tmp_angle = 0;
    for (i=0;i<32;i++)//calculate average
    {
      while(ready_to_capture == 0) {asm("nop");}
      copy_adc_data();
      process_data();
      tmp_angle+=angle;
      update_lcd();
      if ((i % 4) == 0) prosess_brightness();
      delay_ms(10);
      ready_to_capture = 0;//start waiting for data from DMA
    }
    
    draw_number(brightness, 0);//to lcd
    angle2 = tmp_angle/32;
    
    if (STM_EVAL_PBGetState(BUTTON_USER)== Bit_SET) 
    {
      zero_angle = angle2;
      dist_offset = angle2*4.1637;// c / 10MHz / 360 / 2 = 0.0416 meter
    }
    if (angle2< zero_angle) {angle3 = angle2 + 360.0;} else {angle3 = angle2;}//for long distancies
    distance = angle3*4.1637 - dist_offset;
    draw_number((int16_t)distance, 4);//to lcd
  }
}

//AGC emulation (calculate servo position)
void prosess_brightness(void)
{
  int16_t delta = (int16_t)brightness - (int16_t)brightness_level;//higher brightness >>>> delta > 0
  int16_t new_pwm_value;
  
  delta = delta/20;
  
  if (ABS(delta) > 2)
  {
    new_pwm_value = (int16_t)pwm_value - delta;
    if (new_pwm_value < 300) {new_pwm_value = 300;}
    if (new_pwm_value > 400) {new_pwm_value = 400;}
    pwm_value = (uint16_t)new_pwm_value;
    TIM3->CCR3 = pwm_value;//servo timer
    STM_EVAL_LEDOn(LED6);
  }
  else {STM_EVAL_LEDOff(LED6);}
  

}

//calculate phase shift
void process_data(void)
{
  long double tmp_angle;
  
  int32_t s1 = 0;
  int32_t s2 = 0;
  int32_t s3 = 0;
  int32_t s4 = 0;
  
  uint16_t i;
  
  for (i=0;i<(DATA_POINTS_NUMBER);i=i+4)//number of steps = DATA_POINTS_NUMBER/4
  {
    s1+= (int32_t)result_buf[i];
    s2+= (int32_t)result_buf[i+1];
    s3+= (int32_t)result_buf[i+2];
    s4+= (int32_t)result_buf[i+3];
  }
  
  tmp_angle = atan2l((long double)(s4-s2),(long double)(s3-s1));
  tmp_angle= tmp_angle*180/M_PI;
  if (tmp_angle < 0.0) {tmp_angle = 360.0 + tmp_angle;}
  angle = tmp_angle;
  
}

//dma can't be stopped
//copy data from adc_buf[] to result_buf[]
void copy_adc_data(void)
{
  uint16_t i;
  uint32_t tmp = 0;
  for (i=0;i<DATA_POINTS_NUMBER;i++) 
  {
    result_buf[i] = (int16_t)(adc_buf[i]) - 2048;
    tmp+= (uint32_t)(ABS(result_buf[i]));
  }
  tmp = tmp/DATA_POINTS_NUMBER;
  brightness = (uint16_t)tmp;
}


