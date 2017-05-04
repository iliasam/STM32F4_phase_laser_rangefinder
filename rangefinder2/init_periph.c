#include "init_periph.h"
#include "stm32f4xx_conf.h"
#include "main.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dac.h"
#include "lcd_driver.h"

#define ADC1_DR_ADDRESS    ((uint32_t)0x4001204C)
#define PERIOD (uint16_t)16//laser period //160Mhz / 16 = 10Mhz

extern volatile uint16_t adc_buf[POINTS_NUMBER];

void init_all_periph(void)
{
  init_clk();
  init_gpio();
  init_tim2();
  init_adc();
  init_dma();
  init_tim8();
  start_laser();
  
  init_lcd();
  init_tim3();
}


void init_clk(void)
{

  //confugure system clock
  //автоматически настоена на HSI - 168MHZ

  ErrorStatus HSEStartUpStatus;
  RCC_DeInit();
  RCC_HSEConfig(RCC_HSE_ON);
  // Wait till HSE is ready
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  if(HSEStartUpStatus == SUCCESS)
  {
    FLASH->ACR = FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;

    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    RCC_PCLK2Config(RCC_HCLK_Div1);
    RCC_PCLK1Config(RCC_HCLK_Div1);
    
    RCC_PLLConfig(RCC_PLLSource_HSE,8,320,2,7);// (hse = 8mhz) / 8 * 320 / 2 = 160mhz
    RCC_PLLCmd(ENABLE);
    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){;}
    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08){;}
  }
}


//laser timer
void init_tim2(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
 
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_Period = PERIOD-1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
 
  
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = PERIOD/2;//PWM DUTY
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
 
  TIM_OC2Init(TIM2,&TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
  
  TIM_ARRPreloadConfig(TIM2,ENABLE); 
  
}

void start_laser(void)
{
  TIM_Cmd(TIM2,ENABLE);   // start laser timer
}

//PWM timer (servo)
void init_tim3(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
 
  TIM_TimeBaseStructure.TIM_Prescaler = 800-1;//160MHz / 800 = 200kHz
  TIM_TimeBaseStructure.TIM_Period = 4000-1;// (1/200kHz)*4000 = 20ms
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
 
  
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 200; //PWM DUTY (1ms)
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
 
  TIM_OC3Init(TIM3,&TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
  
  TIM_ARRPreloadConfig(TIM3,ENABLE); 
  
  TIM_Cmd(TIM3,ENABLE);   // start laser timer
  
}

//ADC timer
void init_tim8(void)
{
   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
   
   TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
   TIM_TimeBaseStructure.TIM_Period = 100-1;//160Mhz / 100 = 1.6Mhz
   TIM_TimeBaseStructure.TIM_Prescaler = 0;
   TIM_TimeBaseStructure.TIM_ClockDivision = 0;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   
   TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
   TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Update);

   TIM_Cmd(TIM8, ENABLE);
}

void init_gpio(void)
{ 
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  //portb.3 - pwm (laser)
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB,&GPIO_InitStructure);
  
  //portb.0 - pwm (servo)
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
  GPIO_Init(GPIOB,&GPIO_InitStructure);
  
  /* Configure ADC1 Channel 1 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
   
  
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_TIM3);
}

//init ADC
void init_adc(void)
{
  ADC_InitTypeDef          ADC_InitStructure;
  ADC_CommonInitTypeDef   ADC_CommonInitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);
   
  /* ADC1 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_TRGO;//timer8
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 0;
  ADC_Init(ADC1, &ADC_InitStructure);
   
  /* ADC1 regular channel 1 configuration *************************************/
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_3Cycles);
  
   
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);/* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMACmd(ADC1, ENABLE); /* Enable ADC3 DMA */
   
  ADC_Cmd(ADC1, ENABLE);   /* Enable ADC3 */
}

void init_dma(void)
{
  DMA_InitTypeDef        DMA_InitStructure;
  NVIC_InitTypeDef      NVIC_InitStructure;
   
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
   
  /* DMA2 Stream0 DMA_Channel_0 configuration **************************************/
  DMA_DeInit(DMA2_Stream0);
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&adc_buf;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = POINTS_NUMBER;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);
   
   /* Enable the DMA Stream IRQ Channel */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
   
  //DMA_ITConfig(DMA2_Stream0, DMA_IT_HT | DMA_IT_TC, ENABLE);
  DMA_ITConfig(DMA2_Stream0, DMA_IT_HT, ENABLE);//half transfer interrupts
}




void delay_ms(uint32_t ms)
{
  volatile uint32_t nCount;
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq (&RCC_Clocks);
  nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
  for (; nCount!=0; nCount--);
}

