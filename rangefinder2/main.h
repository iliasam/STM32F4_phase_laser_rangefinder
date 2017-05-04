
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4_DISCOVERY_DEMO_H
#define __STM32F4_DISCOVERY_DEMO_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include <stdio.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define POINTS_NUMBER (uint16_t)16384
//#define POINTS_NUMBER (uint16_t)8192
#define DATA_POINTS_NUMBER (uint16_t)(POINTS_NUMBER>>1)



/* Exported macro ------------------------------------------------------------*/
#define ABS(x)         (x < 0) ? (-x) : x
#define MAX(a,b)       (a < b) ? (b) : a
/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);
void Fail_Handler(void);
void copy_adc_data(void);
void process_data(void);
void prosess_brightness(void);

#endif /* __STM32F4_DISCOVERY_DEMO_H */


