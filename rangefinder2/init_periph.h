#ifndef __INIT_PERIPH
#define __INIT_PERIPH

#include <stdint.h>

void init_all_periph(void);
void init_clk(void);
void init_gpio(void);
void init_tim2(void);
void start_laser(void);
void init_tim8(void);
void init_adc(void);
void init_dma(void);
void init_tim3(void);

void delay_ms(uint32_t ms);

#endif
