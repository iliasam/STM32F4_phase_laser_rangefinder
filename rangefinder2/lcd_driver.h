#ifndef __LCD_DRIVER
#define __LCD_DRIVER

#include <stdint.h>

#define LCD_CLK_PORT GPIOC
#define LCD_DATA_PORT GPIOE
#define LCD_RES_PORT GPIOE

#define LCD_CLK_PIN GPIO_Pin_13
#define LCD_DATA_PIN GPIO_Pin_3
#define LCD_RES_PIN GPIO_Pin_5


#define ANODE_A (uint8_t)(1<<3)
#define ANODE_B (uint8_t)(1<<4)
#define ANODE_C (uint8_t)(1<<5)
#define ANODE_D (uint8_t)(1<<6)
#define ANODE_E (uint8_t)(1<<7)
#define ANODE_F (uint8_t)(1<<2)
#define ANODE_G (uint8_t)(1<<0)
#define ANODE_DP (uint8_t)(1<<1)

#define SYMB_0 ANODE_A|ANODE_B|ANODE_C|ANODE_D|ANODE_E|ANODE_F
#define SYMB_1 ANODE_B|ANODE_C
#define SYMB_2 ANODE_A|ANODE_B|ANODE_G|ANODE_E|ANODE_D
#define SYMB_3 ANODE_A|ANODE_B|ANODE_C|ANODE_D|ANODE_G
#define SYMB_4 ANODE_F|ANODE_G|ANODE_B|ANODE_C
#define SYMB_5 ANODE_A|ANODE_F|ANODE_G|ANODE_C|ANODE_D
#define SYMB_6 ANODE_A|ANODE_C|ANODE_D|ANODE_E|ANODE_F|ANODE_G
#define SYMB_7 ANODE_A|ANODE_B|ANODE_C
#define SYMB_8 ANODE_A|ANODE_B|ANODE_C|ANODE_D|ANODE_E|ANODE_F|ANODE_G
#define SYMB_9 ANODE_A|ANODE_B|ANODE_C|ANODE_D|ANODE_F|ANODE_G
#define SYMB_EMPTY 0
#define SYMB_MINUS ANODE_G

void init_lcd(void);
void update_lcd(void);
void snumber_to_buffer(int16_t number, uint8_t* buf);
void draw_number(int16_t number, uint8_t pos);

#endif
