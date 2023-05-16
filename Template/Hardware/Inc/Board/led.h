#ifndef __LED_H
#define __LED_H

#include "main.h"
#include "tim.h"

void Led_Blue_On(void);
void Led_Blue_Off(void);
void Led_Blue_Toggle(void);
void Led_Green_On(void);
void Led_Green_Off(void);
void Led_Green_Toggle(void);
void Led_Red_On(void);
void Led_Red_Off(void);
void Led_Red_Toggle(void);
void Led_RGB(uint8_t R, uint8_t G, uint8_t B);

#endif
