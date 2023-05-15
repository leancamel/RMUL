#include "buzzer.h"
/**
 * @brief 关闭蜂鸣器
 */
void Buzzer_Off(void)
{
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
/**
 * @brief 设置蜂鸣器的定时器时基单元
 * @param PSC 预分频系数
 * @param ARR 自动重装值
 */
void Buzzer_Set_Timer(uint16_t PSC, uint16_t ARR)
{
	__HAL_TIM_PRESCALER(&htim4, PSC);
	__HAL_TIM_SetAutoreload(&htim4, ARR);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, (ARR + 1) / 2);
}
/**
 * @brief 以一定的频率开启蜂鸣器
 * @param frequency 蜂鸣器频率，0~20 ,单位 kHz
 */
void Buzzer_On(float frequency)
{
	if(frequency > 20.0f)
	{
		frequency = 20.0f;
	}
	if(frequency < 0.0f)
	{
		Buzzer_Off();
		return;
	}
	uint32_t PSC = 0,ARR = 65536;
	while(ARR >= 65536)
	{
		PSC++;
		ARR = 65536;
		ARR = 84000.0f / PSC / frequency;
		if(PSC >= 65536)
		{
			Buzzer_Off();
			return;
		}
	}
	Buzzer_Set_Timer(PSC - 1, ARR - 1);
}
