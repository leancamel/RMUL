#include "led.h"

/**
 * @brief 蓝灯亮
 */
void Led_Blue_On(void)
{
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 255000);
}

/**
 * @brief 蓝灯灭
 */
void Led_Blue_Off(void)
{
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 0);
}

/**
 * @brief 蓝灯闪烁
 */
void Led_Blue_Toggle(void)
{
	if(__HAL_TIM_GetCompare(&htim5, TIM_CHANNEL_1) == 0)
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 255000);
	else
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 0);
}

/**
 * @brief 绿灯亮
 */
void Led_Green_On(void)
{
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 255000);
}

/**
 * @brief 绿灯灭
 */
void Led_Green_Off(void)
{
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 0);
}

/**
 * @brief 绿灯闪烁
 */
void Led_Green_Toggle(void)
{
	if(__HAL_TIM_GetCompare(&htim5, TIM_CHANNEL_2) == 0)
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 255000);
	else
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 0);
}

/**
 * @brief 红灯亮
 */
void Led_Red_On(void)
{
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 255000);
}

/**
 * @brief 红灯灭
 */
void Led_Red_Off(void)
{
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 0);
}

/**
 * @brief 红灯闪烁
 */
void Led_Red_Toggle(void)
{
	if(__HAL_TIM_GetCompare(&htim5, TIM_CHANNEL_3) == 0)
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 255000);
	else
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 0);
}

/**
 * @brief LED显示RGB三原色
 * @param R 0~255
 * @param G 0~255
 * @param B 0~255
 */
void Led_RGB(uint8_t R, uint8_t G, uint8_t B)
{
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 1000 * R);
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 1000 * G);
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 1000 * B);
}
