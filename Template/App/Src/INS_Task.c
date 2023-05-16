#include "INS_Task.h"
#include "ist8310driver.h"
#include "BMI088driver.h"
#include "stdio.h"

#if (USE_IST8310 == 1)
	ist8310_real_data_t ist8310_data;
#endif
bmi088_real_data_t bmi088_data;

/**
 * @brief 姿态解算任务初始化
 */
void INS_Init(void)
{
	#if (USE_IST8310 == 1)
		while (ist8310_init());
	#endif
	while (BMI088_init());
}

/**
 * @brief 姿态解算任务主循环
 */
void INS_Task(void)
{
	// BMI088_read(bmi088_data.gyro, bmi088_data.accel, &bmi088_data.temp);
	// printf("%.3f, %.3f, %.3f\n",bmi088_data.accel[0], bmi088_data.accel[1], bmi088_data.accel[2]);
	// printf("%.3f, %.3f, %.3f\n",bmi088_data.gyro[0], bmi088_data.gyro[1], bmi088_data.gyro[2]);
	printf("%.2f, %.2f, %.2f\n",ist8310_data.mag[0], ist8310_data.mag[1], ist8310_data.mag[2]);
}
