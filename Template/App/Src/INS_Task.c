#include "INS_Task.h"
#include "ist8310driver.h"
#include "BMI088driver.h"
#include "serial.h"
#include "pid.h"
#include "tim.h"

#define TEMPERATURE_PID_KP 1600.0f //kp of temperature control PID 
#define TEMPERATURE_PID_KI 0.2f    //ki of temperature control PID 
#define TEMPERATURE_PID_KD 0.0f    //kd of temperature control PID 

#define TEMPERATURE_PID_MAX_OUT 4500.0f  //max out of temperature control PID 
#define TEMPERATURE_PID_MAX_IOUT 4400.0f //max iout of temperature control PID 

const float imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
PidTypeDef imu_temp_pid;

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

	PID_Init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
}

/**
 * @brief 输出 pwm 控制 IMU 温度
 */
void IMU_Temp_Control(void)
{
	uint16_t tempPWM;
	float temp = BMI088_Read_Temp();
	PID_Calc(&imu_temp_pid, temp, 40.0f);
	if (imu_temp_pid.out < 0.0f)
        imu_temp_pid.out = 0.0f;
    tempPWM = (uint16_t)imu_temp_pid.out;
	__HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, tempPWM);
}

/**
 * @brief 姿态解算任务主循环
 */
void INS_Task(void)
{
	IMU_Temp_Control();
	// printf("%.3f, %.3f, %.3f\n",bmi088_data.accel[0], bmi088_data.accel[1], bmi088_data.accel[2]);
	// printf("%.3f, %.3f, %.3f\n",bmi088_data.gyro[0], bmi088_data.gyro[1], bmi088_data.gyro[2]);
	// printf("%.2f, %.2f, %.2f\n",ist8310_data.mag[0], ist8310_data.mag[1], ist8310_data.mag[2]);
}
