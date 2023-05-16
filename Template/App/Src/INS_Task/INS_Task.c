#include "INS_Task.h"
#include "ist8310driver.h"
#include "BMI088driver.h"
#include "serial.h"
#include "pid.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "MahonyAHRS.h"
#include "math.h"

#define TEMPERATURE_PID_KP 1600.0f // kp of temperature control PID
#define TEMPERATURE_PID_KI 0.2f	   // ki of temperature control PID
#define TEMPERATURE_PID_KD 0.0f	   // kd of temperature control PID

#define TEMPERATURE_PID_MAX_OUT 4500.0f	 // max out of temperature control PID
#define TEMPERATURE_PID_MAX_IOUT 4400.0f // max iout of temperature control PID

const float imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
PidTypeDef imu_temp_pid;

#if (USE_IST8310 == 1)
	ist8310_real_data_t ist8310_data;
#endif
bmi088_real_data_t bmi088_data;

extern TaskHandle_t INS_task_local_handler;
extern uint8_t INT_Task_Start;

float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3] = {0.0f, 0.0f, 0.0f}; // euler angle, unit rad.欧拉角 单位 rad

/**
 * @brief 初始化姿态解算所需数据
 * @param quat 	四元数
 * @param accel 加速度
 * @param mag 	磁力计
 */
void AHRS_init(float quat[4], float accel[3], float mag[3])
{
	quat[0] = 1.0f;
	quat[1] = 0.0f;
	quat[2] = 0.0f;
	quat[3] = 0.0f;
}

/**
 * @brief 姿态更新
 * @param quat 	四元数
 * @param gyro	角速度
 * @param accel 加速度
 * @param mag	磁力计
 */
void AHRS_update(float quat[4], float gyro[3], float accel[3], float mag[3])
{
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
}

/**
 * @brief 四元数转换为欧拉角
 * @param quat	四元数
 * @param yaw	yaw轴偏角，单位 rad/s
 * @param pitch	pitch轴偏角，单位 rad/s
 * @param roll	roll轴偏角，单位 rad/s
 */
void get_angle(float quat[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(quat[0]*quat[3]+quat[1]*quat[2]), 2.0f*(quat[0]*quat[0]+quat[1]*quat[1])-1.0f);
    *pitch = asinf(-2.0f*(quat[1]*quat[3]-quat[0]*quat[2]));
    *roll = atan2f(2.0f*(quat[0]*quat[1]+quat[2]*quat[3]),2.0f*(quat[0]*quat[0]+quat[3]*quat[3])-1.0f);
}

/**
 * @brief 姿态解算任务初始化
 */
void INS_Init(void)
{
	#if (USE_IST8310 == 1)
		while (ist8310_init());
	#endif
	while (BMI088_init());
	bmi088_data.status = 0x00;

	PID_Init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
	BMI088_read(bmi088_data.gyro, bmi088_data.accel, &bmi088_data.temp);
	AHRS_init(INS_quat, bmi088_data.accel, ist8310_data.mag);
}

/**
 * @brief 输出 pwm 控制 IMU 温度
 */
void IMU_Temp_Control(void)
{
	uint16_t tempPWM;
	PID_Calc(&imu_temp_pid, bmi088_data.temp, 40.0f);
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

	AHRS_update(INS_quat, bmi088_data.gyro, bmi088_data.accel, ist8310_data.mag);
	get_angle(INS_quat, INS_angle + 0, INS_angle + 1, INS_angle + 2);

	printf("%.2f, %.2f, %.2f\n",INS_angle[0], INS_angle[1], INS_angle[2]);
	// printf("%.3f, %.3f, %.3f\n", bmi088_data.accel[0], bmi088_data.accel[1], bmi088_data.accel[2]);
	// printf("%.3f, %.3f, %.3f\n",bmi088_data.gyro[0], bmi088_data.gyro[1], bmi088_data.gyro[2]);
	// printf("%.2f, %.2f, %.2f\n",ist8310_data.mag[0], ist8310_data.mag[1], ist8310_data.mag[2]);
}

/**
 * @brief 唤醒 INS_Task
 */
void INS_Task_Resume(void)
{
	if (INT_Task_Start)
	{
		if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
		{
			static BaseType_t xHigherPriorityTaskWoken;
			vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}

/**
 * @brief 根据状态位开启DMA
 */
void IMU_Cmd_Spi_DMA(void)
{
	if (bmi088_data.status & 0x01)
	{
		BMI088_Read_Accel(bmi088_data.accel);
		bmi088_data.temp = BMI088_Read_Temp();
		bmi088_data.status &= ~0x01;
		INS_Task_Resume();
	}
	if (bmi088_data.status & 0x10)
	{
		BMI088_Read_Gyro(bmi088_data.gyro);
		bmi088_data.status &= ~0x10;
		INS_Task_Resume();
	}
}

/**
 * @brief 加速度计数据中断函数
 */
void INT1_ACCEL_Func(void)
{
	bmi088_data.status |= 0x01;
	IMU_Cmd_Spi_DMA();
}

/**
 * @brief 角速度计数据中断函数
 */
void INT1_Gyro_Func(void)
{
	bmi088_data.status |= 0x10;
	IMU_Cmd_Spi_DMA();
}
