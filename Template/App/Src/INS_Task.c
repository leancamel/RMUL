#include "INS_Task.h"
#include "ist8310driver.h"

#if (USE_IST8310 == 1)
	ist8310_real_data_t ist_data;
#endif

/**
 * @brief 姿态解算任务初始化
 */
void INS_Init(void)
{
	#if (USE_IST8310 == 1)
		while(ist8310_init());
	#endif
}

/**
 * @brief 姿态解算任务主循环
 */
void INS_Task(void)
{
	;
}
