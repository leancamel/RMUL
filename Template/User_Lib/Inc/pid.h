/**
  *******************************RM Warrior 2023********************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-1-1        pxx             1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *******************************RM Warrior 2023********************************
  */
#ifndef PID_H
#define PID_H

#include "main.h"

enum PID_MODE
{
    PID_POSITION = 0,//位置式PID
    PID_DELTA        //增量式PID
};

typedef struct
{
    uint8_t mode;
    //PID 参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;      //设定值
    float fdb;      //实际值

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} PidTypeDef;
extern void PID_Init(PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);
extern float PID_Calc(PidTypeDef *pid, float ref, float set);
extern void PID_clear(PidTypeDef *pid);
#endif
