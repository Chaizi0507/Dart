/**
 * @file crt_gimbal.cpp
 * @author lez by wanghongxi
 * @brief 云台
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_gimbal.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/


/**
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    //imu初始化
    Boardc_BMI.Init(); 

    Motor_Pitch_L.PID_Angle.Init(25.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Pitch_L.PID_Omega.Init(3000.0f, 10.0f, 0.001f, 0.0f, Motor_Pitch_L.Get_Output_Max(), Motor_Pitch_L.Get_Output_Max());
    Motor_Pitch_L.Init(&hcan2, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA);

    Motor_Pitch_R.PID_Angle.Init(25.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Pitch_R.PID_Omega.Init(3000.0f, 10.0f, 0.001f, 0.0f, Motor_Pitch_R.Get_Output_Max(), Motor_Pitch_R.Get_Output_Max());
    Motor_Pitch_R.Init(&hcan2, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA);

    Motor_Yaw.PID_Angle.Init(25.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Yaw.PID_Omega.Init(3000.0f, 10.0f, 0.001f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.Init(&hcan2, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_OMEGA);

}

/**
 * @brief 输出到电机
 *
 */
void Class_Gimbal::Output()
{
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        //云台失能
        Motor_Pitch_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Pitch_L.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch_L.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch_L.Set_Out(0.0f);

        Motor_Pitch_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Pitch_R.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch_R.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch_R.Set_Out(0.0f);

        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw.Set_Out(0.0f);

    }
    else // 非失能模式
    {   
        
    }
}


/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    Output();
    
    //根据不同c板的放置方式来修改这几个函数
    Motor_Pitch_L.Set_Transform_Angle(Boardc_BMI.Get_Angle_Pitch());
    Motor_Pitch_R.Set_Transform_Angle(Boardc_BMI.Get_Angle_Pitch());

    Motor_Pitch_L.Set_Transform_Omega(Boardc_BMI.Get_Gyro_Pitch());
    Motor_Pitch_R.Set_Transform_Omega(Boardc_BMI.Get_Gyro_Pitch());

    //PID
    Motor_Yaw.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch_L.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch_R.TIM_PID_PeriodElapsedCallback();

}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
