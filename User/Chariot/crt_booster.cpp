/**
 * @file crt_booster.cpp
 * @author lez by wanghongxi
 * @brief 发射机构
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_booster.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

void Class_FSM_Shooting::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    //自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
    case (0):
    {
        //正常状态
    }
    break;
    case (1):
    {
        //发射状态
    }
    break;
    case (2):
    {
        //卡弹状态
    }
    break;
    case (3):
    {
        //停机状态
    }
    break;
    }
}

/**
 * @brief 发射机构初始化
 *
 */
void Class_Booster::Init()
{

    FSM_Shooting.Booster = this;
    FSM_Shooting.Init(9, 0);

    //拉力电机
    Motor_Pull.PID_Angle.Init(25.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Pull.PID_Omega.Init(3000.0f, 10.0f, 0.001f, 0.0f, Motor_Pull.Get_Output_Max(), Motor_Pull.Get_Output_Max());
    Motor_Pull.Init(&hcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA);

}

/**
 * @brief 输出到电机
 *
 */
float te_omega = 0.f;
float te_angle = 0.f;
void Class_Booster::Output()
{
    //控制拨弹轮
    switch (Booster_Control_Type)
    {
        case (Booster_Control_Type_DISABLE):
        {
            // Motor_Shoot.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);

            // Motor_Shoot.PID_Angle.Set_Integral_Error(0.0f);
            // Motor_Shoot.PID_Omega.Set_Integral_Error(0.0f);

            // Motor_Shoot.Set_Out(0.0f);

            // Motor_Shoot.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            // Motor_Shoot.Set_Target_Omega_Radian(te_omega);

            // Motor_Shoot.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            // Motor_Shoot.Set_Target_Radian(te_angle);
        }
        break;
        case (Booster_Control_Type_CEASEFIRE):
        {
        }
        break;
    }

}

/**
 * @brief 定时器计算函数
 *
 */
void Class_Booster::TIM_Calculate_PeriodElapsedCallback()
{     

    FSM_Shooting.Reload_TIM_Status_PeriodElapsedCallback();

    //Motor_Shoot.TIM_PID_PeriodElapsedCallback();


}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
