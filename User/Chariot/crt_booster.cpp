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

/**
 * @brief 定时器处理函数
 * 这是一个模板, 使用时请根据不同处理情况在不同文件内重新定义
 *
 */
void Class_FSM_Heat_Detect::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    //自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
    case (0):
    {
        //正常状态

        if (abs(Booster->Motor_Friction_Right.Get_Now_Torque()) >= Booster->Friction_Torque_Threshold)
        {
            //大扭矩->检测状态
            Set_Status(1);
        }
        else if (Booster->Booster_Control_Type == Booster_Control_Type_DISABLE)
        {
            //停机->停机状态
            Set_Status(3);
        }
    }
    break;
    case (1):
    {
        //发射嫌疑状态

        if (Status[Now_Status_Serial].Time >= 15)
        {
            //长时间大扭矩->确认是发射了
            Set_Status(2);
        }
    }
    break;
    case (2):
    {
        //发射完成状态->加上热量进入下一轮检测

        Heat += 10.0f;
        Set_Status(0);
    }
    break;
    case (3):
    {
        //停机状态

        if (abs(Booster->Motor_Friction_Right.Get_Now_Omega_Radian()) >= Booster->Friction_Omega_Threshold)
        {
            //开机了->正常状态
            Set_Status(0);
        }
    }
    break;
    }

    //热量冷却到0
    if (Heat > 0)
    {
        //Heat -= Booster->Referee->Get_Booster_17mm_1_Heat_CD() / 1000.0f;
    }
    else
    {
        Heat = 0;
    }
}

/**
 * @brief 卡弹策略有限自动机
 *
 */
void Class_FSM_Antijamming::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    //自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
        case (0):
        {
            //正常状态
            Booster->Output();

            if (abs(Booster->Motor_Driver.Get_Now_Torque()) >= Booster->Driver_Torque_Threshold)
            {
                //大扭矩->卡弹嫌疑状态
                Set_Status(1);
            }
        }
        break;
        case (1):
        {
            //卡弹嫌疑状态
            Booster->Output();

            if (Status[Now_Status_Serial].Time >= 100)
            {
                //长时间大扭矩->卡弹反应状态
                Set_Status(2);
            }
            else if (abs(Booster->Motor_Driver.Get_Now_Torque()) < Booster->Driver_Torque_Threshold)
            {
                //短时间大扭矩->正常状态
                Set_Status(0);
            }
        }
        break;
        case (2):
        {
            //卡弹反应状态->准备卡弹处理
            Booster->Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Booster->Drvier_Angle = Booster->Motor_Driver.Get_Now_Radian() + PI / 12.0f;
            Booster->Motor_Driver.Set_Target_Radian(Booster->Drvier_Angle);
            Set_Status(3);
        }
        break;
        case (3):
        {
            //卡弹处理状态

            if (Status[Now_Status_Serial].Time >= 300)
            {
                //长时间回拨->正常状态
                Set_Status(0);
            }
        }
        break;
    }
}

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
    //正常状态, 发射嫌疑状态, 发射完成状态, 停机状态
    FSM_Heat_Detect.Booster = this;
    FSM_Heat_Detect.Init(3, 3);

    //正常状态, 卡弹嫌疑状态, 卡弹反应状态, 卡弹处理状态
    FSM_Antijamming.Booster = this;
    FSM_Antijamming.Init(4, 0);

    //拨弹盘电机
    Motor_Shoot.PID_Angle.Init(25.0f, 0.f, 0.0f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Shoot.PID_Omega.Init(3000.0f, 10.0f, 0.001f, 0.0f, Motor_Driver.Get_Output_Max(), Motor_Driver.Get_Output_Max());
    Motor_Shoot.Init(&hcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA);

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
            Motor_Shoot.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);

            Motor_Shoot.PID_Angle.Set_Integral_Error(0.0f);
            Motor_Shoot.PID_Omega.Set_Integral_Error(0.0f);

            Motor_Shoot.Set_Out(0.0f);

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
    
    //无需裁判系统的热量控制计算
    FSM_Heat_Detect.Reload_TIM_Status_PeriodElapsedCallback();
    //卡弹处理
    FSM_Antijamming.Reload_TIM_Status_PeriodElapsedCallback();

    Motor_Shoot.TIM_PID_PeriodElapsedCallback();


}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
