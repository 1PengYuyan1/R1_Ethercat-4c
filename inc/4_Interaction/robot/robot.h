#ifndef ROBOT_H
#define ROBOT_H

#include "crt_chassis.h"

class Class_Robot
{
public:

    Class_Chassis Chassis;
    // Class_Navigation Navigation;
    // Class_DR16 DR16;
    // Class_LogF710 LogF710;
    // Class_OPS ops;

    void Init(linkx_t *__LinkX_Handler);

    void Loop();

    void CAN_Rx_Callback(uint8_t CAN_Channel, uint32_t CAN_ID, uint8_t *CAN_Data);

    void TIM_1000ms_Alive_PeriodElapsedCallback();
    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_100ms_Calculate_Callback();
    void TIM_10ms_Calculate_PeriodElapsedCallback();
    void TIM_2ms_Calculate_PeriodElapsedCallback();
    void TIM_1ms_Calculate_Callback();

protected:
    linkx_t *LinkX_Handler;
    // 底盘解算方向对底盘速度的前馈
    float AHRS_Chassis_Omega_Feedforward = 0.07f;
    float Chassis_Omega_Feedforward = 0.10f;

    void _Chassis_Control();
    void _Chassis_Navigation_Control();
};

#endif // USTC_STREETING_ROBOT_H