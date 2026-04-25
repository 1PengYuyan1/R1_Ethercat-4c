//
// Created by pzx on 2025/12/13.
//
#include "robot.h"

// // 预设路线 A：取球路线 (绕开出生区柱子)
// const Struct_Waypoint Route_Get_Ball[] = {
//     // x(mm), y(mm), yaw(度), max_speed(m/s), pass_radius(mm)
//     {1500.0f, 0.0f, 0.0f, 1.5f, 200.0f}, // 途经点1：先直走1.5米出基地区，误差20cm内即可切点
//     {1500.0f, 2000.0f, 90.0f, 2.0f, 300.0f}, // 途经点2：高速横移，车头转向90度
//     {2500.0f, 2000.0f, 90.0f, 1.0f, 0.0f} // 终点：精准停靠取球区 (最后一点的 pass_radius 无效)
// };

// // 预设路线 B：退回原点
// const Struct_Waypoint Route_Go_Home[] = {
//     {0.0f, 0.0f, 0.0f, 1.5f, 0.0f} // 只有一个点，直接回家
// };

/**
 * @brief 底盘统一接收回调分发函数
 * @param CAN_Channel 来自 LinkX 的通道号 (0-3)
 * @param CAN_ID 接收到的标准/扩展 CAN ID
 * @param CAN_Data 指向 8 字节数据的指针
 */
void Class_Robot::CAN_Rx_Callback(uint8_t CAN_Channel, uint32_t CAN_ID, uint8_t *CAN_Data)
{
    // 分发给舵向达妙电机 (通常接在通道 0)
    if (CAN_Channel == 0)
    {
        for (int i = 0; i < 4; i++)
        {
            // 达妙电机反馈 ID 匹配 (Init 时设置的 CAN_Rx_ID)
            if (CAN_ID == Chassis.Motor_Steer[i].DM_CAN_Rx_ID)
            {
                Chassis.Motor_Steer[i].CAN_RxCpltCallback(CAN_Data);
                return;
            }
        }
    }

    // 分发给舵向编码器 (假设接在通道 1)
    else if (CAN_Channel == 1)
    {
        for (int i = 0; i < 4; i++)
        {
            // 编码器 ID 匹配
            if (CAN_ID == Chassis.Encoder_Steer[i].Get_Can_ID())
            {
                Chassis.Encoder_Steer[i].CAN_RxCpltCallback(CAN_Data);
                return;
            }
        }
    }

    // 分发给轮向 ODrive 电机 (假设接在通道 2)
    else if (CAN_Channel == 2)
    {
        // ODrive 的 ID 构造为 (NodeID << 5) | CommandID
        uint32_t node_id = (CAN_ID >> 5);

        for (int i = 0; i < 4; i++)
        {
            if (node_id == Chassis.ODrive_Motor_Steer[i].Get_node_id())
            {
                // 注意：ODrive 需要 ID 来判断是心跳还是反馈数据
                Chassis.ODrive_Motor_Steer[i].CAN_RxCpltCallback(CAN_Data, Chassis.ODrive_Motor_Steer[i].Get_node_id());
                return;
            }
        }
    }
}

/**
 * @brief 控制交互端初始化
 *
 */
void Class_Robot::Init(linkx_t *__LinkX_Handler)
{
    LinkX_Handler = __LinkX_Handler;
    Chassis.Init(LinkX_Handler);
    Chassis.Steer_Calibration_Init();
    Chassis.Init_Motor_Params();

    // Navigation.Init();
}

/**
 * @brief 机器人执行循环
 * 在 task.cpp 中每一帧(1ms) 接收完 CAN 数据并执行完定时任务后调用
 */
void Class_Robot::Loop()
{

    // 底盘解算：将 _Chassis_Control 计算出的目标速度转为 4 个轮子的位置/速度
    _Chassis_Control();
    // 更新电机指令：根据状态机（使能/校准/驱动）将数据填入发送缓冲区
    Chassis.TIM_2ms_Control_PeriodElapsedCallback();

}

/**
 * @brief 定时器执行器存活
 *
 */
void Class_Robot::TIM_100ms_Alive_PeriodElapsedCallback()
{

    Chassis.TIM_100ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief 定时器遥控控制
 *
 */
void Class_Robot::TIM_2ms_Calculate_PeriodElapsedCallback()
{
    Chassis.TIM_2ms_Resolution_PeriodElapsedCallback();
    Chassis.TIM_2ms_Control_PeriodElapsedCallback();
}

void Class_Robot::TIM_1ms_Calculate_Callback()
{

    // ops.getData();
    // if (Navigation.Get_Status() == Nav_Status_NAVIGATING)
    // {
    //     Navigation.Calculate(ops.getData());
    // }

    _Chassis_Control();
}

/**
 * @brief 底盘控制逻辑
 *
 */
void Class_Robot::_Chassis_Control()
{
    //     // 统一声明目标速度
    //     float target_vx = 0.0f;
    //     float target_vy = 0.0f;
    //     float target_omega = 0.0f;

    //     // ==============================================================
    //     // 优先级 1: ROS 上位机控制
    //     // ==============================================================
    //     if (Dvc_MiniPC.Is_Connected())
    //     {
    //         Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_ENABLE);

    //         target_vx = Dvc_MiniPC.Get_Vx();
    //         target_vy = Dvc_MiniPC.Get_Vy();
    //         target_omega = Dvc_MiniPC.Get_Vw();

    //         Chassis.Set_Target_Velocity_X(target_vx);
    //         Chassis.Set_Target_Velocity_Y(target_vy);
    //         Chassis.Set_Target_Omega(target_omega);

    //         return; // 拦截成功，ROS 在线时直接结束本周期
    //     }

    //     // ==============================================================
    //     // 优先级 2: LogF710 无线手柄控制
    //     // ==============================================================
    //     else if (LogF710.Is_Connected())
    //     {
    //         LogF710.Judge_Status();

    //         if (LogF710.Is_ROS_Control_Enabled && LogF710.Get_Status() != LogF710_Status_DISABLE)
    //         {
    //             Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_ENABLE);

    //             target_vx = LogF710.Get_Left_X();
    //             target_vy = LogF710.Get_Left_Y();
    //             target_omega = LogF710.Get_Right_X();

    //             Chassis.Set_Target_Velocity_X(target_vx);
    //             Chassis.Set_Target_Velocity_Y(target_vy);
    //             Chassis.Set_Target_Omega(target_omega);
    //         }
    //         else
    //         {
    //             Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
    //         }

    //         return; // 拦截成功，LogF710 在线时直接结束
    //     }

    //     // ==============================================================
    //     // 优先级 3: DR16 遥控器 (半自动导航 + 纯手动控制)
    //     // ==============================================================
    //     else if (DR16.Is_Connected())
    //     {
    //     if (DR16.Get_Status() == DR16_Status_DISABLE || DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN)
    //     {
    //         Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

    //         if (Navigation.Get_Status() != Nav_Status_IDLE)
    //         {
    //             Navigation.Stop_Navigation();
    //         }
    //         return;
    //     }

    //     // DR16 状态正常，使能底盘
    //     Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_ENABLE);

    //     Enum_DR16_Switch_Status left_sw = DR16.Get_Left_Switch();
    //     Enum_DR16_Switch_Status right_sw = DR16.Get_Right_Switch();

    //     // 逻辑 A: 停止导航，回到手动模式 (左 MID + 右 MID)
    //     if (left_sw == DR16_Switch_Status_MIDDLE && right_sw == DR16_Switch_Status_MIDDLE)
    //     {
    //         if (Navigation.Get_Status() != Nav_Status_IDLE)
    //         {
    //             Navigation.Stop_Navigation();
    //         }
    //     }
    //     // 逻辑 B: 开启“自动模式准备状态” (左 MID + 右 DOWN)
    //     else if (left_sw == DR16_Switch_Status_MIDDLE && right_sw == DR16_Switch_Status_DOWN)
    //     {
    //         // 如果你的 Navigation 类有“预备”接口可以在此调用
    //         // 这里主要起到占位作用，确保从 MID/MID 切换过来时不会立刻跑车
    //     }
    //     // 逻辑 C: 触发具体导航路径 (左 UP + 右 DOWN)
    //     else if (left_sw == DR16_Switch_Status_UP && right_sw == DR16_Switch_Status_DOWN)
    //     {
    //         // 只有当前处于空闲时才触发，防止重复触发导致路径重置
    //         if (Navigation.Get_Status() == Nav_Status_IDLE)
    //         {
    //             // 这里触发你指定的路径，例如 Route_Get_Ball
    //             Navigation.Set_Route(Route_Get_Ball, sizeof(Route_Get_Ball) / sizeof(Struct_Waypoint));
    //             Navigation.Start_Navigation();
    //         }
    //     }

    //     // --- 3.2 导航接管底盘 (拦截) ---
    //     if (Navigation.Get_Status() == Nav_Status_NAVIGATING)
    //     {
    //         target_vx = Navigation.Get_Target_Vx();
    //         target_vy = Navigation.Get_Target_Vy();
    //         target_omega = Navigation.Get_Target_Omega();

    //         Chassis.Set_Target_Velocity_X(target_vx);
    //         Chassis.Set_Target_Velocity_Y(target_vy);
    //         Chassis.Set_Target_Omega(target_omega);

    //         return; // 导航中，跳过下方摇杆读取
    //     }

    //     // --- 3.3 纯手动接管逻辑 (当导航处于 IDLE 状态时) ---
    //     float dr16_left_x = DR16.Get_Left_X();
    //     float dr16_left_y = DR16.Get_Left_Y();
    //     float dr16_yaw = DR16.Get_Yaw();

    //     // 排除死区
    //     dr16_left_x = Math_Abs(dr16_left_x) > DR16_Rocker_Dead_Zone ? dr16_left_x : 0.0f;
    //     dr16_left_y = Math_Abs(dr16_left_y) > DR16_Rocker_Dead_Zone ? dr16_left_y : 0.0f;
    //     dr16_yaw = Math_Abs(dr16_yaw) > DR16_Rocker_Dead_Zone ? dr16_yaw : 0.0f;

    //     target_vx = dr16_left_y;
    //     target_vy = -dr16_left_x;
    //     target_omega = dr16_yaw;

    //     Chassis.Set_Target_Velocity_X(target_vx);
    //     Chassis.Set_Target_Velocity_Y(target_vy);
    //     Chassis.Set_Target_Omega(target_omega);

    //     return;
    // }

    //     Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

    //     if (Navigation.Get_Status() != Nav_Status_IDLE)
    //     {
    //         Navigation.Stop_Navigation();
    //     }
}
