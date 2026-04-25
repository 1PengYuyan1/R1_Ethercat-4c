//
// Created by pzx on 2025/12/20.
//

#ifndef USTC_STEERING_1_CRT_CHASSIS_H
#define USTC_STEERING_1_CRT_CHASSIS_H

#include "dvc_encoder.h"
#include "dvc_odrive.h"
#include "dvc_motor_dm.h"

#include <cstdint>
#include <cmath>
#include "linkx4c_handler.h"

#define STEER_NUM 4
#define REDUCTION_RATIO 3.5f

struct SteerWheelParams
{
    // 舵向参数
    float steer_kp;             // 舵向角速度P增益
    float steer_kd;             // 舵向角速度D增益
    float steer_omega_deadzone; // 舵向角速度死区
    float steer_rad_deadzone;   // 舵向角度死区

    // 轮向参数
    float wheel_omega_deadzone; // 轮向死区
    float wheel_feedforward;    // 轮向前馈系数
    float wheel_direction;      // 轮向方向(+1或-1)

    // 翻轮参数
    float flip_speed_threshold; // 翻轮速度阈值
    float flip_drive_scale;     // 翻轮期间速度缩放
};

/**
 * @brief 校准控制类型
 *
 */
enum Enum_Calib_State
{
    CALIB_STATE_WAIT_STABLE = 0, // 等待系统稳定和编码器数据
    CALIB_STATE_CALCULATE,       // 计算校准目标
    CALIB_STATE_EXECUTING,       // 执行电机运动
    CALIB_STATE_DONE             // 校准完成
};

/**
 * @brief 底盘控制类型
 *
 */
enum Enum_Chassis_Control_Type
{
    Chassis_Control_Type_DISABLE = 0,
    Chassis_Control_Type_ENABLE,
};

/**
 * @brief 舵向电机状态判断
 *
 */
typedef enum
{
    STEER_STATE_IDLE = 0, // 无运动 / 轮速极低
    STEER_STATE_ALIGN,    // 舵向对准中
    STEER_STATE_DRIVE     // 舵向已对准，允许驱动
} SteerDriveState_e;

/**
 * @brief 舵轮状态判断
 *
 */
typedef struct
{
    SteerDriveState_e state;
    uint8_t flip_locked;            // 翻轮锁（只允许一次）
    uint8_t angle_reached;          // 舵向到位锁存
    uint8_t request_drive_slowdown; // 翻轮发生的那一刻
} SteerCtrl_t;

/**
 * @brief Specialized, 舵轮底盘类
 *
 */
class Class_Chassis
{
public:
    SteerWheelParams steer_wheel_params_[STEER_NUM]; // 舵轮底盘变量
    Class_Motor_DM_Normal Motor_Steer[STEER_NUM];    // 舵向电机
    Class_Encoder_BRT Encoder_Steer[STEER_NUM];      // 舵向编码器
    Class_ODrive ODrive_Motor_Steer[STEER_NUM];      // 轮向电机

    // 舵向校准相关函数
    void Steer_Calibration_Init();                    // 校准初始化(Reset时调用一次)
    uint8_t Steer_Calibration_Process();              // 校准执行(控制周期中调用)
    uint8_t Is_Calibration_Complete();                // 查询是否校准完成
    void Force_Recalibration();                       // 强制重新校准
    uint8_t Is_Wheel_Calibration_Complete(int index); // 查询单个轮子是否校准完成

    // 舵向初始化校准

    void Init(linkx_t *__LinkX_Handler);
    void Init_Motor_Params();
    bool Is_Steer_Angle_Reached(int index); // 判断角度是否到达

    // 获取信息
    inline float Get_Now_Velocity_X();
    inline float Get_Now_Velocity_Y();
    inline float Get_Now_Omega();
    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();
    inline float Get_Target_Velocity_X();
    inline float Get_Target_Velocity_Y();
    inline float Get_Target_Omega();

    // 设立信息
    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Target_Velocity_X(float __Target_Velocity_X);
    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);
    inline void Set_Target_Omega(float __Target_Omega);

    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_2ms_Resolution_PeriodElapsedCallback();
    void TIM_2ms_Control_PeriodElapsedCallback();

    void Send_Encoder_CAN_Frame();

protected:
    linkx_t *LinkX_Handler;
    float Dynamic_Resistance_Wheel_Current[STEER_NUM] = {0.0f};
    uint8_t steer_calibration_done[STEER_NUM] = {0, 0, 0, 0};             // 校准状态标志位(每个轮子独立)
    float steer_calibration_target[STEER_NUM] = {0.0f, 0.0f, 0.0f, 0.0f}; // 校准目标位置记录(每个轮子独立存储,单位:电机轴角度rad)
    uint8_t all_calibration_complete = 0;                                 // 校准完成标志(全局)
    uint32_t calibration_wait_tick = 0;                                   // 等待计时器
    bool is_calib_target_calculated = false;                              // 目标是否已计算标志

    uint8_t steer_flipped[STEER_NUM] = {0};                        // 轮子状态位
    uint8_t steer_filter_init[STEER_NUM] = {0};                    // 某个舵轮的滤波器是否已经初始化
    float steer_ref_filtered[STEER_NUM] = {0};                     // 存放 舵向参考值的滤波后结果
    uint8_t steer_flip_locked[STEER_NUM] = {0};                    // 翻轮锁存标志
    SteerCtrl_t steer_ctrl[STEER_NUM] = {STEER_STATE_IDLE};        // 舵轮状态判断
    SteerDriveState_e steer_state[STEER_NUM] = {STEER_STATE_IDLE}; // 舵向状态判断
    float wheel_speed_scale[STEER_NUM] = {1.0f};                   // 轮速缩放系数

    float Target_Steer_Rad[STEER_NUM];    // 舵向电机角度目标值
    float Target_Steer_Omage[STEER_NUM];  // 舵向电机角速度目标值
    float Target_Steer_Torque[STEER_NUM]; // 舵向电机力矩目标值
    float Target_Wheel_Omega[STEER_NUM];  // 轮向电机角速度目标值
    float Target_Wheel_torque[STEER_NUM]; // 轮向电机力矩

    const float angle_tolerance = 0.05f; // 角度容差，单位弧度（约2.86度）
    const float Wheel_Radius = 0.018f;   // 轮组半径
    const float Wheel_To_Core_Distance[STEER_NUM] = {
        0.707f,
        0.707f,
        0.707f,
        0.707f,
    }; // 轮距中心长度

    // 轮组方位角
    const float Wheel_Azimuth[STEER_NUM] = {
        PI / 4.0f,        // 45 左前
        3.0f * PI / 4.0f, // 135   左后
        5.0f * PI / 4.0f, // 225   右后
        7.0f * PI / 4.0f,
    }; // 315  右前

    float Wheel_Resistance_Omega_Threshold = 1.0f; // 轮向电机摩擦阻力连续化的角速度阈值

    // 防单轮超速系数
    float Wheel_Speed_Limit_Factor = 0.5f; // 防单轮超速系数
    float MAX_STEER_OMEGA = 50;            // 限制最大转向速度
    const float MAX_CHASSIS_SPEED = 30;    // 底盘最大速度

    const float Center_Height = 0.15f; // 重心高度
    const float Class_Mass = 30.0f;    // 底盘质量
    const float Class_Inertia = 2.0f;  // 转动惯量

    float Now_Velocity_X = 0.0f; // 当前速度X
    float Now_Velocity_Y = 0.0f; // 当前速度Y
    float Now_Omega = 0.0f;      // 当前角速度

    float Target_Velocity_X = 0.0f; // 目标速度X
    float Target_Velocity_Y = 0.0f; // 目标速度Y
    float Target_Omega = 0.0f;      // 目标角速度

    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;
    Enum_Calib_State calib_step = CALIB_STATE_WAIT_STABLE;

    void Steer_Calibration_Start();                               // 启动校准(内部调用)
    float Get_Now_Steer_Radian(int index);                        // 获取当前舵向角度 (通过达妙反馈 + 减速比换算)
    float Steer_To_Motor_Position(float target_steer, int index); // 舵向目标角度 → 达妙位置指令
    bool All_Steer_Angle_Reached();                               // 判断每个舵向角度是否到达
    void Self_Resolution();                                       // 自身姿态，速度解算
    void Kinematics_Inverse_Resolution();                         // 舵轮运动学逆结算
    void _Steer_Motor_Kinematics_Nearest_Transposition();         // 舵向电机就近解算
    void Output_To_Dynamics();                                    // 输出动力学
    void Dynamics_Inverse_Resolution();                           // 动力学逆解算
    void Update_Steer_State(int i);                               // 处理状态切换逻辑
    void Execute_Steer_State(int i);                              // 处理电机输出逻辑
    void Output_To_Motor();                                       // 输出到电机
    void Send_DMMotor_CAN_Frame();                                //
    void Send_ODrive_CAN_Frame();                                 //
};

/**
 * @brief 获取当前速度X
 *
 * @return float 当前速度X
 */
inline float Class_Chassis::Get_Now_Velocity_X()
{
    return (Now_Velocity_X);
}

/**
 * @brief 获取当前速度Y
 *
 * @return float 当前速度Y
 */
inline float Class_Chassis::Get_Now_Velocity_Y()
{
    return (Now_Velocity_Y);
}

/**
 * @brief 获取当前角速度
 *
 * @return float 当前角速度
 */
inline float Class_Chassis::Get_Now_Omega()
{
    return (Now_Omega);
}

/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 */
inline Enum_Chassis_Control_Type Class_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}

/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
inline float Class_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}

/**
 * @brief 获取目标速度Y
 *
 * @return float 目标速度Y
 */
inline float Class_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 */
inline void Class_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
inline void Class_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
inline void Class_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
inline void Class_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

#endif
