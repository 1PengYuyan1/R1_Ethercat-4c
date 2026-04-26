//
// Created by pzx on 2025/12/20.
//

#ifndef USTC_STEERING_1_CRT_CHASSIS_H
#define USTC_STEERING_1_CRT_CHASSIS_H

#include "alg_pid.h"
#include "math.h"
#include "dvc_encoder.h"
#include "dvc_motor_dm.h"
#include "dvc_odrive.h"

#define STEER_NUM 4
#define REDUCTION_RATIO 3.5f

/** @brief 每个舵轮的可调参数 */
struct SteerWheelParams {
  float steer_kp;  // 舵向角速度 P 增益
  float steer_kd;  // 舵向角速度 D 增益
  /** steer_omega_deadzone：PD 输出最小角速度，【舵向侧（输出轴）rad/s】 */
  float steer_omega_deadzone;
  float steer_rad_deadzone;  // 舵向角度死区 [rad]

  float wheel_omega_deadzone;  // 轮向（驱动轮）死区 [rad/s]
  float wheel_feedforward;     // 轮向前馈系数
  float wheel_direction;       // 轮向安装方向（+1 或 -1）

  float flip_speed_threshold;  // 翻轮速度阈值
  float flip_drive_scale;      // 翻轮期间速度缩放
};

/** @brief 校准流程状态 */
enum Enum_Calib_State {
  CALIB_STATE_WAIT_STABLE = 0,
  CALIB_STATE_CALCULATE,
  CALIB_STATE_EXECUTING,
  CALIB_STATE_DONE
};

/** @brief 底盘使能状态 */
enum Enum_Chassis_Control_Type {
  Chassis_Control_Type_DISABLE = 0,
  Chassis_Control_Type_ENABLE,
};

/** @brief 单个舵轮对准/驱动状态 */
typedef enum {
  STEER_STATE_IDLE = 0,
  STEER_STATE_ALIGN,
  STEER_STATE_DRIVE
} SteerDriveState_e;

typedef struct {
  SteerDriveState_e state;
  uint8_t flip_locked;
  uint8_t angle_reached;
  uint8_t request_drive_slowdown;
} SteerCtrl_t;

/**
 * @brief 四舵轮底盘类
 */
class Class_Chassis {
 public:
  SteerWheelParams steer_wheel_params_[STEER_NUM];
  Class_Motor_DM_Normal Motor_Steer[STEER_NUM];
  Class_Encoder_BRT Encoder_Steer[STEER_NUM];
  Class_ODrive ODrive_Motor_Steer[STEER_NUM];
  Class_PID PID_Heading;

  // 初始化
  void Init(linkx_t *__LinkX_Handler);
  void Init_Motor_Params();

  // 校准
  void Align_Steer_Encoders() ;
  void Steer_Calibration_Init();
  uint8_t Steer_Calibration_Process();
  uint8_t Is_Calibration_Complete();
  void Force_Recalibration();
  uint8_t Is_Wheel_Calibration_Complete(int index);
  bool Is_Steer_Angle_Reached(int index);

  // 航向补偿
  void Apply_Heading_Correction(float& omega, float vx, float vy, float yaw);

  // 定时器回调
  void TIM_100ms_Alive_PeriodElapsedCallback();
  void TIM_2ms_Resolution_PeriodElapsedCallback();
  void TIM_2ms_Control_PeriodElapsedCallback();

  // Getter
  inline float Get_Now_Velocity_X() { return Now_Velocity_X; }
  inline float Get_Now_Velocity_Y() { return Now_Velocity_Y; }
  inline float Get_Now_Omega() { return Now_Omega; }
  inline Enum_Chassis_Control_Type Get_Chassis_Control_Type() {
    return Chassis_Control_Type;
  }
  inline float Get_Target_Velocity_X() { return Target_Velocity_X; }
  inline float Get_Target_Velocity_Y() { return Target_Velocity_Y; }
  inline float Get_Target_Omega() { return Target_Omega; }

  // Setter
  inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type t) {
    Chassis_Control_Type = t;
  }
  inline void Set_Target_Velocity_X(float v) { Target_Velocity_X = v; }
  inline void Set_Target_Velocity_Y(float v) { Target_Velocity_Y = v; }
  inline void Set_Target_Omega(float v) { Target_Omega = v; }

 protected:
  linkx_t *LinkX_Handler = nullptr;
  // 航向锁定
  float Target_Heading = 0.0f;
  bool Heading_Lock_Flag = false;

  // 调试
  float debug_wheel_error[STEER_NUM] = {0};
  float debug_encoder_rad[STEER_NUM] = {0};

  // 校准
  float Motor_Zero_Offset[STEER_NUM] = {0};
  uint8_t steer_calibration_done[STEER_NUM] = {0};
  float steer_calibration_target[STEER_NUM] = {0};
  uint8_t all_calibration_complete = 0;
  uint32_t calibration_wait_tick = 0;
  uint8_t calibration_total_timeout = 0;
  Enum_Calib_State calib_step = CALIB_STATE_WAIT_STABLE;
  bool is_steer_aligned = false;
  bool is_calib_target_calculated = false;

  // 舵轮状态
  uint8_t steer_flipped[STEER_NUM] = {0};
  uint8_t steer_filter_init[STEER_NUM] = {0};
  float steer_ref_filtered[STEER_NUM] = {0};
  SteerCtrl_t steer_ctrl[STEER_NUM] = {STEER_STATE_IDLE};
  SteerDriveState_e steer_state[STEER_NUM] = {STEER_STATE_IDLE};
  float wheel_speed_scale[STEER_NUM] = {1.0f};

  // 动力学辅助
  float Dynamic_Resistance_Wheel_Current[STEER_NUM] = {0};

  float Steer_Inertia = 0.001f;         // 舵向转动惯量，根据实际机构调整
  float Steer_Damping = 0.01f;          // 舵轴阻尼
  float Steer_Static_Friction = 0.05f;  // 舵轴静摩擦补偿

  // 目标量
  // ---------------------------------------------------------------
  // 舵向量均定义在【舵向侧（输出轴）】坐标系下：
  //   - Target_Steer_Rad    [rad]       舵向目标角度（输出轴）
  //   - Target_Steer_Omage  [rad/s]     舵向目标角速度（输出轴）
  //   - Target_Steer_Torque [N·m]       舵向目标力矩（输出轴）
  // 发给 DM 电机时在 Execute_Steer_State() 统一乘以 REDUCTION_RATIO（角速度）
  // 或除以 REDUCTION_RATIO（力矩），位置由 Steer_To_Motor_Position() 换算。
  // ---------------------------------------------------------------
  float Target_Steer_Rad[STEER_NUM] = {0};
  float Target_Steer_Omage[STEER_NUM] = {0};
  float Target_Steer_Torque[STEER_NUM] = {0};
  float Target_Wheel_Omega[STEER_NUM] = {0};
  float Target_Wheel_torque[STEER_NUM] = {0};

  // 机械参数
  const float angle_tolerance = 0.05f;
  const float Wheel_Radius = 0.018f;
  const float Wheel_To_Core_Distance[STEER_NUM] = {0.707f, 0.707f, 0.707f,
                                                   0.707f};
  const float Wheel_Azimuth[STEER_NUM] = {
      PI / 4.0f,         //  45° 左前
      3.0f * PI / 4.0f,  // 135° 左后
      5.0f * PI / 4.0f,  // 225° 右后
      7.0f * PI / 4.0f,  // 315° 右前
  };

  float Wheel_Resistance_Omega_Threshold = 1.0f;
  float Wheel_Speed_Limit_Factor = 0.5f;
  /** MAX_STEER_OMEGA：舵向侧（输出轴）最大角速度 [rad/s]，输出到电机时乘以
   * REDUCTION_RATIO */

  float MAX_STEER_OMEGA = 30.0f;
  const float MAX_CHASSIS_SPEED = 1.0f;

  const float Center_Height = 0.35f;
  const float Class_Mass = 30.0f;
  const float Class_Inertia = 2.0f;

  // 当前状态量
  float Now_Velocity_X = 0.0f;
  float Now_Velocity_Y = 0.0f;
  float Now_Omega = 0.0f;

  float Target_Velocity_X = 0.0f;
  float Target_Velocity_Y = 0.0f;
  float Target_Omega = 0.0f;

  Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;

  // 内部函数
  float Get_Now_Steer_Radian(int index);
  float Steer_To_Motor_Position(float target_steer, int index);
  bool All_Steer_Angle_Reached();
  void Steer_Calibration_Start();
  void Self_Resolution();
  void Kinematics_Inverse_Resolution();
  void _Steer_Motor_Kinematics_Nearest_Transposition();
  void Dynamics_Inverse_Resolution();
  void Update_Steer_State(int i);
  void Execute_Steer_State(int i);
  void Output_To_Motor();
  void Send_DMMotor_CAN_Frame();
  void Send_ODrive_CAN_Frame();
  void Send_Encoder_CAN_Frame();
};

#endif
