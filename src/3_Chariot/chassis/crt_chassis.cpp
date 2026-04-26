//
// Created by pzx on 2025/12/20.
//
#include "crt_chassis.h"
#include <math.h>

/**
 * @brief 初始化硬件
 */
void Class_Chassis::Init(linkx_t *__LinkX_Handler)
{
  LinkX_Handler = __LinkX_Handler;

  Motor_Steer[0].Init(LinkX_Handler, 0, 0x11, 0x01,
    Motor_DM_Control_Method_NORMAL_MIT, 21.92f, 150.0f, 5.0f, 15.0f);
  Motor_Steer[1].Init(LinkX_Handler, 0, 0x12, 0x02,
    Motor_DM_Control_Method_NORMAL_MIT, 21.92f, 150.0f, 5.0f, 15.0f);
  Motor_Steer[2].Init(LinkX_Handler, 0, 0x13, 0x03,
    Motor_DM_Control_Method_NORMAL_MIT, 21.92f, 150.0f, 5.0f, 15.0f);
  Motor_Steer[3].Init(LinkX_Handler, 0, 0x14, 0x04,
    Motor_DM_Control_Method_NORMAL_MIT, 21.92f, 150.0f, 5.0f, 15.0f);

  ODrive_Motor_Steer[0].Init(LinkX_Handler, 1, 0x10);
  ODrive_Motor_Steer[1].Init(LinkX_Handler, 1, 0x18);
  ODrive_Motor_Steer[2].Init(LinkX_Handler, 1, 0x20);
  ODrive_Motor_Steer[3].Init(LinkX_Handler, 1, 0x28);

  Encoder_Steer[0].Init(LinkX_Handler, 2, 0x05, 4096);
  Encoder_Steer[1].Init(LinkX_Handler, 2, 0x06, 4096);
  Encoder_Steer[2].Init(LinkX_Handler, 2, 0x07, 4096);
  Encoder_Steer[3].Init(LinkX_Handler, 2, 0x08, 4096);

  PID_Heading.Init(1.5f, 0.0f, 0.05f, 0.0f, 1.0f, 10.0f, 0.001f, 0.5f);
}

/**
 * @brief 初始化参数
 * @note 四个轮参数相同，用循环统一赋值
 */
void Class_Chassis::Init_Motor_Params()
{
  SteerWheelParams common_params = {
    .steer_kp = 30.0f,
    .steer_kd = 0.5f,
    .steer_omega_deadzone = 1.2f / REDUCTION_RATIO,
    .steer_rad_deadzone = (2.86f * PI / 180.0f),
    .wheel_omega_deadzone = 0.05f,
    .wheel_feedforward = 0.5f,
    .wheel_direction = 1.0f,
    .flip_speed_threshold = 2.0f,
    .flip_drive_scale = 0.4f,
  };
  for (int i = 0; i < STEER_NUM; i++) steer_wheel_params_[i] = common_params;

  Steer_Inertia = 1.000f; // 舵向转动惯量，根据实际机构调整
  Steer_Damping = 1.00f; // 舵轴阻尼
  Steer_Static_Friction = 1.00f; // 舵轴静摩擦补偿
}

/**
 * @brief 100ms 定时器：检测电机心跳
 */
void Class_Chassis::TIM_100ms_Alive_PeriodElapsedCallback()
{
  for (int i = 0; i < STEER_NUM; i++)
  {
    Motor_Steer[i].TIM_Alive_PeriodElapsedCallback();
    ODrive_Motor_Steer[i].TIM_Alive_CheckCallback();
  }

  if (Chassis_Control_Type != Chassis_Control_Type_ENABLE ||
    !all_calibration_complete)
    return;

  for (int i = 0; i < STEER_NUM; i++)
  {
    if (Motor_Steer[i].Get_Status() != Motor_DM_Status_ENABLE)
      Motor_Steer[i].CAN_Send_Enter();

    uint32_t current_error = ODrive_Motor_Steer[i].Get_Axis_Error();
    if (current_error != AXIS_ERROR_NONE)
    {
      ODrive_Motor_Steer[i].Clear_Errors();
      continue;
    }

    if (ODrive_Motor_Steer[i].Get_Axis_State() != ODRIVE_STATE_CLOSED_LOOP_CONTROL)
      ODrive_Motor_Steer[i].SET_ClosedLoop();
  }
}

/**
 * @brief 2ms 定时器：姿态/速度解算
 */
void Class_Chassis::TIM_2ms_Resolution_PeriodElapsedCallback()
{
  Self_Resolution();
}

/**
 * @brief 2ms 定时器：底盘控制主循环
 */
void Class_Chassis::TIM_2ms_Control_PeriodElapsedCallback()
{
  if (!Is_Calibration_Complete())
  {
    if (calib_step != CALIB_STATE_DONE)
    {
      for (int i = 0; i < STEER_NUM; i++)
      {
        if (Motor_Steer[i].Get_Status() != Motor_DM_Status_ENABLE)
          Motor_Steer[i].CAN_Send_Enter();
      }
    }
    Steer_Calibration_Process();

    if (calib_step != CALIB_STATE_DONE)
    {
      for (int i = 0; i < STEER_NUM; i++)
        Motor_Steer[i].TIM_Send_PeriodElapsedCallback();
    }

    return;
  }

  switch (Chassis_Control_Type)
  {
  case Chassis_Control_Type_DISABLE:
    // 停止所有电机，清除目标
    for (int i = 0; i < STEER_NUM; i++)
    {
      Motor_Steer[i].Set_Control_Status(Motor_DM_Status_DISABLE);
      Motor_Steer[i].Set_Control_Torque_P_D_MIT(0.0f,
        steer_wheel_params_[i].steer_kp, steer_wheel_params_[i].steer_kd);
      Motor_Steer[i].Set_Control_Parameter_MIT(0.0f, 0.0f);
      if (Motor_Steer[i].Get_Now_Control_Status() != Motor_DM_Status_DISABLE)
        Motor_Steer[i].CAN_Send_Exit();

      if (ODrive_Motor_Steer[i].Get_Axis_State() != ODRIVE_STATE_IDLE)
      {
        ODrive_Motor_Steer[i].Emergency_Stop();
        ODrive_Motor_Steer[i].Set_Velocity(0.0f);
        ODrive_Motor_Steer[i].Set_Torque(0.0f);
      }

      Target_Wheel_Omega[i] = 0.0f;
      Target_Wheel_torque[i] = 0.0f;
    }
    Target_Velocity_X = 0.0f;
    Target_Velocity_Y = 0.0f;
    Target_Omega = 0.0f;
    return;

  case Chassis_Control_Type_ENABLE:

    Kinematics_Inverse_Resolution();
    // Dynamics_Inverse_Resolution();
    Output_To_Motor();
    break;
  }
}

uint8_t Class_Chassis::Is_Calibration_Complete()
{
  return all_calibration_complete;
}

void Class_Chassis::Force_Recalibration()
{
  all_calibration_complete = 0;
  calibration_wait_tick = 0;
  is_calib_target_calculated = false;
}

uint8_t Class_Chassis::Is_Wheel_Calibration_Complete(int index)
{
  if (index >= 0 && index < STEER_NUM) return steer_calibration_done[index];
  return 0;
}

void Class_Chassis::Steer_Calibration_Init()
{
  for (int i = 0; i < STEER_NUM; i++)
  {
    Motor_Steer[i].Set_Control_Torque_P_D_MIT(0.0f,
      steer_wheel_params_[i].steer_kp, steer_wheel_params_[i].steer_kd);
  }
  calib_step = CALIB_STATE_WAIT_STABLE;
  all_calibration_complete = 0;
  calibration_wait_tick = 0;
  is_calib_target_calculated = false;

  for (int i = 0; i < STEER_NUM; i++)
  {
    steer_calibration_done[i] = 0;
    steer_calibration_target[i] = 0.0f;
  }
}

/**
 * @brief 计算每个轮子的校准目标位置（电机轴角度）
 */
void Class_Chassis::Steer_Calibration_Start()
{
  for (int i = 0; i < STEER_NUM; i++)
  {
    float encoder_angle_rad = Encoder_Steer[i].Get_Wheel_Posture_radian();
    float error_rad = -encoder_angle_rad;
    if (error_rad > PI) error_rad -= 2.0f * PI;
    if (error_rad < -PI) error_rad += 2.0f * PI;

    steer_calibration_target[i] =
      Motor_Steer[i].Get_Now_Radian() + (error_rad * REDUCTION_RATIO);
    steer_calibration_done[i] = 0;
  }
}

/**
 * @brief 校准状态机，在控制周期内循环调用
 * @return 1-完成, 0-进行中
 */
uint8_t Class_Chassis::Steer_Calibration_Process()
{
  if (all_calibration_complete) return 1;

  switch (calib_step)
  {
  case CALIB_STATE_WAIT_STABLE:
    for (int i = 0; i < STEER_NUM; i++)
      Encoder_Steer[i].TIM_Query_PeriodElapsedCallback();

    if (++calibration_wait_tick > 1000)
    {
      uint8_t data_ready = 1;
      for (int i = 0; i < STEER_NUM; i++)
        if (Encoder_Steer[i].Get_Wheel_Posture_radian() == 0.0f)
        {
          data_ready = 0;
          break;
        }

      if (data_ready) calib_step = CALIB_STATE_CALCULATE;
    }
    break;

  case CALIB_STATE_CALCULATE:
    Steer_Calibration_Start();
    calib_step = CALIB_STATE_EXECUTING;
    break;

  case CALIB_STATE_EXECUTING:
    {
      uint8_t all_ready = 1;
      for (int i = 0; i < STEER_NUM; i++)
      {
        Motor_Steer[i].Set_Control_Torque_P_D_MIT(8.0f, 0.4f, 0.0f);
      }
      for (int i = 0; i < STEER_NUM; i++)
      {
        Encoder_Steer[i].TIM_Query_PeriodElapsedCallback();
        Motor_Steer[i].Set_Control_Parameter_MIT(
          steer_calibration_target[i], 0.0f);

        float enc_rad = Encoder_Steer[i].Get_Wheel_Posture_radian();
        float wheel_err = 0.0f - enc_rad;
        while (wheel_err > PI) wheel_err -= 2.0f * PI;
        while (wheel_err < -PI) wheel_err += 2.0f * PI;

        if (fabsf(wheel_err) < 0.03f) steer_calibration_done[i] = 1;
        if (!steer_calibration_done[i]) all_ready = 0;
      }
      if (all_ready)
      {
        calib_step = CALIB_STATE_DONE;
        calibration_wait_tick = 0;
      }
      break;
    }

  case CALIB_STATE_DONE:
    calibration_wait_tick++;
    if (calibration_wait_tick == 100)
    {
      for (int i = 0; i < STEER_NUM; i++)
      {
        Motor_Steer[i].Set_Control_Torque_P_D_MIT(0.0f, 1.0f, 0.0f);
        steer_calibration_target[i] = 0.0f;
        steer_ref_filtered[i] = 0.0f; // 清除滤波器状态
        steer_filter_init[i] = 0; // 重新触发滤波器初始化
      }
    }

    else if (calibration_wait_tick == 200)
    {
      for (int i = 0; i < STEER_NUM; i++) Motor_Steer[i].CAN_Send_Exit();

    }

    else if (calibration_wait_tick == 500)
    {
      for (int i = 0; i < STEER_NUM; i++) Motor_Steer[i].CAN_Send_Save_Zero();

    }

    else if (calibration_wait_tick == 800)
    {
      Target_Heading = 0.0f;
      all_calibration_complete = 1;
      return 1;
    }

    break;
  }
  return 0;
}

/**
 * @brief 获取当前舵向角度（直接读取电机驱动层的 Wheel_Rad）
 * @note  Wheel_Rad 由 Data_Process() 在电机驱动层完成 Now_Rad / REDUCTION_RATIO
 *        并归一化到 [-PI, PI]；此处再统一映射到 [0, 2PI] 与内部坐标系保持一致。
 *        Steer_To_Motor_Position() 仍使用
 * Get_Now_Radian()（电机轴），无需改动。
 */
float Class_Chassis::Get_Now_Steer_Radian(int index)
{
  float real_wheel_rad = Motor_Steer[index].Get_Now_Radian() / REDUCTION_RATIO;
  return Math_Modulus_Normalization(real_wheel_rad ,2.0f * PI);
}

/**
 * @brief 将舵向目标角度转换为达妙电机位置指令（最短路径）
 */
float Class_Chassis::Steer_To_Motor_Position(float target_steer, int index)
{
  float current_steer = Get_Now_Steer_Radian(index);
  float error = target_steer - current_steer;
  if (error > PI) error -= 2.0f * PI;
  if (error < -PI) error += 2.0f * PI;

  float motor_target =
    Motor_Steer[index].Get_Now_Radian() + error * REDUCTION_RATIO;

  return motor_target;
}

// bool Class_Chassis::Is_Steer_Angle_Reached(int index) {
//   float current = Get_Now_Steer_Radian(index);
//   float error =
//       Math_Modulus_Normalization(Target_Steer_Rad[index] - current, 2.0f * PI);
//   if (error > PI) error -= 2.0f * PI;
//   return fabsf(error) < angle_tolerance;
// }
//
// bool Class_Chassis::All_Steer_Angle_Reached() {
//   for (int i = 0; i < STEER_NUM; i++)
//     if (!Is_Steer_Angle_Reached(i)) return false;
//   return true;
// }

/**
 * @brief 整车速度正解算
 * @note  每周期先清零，再从各轮积分
 */
void Class_Chassis::Self_Resolution()
{
  Now_Velocity_X = 0.0f;
  Now_Velocity_Y = 0.0f;
  Now_Omega = 0.0f;

  for (int i = 0; i < STEER_NUM; i++)
  {
    float omega = ODrive_Motor_Steer[i].Get_Omega();
    float enc_rad = Encoder_Steer[i].Get_Wheel_Posture_radian();

    Now_Velocity_X += omega * Wheel_Radius * cosf(enc_rad) / 4.0f;
    Now_Velocity_Y += omega * Wheel_Radius * sinf(enc_rad) / 4.0f;
    Now_Omega += omega * Wheel_Radius *
      sinf(enc_rad - Wheel_Azimuth[i]) /
      (Wheel_To_Core_Distance[i] * 4.0f);
  }
}

/**
 * @brief 运动学逆解算：由目标速度计算各轮舵向目标角和轮速目标
 */
void Class_Chassis::Kinematics_Inverse_Resolution()
{
  // 全局死区
  float chassis_input_mod = sqrtf(Target_Velocity_X * Target_Velocity_X +
    Target_Velocity_Y * Target_Velocity_Y +
    Target_Omega * Target_Omega);
  if (chassis_input_mod < 0.02f)
  {
    for (int i = 0; i < STEER_NUM; i++) Target_Wheel_Omega[i] = 0.0f;
    return;
  }

  // 平移速度限幅
  float chassis_speed = sqrtf(Target_Velocity_X * Target_Velocity_X +
    Target_Velocity_Y * Target_Velocity_Y);
  if (chassis_speed > MAX_CHASSIS_SPEED)
  {
    float scale = MAX_CHASSIS_SPEED / chassis_speed;
    Target_Velocity_X *= scale;
    Target_Velocity_Y *= scale;
  }

  // 逆解算各轮
  float max_wheel_omega = 0.0f;
  for (int i = 0; i < STEER_NUM; i++)
  {
    float sin_phi = sinf(Wheel_Azimuth[i]);
    float cos_phi = cosf(Wheel_Azimuth[i]);
    if (fabsf(sin_phi) < 0.001f) sin_phi = 0.001f;
    if (fabsf(cos_phi) < 0.001f) cos_phi = 0.001f;

    float vx =
      Target_Velocity_X - Target_Omega * Wheel_To_Core_Distance[i] * sin_phi;
    float vy =
      Target_Velocity_Y + Target_Omega * Wheel_To_Core_Distance[i] * cos_phi;
    float v_mod = sqrtf(vx * vx + vy * vy);

    if (v_mod < 0.015f)
    {
      Target_Wheel_Omega[i] = 0.0f;
      continue;
    }

    Target_Steer_Rad[i] = atan2f(vy, vx);
    Target_Wheel_Omega[i] = v_mod / Wheel_Radius;
    if (Target_Wheel_Omega[i] > max_wheel_omega)
      max_wheel_omega = Target_Wheel_Omega[i];
  }

  // 轮速归一化（在全部轮子解算完成后统一限幅）
  const float MAX_WHEEL_OMEGA = MAX_CHASSIS_SPEED / Wheel_Radius;
  if (max_wheel_omega > MAX_WHEEL_OMEGA && max_wheel_omega > 1e-4f)
  {
    float scale = MAX_WHEEL_OMEGA / max_wheel_omega;
    for (int i = 0; i < STEER_NUM; i++) Target_Wheel_Omega[i] *= scale;
  }

  _Steer_Motor_Kinematics_Nearest_Transposition();

  // 舵向 PD 控制
  static float last_error[STEER_NUM] = {0};
  for (int i = 0; i < STEER_NUM; i++)
  {
    float current = Get_Now_Steer_Radian(i);
    float error =
      Math_Modulus_Normalization(Target_Steer_Rad[i] - current, 2.0f * PI);
    if (error > PI) error -= 2.0f * PI;

    if (fabsf(error) < angle_tolerance)
    {
      Target_Steer_Omage[i] = 0.0f;
      last_error[i] = error;
      continue;
    }

    float p_term = steer_wheel_params_[i].steer_kp * error;
    float d_term =
      steer_wheel_params_[i].steer_kd * (error - last_error[i]) / 0.002f;
    last_error[i] = error;

    float tmp_omega = p_term + d_term;
    /* min_omega / MAX_STEER_OMEGA 均为舵向侧 rad/s（输出轴）；
     * 大误差时的最小角速度同样保持在舵向侧单位（3.0 motor rad/s → ÷
     * REDUCTION_RATIO） */
    float min_omega = steer_wheel_params_[i].steer_omega_deadzone;
    if (fabsf(error) > 0.3f) min_omega = 3.0f / REDUCTION_RATIO;

    if (fabsf(tmp_omega) < min_omega)
      tmp_omega = (tmp_omega >= 0.0f ? 1.0f : -1.0f) * min_omega;
    if (tmp_omega > MAX_STEER_OMEGA) tmp_omega = MAX_STEER_OMEGA;
    if (tmp_omega < -MAX_STEER_OMEGA) tmp_omega = -MAX_STEER_OMEGA;

    Target_Steer_Omage[i] = tmp_omega;
  }

  // 舵向目标低通滤波
  for (int i = 0; i < STEER_NUM; i++)
  {
    if (!steer_filter_init[i])
    {
      steer_ref_filtered[i] = Target_Steer_Rad[i];
      steer_filter_init[i] = 1;
    }
    else
    {
      float err = Math_Modulus_Normalization(
        Target_Steer_Rad[i] - steer_ref_filtered[i], 2.0f * PI);
      if (err > PI) err -= 2.0f * PI;

      float alpha;
      if (fabsf(err) > 0.5f)
        alpha = 0.5f;
      else if (fabsf(err) > 0.2f)
        alpha = 0.3f;
      else
        alpha = 0.15f;

      steer_ref_filtered[i] += alpha * err;
      steer_ref_filtered[i] =
        Math_Modulus_Normalization(steer_ref_filtered[i], 2.0f * PI);
    }
    Target_Steer_Rad[i] = steer_ref_filtered[i];
  }
}

/**
 * @brief 就近转位解算：判断是否翻轮（反向取近路）
 *
 * @note  修复：旋转主导时禁用翻轮。
 *        在纯旋转或旋转成分大于平移时，舵向需持续跟踪切向方向，
 *        此时翻轮+轮速取反会导致底盘反向旋转。
 */
void Class_Chassis::_Steer_Motor_Kinematics_Nearest_Transposition()
{
  float translation_speed = sqrtf(Target_Velocity_X * Target_Velocity_X +
    Target_Velocity_Y * Target_Velocity_Y);
  // 旋转主导标志：有旋转指令 且 旋转量大于平移量
  bool rotation_dominant =
    (fabsf(Target_Omega) > 0.1f) && (fabsf(Target_Omega) > translation_speed);

  for (int i = 0; i < STEER_NUM; i++)
  {
    float current_rad = Get_Now_Steer_Radian(i);
    float target_rad =
      Math_Modulus_Normalization(Target_Steer_Rad[i], 2.0f * PI);
    float delta =
      Math_Modulus_Normalization(target_rad - current_rad, 2.0f * PI);
    if (delta > PI) delta -= 2.0f * PI;

    float omega = fabsf(Target_Wheel_Omega[i]);

    // 轮速极低：冻结舵向
    if (omega < steer_wheel_params_[i].wheel_omega_deadzone)
    {
      Target_Steer_Rad[i] = current_rad;
      steer_flipped[i] = 0;
      wheel_speed_scale[i] = 1.0f;
      continue;
    }

    // 翻轮条件：误差>90°、速度安全、未被锁定，且不处于旋转主导状态
    bool need_flip =
      fabsf(delta) > (PI / 2.0f + steer_wheel_params_[i].steer_rad_deadzone);
    bool allow_flip = omega < 10.0f;

    // 翻轮迟滞解锁
    if (steer_flipped[i] &&
      fabsf(delta) <
      (PI / 2.0f - steer_wheel_params_[i].steer_rad_deadzone))
    {
      steer_flipped[i] = 0;
    }

    if (need_flip && allow_flip && !steer_flipped[i] && !rotation_dominant)
    {
      steer_flipped[i] = 1;
    }

    if (steer_flipped[i])
    {
      target_rad -= PI;
      Target_Wheel_Omega[i] *= -1.0f;
    }

    Target_Steer_Rad[i] = Math_Modulus_Normalization(target_rad, 2.0f * PI);

    // 动态轮速缩放（舵向误差越大，轮速越低）
    float final_err = fabsf(Math_Modulus_Normalization(
      Target_Steer_Rad[i] - current_rad, 2.0f * PI));
    if (final_err > PI) final_err = fabsf(final_err - 2.0f * PI);

    if (final_err > (120.0f * PI / 180.0f))
      wheel_speed_scale[i] = 0.5f;
    else if (final_err > (90.0f * PI / 180.0f))
      wheel_speed_scale[i] = 0.7f;
    else if (final_err > 1.0f)
      wheel_speed_scale[i] = 0.8f;
    else
      wheel_speed_scale[i] = 1.0f;

    Target_Wheel_Omega[i] *= wheel_speed_scale[i];
  }
}

/**
 * @brief 动力学逆解算：叠加加速度前馈和阻尼力矩
 */
void Class_Chassis::Dynamics_Inverse_Resolution()
{
  static float last_vx = 0.0f, last_vy = 0.0f, last_omega = 0.0f;
  const float dt = 0.002f;
  const float MAX_ACCEL = 40.0f;
  static float last_steer_omega[STEER_NUM] = {0};

  // 1. 加速度斜坡处理：防止瞬时速度跳变导致力矩爆表
  float raw_ax = (Target_Velocity_X - last_vx) / dt;
  float raw_ay = (Target_Velocity_Y - last_vy) / dt;
  float raw_alpha = (Target_Omega - last_omega) / dt;

  // 对加速度进行限幅，保证启动平滑
  Math_Constrain(&raw_ax, -MAX_ACCEL, MAX_ACCEL);
  Math_Constrain(&raw_ay, -MAX_ACCEL, MAX_ACCEL);
  Math_Constrain(&raw_alpha, -5.0f, 5.0f); // 角加速度限幅
  float ax = raw_ax;
  float ay = raw_ay;
  float alpha = raw_alpha;

  last_vx = Target_Velocity_X;
  last_vy = Target_Velocity_Y;
  last_omega = Target_Omega;

  // 2. 计算理想状态下的系统总力/力矩
  float force_x = Class_Mass * ax;
  float force_y = Class_Mass * ay;
  float torque_omega = Class_Inertia * alpha;

  // 1. 驱动轴（Wheel）动力学
  for (int i = 0; i < STEER_NUM; i++)
  {
    // 获取当前轮子的物理状态
    float current_steer_rad = Encoder_Steer[i].Get_Wheel_Posture_radian();
    float target_steer_rad = Target_Steer_Rad[i]; // 逆解出的目标舵角

    // 3. 计算舵向对准权重 (Confidence)
    // 使用余弦值作为权重：误差 0° 权重为 1；误差 90° 权重为 0
    float steer_err = fabsf(Math_Modulus_Normalization(
      target_steer_rad - current_steer_rad, 2.0f * PI));
    float confidence = cosf(steer_err);
    if (confidence < 0.0f) confidence = 0.0f; // 角度偏差超过 90 度时不给动力

    // 4. 计算该轮子在当前角度下应分担的切向力
    // 这一步是核心：将整体合力投影到轮子当前的滚动方向上
    float tmp_force = force_x * cosf(current_steer_rad) +
      force_y * sinf(current_steer_rad) +
      (torque_omega / Wheel_To_Core_Distance[i]) *
      sinf(Wheel_Azimuth[i] - current_steer_rad);

    // 5. 合成最终前馈力矩
    // a. 惯性力矩 (F=ma)
    float torque_static = tmp_force * Wheel_Radius;

    // b. 摩擦力补偿 (克服系统静摩擦和滚动摩擦)
    float omega_tgt = Target_Wheel_Omega[i];
    float tau_fric = Dynamic_Resistance_Wheel_Current[i] *
      tanhf(omega_tgt / Wheel_Resistance_Omega_Threshold);

    Target_Wheel_torque[i] = (torque_static + tau_fric) * confidence;
  }

  // 2. 舵向轴（Steer）动力学解算
  for (int i = 0; i < STEER_NUM; i++)
  {
    // 计算舵向角加速度 (rad/s^2)
    float steer_alpha = (Target_Steer_Omage[i] - last_steer_omega[i]) / dt;

    // 限制加速度斜坡，防止指令跳变导致电流过大
    // Math_Constrain(&steer_alpha, -200.0f, 200.0f);

    // 动力学方程：T = J*alpha + B*omega + T_static*sgn(omega)
    // Steer_Inertia (J): 舵向转动惯量
    // Steer_Damping (B): 阻尼系数
    // Steer_Static_Friction: 静摩擦力矩
    float torque_ff =
      Steer_Inertia * steer_alpha + Steer_Damping * Target_Steer_Omage[i] +
      Steer_Static_Friction * tanhf(Target_Steer_Omage[i] / 0.1f);

    Target_Steer_Torque[i] = torque_ff;
    last_steer_omega[i] = Target_Steer_Omage[i];
  }
}

/**
 * @brief 状态机：判断舵轮状态切换
 */
void Class_Chassis::Update_Steer_State(int i)
{
  float current = Get_Now_Steer_Radian(i);
  float delta =
    Math_Modulus_Normalization(Target_Steer_Rad[i] - current, 2.0f * PI);
  if (delta > PI) delta -= 2.0f * PI;

  float omega_thresh = steer_wheel_params_[i].wheel_omega_deadzone;

  switch (steer_state[i])
  {
  case STEER_STATE_IDLE:
    if (fabsf(Target_Wheel_Omega[i]) > omega_thresh)
      steer_state[i] = STEER_STATE_ALIGN;
    break;

  case STEER_STATE_ALIGN:
    if (fabsf(delta) < 0.15f)
      steer_state[i] = STEER_STATE_DRIVE;
    else if (fabsf(Target_Wheel_Omega[i]) < omega_thresh)
      steer_state[i] = STEER_STATE_IDLE;
    break;

  case STEER_STATE_DRIVE:
    if (fabsf(delta) > 0.15f && fabsf(Target_Wheel_Omega[i]) > omega_thresh)
      steer_state[i] = STEER_STATE_ALIGN;
    else if (fabsf(Target_Wheel_Omega[i]) < omega_thresh)
      steer_state[i] = STEER_STATE_IDLE;
    break;

  default:
    steer_state[i] = STEER_STATE_IDLE;
    break;
  }
}

/**
 * @brief 状态机：根据当前状态执行电机指令
 */
void Class_Chassis::Execute_Steer_State(int i)
{
  float motor_pos_continuous = Steer_To_Motor_Position(Target_Steer_Rad[i], i);
  float dir = steer_wheel_params_[i].wheel_direction;

  // 2. 核心修正：指令归一化
  // 将 motor_pos 映射到电机能识别的 [-Radian_Max, Radian_Max] 范围内
  // Radian_Max 对 DM2325 通常是 21.92f
  float motor_limit = 21.92f;
  float range = 2.0f * motor_limit;
  float motor_pos_cmd = motor_pos_continuous;

  motor_pos_cmd = fmodf(motor_pos_cmd + motor_limit, range);
  if (motor_pos_cmd < 0) motor_pos_cmd += range;
  motor_pos_cmd -= motor_limit;

  switch (steer_state[i])
  {
  case STEER_STATE_IDLE:
    /* 位置使用电机轴角度（Steer_To_Motor_Position 内部已完成舵→电机换算）
     * omega = 0，无需换算 */
    Motor_Steer[i].Set_Control_Torque_P_D_MIT(0.0f,
      steer_wheel_params_[i].steer_kp, steer_wheel_params_[i].steer_kd);
    Motor_Steer[i].Set_Control_Parameter_MIT(motor_pos_cmd, 0.0f);
    ODrive_Motor_Steer[i].Set_target_omega(0.0f);
    ODrive_Motor_Steer[i].Set_target_torque(0.0f);
    break;

  case STEER_STATE_ALIGN:
    {
      /* Target_Steer_Omage[i]：舵向侧 rad/s → 电机轴 rad/s = × REDUCTION_RATIO
       */
      Motor_Steer[i].Set_Control_Torque_P_D_MIT(
        Target_Steer_Torque[i],
        steer_wheel_params_[i].steer_kp, steer_wheel_params_[i].steer_kd);
      Motor_Steer[i].Set_Control_Parameter_MIT(
        motor_pos_cmd, Target_Steer_Omage[i] * REDUCTION_RATIO);

      float current = Get_Now_Steer_Radian(i);
      float err =  Math_Modulus_Normalization(Target_Steer_Rad[i] - current, 2.0f * PI);
      float proj = cosf(err);
      if (proj < 0.0f) proj = 0.0f;

      ODrive_Motor_Steer[i].Set_target_omega(Target_Wheel_Omega[i] * proj);
      ODrive_Motor_Steer[i].Set_target_torque(0.0f);
      break;
    }

  case STEER_STATE_DRIVE:

    Motor_Steer[i].Set_Control_Torque_P_D_MIT(
      Target_Steer_Torque[i], steer_wheel_params_[i].steer_kp,
      steer_wheel_params_[i].steer_kd);
    Motor_Steer[i].Set_Control_Parameter_MIT(
      motor_pos_cmd, Target_Steer_Omage[i] * REDUCTION_RATIO);
    ODrive_Motor_Steer[i].Set_target_omega(Target_Wheel_Omega[i] * dir);
    ODrive_Motor_Steer[i].Set_target_torque(Target_Wheel_torque[i] * dir);
    break;
  }
}

/**
 * @brief 输出到电机
 */
void Class_Chassis::Output_To_Motor()
{
  for (int i = 0; i < STEER_NUM; i++)
  {
    Update_Steer_State(i);
    Execute_Steer_State(i);
  }

  for (int i = 0; i < STEER_NUM; i++)
    Motor_Steer[i].TIM_Send_PeriodElapsedCallback();
  for (int i = 0; i < STEER_NUM; i++)
    ODrive_Motor_Steer[i].TIM_Send_PeriodElapsedCallback();
}

/**
 * @brief 航向锁定 PID 补偿（平移时保持航向）
 */
void Class_Chassis::Apply_Heading_Correction(float& omega, float vx, float vy,
                                             float yaw)
{
  bool moving = (Math_Abs(vx) > 0.01f || Math_Abs(vy) > 0.01f);
  bool no_spin = (Math_Abs(omega) < 0.005f);

  if (no_spin && moving)
  {
    if (!Heading_Lock_Flag)
    {
      Target_Heading = yaw;
      Heading_Lock_Flag = true;
    }
    float error = Target_Heading - yaw;
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    PID_Heading.Set_Now(yaw);
    PID_Heading.Set_Target(yaw + error);
    PID_Heading.TIM_Calculate_PeriodElapsedCallback();
    omega = PID_Heading.Get_Out();
  }
  else
  {
    Heading_Lock_Flag = false;
    PID_Heading.Set_Integral_Error(0.0f);
  }
}
