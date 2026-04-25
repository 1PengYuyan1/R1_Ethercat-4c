//
// Created by pzx on 2025/12/20.
//
#include "crt_chassis.h"
#include "math.h"
#include "cmath"

/**
 * @brief 初始化硬件
 */
void Class_Chassis::Init(linkx_t *__LinkX_Handler)
{
    LinkX_Handler = __LinkX_Handler;

    Motor_Steer[0].Init(LinkX_Handler, 0, 0x11, 0x01, Motor_DM_Control_Method_NORMAL_MIT, 21.92f, 150.0f, 5.0f, 10.0f);
    Motor_Steer[1].Init(LinkX_Handler, 0, 0x12, 0x02, Motor_DM_Control_Method_NORMAL_MIT, 21.92f, 150.0f, 5.0f, 10.0f);
    Motor_Steer[2].Init(LinkX_Handler, 0, 0x13, 0x03, Motor_DM_Control_Method_NORMAL_MIT, 21.92f, 150.0f, 5.0f, 10.0f);
    Motor_Steer[3].Init(LinkX_Handler, 0, 0x14, 0x04, Motor_DM_Control_Method_NORMAL_MIT, 21.92f, 150.0f, 5.0f, 10.0f);

    // 2. 补全编码器 Init 参数 (添加第4个参数：单圈分辨率，通常是 4096)
    Encoder_Steer[0].Init(LinkX_Handler, 1, 0x05, 4096);
    Encoder_Steer[1].Init(LinkX_Handler, 1, 0x06, 4096);
    Encoder_Steer[2].Init(LinkX_Handler, 1, 0x07, 4096);
    Encoder_Steer[3].Init(LinkX_Handler, 1, 0x08, 4096);

    ODrive_Motor_Steer[0].Init(LinkX_Handler, 2, 0x01);
    ODrive_Motor_Steer[1].Init(LinkX_Handler, 2, 0x02);
    ODrive_Motor_Steer[2].Init(LinkX_Handler, 2, 0x03);
    ODrive_Motor_Steer[3].Init(LinkX_Handler, 2, 0x04);
}

/**
 * @brief 初始化参数
 */
void Class_Chassis::Init_Motor_Params()
{
    {
        steer_wheel_params_[0] = {
            .steer_kp = 0.4f, // 舵向角速度P增益
            .steer_kd = 0.3f, // 舵向角速度D增益

            .steer_omega_deadzone = 1.2f,                // 舵向角速度死区
            .steer_rad_deadzone = (2.86f * PI / 180.0f), // 舵向角度死区

            .wheel_omega_deadzone = 0.05f, // 轮向最大角速度
            .wheel_feedforward = 0.5,      // 轮向前馈系数

            .wheel_direction = 1, // 轮向方向(+1或-1)

            .flip_speed_threshold = 2.0f, // 翻轮速度阈值
            .flip_drive_scale = 0.4       // 翻轮期间速度缩放
        };

        steer_wheel_params_[1] = {
            .steer_kp = 0.4f, // 舵向角速度P增益
            .steer_kd = 0.3f, // 舵向角速度D增益

            .steer_omega_deadzone = 1.2f,                // 舵向角速度死区
            .steer_rad_deadzone = (2.86f * PI / 180.0f), // 舵向角度死区

            .wheel_omega_deadzone = 0.05f, // 轮向最大角速度
            .wheel_feedforward = 0.5,      // 轮向前馈系数

            .wheel_direction = 1, // 轮向方向(+1或-1)

            .flip_speed_threshold = 2.0f, // 翻轮速度阈值
            .flip_drive_scale = 0.4       // 翻轮期间速度缩放

        };

        steer_wheel_params_[2] = {
            .steer_kp = 0.4f, // 舵向角速度P增益
            .steer_kd = 0.3f, // 舵向角速度D增益

            .steer_omega_deadzone = 1.2f,                // 舵向角速度死区
            .steer_rad_deadzone = (2.86f * PI / 180.0f), // 舵向角度死区

            .wheel_omega_deadzone = 0.05f, // 轮向最大角速度
            .wheel_feedforward = 0.5,      // 轮向前馈系数

            .wheel_direction = 1, // 轮向方向(+1或-1)

            .flip_speed_threshold = 2.0f, // 翻轮速度阈值
            .flip_drive_scale = 0.4       // 翻轮期间速度缩放

        };

        steer_wheel_params_[3] = {
            .steer_kp = 0.4f, // 舵向角速度P增益
            .steer_kd = 0.3f, // 舵向角速度D增益

            .steer_omega_deadzone = 1.2f,                // 舵向角速度死区
            .steer_rad_deadzone = (2.86f * PI / 180.0f), // 舵向角度死区

            .wheel_omega_deadzone = 0.05f, // 轮向最大角速度
            .wheel_feedforward = 0.5,      // 轮向前馈系数

            .wheel_direction = 1, // 轮向方向(+1或-1)

            .flip_speed_threshold = 2.0f, // 翻轮速度阈值
            .flip_drive_scale = 0.4       // 翻轮期间速度缩放

        };
    }
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Chassis::TIM_100ms_Alive_PeriodElapsedCallback()
{
    for (int i = 0; i < STEER_NUM; i++)
    {
        Motor_Steer[i].TIM_Alive_PeriodElapsedCallback();
        ODrive_Motor_Steer[i].TIM_Alive_CheckCallback();
    }

    if (Chassis_Control_Type == Chassis_Control_Type_ENABLE)
    {
        for (int i = 0; i < STEER_NUM; i++)
        {
            // --- 达妙电机检查 (保持不变) ---
            if (Motor_Steer[i].Get_Status() != Motor_DM_Control_Status_ENABLE)
            {
                Motor_Steer[i].CAN_Send_Enter();
            }

            // --- ODrive 逻辑修复 ---
            // 获取当前 ODrive 的状态和错误
            uint32_t current_state = ODrive_Motor_Steer[i].Get_Axis_State();
            uint32_t current_error = ODrive_Motor_Steer[i].Get_Axis_Error();

            //  如果有错误 (Error != 0)
            if (current_error != AXIS_ERROR_NONE)
            {
                ODrive_Motor_Steer[i].Clear_Errors();
                continue;
            }
            // 没有错误，但状态不对 (比如变成了 IDLE)
            if (current_state != ODRIVE_STATE_CLOSED_LOOP_CONTROL)
            {
                ODrive_Motor_Steer[i].SET_ClosedLoop();
            }
        }
    }
}

/**
 * @brief TIM定时器中断自身姿态，速度解算回调函数
 *
 */
void Class_Chassis::TIM_2ms_Resolution_PeriodElapsedCallback()
{
    Self_Resolution();
}

/**
 * @brief TIM定时器中断底盘控制回调函数
 *
 */
void Class_Chassis::TIM_2ms_Control_PeriodElapsedCallback()
{
    // 如果校准未完成,优先执行校准
    if (!Is_Calibration_Complete())
    {
        for (int i = 0; i < STEER_NUM; i++)
        {
            if (Motor_Steer[i].Get_Status() != Motor_DM_Control_Status_ENABLE)
            {
                Motor_Steer[i].CAN_Send_Enter();
            }
        }
        Steer_Calibration_Process();

        // 3. 发送CAN指令 (统一在这里发送!)
        for (int i = 0; i < STEER_NUM; i++)
        {
            Motor_Steer[i].TIM_Send_PeriodElapsedCallback();
        }

        return;
    }

    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        // --- 失能状态处理 ---

        // 1. 停止所有电机输出
        for (int i = 0; i < STEER_NUM; i++)
        {
            // 舵向电机失能
            Motor_Steer[i].Set_Control_Status(Motor_DM_Status_DISABLE);
            Motor_Steer[i].Set_Control_Parameter_MIT(0.0f, 0.0f); // 安全起见清零
            if (Motor_Steer[i].Get_Now_Control_Status() != Motor_DM_Status_DISABLE)
                Motor_Steer[i].CAN_Send_Exit();

            // 轮向电机进入空闲模式 (比给0速度更安全)
            uint32_t state = ODrive_Motor_Steer[i].Get_Axis_State();

            if (state != ODRIVE_STATE_IDLE)
            {
                ODrive_Motor_Steer[i].Emergency_Stop();
                ODrive_Motor_Steer[i].Set_Velocity(0.0f);
                ODrive_Motor_Steer[i].Set_Torque(0.0f);
            }

            // 2. 清除解算过程量 (可选，防止切回使能瞬间猛冲)
            Target_Velocity_X = 0.0f;
            Target_Velocity_Y = 0.0f;
            Target_Omega = 0.0f;

            for (int i = 0; i < STEER_NUM; i++)
            {
                Target_Wheel_Omega[i] = 0.0f;
                Target_Wheel_torque[i] = 0.0f;
            }

            // // 失能状态下，直接返回，不进行后续解算和输出
            // return;
        }
        // 失能状态下，直接返回，不进行后续解算和输出
        return;
    case (Chassis_Control_Type_ENABLE):
    {
        Kinematics_Inverse_Resolution();

        // Output_To_Dynamics();

        Dynamics_Inverse_Resolution();

        Output_To_Motor();
    }
    break;
    }
    }
}

/**
 * @brief 获取校准状态
 * @return 1-已完成校准, 0-未完成
 */
uint8_t Class_Chassis::Is_Calibration_Complete()
{
    return all_calibration_complete;
}

/**
 * @brief 强制重新校准
 * @note 用于需要重新校准的场景
 */
void Class_Chassis::Force_Recalibration()
{
    all_calibration_complete = 0;
    calibration_wait_tick = 0;
    is_calib_target_calculated = false;
}

/**
 * @brief 获取单个轮子的校准状态
 * @param index 轮子索引
 * @return 1-已完成, 0-未完成
 */
uint8_t Class_Chassis::Is_Wheel_Calibration_Complete(int index)
{
    if (index >= 0 && index < STEER_NUM)
    {
        return steer_calibration_done[index];
    }
    return 0;
}

/**
 * @brief 舵向角度校准初始化
 * @note 在Reset或系统初始化时调用,仅运行一次
 */
void Class_Chassis::Steer_Calibration_Init()
{
    for (int i = 0; i < STEER_NUM; i++)
    {
        Motor_Steer[i].Set_Control_Torque_P_D_MIT(0.0f, 4.0, 0.2f); // 参数可根据实际情况微调
    }
    calib_step = CALIB_STATE_WAIT_STABLE; // <-- 添加这句
    // 2. 关键：初始化状态机变量
    all_calibration_complete = 0;
    calibration_wait_tick = 0;          // 计时器归零
    is_calib_target_calculated = false; // 标记“还没计算过目标”

    // 清空校准状态
    for (int i = 0; i < STEER_NUM; i++)
    {
        steer_calibration_done[i] = 0;
        steer_calibration_target[i] = 0.0f;
    }
}

/**
 * @brief 启动舵向角度校准
 * @note 读取绝对编码器,计算校准目标位置
 */
void Class_Chassis::Steer_Calibration_Start()
{
    for (int i = 0; i < STEER_NUM; i++)
    {
        // 读取绝对编码器的当前角度
        // 使用 Encoder_Steer[i].Get_Wheel_Posture_Radian() 获取已处理好的舵向角度[0, 2π]
        float encoder_angle_rad = Encoder_Steer[i].Get_Wheel_Posture_radian();
        // 读取达妙电机当前反馈的电机轴角度
        float motor_raw_angle = Motor_Steer[i].Get_Now_Radian();
        // 计算舵向实际角度(电机角度/减速比)
        float motor_steer_angle = motor_raw_angle / REDUCTION_RATIO;
        // 计算到0点的最短路径偏移量
        float angle_to_zero;

        // 编码器在 [0, π] 区间,逆时针旋转到0更近
        if (encoder_angle_rad <= PI)
            angle_to_zero = -encoder_angle_rad;
        // 编码器在 (π, 2π] 区间,顺时针旋转到0(即2π)更近
        else
            angle_to_zero = 2.0f * PI - encoder_angle_rad;

        // 目标舵向角度 = 当前舵向角度 + 到零点的偏移
        float target_steer_angle = motor_steer_angle + angle_to_zero;
        // 将舵向目标转换为电机目标(乘以减速比)
        float calibration_motor_target = target_steer_angle * REDUCTION_RATIO;

        // 保存校准目标位置
        steer_calibration_target[i] = calibration_motor_target;
        // 标记该轮未完成校准
        steer_calibration_done[i] = 0;
    }
}

/**
 * @brief 舵向校准执行函数
 * @note 在控制周期中循环调用,直到所有轮子校准完成
 * @return 1-校准完成, 0-校准进行中
 */
uint8_t Class_Chassis::Steer_Calibration_Process()
{
    if (all_calibration_complete)
    {
        // Buzzer_Play_Confirm();
        return 1;
    }
    switch (calib_step)
    {
    case CALIB_STATE_WAIT_STABLE:
        // 1. 持续发送编码器请求指令
        // Send_Encoder_CAN_Frame();
        for (int i = 0; i < STEER_NUM; i++)
        {
            Encoder_Steer[i].TIM_Query_PeriodElapsedCallback();
        }
        // 2. 等待一段时间（比如 1s/500次），确保 CAN 中断有足够时间填入数据
        calibration_wait_tick++;
        if (calibration_wait_tick > 500)
        {
            // 3. 检查数据是否真的来了（判断不为 0）
            uint8_t data_ready = 1;
            for (int i = 0; i < STEER_NUM; i++)
            {
                if (Encoder_Steer[i].Get_Wheel_Posture_radian() == 0.0f)
                    data_ready = 0;
            }

            if (data_ready)
                calib_step = CALIB_STATE_CALCULATE;
        }
        break;

    case CALIB_STATE_CALCULATE:
        // 只执行一次计算
        Steer_Calibration_Start();
        calib_step = CALIB_STATE_EXECUTING;
        break;

    case CALIB_STATE_EXECUTING:
    {
        uint8_t all_done = 1;
        for (int i = 0; i < STEER_NUM; i++)
        {
            if (steer_calibration_done[i])
            {

                Motor_Steer[i].Set_Control_Parameter_MIT(0.0f, 0.0f);
                continue;
            }
            float current_motor_angle = Motor_Steer[i].Get_Now_Radian();
            // 误差计算
            float angle_error = Math_Modulus_Normalization(steer_calibration_target[i] - current_motor_angle,
                                                           2.0f * PI);
            if (angle_error > PI)
                angle_error -= 2.0f * PI;

            // MIT 模式平稳驱动 (不要在里面加 HAL_Delay!)
            Motor_Steer[i].Set_Control_Parameter_MIT(steer_calibration_target[i], 0.0f);

            if (fabsf(angle_error) < 0.5f)
                steer_calibration_done[i] = 1;
            else
                all_done = 0;
        }

        if (all_done)
        {
            for (int i = 0; i < STEER_NUM; i++)
                Motor_Steer[i].CAN_Send_Save_Zero();
            calib_step = CALIB_STATE_DONE;
        }
        break;
    }

    case CALIB_STATE_DONE:
        all_calibration_complete = 1;
        return 1;
    }
    return 0;
}

/**
 * @brief 获取当前舵向角度 (通过达妙反馈 + 减速比换算)
 *
 * @param index 电机索引
 * @return 舵向角度 [0, 2π]
 */
float Class_Chassis::Get_Now_Steer_Radian(int index)
{
    // 达妙反馈 (电机轴角度)
    float motor_rad = Motor_Steer[index].Get_Now_Radian();

    // 除以减速比得到舵向角度
    float steer_rad = motor_rad / REDUCTION_RATIO;

    // 归一化到 [0, 2π]
    steer_rad = Math_Modulus_Normalization(steer_rad, 2.0f * PI);

    return steer_rad;
}

/**
 * @brief 舵向目标角度 → 达妙位置指令
 *
 * @param target_steer 舵向目标 [0, 2π]
 * @param index 电机索引
 * @return 达妙位置指令
 */
float Class_Chassis::Steer_To_Motor_Position(float target_steer, int index)
{
    // 当前舵向角度
    float current_steer = Get_Now_Steer_Radian(index);

    // 计算最短路径误差
    float error = target_steer - current_steer;
    if (error > PI)
        error -= 2.0f * PI;
    if (error < -PI)
        error += 2.0f * PI;

    // 达妙目标 = 当前 + 误差 × 减速比
    float motor_current = Motor_Steer[index].Get_Now_Radian();
    float motor_target = motor_current + error * REDUCTION_RATIO;

    // 安全限幅
    if (motor_target > Motor_Steer[index].Get_Radian_Max())
        motor_target = Motor_Steer[index].Get_Radian_Max();
    if (motor_target < -Motor_Steer[index].Get_Radian_Max())
        motor_target = -Motor_Steer[index].Get_Radian_Max();

    return motor_target;
}

/**
 * @brief 自身姿态，速度解算
 *
 */
void Class_Chassis::Self_Resolution()
{
    // 计算整车速度

    for (int i = 0; i < STEER_NUM; i++)
    {
        Now_Velocity_X += (ODrive_Motor_Steer[i].Get_Omega() * Wheel_Radius * acoshf32(Encoder_Steer[i].Get_Wheel_Posture_radian()) / 4.0f);

        Now_Velocity_Y += (ODrive_Motor_Steer[i].Get_Omega() * Wheel_Radius * asinhf32(Encoder_Steer[i].Get_Wheel_Posture_radian()) / 4.0f);

        Now_Omega += (ODrive_Motor_Steer[i].Get_Omega() *
                      asinhf32(Encoder_Steer[i].Get_Wheel_Posture_radian() - Wheel_Azimuth[i]) *
                      Wheel_Radius / Wheel_To_Core_Distance[i]) /
                     4.0f;
    }
}

/**
 * @brief 运动学逆解算
 *
 */
void Class_Chassis::Kinematics_Inverse_Resolution()
{

    // 1. 计算整车合成速度
    float chassis_input_mod = sqrtf(Target_Velocity_X * Target_Velocity_X +
                                    Target_Velocity_Y * Target_Velocity_Y +
                                    Target_Omega * Target_Omega);

    // 2. 设置一个合理的死区（例如 0.02 m/s 或 rad/s）
    if (chassis_input_mod < 0.02f)
    {
        for (int i = 0; i < STEER_NUM; i++)
        {
            Target_Wheel_Omega[i] = 0.0f;
            // 角度保持不变，直接跳过本轮解算
        }
        return;
    }

    float chassis_speed = sqrtf(Target_Velocity_X * Target_Velocity_X +
                                Target_Velocity_Y * Target_Velocity_Y);

    if (chassis_speed > MAX_CHASSIS_SPEED)
    {
        float scale = MAX_CHASSIS_SPEED / chassis_speed;
        Target_Velocity_X *= scale;
        Target_Velocity_Y *= scale;
    }

    float max_wheel_omega = 0.0f;

    for (int i = 0; i < STEER_NUM; i++)
    {
        float tmp_velocity_x, tmp_velocity_y, tmp_velocity_modulus;

        // 添加边界保护
        float sin_theta = asinhf32(Wheel_Azimuth[i]);
        float cos_theta = acoshf32(Wheel_Azimuth[i]);
        // 防止除以0
        if (fabsf(sin_theta) < 0.001f)
            sin_theta = 0.001f;
        if (fabsf(cos_theta) < 0.001f)
            cos_theta = 0.001f;

        // 解算到每个轮组的具体线速度
        tmp_velocity_x = Target_Velocity_X - Target_Omega * Wheel_To_Core_Distance[i] * sin_theta;
        tmp_velocity_y = Target_Velocity_Y + Target_Omega * Wheel_To_Core_Distance[i] * cos_theta;
        tmp_velocity_modulus = sqrtf(tmp_velocity_x * tmp_velocity_x + tmp_velocity_y * tmp_velocity_y);

        float v_mod = sqrtf(tmp_velocity_x * tmp_velocity_x +
                            tmp_velocity_y * tmp_velocity_y);

        /* ---------- 低速保护 ---------- */
        if (v_mod < 1e-4f)
        {
            Target_Wheel_Omega[i] = 0.0f;
            // Target_Steer_Rad[i] = Get_Now_Steer_Radian(i);
            continue;
        }

        /* —— 舵向角：由期望速度方向决定 —— */
        float steer_theta = atan2f(tmp_velocity_y, tmp_velocity_x);
        Target_Steer_Rad[i] = steer_theta;

        /* ---------- 正常轮速（始终正转） ---------- */
        Target_Wheel_Omega[i] = tmp_velocity_modulus / Wheel_Radius;
        if (Target_Wheel_Omega[i] > max_wheel_omega)
            max_wheel_omega = Target_Wheel_Omega[i];

        /* ================= 3. 轮速二次归一化（关键！！！） ================= */
        const float MAX_WHEEL_OMEGA = MAX_CHASSIS_SPEED / Wheel_Radius;

        if (max_wheel_omega > MAX_WHEEL_OMEGA && max_wheel_omega > 1e-4f)
        {
            float scale = MAX_WHEEL_OMEGA / max_wheel_omega;
            for (int i = 0; i < STEER_NUM; i++)
            {
                Target_Wheel_Omega[i] *= scale;
            }
        }
    }

    _Steer_Motor_Kinematics_Nearest_Transposition();

    for (int i = 0; i < STEER_NUM; i++)
    {
        float current = Get_Now_Steer_Radian(i);
        float error = Math_Modulus_Normalization(Target_Steer_Rad[i] - current, 2.0f * PI);
        if (error > PI)
            error -= 2.0f * PI;

        /* ---- 死区 ---- */
        if (fabsf(error) < angle_tolerance)
        {
            Target_Steer_Omage[i] = 0.0f;
            continue;
        }

        /* ----  PD控制 (增加D项) ---- */
        static float last_error[STEER_NUM] = {0};
        float p_term = steer_wheel_params_[0].steer_kp * error;
        float d_term = steer_wheel_params_[0].steer_kd * (error - last_error[i]) / 0.002f; // 假设2ms周期
        float tmp_omega = p_term + d_term;
        last_error[i] = error;

        /* ---- 动态最小速度: 误差大时提高最小速度 ---- */
        float min_omega = steer_wheel_params_[i].steer_omega_deadzone;
        if (fabsf(error) > 0.3f)
            min_omega = 3.0f; // >17°时提高最小速度
        if (fabsf(tmp_omega) < min_omega)
            tmp_omega = (tmp_omega > 0 ? 1 : -1) * min_omega;

        /* ---- 最大角速度 ---- */
        if (tmp_omega > MAX_STEER_OMEGA)
            tmp_omega = MAX_STEER_OMEGA;
        if (tmp_omega < -MAX_STEER_OMEGA)
            tmp_omega = -MAX_STEER_OMEGA;

        Target_Steer_Omage[i] = tmp_omega;
    }
}

/**
 * @brief 舵向电机依照轮向电机目标角速度就近转位解算
 *
 */
void Class_Chassis::_Steer_Motor_Kinematics_Nearest_Transposition()
{
    for (int i = 0; i < STEER_NUM; i++)
    {
        float tmp_current_rad = Get_Now_Steer_Radian(i);
        float tmp_target_rad = Math_Modulus_Normalization(Target_Steer_Rad[i], 2.0f * PI);
        float tmp_delta_rad = Math_Modulus_Normalization(tmp_target_rad - tmp_current_rad, 2.0f * PI);
        if (tmp_delta_rad > PI)
            tmp_delta_rad -= 2.0f * PI;

        float tmp_omega = fabsf(Target_Wheel_Omega[i]);

        /* ---------- 1. 轮速极低：冻结舵向 ---------- */
        if (tmp_omega < steer_wheel_params_[i].wheel_omega_deadzone)
        {
            Target_Steer_Rad[i] = tmp_current_rad; // 锁死当前角度
            steer_flipped[i] = 0;                  // 状态复位
            wheel_speed_scale[i] = 1.0f;
            continue; // 速度太低直接跳过后续解算
        }

        /* ---------- 2. 翻轮逻辑判断 (加入迟滞) ---------- */
        // 触发翻转：原始误差 > 90度 + 死区
        bool rad_need_flip = fabsf(tmp_delta_rad) > (PI / 2.0f + steer_wheel_params_[i].steer_rad_deadzone);
        bool rad_allow_flip = tmp_omega < 10.0f; // 允许翻转的安全速度上限

        // 解锁翻转：原始误差回到 90度 - 死区 以内 (迟滞区间，防止频繁死锁震荡)
        if (steer_flipped[i] && fabsf(tmp_delta_rad) < (PI / 2.0f - steer_wheel_params_[i].steer_rad_deadzone))
        {
            steer_flipped[i] = 0;
        }

        // 满足条件则进入翻转状态
        if (rad_need_flip && rad_allow_flip && !steer_flipped[i])
        {
            steer_flipped[i] = 1;
        }

        /* ---------- 3. 执行翻转与倒车 ---------- */
        if (steer_flipped[i])
        {
            tmp_target_rad -= PI;           // 舵向目标转180度
            Target_Wheel_Omega[i] *= -1.0f; // 关键：轮向速度反向(倒车)
        }

        // 统一归一化处理最终目标角度
        Target_Steer_Rad[i] = Math_Modulus_Normalization(tmp_target_rad, 2.0f * PI);

        /* ---------- 4. 动态速度缩放 (防猛冲) ---------- */
        // 计算翻转后(最终要去的方向)的真实误差
        float final_error = fabsf(Math_Modulus_Normalization(Target_Steer_Rad[i] - tmp_current_rad, 2.0f * PI));
        if (final_error > PI)
            final_error = fabsf(final_error - 2.0f * PI);

        // 误差过大时限制轮速
        if (final_error > (120.0f * PI / 180.0f))
            wheel_speed_scale[i] = 0.2f;
        else if (final_error > (100.0f * PI / 180.0f))
            wheel_speed_scale[i] = 0.4f;
        else if (final_error > 0.2f)
            wheel_speed_scale[i] = 0.6f;
        else
            wheel_speed_scale[i] = 1.0f;

        // 应用速度缩放
        Target_Wheel_Omega[i] *= wheel_speed_scale[i];

        /* ---------- 5. 舵向目标低通滤波 ---------- */
        if (!steer_filter_init[i])
        {
            steer_ref_filtered[i] = Target_Steer_Rad[i];
            steer_filter_init[i] = 1;
        }
        else
        {
            float err = Math_Modulus_Normalization(Target_Steer_Rad[i] - steer_ref_filtered[i], 2.0f * PI);
            if (err > PI)
                err -= 2.0f * PI;

            float dynamic_alpha;
            if (fabsf(err) > 0.5f)
                dynamic_alpha = 0.5f;
            else if (fabsf(err) > 0.2f)
                dynamic_alpha = 0.3f;
            else
                dynamic_alpha = 0.15f;

            steer_ref_filtered[i] += dynamic_alpha * err;
            steer_ref_filtered[i] = Math_Modulus_Normalization(steer_ref_filtered[i], 2.0f * PI);
        }

        Target_Steer_Rad[i] = steer_ref_filtered[i];
    }
}

/**
 * @brief 动力学解算
 * @prif 输出转换到动力学状态
 */
void Class_Chassis::Output_To_Dynamics()
{
}

/**
 * @brief 动力学逆解算
 *
 */
void Class_Chassis::Dynamics_Inverse_Resolution()
{
    static float last_vx = 0.0f;
    static float last_vy = 0.0f;
    static float last_omega = 0.0f;

    const float dt = 0.002f; // 2ms 控制周期

    /* ================= 1. 计算加速度 ================= */
    float ax = (Target_Velocity_X - last_vx) / dt;
    float ay = (Target_Velocity_Y - last_vy) / dt;
    float alpha = (Target_Omega - last_omega) / dt;

    last_vx = Target_Velocity_X;
    last_vy = Target_Velocity_Y;
    last_omega = Target_Omega; // 初始速度为0，末速度为当前速度

    /* ================= 2. 底盘合力 / 合力矩 ================= */
    float force_x = Class_Mass * ax;
    float force_y = Class_Mass * ay;            // 平动状态下 F = m * a
    float torque_omega = Class_Inertia * alpha; // 旋转状态下 τ = I * a

    // 每个轮的扭力
    float tmp_force[STEER_NUM];

    /* ================= 3. 合力 → 各轮等效切向力 ================= */
    for (int i = 0; i < STEER_NUM; i++)
    {
        float steer_radian = Encoder_Steer[i].Get_Wheel_Posture_radian(); // 使用编码器角度(θ)

        // 修正力分解公式
        // tmp_force[i] = Fx·cosθ + Fy·sinθ +       // 平移力在轮子方向的分量
        // (τ/L)·sin(φ-θ)                           // 旋转力在轮子方向的分量

        tmp_force[i] = force_x * acoshf32(steer_radian) +
                       force_y * asinhf32(steer_radian) +
                       (torque_omega / Wheel_To_Core_Distance[i]) * asinhf32(Wheel_Azimuth[i] - steer_radian);
    }

    /* ================= 4. 各轮力 → 力矩 + 阻尼 ================= */
    for (int i = 0; i < STEER_NUM; i++)
    {
        // 获取轮子实际角速度（不是角度！）
        float current_wheel_omega = ODrive_Motor_Steer[i].Get_Omega();

        /* —— 静力学力矩 —— */
        float torque_static = tmp_force[i] * Wheel_Radius; // 静力学部分：克服外力所需的扭矩

        /* —— 等效速度阻尼（不是控制环） —— */
        float torque_damping = Wheel_Speed_Limit_Factor *
                               (Target_Wheel_Omega[i] - current_wheel_omega); // 动力学部分：速度跟踪的前馈补偿

        Target_Wheel_torque[i] = torque_static + torque_damping;

        /* —— 摩擦连续化 —— */
        float omega = Target_Wheel_Omega[i];
        float tau_fric = Dynamic_Resistance_Wheel_Current[i] *
                         tanhf(omega / Wheel_Resistance_Omega_Threshold);

        Target_Wheel_torque[i] += tau_fric;
    }
}

/**
 * @brief 更新舵轮状态机 (状态流转)
 * @param i 轮子索引
 */
void Class_Chassis::Update_Steer_State(int i)
{
    float current = Get_Now_Steer_Radian(i);
    float target = Target_Steer_Rad[i];
    float delta = Math_Modulus_Normalization(target - current, 2.0f * PI);
    if (delta > PI)
        delta -= 2.0f * PI;

    switch (steer_state[i])
    {
    case STEER_STATE_IDLE:
        // 如果有明显的速度请求，进入对齐模式
        if (fabsf(Target_Wheel_Omega[i]) > steer_wheel_params_[i].wheel_omega_deadzone)
        {
            steer_state[i] = STEER_STATE_ALIGN;
        }
        break;

    case STEER_STATE_ALIGN:
        // 角度对齐到阈值内 (0.15rad 约 8.6度)，允许全功率驱动
        if (fabsf(delta) < 0.15f)
        {
            steer_state[i] = STEER_STATE_DRIVE;
        }
        // 如果对齐中途摇杆归零，退回 IDLE
        else if (fabsf(Target_Wheel_Omega[i]) < steer_wheel_params_[i].wheel_omega_deadzone)
        {
            steer_state[i] = STEER_STATE_IDLE;
        }
        break;

    case STEER_STATE_DRIVE:
        // 如果外力剧烈冲撞或摇杆瞬间大转向导致误差过大，退回 ALIGN 重新对齐
        if (fabsf(delta) > 0.15f && fabsf(Target_Wheel_Omega[i]) > steer_wheel_params_[i].wheel_omega_deadzone)
        {
            steer_state[i] = STEER_STATE_ALIGN;
        }
        // 停车，退回 IDLE
        else if (fabsf(Target_Wheel_Omega[i]) < steer_wheel_params_[i].wheel_omega_deadzone)
        {
            steer_state[i] = STEER_STATE_IDLE;
        }
        break;

    default:
        steer_state[i] = STEER_STATE_IDLE;
        break;
    }
}

/**
 * @brief 执行舵轮控制输出 (电机指令)
 * @param i 轮子索引
 */
void Class_Chassis::Execute_Steer_State(int i)
{
    switch (steer_state[i])
    {
    case STEER_STATE_IDLE:
    {
        // /* IDLE: 舵向锁当前角，轮向停止 */
        // float current_steer = Get_Now_Steer_Radian(i);
        // float motor_pos = Steer_To_Motor_Position(current_steer, i);
        // Motor_Steer[i].Set_Control_Parameter_MIT(motor_pos, 0.0f);

        /* IDLE: 舵向锁目标角 (Last Heading)，轮向停止 */
        // 【关键】锁定到 Target_Steer_Rad，即停止前的最后指令位置
        // 这样舵机会尽力维持在预定位置，即使有外力干扰或停止瞬间的冲击，也不会因为重置参考点而偏移
        float motor_pos = Steer_To_Motor_Position(Target_Steer_Rad[i], i);
        Motor_Steer[i].Set_Control_Parameter_MIT(motor_pos, 0.0f);

        ODrive_Motor_Steer[i].Set_target_omega(0.0f);
        ODrive_Motor_Steer[i].Set_target_torque(0.0f);
    }
    break;

    case STEER_STATE_ALIGN:
    {
        /* ALIGN: 舵向对齐，轮向根据投影系数慢转 */

        // 1. 舵向控制
        float motor_pos = Steer_To_Motor_Position(Target_Steer_Rad[i], i);
        Motor_Steer[i].Set_Control_Parameter_MIT(motor_pos, Target_Steer_Omage[i]);

        // 2. 轮向控制 (加入余弦投影优化)
        float current = Get_Now_Steer_Radian(i);
        float target = Target_Steer_Rad[i];
        float error = Math_Modulus_Normalization(target - current, 2.0f * PI);

        // 计算投影系数
        float projection_factor = acoshf32(error);
        if (projection_factor < 0.0f)
            projection_factor = 0.0f;

        // 应用投影速度
        ODrive_Motor_Steer[i].Set_target_omega(Target_Wheel_Omega[i] * projection_factor);
        ODrive_Motor_Steer[i].Set_target_torque(0.0f);
    }
    break;

    case STEER_STATE_DRIVE:
    {
        /* DRIVE: 正常行驶 */
        float motor_pos = Steer_To_Motor_Position(Target_Steer_Rad[i], i);

        // 保持舵向微调 (30% 力度)
        Motor_Steer[i].Set_Control_Parameter_MIT(
            motor_pos,
            Target_Steer_Omage[i] * 0.3f);

        // 轮向全速/力矩控制
        ODrive_Motor_Steer[i].Set_target_omega(Target_Wheel_Omega[i]);
        ODrive_Motor_Steer[i].Set_target_torque(Target_Wheel_torque[i]);
    }
    break;
    }
}

/**
 * @brief 输出到电机
 *
 */
void Class_Chassis::Output_To_Motor()
{
    for (int i = 0; i < STEER_NUM; i++)
    {
        // 1. 更新状态机 (判断是否切换状态)
        Update_Steer_State(i);

        // 2. 执行状态机 (发送电机指令)
        Execute_Steer_State(i);
    }

    // 中断回调通讯函数
    for (int i = 0; i < STEER_NUM; i++)
    {
        Motor_Steer[i].TIM_Send_PeriodElapsedCallback();
        ODrive_Motor_Steer[i].TIM_Send_PeriodElapsedCallback();
    }
}

void Class_Chassis::Send_DMMotor_CAN_Frame()
{
    static uint8_t idx = 0;
    Motor_Steer[idx].TIM_Send_PeriodElapsedCallback();
    idx++;
    if (idx >= STEER_NUM)
        idx = 0;
}

void Class_Chassis::Send_ODrive_CAN_Frame()
{
    static uint8_t idx = 0;
    ODrive_Motor_Steer[idx].TIM_Send_PeriodElapsedCallback();
    idx++;
    if (idx >= STEER_NUM)
        idx = 0;
}

void Class_Chassis::Send_Encoder_CAN_Frame()
{
    static uint8_t idx = 0;
    Encoder_Steer[idx].TIM_Query_PeriodElapsedCallback();
    idx++;
    if (idx >= STEER_NUM)
        idx = 0;
}
