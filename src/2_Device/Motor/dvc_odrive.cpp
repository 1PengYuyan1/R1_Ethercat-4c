/**
 * @file dvc_odrive.cpp
 * @brief ODrive简化版驱动实现
 */

#include "dvc_odrive.h"
#include <string.h>
#include <chrono>

/* 通信超时时间（毫秒） */
#define ODRIVE_COMM_TIMEOUT_MS 1000

static uint32_t HAL_GetTick()
{
    auto now = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return static_cast<uint32_t>(ms.count());
}

/**
 * @brief 简化初始化函数
 */
void Class_ODrive::Init(linkx_t *__LinkX_Handler, uint8_t __CAN_Channel, uint8_t __Node_ID)
{
    // 绑定 LinkX 资源
    LinkX_Handler = __LinkX_Handler;
    CAN_Channel = __CAN_Channel;
    config.node_id = __Node_ID;

    /* 数据初始化 */
    memset(&data, 0, sizeof(data));
    data.is_connected = 0;
    last_comm_time = HAL_GetTick();
    comm_timeout_counter = 0;
}

/**
 * @brief 设置轴状态
 */
void Class_ODrive::Set_Axis_State(Enum_ODrive_Axis_State state)
{
    uint8_t cmd_data[8] = {0};
    cmd_data[0] = (uint8_t)state;
    Send_Command(ODRIVE_CMD_SET_AXIS_STATE, cmd_data, 8);
}

/**
 * @brief 设置控制模式
 */
void Class_ODrive::Set_Control_Mode(Enum_ODrive_Control_Mode ctrl_mode,
                                    Enum_ODrive_Input_Mode input_mode)
{
    uint8_t cmd_data[8] = {0};
    cmd_data[0] = (uint8_t)ctrl_mode;  // 控制模式
    cmd_data[4] = (uint8_t)input_mode; // 输入模式
    Send_Command(ODRIVE_CMD_SET_CONTROLLER_MODES, cmd_data, 8);
}

/**
 * @brief 设置位置控制
 */
void Class_ODrive::Set_Position(float position, float vel_ff, float torque_ff)
{
    uint8_t cmd_data[8];
    Float_To_Bytes(position, &cmd_data[0]);
    Float_To_Bytes(vel_ff, &cmd_data[4]);
    Send_Command(ODRIVE_CMD_SET_INPUT_POS, cmd_data, 8);
}

/**
 * @brief 设置速度控制
 */
void Class_ODrive::Set_Velocity(float velocity, float torque_ff)
{
    uint8_t cmd_data[8];
    Float_To_Bytes(velocity, &cmd_data[0]);
    Float_To_Bytes(torque_ff, &cmd_data[4]);
    Send_Command(ODRIVE_CMD_SET_INPUT_VEL, cmd_data, 8);
}

/**
 * @brief 设置扭矩控制
 */
void Class_ODrive::Set_Torque(float torque)
{
    uint8_t cmd_data[4];
    Float_To_Bytes(torque, cmd_data);
    Send_Command(ODRIVE_CMD_SET_INPUT_TORQUE, cmd_data, 4);
}

/**
 * @brief 设置限制参数
 */
void Class_ODrive::Set_Limits(float vel_limit, float current_limit)
{
    uint8_t cmd_data[8];
    Float_To_Bytes(vel_limit, &cmd_data[0]);
    Float_To_Bytes(current_limit, &cmd_data[4]);
    Send_Command(ODRIVE_CMD_SET_LIMITS, cmd_data, 8);
}

/**
 * @brief 闭环模式
 */
void Class_ODrive::SET_ClosedLoop()
{
    Set_Axis_State(ODRIVE_STATE_CLOSED_LOOP_CONTROL);
}

/**
 * @brief 紧急停止
 */
void Class_ODrive::Emergency_Stop()
{
    Set_Axis_State(ODRIVE_STATE_IDLE);
}

/**
 * @brief 清除错误
 */
void Class_ODrive::Clear_Errors()
{
    uint8_t cmd_data[8] = {0};
    Send_Command(ODRIVE_CMD_CLEAR_ERRORS, cmd_data, 0);
}

/**
 * @brief 请求编码器数据
 */
void Class_ODrive::Request_Encoder_Data()
{
    uint8_t cmd_data[8] = {0};
    Send_Command_RTR(ODRIVE_CMD_GET_ENCODER_ESTIMATES, cmd_data, 8);
}

/**
 * @brief 请求总线电压
 */
void Class_ODrive::Request_Bus_Voltage()
{
    uint8_t cmd_data[8] = {0};
    Send_Command(ODRIVE_CMD_GET_BUS_VOLTAGE, cmd_data, 8);
}

/**
 * @brief 请求IQ数据
 */
void Class_ODrive::Request_IQ_Data()
{
    uint8_t cmd_data[8] = {0};
    Send_Command(ODRIVE_CMD_GET_IQ, cmd_data, 8);
}

/**
 * @brief 请求重新启动
 */
void Class_ODrive::Request_Reset_Data()
{
    uint8_t cmd_data[8] = {0};
    Send_Command(ODRIVE_CMD_RESET, cmd_data, 0);
}

/**
 * @brief
 */
void Class_ODrive::Set_loop()
{
    Clear_Errors();

    SET_ClosedLoop();
}

/**
 * @brief CAN接收回调
 */
void Class_ODrive::CAN_RxCpltCallback(uint8_t *rx_data, uint32_t __can_id)
{
// 直接使用传入的 __can_id
    uint32_t node_id = (__can_id >> 5);
    uint32_t cmd_id = (__can_id & 0x1F);

    if (node_id == config.node_id) {
        data.last_update = HAL_GetTick(); // 更新时间戳
        Process_Response((Enum_ODrive_Command)cmd_id, rx_data);
    }
}
/**
 * @brief TIM定时器中断计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void Class_ODrive::TIM_Send_PeriodElapsedCallback()
{

    if (data.motor_mode == ODRIVE_CTRL_VOLTAGE)
    {

        Set_Velocity(data.target_omage, data.target_torque);
    }

    else if (data.motor_mode == ODRIVE_CTRL_TORQUE)
    {
        Set_Torque(data.target_torque);
    }

    else if (data.motor_mode == ODRIVE_CTRL_VELOCITY)
    {
        Set_Velocity(data.target_omage, data.target_torque);
    }

    else if (data.motor_mode == ODRIVE_CTRL_POSITION)
    {
        Set_Position(data.target_postion);
    }
}

/**
 * @brief 定时存活检查
 */
void Class_ODrive::TIM_Alive_CheckCallback()
{
    uint32_t current_time = HAL_GetTick();

    if (current_time - data.last_update > ODRIVE_COMM_TIMEOUT_MS)
    {
        data.is_connected = 0;
        // Buzzer_Play_SystemError();
    }
    else
    {
        data.is_connected = 1;
    }
}

/**
 * @brief 掉线拯救
 */
void Class_ODrive::TIM_ODrive_DropLineSave()
{
    switch (ODrive_Status)
    {
    case (Odrive_Status_ENABLE):
    {
    }
    case (Odrive_Status_DISABLE):
    {
    }
    }
}

/**
 * @brief 发送CAN命令
 */
void Class_ODrive::Send_Command(Enum_ODrive_Command cmd, const uint8_t *send_data, uint8_t len)
{
    // ODrive CAN ID = (NodeID << 5) | CommandID
    uint32_t can_id = (config.node_id << 5) | (uint32_t)cmd;

    // 准备发送缓冲区 (ODrive 固定 8 字节)
    uint8_t tmp_tx_buffer[8] = {0};
    if (send_data != nullptr && len > 0)
    {
        memcpy(tmp_tx_buffer, send_data, (len > 8) ? 8 : len);
    }

    // 调用 EtherCAT-CAN 快速发送接口
    linkx_quick_can_send(LinkX_Handler, CAN_Channel, can_id, tmp_tx_buffer);
}

/**
 * @brief 发送远程帧CAN命令
 */
void Class_ODrive::Send_Command_RTR(Enum_ODrive_Command cmd, const uint8_t *send_data, uint8_t len)
{
    // ODrive CAN ID = (NodeID << 5) | CommandID
    uint32_t can_id = (config.node_id << 5) | (uint32_t)cmd;

    // 准备发送缓冲区 (ODrive 固定 8 字节)
    uint8_t tmp_tx_buffer[8] = {0};
    if (send_data != nullptr && len > 0)
    {
        memcpy(tmp_tx_buffer, send_data, (len > 8) ? 8 : len);
    }

    // 调用 EtherCAT-CAN 快速发送接口
    linkx_quick_can_send(LinkX_Handler, CAN_Channel, can_id, tmp_tx_buffer);
}

/**
 * @brief 处理响应数据
 */
void Class_ODrive::Process_Response(Enum_ODrive_Command cmd, const uint8_t *rx_data)
{
    if (rx_data == nullptr)
        return;

    switch (cmd)
    {
    case ODRIVE_CMD_GET_ENCODER_ESTIMATES:
        data.position = Bytes_To_Float(&rx_data[0]);
        data.omega = Bytes_To_Float(&rx_data[4]);
        break;

    case ODRIVE_CMD_GET_BUS_VOLTAGE:
        data.bus_voltage = Bytes_To_Float(rx_data);
        break;

    case ODRIVE_CMD_GET_IQ:
        data.iq_setpoint = Bytes_To_Float(&rx_data[0]);
        data.iq_measured = Bytes_To_Float(&rx_data[4]);
        break;

    case ODRIVE_CMD_SET_AXIS_STATE:
        uint32_t axis_error;
        uint32_t axis_state;

        memcpy(&axis_error, &rx_data[0], 4);
        memcpy(&axis_state, &rx_data[4], 4);

        data.axis_error = (Enum_ODrive_Axis_Error)axis_error;
        data.axis_state = (Enum_ODrive_Axis_State)axis_state;

        break;

    default:
        break;
    }
}

/**
 * @brief 浮点数转字节数组
 */
void Class_ODrive::Float_To_Bytes(float value, uint8_t *bytes)
{
    union
    {
        float f;
        uint8_t b[4];
    } converter;

    converter.f = value;
    memcpy(bytes, converter.b, 4);
}

/**
 * @brief 字节数组转浮点数
 */
float Class_ODrive::Bytes_To_Float(const uint8_t *bytes)
{
    union
    {
        float f;
        uint8_t b[4];
    } converter;

    memcpy(converter.b, bytes, 4);
    return converter.f;
}