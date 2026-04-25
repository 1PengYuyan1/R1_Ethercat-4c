/**
 * @file dvc_encoder_brt.cpp
 * @author Your_Name (your@email.com)
 * @brief 布瑞特编码器CAN通信实现
 * @version 0.1
 * @date 2024-01-01
 *
 * @copyright Your_Company (c) 2024
 */
#include "dvc_encoder.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 编码器初始化
 *
 * @param hcan CAN句柄
 * @param encoder_id 编码器ID(1-255)
 * @param baudrate 通信波特率
 * @param single_turn_res 单圈分辨率
 */
void Class_Encoder_BRT::Init(linkx_t *linkx_ptr, uint16_t slave_index, uint8_t encoder_id, uint16_t single_turn_res)
{
    this->LinkX_Ptr = linkx_ptr; // 保存网关指针
    this->Slave_Index = slave_index;
    this->config.can_id = encoder_id;
    this->config.single_turn_resolution = single_turn_res; // 初始化配置参数
    config.single_turn_resolution = BRT_ENCODER_SINGLE_TURN_RES;

    // 初始化配置参数
    config.can_id = encoder_id;
    config.single_turn_resolution = single_turn_res;
    config.max_turns = 50;              // 默认最大圈数
    config.auto_send_time = 1000;       // 默认1ms
    config.velocity_sample_time = 1000; // 默认100ms

    // 初始化接收数据
    data = {0};
    data.status = BRT_STATUS_DISABLE;
}

/**
 * @brief CAN接收完成回调函数
 *
 * @param Rx_Data 接收到的数据
 */
void Class_Encoder_BRT::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    // 更新存活标志
    alive_flag += 1;

    // 数据处理
    Data_Process(Rx_Data);
}

/**
 * @brief 定时器存活检测回调函数
 *
 */
void Class_Encoder_BRT::TIM_Alive_PeriodElapsedCallback()
{
    // 检测编码器是否在线
    if (alive_flag == pre_alive_flag)
    {
        data.status = BRT_STATUS_DISABLE;
    }
    else
    {
        data.status = BRT_STATUS_ENABLE;
    }
    pre_alive_flag = alive_flag;
}

/**
 * @brief 定时查询回调函数
 *
 */
void Class_Encoder_BRT::TIM_Query_PeriodElapsedCallback()
{
    if (config.work_mode == BRT_MODE_QUERY)
    {
        // 查询模式下定期读取编码器值
        CAN_Send_ReadEncoderValue();
    }
}

/**
 * @brief 发送读取编码器值指令
 *
 */
void Class_Encoder_BRT::CAN_Send_ReadEncoderValue()
{
    uint8_t data[1] = {0x00};
    CAN_Send_Command(BRT_CMD_READ_ENCODER_VALUE, data, 1);
}

/**
 * @brief 发送设置编码器ID指令
 *
 * @param new_id 新的编码器ID(1-255)
 */
void Class_Encoder_BRT::CAN_Send_SetEncoderID(uint8_t new_id)
{
    uint8_t data[1] = {new_id};
    CAN_Send_Command(BRT_CMD_SET_ENCODER_ID, data, 1);
}

/**
 * @brief 发送设置波特率指令
 *
 * @param baudrate 波特率枚举值
 */
void Class_Encoder_BRT::CAN_Send_SetBaudrate(Enum_BRT_Encoder_Baudrate baudrate)
{
    uint8_t data[1] = {(uint8_t)baudrate};
    CAN_Send_Command(BRT_CMD_SET_BAUDRATE, data, 1);
}

/**
 * @brief 发送设置工作模式指令
 *
 * @param mode 工作模式
 */
void Class_Encoder_BRT::CAN_Send_SetMode(Enum_BRT_Encoder_Mode mode)
{
    uint8_t data[1] = {(uint8_t)mode};
    CAN_Send_Command(BRT_CMD_SET_MODE, data, 1);
    config.work_mode = mode;
}

/**
 * @brief 发送设置自动发送时间指令
 *
 * @param time_us 自动发送时间(微秒)
 */
void Class_Encoder_BRT::CAN_Send_SetAutoSendTime(uint16_t time_us)
{
    uint8_t data[2];
    data[0] = time_us & 0xFF;        // 低字节在前
    data[1] = (time_us >> 8) & 0xFF; // 高字节
    CAN_Send_Command(BRT_CMD_SET_AUTO_SEND_TIME, data, 2);
    config.auto_send_time = time_us;
}

/**
 * @brief 发送设置零点指令
 *
 */
void Class_Encoder_BRT::CAN_Send_SetZero()
{
    uint8_t data[1] = {0x00};
    CAN_Send_Command(BRT_CMD_SET_ZERO, data, 1);
}

/**
 * @brief 发送设置方向指令
 *
 * @param direction 递增方向
 */
void Class_Encoder_BRT::CAN_Send_SetDirection(Enum_BRT_Encoder_Direction direction)
{
    uint8_t data[1] = {(uint8_t)direction};
    CAN_Send_Command(BRT_CMD_SET_DIRECTION, data, 1);
}

/**
 * @brief 发送读取角速度指令
 *
 */
void Class_Encoder_BRT::CAN_Send_ReadAngularVelocity()
{
    uint8_t data[1] = {0x00};
    CAN_Send_Command(BRT_CMD_READ_ANGULAR_VELOCITY, data, 1);
}

/**
 * @brief 发送设置速度采样时间指令
 *
 * @param time_ms 采样时间(毫秒)
 */
void Class_Encoder_BRT::CAN_Send_SetVelocitySampleTime(uint16_t time_ms)
{
    uint8_t data[2];
    data[0] = time_ms & 0xFF;        // 低字节在前
    data[1] = (time_ms >> 8) & 0xFF; // 高字节
    CAN_Send_Command(BRT_CMD_SET_VELOCITY_SAMP_TIME, data, 2);
    config.velocity_sample_time = time_ms;
}

/**
 * @brief 发送设置中点指令
 *
 */
void Class_Encoder_BRT::CAN_Send_SetMidpoint()
{
    uint8_t data[1] = {0x01};
    CAN_Send_Command(BRT_CMD_SET_MIDPOINT, data, 1);
}

/**
 * @brief 发送设置当前值指令
 *
 * @param value 要设置的编码器值
 */
void Class_Encoder_BRT::CAN_Send_SetCurrentValue(uint32_t value)
{
    uint8_t data[4];
    data[0] = value & 0xFF; // 低字节在前
    data[1] = (value >> 8) & 0xFF;
    data[2] = (value >> 16) & 0xFF;
    data[3] = (value >> 24) & 0xFF;
    CAN_Send_Command(BRT_CMD_SET_CURRENT_VALUE, data, 4);
}

/**
 * @brief 发送设置5圈值指令
 *
 */
void Class_Encoder_BRT::CAN_Send_Set5TurnValue()
{
    uint8_t data[1] = {0x01};
    CAN_Send_Command(BRT_CMD_SET_5_TURN_VALUE, data, 1);
}

/**
 * @brief 数据处理函数
 *
 * @param rx_data_buffer 接收数据缓冲区
 */
void Class_Encoder_BRT::Data_Process(uint8_t *inputs_ptr)
{
    // 解析数据长度
    uint8_t data_len = inputs_ptr[0];
    uint8_t encoder_id = inputs_ptr[1];
    uint8_t command = inputs_ptr[2];

    // 检查ID是否匹配
    if (encoder_id != config.can_id)
        return;

    switch (command)
    {
    case BRT_CMD_READ_ENCODER_VALUE:
    {
        if (data_len == 7)
        {
            // ================= 编码器原始值解析 =================
            data.encoder_value = (uint32_t)inputs_ptr[3] |
                                 ((uint32_t)inputs_ptr[4] << 8) |
                                 ((uint32_t)inputs_ptr[5] << 16) |
                                 ((uint32_t)inputs_ptr[6] << 24);

            // ================= 计算当前圈数和单圈值 =================
            uint32_t current_circle = data.encoder_value / 4096;    // 当前在第几圈
            uint32_t single_turn_value = data.encoder_value % 4096; // 当前单圈编码器值

            // ================= 舵向弧度（相对中点） =================
            // 中点：第25圈，单圈值0
            int32_t circle_offset = (int32_t)current_circle - 25;  // 圈数偏移
            int32_t value_offset = (int32_t)single_turn_value - 0; // 单圈值偏移

            // 总的编码器弧度（相对中点）
            data.current_radian = (float)circle_offset * 2.0f * PI + (float)value_offset * 2.0f * PI / 4096.0f;

            // ================= 轮向弧度 =================
            data.wheel_posture_radian = data.current_radian / 3.5f;

            // 归一化到 [0, 2π)
            data.wheel_posture_radian = fmodf(data.wheel_posture_radian, 2.0f * PI);
            if (data.wheel_posture_radian < 0)
                data.wheel_posture_radian += 2.0f * PI;

            // 角度
            data.wheel_posture_angle = data.wheel_posture_radian * 180.0f / PI;
        }
    }
    break;

    case BRT_CMD_READ_ANGULAR_VELOCITY:
    {
        if (data_len == 7) // 4字节数据 + 3字节头
        {
            // 解析32位角速度值(有符号整数)
            int32_t angular_velocity_raw = (int32_t)inputs_ptr[3] |
                                           ((int32_t)inputs_ptr[4] << 8) |
                                           ((int32_t)inputs_ptr[5] << 16) |
                                           ((int32_t)inputs_ptr[6] << 24);

            // 计算角速度(转/分钟)
            data.current_omega = (float)angular_velocity_raw /
                                 config.single_turn_resolution /
                                 (config.velocity_sample_time / 1000.0f / 60.0f);
        }
    }
    break;

    default:
        // 其他指令的响应处理
        break;
    }
}

/**
 * @brief 编码器值转换为角度/弧度
 *
 */
void Class_Encoder_BRT::EncoderValue_To_Angle()
{
    // 计算当前角度: 编码器值 * 360 / 单圈分辨率
    data.current_angle = data.current_circle_value * 360.0f /
                         (float)config.single_turn_resolution;
}

/**
 * @brief 发送CAN指令
 *
 * @param command 指令码
 * @param data 数据缓冲区
 * @param data_len 数据长度
 */
void Class_Encoder_BRT::CAN_Send_Command(uint8_t command, uint8_t *data, uint8_t data_len)
{
    // 构造发送数据包
    tx_data[0] = data_len + 3; // 总长度: 数据长度 + 3字节头
    tx_data[1] = config.can_id;
    tx_data[2] = command;

    // 拷贝数据
    for (uint8_t i = 0; i < data_len; i++)
    {
        tx_data[3 + i] = data[i];
    }

    // 填充剩余字节为0
    for (uint8_t i = data_len + 3; i < 8; i++)
    {
        tx_data[i] = 0x00;
    }

    // 发送CAN数据
    linkx_quick_can_send(LinkX_Ptr, 0, 0x200 + config.can_id,tx_data);

}

/************************ End of file ************************/