/**
 * @file dvc_encoder_brt.h
 * @author Your_Name (your@email.com)
 * @brief 布瑞特编码器CAN通信驱动
 * @version 0.1
 * @date 2024-01-01
 *
 * @copyright Your_Company (c) 2024
 */

#ifndef DVC_ENCODER_H
#define DVC_ENCODER_H

#include <cstdint>
#include <cmath>
#include "linkx4c_handler.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// 布瑞特编码器默认参数
#define BRT_ENCODER_DEFAULT_BAUDRATE 1000000 // 1000kbps
#define BRT_ENCODER_SINGLE_TURN_RES 1024     // 默认单圈分辨率

// CAN指令码定义
#define BRT_CMD_READ_ENCODER_VALUE 0x01
#define BRT_CMD_SET_ENCODER_ID 0x02
#define BRT_CMD_SET_BAUDRATE 0x03
#define BRT_CMD_SET_MODE 0x04
#define BRT_CMD_SET_AUTO_SEND_TIME 0x05
#define BRT_CMD_SET_ZERO 0x06
#define BRT_CMD_SET_DIRECTION 0x07
#define BRT_CMD_READ_ANGULAR_VELOCITY 0x0A
#define BRT_CMD_SET_VELOCITY_SAMP_TIME 0x0B
#define BRT_CMD_SET_MIDPOINT 0x0C
#define BRT_CMD_SET_CURRENT_VALUE 0x0D
#define BRT_CMD_SET_5_TURN_VALUE 0x0F

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 布瑞特编码器工作模式
 */
typedef enum
{
    BRT_MODE_QUERY = 0x00,               // 查询模式
    BRT_MODE_AUTO_RETURN_ANGLE = 0xAA,   // 自动返回角度值
    BRT_MODE_AUTO_RETURN_VELOCITY = 0x02 // 自动返回角速度值
} Enum_BRT_Encoder_Mode;

/**
 * @brief 布瑞特编码器波特率
 */
typedef enum
{
    BRT_BAUDRATE_500K = 0x00, // 默认500kbps
    BRT_BAUDRATE_1M = 0x01,   // 1Mbps
    BRT_BAUDRATE_250K = 0x02, // 250kbps
    BRT_BAUDRATE_125K = 0x03, // 125kbps
    BRT_BAUDRATE_100K = 0x04  // 100kbps
} Enum_BRT_Encoder_Baudrate;

/**
 * @brief 布瑞特编码器递增方向
 */
typedef enum
{
    BRT_DIRECTION_CW = 0x00, // 顺时针递增
    BRT_DIRECTION_CCW = 0x01 // 逆时针递增
} Enum_BRT_Encoder_Direction;

/**
 * @brief 布瑞特编码器状态
 */
typedef enum
{
    BRT_STATUS_DISABLE = 0,
    BRT_STATUS_ENABLE,
    BRT_STATUS_ERROR
} Enum_BRT_Encoder_Status;

/**
 * @brief 布瑞特编码器接收数据结构
 */
typedef struct
{
    uint32_t encoder_value; // 编码器原始值

    float current_circle_value; // 当前单圈编码器的值

    float current_angle;        // 当前编码器角度(度)
    float current_radian;       // 当前编码器弧度（弧度）
    float wheel_posture_angle;  // 当前轮组角度
    float wheel_posture_radian; // 当前轮组弧度

    float Last_radian;               // 上一次单圈编码器的的弧度值
    float wheel_posture_radian_last; // 上一次轮组弧度（rad）

    float current_omega;              // 编码器角速度(转/分钟)
    float wheel_current_omega_radian; // 映射到轮组的角速度（弧度/分钟）

    float target_omega;  // 编码器目标角速度(转/分钟)
    float target_radian; // 编码器目标角度

    float current_rounds; // 编码器当前圈数

    float position_delta; // 编码器位置差

    float pre_encoder;        // 编码器前一次编码器值
    float pre_circle_encoder; // 编码器上一次单圈编码器的值

    uint8_t speed_init_flag;        //
    Enum_BRT_Encoder_Status status; // 编码器状态
} Struct_BRT_Encoder_Data;

/**
 * @brief 布瑞特编码器配置参数
 */
typedef struct
{
    uint8_t can_id;                  // CAN节点ID
    uint32_t baudrate;               // 通信波特率
    uint16_t single_turn_resolution; // 单圈分辨率
    uint32_t max_turns;              // 最大圈数
    Enum_BRT_Encoder_Mode work_mode; // 工作模式
    uint16_t auto_send_time;         // 自动发送时间(微秒)
    uint16_t velocity_sample_time;   // 速度采样时间(毫秒)
} Struct_BRT_Encoder_Config;

/**
 * @brief 布瑞特编码器类
 */
class Class_Encoder_BRT
{
public:
    void Init(linkx_t *linkx_ptr, uint16_t slave_index, uint8_t encoder_id, uint16_t single_turn_res); // 基本功能函数
    void CAN_RxCpltCallback(uint8_t *Rx_Data);

    // 指令发送函数
    void CAN_Send_ReadEncoderValue();
    void CAN_Send_SetEncoderID(uint8_t new_id);
    void CAN_Send_SetBaudrate(Enum_BRT_Encoder_Baudrate baudrate);
    void CAN_Send_SetMode(Enum_BRT_Encoder_Mode mode);
    void CAN_Send_SetAutoSendTime(uint16_t time_us);
    void CAN_Send_SetZero();
    void CAN_Send_SetDirection(Enum_BRT_Encoder_Direction direction);
    void CAN_Send_ReadAngularVelocity();
    void CAN_Send_SetVelocitySampleTime(uint16_t time_ms);
    void CAN_Send_SetMidpoint();
    void CAN_Send_SetCurrentValue(uint32_t value);
    void CAN_Send_Set5TurnValue();

    // 数据获取函数
    inline uint32_t Get_EncoderValue();
    inline float Get_CurrentAngle();
    inline float Get_Wheel_Angle();
    inline float Get_Wheel_Posture_radian();
    inline float Get_AngularVelocity();
    inline uint32_t Get_TotalRounds();
    inline Enum_BRT_Encoder_Status Get_Status();

    inline uint8_t Get_Can_ID();

    // 参数设置函数
    inline void Set_SingleTurnResolution(uint16_t resolution);
    inline void Set_MaxTurns(uint32_t turns);
    inline void Set_Target_Angle(float Target_Angle);
    inline void Set_Omega(float Target_Omega);

    void TIM_Alive_PeriodElapsedCallback();
    void TIM_Query_PeriodElapsedCallback();

protected:
    // 初始化相关变量
    linkx_t *LinkX_Ptr;
    uint16_t Slave_Index;
    Struct_BRT_Encoder_Config config;

    // 接收数据
    Struct_BRT_Encoder_Data data;

    // 内部变量
    uint32_t alive_flag = 0;
    uint32_t pre_alive_flag = 0;
    uint8_t tx_data[8];

    // 内部函数
    void Data_Process(uint8_t *inputs_ptr);
    void EncoderValue_To_Angle();
    void CAN_Send_Command(uint8_t command, uint8_t *data, uint8_t data_len);
};

inline uint32_t Class_Encoder_BRT::Get_EncoderValue()
{
    return data.encoder_value;
}

inline float Class_Encoder_BRT::Get_CurrentAngle()
{
    return data.current_angle;
}

inline float Class_Encoder_BRT::Get_Wheel_Angle()
{
    return data.wheel_posture_angle;
}
inline float Class_Encoder_BRT::Get_Wheel_Posture_radian()
{
    return data.wheel_posture_radian;
}

inline float Class_Encoder_BRT::Get_AngularVelocity()
{
    return data.current_omega;
}

inline uint32_t Class_Encoder_BRT::Get_TotalRounds()
{
    return data.current_rounds;
}

inline Enum_BRT_Encoder_Status Class_Encoder_BRT::Get_Status()
{
    return data.status;
}

inline uint8_t Class_Encoder_BRT::Get_Can_ID()
{
    return config.can_id;
}

inline void Class_Encoder_BRT::Set_Target_Angle(float Target_Angle)
{
    data.target_radian = Target_Angle;
}

inline void Class_Encoder_BRT::Set_Omega(float Target_Omega)
{
    data.target_omega = Target_Omega;
}

// 参数设置函数
inline void Class_Encoder_BRT::Set_SingleTurnResolution(uint16_t resolution)
{
    config.single_turn_resolution = resolution;
}

inline void Class_Encoder_BRT::Set_MaxTurns(uint32_t turns)
{
    config.max_turns = turns;
}

#endif // USTC_STREETING_ENCODER_H
