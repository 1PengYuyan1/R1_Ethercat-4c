/**
 * @file dvc_odrive.h
 * @brief ODrive电机控制器简化版驱动
 * @version 2.0
 * @date 2024-01-01
 */

#ifndef DVC_ODRIVE_H
#define DVC_ODRIVE_H

#include <cstdint>
#include <cmath>
#include "linkx.h"
#include "linkx4c_handler.h"

/* ODrive默认参数 */
#define ODRIVE_DEFAULT_NODE_ID 0x00
#define ODRIVE_DEFAULT_BAUDRATE_1000000 1000000
#define ODRIVE_DEFAULT_BAUDRATE_500000 500000

enum Enum_ODrive_Status
{
    Odrive_Status_DISABLE = 0,
    Odrive_Status_ENABLE,
};

/* ODrive核心CAN命令ID */
typedef enum
{
    ODRIVE_CMD_SET_AXIS_STATE = 0x007,
    ODRIVE_CMD_GET_ENCODER_ESTIMATES = 0x009,
    ODRIVE_CMD_SET_CONTROLLER_MODES = 0x00B,
    ODRIVE_CMD_SET_INPUT_POS = 0x00C,
    ODRIVE_CMD_SET_INPUT_VEL = 0x00D,
    ODRIVE_CMD_SET_INPUT_TORQUE = 0x00E,
    ODRIVE_CMD_SET_LIMITS = 0x00F,
    ODRIVE_CMD_GET_IQ = 0x014,
    ODRIVE_CMD_RESET = 0x016,
    ODRIVE_CMD_GET_BUS_VOLTAGE = 0x017,
    ODRIVE_CMD_CLEAR_ERRORS = 0x018
} Enum_ODrive_Command;

/* 轴错误枚举 */
typedef enum
{
    AXIS_ERROR_NONE = 0x00000000,

    AXIS_ERROR_INVALID_STATE = 0x00000001,
    AXIS_ERROR_WATCHDOG_TIMER_EXPIRED = 0x00000002,
    AXIS_ERROR_MIN_ENDSTOP_PRESSED = 0x00000004,
    AXIS_ERROR_MAX_ENDSTOP_PRESSED = 0x00000008,

    AXIS_ERROR_ESTOP_REQUESTED = 0x00000010,
    AXIS_ERROR_HOMING_WITHOUT_ENDSTOP = 0x00000020,
    AXIS_ERROR_OVER_TEMP = 0x00000040,
    AXIS_ERROR_UNKNOWN_POSITION = 0x00000080,

    AXIS_ERROR_BRAKE_RESISTOR_DISARMED = 0x00000100,
    AXIS_ERROR_SYSTEM_LEVEL = 0x00000200,
} Enum_ODrive_Axis_Error;

/* 轴状态枚举 */
typedef enum
{
    ODRIVE_STATE_UNDEFINED = 0,
    ODRIVE_STATE_IDLE = 1,
    ODRIVE_STATE_STARTUP_SEQUENCE = 2,
    ODRIVE_STATE_FULL_CALIBRATION = 3,
    ODRIVE_STATE_MOTOR_CALIBRATION = 4,
    ODRIVE_STATE_ENCODER_INDEX_SEARCH = 6,
    ODRIVE_STATE_ENCODER_OFFSET_CALIB = 7,
    ODRIVE_STATE_CLOSED_LOOP_CONTROL = 8
} Enum_ODrive_Axis_State;

/* 控制模式枚举 */
typedef enum
{
    ODRIVE_CTRL_VOLTAGE = 0,
    ODRIVE_CTRL_TORQUE = 1,
    ODRIVE_CTRL_VELOCITY = 2,
    ODRIVE_CTRL_POSITION = 3
} Enum_ODrive_Control_Mode;

/* 输入模式枚举 */
typedef enum
{
    ODRIVE_INPUT_PASSTHROUGH = 1,
    ODRIVE_INPUT_VEL_RAMP = 2,
    ODRIVE_INPUT_POS_FILTER = 3,
    ODRIVE_INPUT_TRAP_TRAJ = 5
} Enum_ODrive_Input_Mode;

/* ODrive状态数据结构 */
typedef struct
{
    float position;    // 当前位置 (rad)
    float omega;       // 当前速度 (rad/s)
    float torque;      // 当前扭矩 (Nm)
    float bus_voltage; // 总线电压 (V)
    float iq_setpoint; // Q轴电流设定值 (A)
    float iq_measured; // Q轴电流测量值 (A)

    float target_postion;
    float target_omage;
    float target_torque;

    Enum_ODrive_Axis_Error axis_error;
    Enum_ODrive_Axis_State axis_state;   // 轴状态
    Enum_ODrive_Control_Mode motor_mode; // 控制模式
    uint32_t last_update;                // 最后更新时间戳
    uint8_t is_connected;                // 连接状态
} Struct_ODrive_Data;

/* ODrive配置结构（简化版） */
typedef struct
{
    uint8_t node_id;   // CAN节点ID
    uint32_t baudrate; // 通信波特率
} Struct_ODrive_Config;

/* ODrive主类 */
class Class_ODrive
{
public:
    /* 初始化函数（简化版） */
    void Init(linkx_t *__LinkX_Handler, uint8_t __CAN_Channel, uint8_t __Node_ID);
    /* 核心控制函数 */
    void Set_Axis_State(Enum_ODrive_Axis_State state);
    void Set_Control_Mode(Enum_ODrive_Control_Mode ctrl_mode,
                          Enum_ODrive_Input_Mode input_mode);
    void Set_Position(float position, float vel_ff = 0, float torque_ff = 0);
    void Set_Velocity(float velocity, float torque_ff = 0);
    void Set_Torque(float torque);

    /* 状态查询函数 */
    void Request_Encoder_Data();
    void Request_Bus_Voltage();
    void Request_IQ_Data();
    void Request_Reset_Data();

    /* 实用函数 */
    void SET_ClosedLoop();
    void Emergency_Stop();
    void Clear_Errors();

    void Set_Limits(float vel_limit, float current_limit);
    void Set_loop();

    /* 数据获取函数 */
    inline float Get_Position();
    inline float Get_Omega();
    inline float Get_Torque();
    inline float Get_Axis_Error();
    inline float Get_Axis_State();
    inline float Get_Bus_Voltage();

    inline uint8_t Is_Connected();

    inline Enum_ODrive_Control_Mode Get_Motor_Mode();
    inline void Get_Axis_State(Enum_ODrive_Axis_State __state);
    inline void Get_Motor_Mode(Enum_ODrive_Control_Mode __mode);

    inline uint8_t Get_node_id();

    // 设立目标值
    inline void Set_target_omega(float target_omega);
    inline void Set_target_torque(float target_torque);

    /* CAN回调函数 */
    void CAN_RxCpltCallback(uint8_t *rx_data, uint32_t __can_id);

    void TIM_Alive_CheckCallback();
    void TIM_Send_PeriodElapsedCallback();
    void TIM_ODrive_DropLineSave();

private:
    linkx_t *LinkX_Handler;
    uint8_t CAN_Channel;
    Struct_ODrive_Config config;
    Struct_ODrive_Data data;
    Enum_ODrive_Status ODrive_Status = Odrive_Status_DISABLE;

    uint32_t comm_timeout_counter;
    uint32_t last_comm_time;
    uint8_t tx_buffer[8];

    void Send_Command(Enum_ODrive_Command cmd, const uint8_t *send_data, uint8_t len);
    void Send_Command_RTR(Enum_ODrive_Command cmd, const uint8_t *data, uint8_t len);

    void Process_Response(Enum_ODrive_Command cmd, const uint8_t *rx_data);

    void Float_To_Bytes(float value, uint8_t *bytes);
    float Bytes_To_Float(const uint8_t *bytes);
};

inline float Class_ODrive::Get_Omega()
{
    return (data.omega);
}

inline float Class_ODrive::Get_Torque()
{

    return (data.torque);
}

inline float Class_ODrive::Get_Bus_Voltage()
{

    return (data.bus_voltage);
}

inline float Class_ODrive::Get_Axis_Error()
{

    return (data.axis_error);
}

inline float Class_ODrive::Get_Axis_State()
{

    return (data.axis_state);
}

inline Enum_ODrive_Control_Mode Class_ODrive::Get_Motor_Mode()
{

    return (data.motor_mode);
}

inline uint8_t Class_ODrive::Get_node_id()
{

    return config.node_id;
}

inline void Class_ODrive::Get_Axis_State(Enum_ODrive_Axis_State __state)
{

    data.axis_state = __state;
}

inline void Class_ODrive::Get_Motor_Mode(Enum_ODrive_Control_Mode __mode)
{

    data.motor_mode = __mode;
}

inline void Class_ODrive::Set_target_omega(float target_omega)
{
    data.target_omage = target_omega;
}

inline void Class_ODrive::Set_target_torque(float target_torque)
{
    data.target_torque = target_torque;
}
#endif
