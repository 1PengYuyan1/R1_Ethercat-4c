//
// Created by pzx on 2025/12/15.
//
#include "dvc_motor_dm.h"
#include "math.h"

uint8_t DM_Motor_CAN_Message_Clear_Error[8] = {
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfb
};
// 使能电机, 传统模式有效
uint8_t DM_Motor_CAN_Message_Enter[8] = {
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfc
};
// 失能电机, 传统模式有效
uint8_t DM_Motor_CAN_Message_Exit[8] = {
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfd
};
// 保存当前电机位置为零点, 传统模式有效
uint8_t DM_Motor_CAN_Message_Save_Zero[8] = {
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfe
};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致, 传统模式有效
 * @param __CAN_Tx_ID 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量, 传统模式有效
 * @param __Motor_DM_Control_Method 电机控制方式
 * @param __Angle_Max 最大位置, 与上位机控制幅值PMAX保持一致, 传统模式有效
 * @param __Omega_Max 最大速度, 与上位机控制幅值VMAX保持一致, 传统模式有效
 * @param __Torque_Max 最大扭矩, 与上位机控制幅值TMAX保持一致, 传统模式有效
 */
void Class_Motor_DM_Normal::Init(linkx_t *__LinkX_Handler, uint8_t __CAN_Channel, uint8_t __CAN_Rx_ID, uint8_t __CAN_Tx_ID,
                                 Enum_Motor_DM_Control_Method __Motor_DM_Control_Method, float __Angle_Max,
                                 float __Omega_Max, float __Torque_Max, float __Current_Max) {
    
    // 保存 EtherCAT LinkX 句柄和通道
    LinkX_Handler = __LinkX_Handler;
    CAN_Channel = __CAN_Channel;

    // 处理控制模式对应的 ID 偏移
    switch (__Motor_DM_Control_Method) {
        case (Motor_DM_Control_Method_NORMAL_MIT): {
            DM_CAN_Tx_ID = __CAN_Tx_ID;
            break;
        }
        case (Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA): {
            DM_CAN_Tx_ID = __CAN_Tx_ID + 0x100;
            break;
        }
        case (Motor_DM_Control_Method_NORMAL_OMEGA): {
            DM_CAN_Tx_ID = __CAN_Tx_ID + 0x200;
            break;
        }
        case (Motor_DM_Control_Method_NORMAL_EMIT): {
            DM_CAN_Tx_ID = __CAN_Tx_ID + 0x300;
            break;
        }
    }

    DM_CAN_Rx_ID = __CAN_Rx_ID;
    Motor_DM_Control_Method = __Motor_DM_Control_Method;
    Radian_Max = __Angle_Max;
    Omega_Max = __Omega_Max;
    Torque_Max = __Torque_Max;
    Current_Max = __Current_Max;
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_Motor_DM_Normal::CAN_RxCpltCallback(uint8_t *Rx_Data) {
    // 滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process(Rx_Data);
}

/**
 * @brief 发送清除错误信息
 *
 */
void Class_Motor_DM_Normal::CAN_Send_Clear_Error() {
linkx_quick_can_send(LinkX_Handler, CAN_Channel, DM_CAN_Tx_ID, DM_Motor_CAN_Message_Clear_Error);
}

/**
 * @brief 发送使能电机
 *
 */
void Class_Motor_DM_Normal::CAN_Send_Enter() {

    linkx_quick_can_send(LinkX_Handler, CAN_Channel, DM_CAN_Tx_ID, DM_Motor_CAN_Message_Enter);
}

/**
 * @brief 发送失能电机
 *
 */
void Class_Motor_DM_Normal::CAN_Send_Exit() {
    linkx_quick_can_send(LinkX_Handler, CAN_Channel, DM_CAN_Tx_ID, DM_Motor_CAN_Message_Exit);
}

/**
 * @brief 发送保存当前位置为零点
 *
 */
void Class_Motor_DM_Normal::CAN_Send_Save_Zero() {
    linkx_quick_can_send(LinkX_Handler, CAN_Channel, DM_CAN_Tx_ID, DM_Motor_CAN_Message_Save_Zero);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活, 检测周期取决于电机掉线时长
 *
 */
void Class_Motor_DM_Normal::TIM_Alive_PeriodElapsedCallback() {

    // 判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag) {
        // 电机断开连接
        Motor_DM_Status = Motor_DM_Status_DISABLE;
        // Buzzer_Play_SystemError();
    } else {
        // 电机保持连接
        Motor_DM_Status = Motor_DM_Status_ENABLE;
    }

    Pre_Flag = Flag;

}

/**
 * @brief TIM定时器中断发送出去的回调函数, 计算周期取决于自主设置的控制周期
 *
 */
void Class_Motor_DM_Normal::TIM_Send_PeriodElapsedCallback() {

    if (data.Control_Status == Motor_DM_Status_ENABLE) {
        // 电机在线, 正常控制
        Math_Constrain(&Control_Radian, -Radian_Max, Radian_Max);
        Math_Constrain(&Control_Omega, -Omega_Max, Omega_Max);
        Math_Constrain(&Control_Torque, -Torque_Max, Torque_Max);
        Math_Constrain(&Control_Current, -Current_Max, Current_Max);
        Math_Constrain(&K_P, 0.0f, 500.0f);
        Math_Constrain(&K_D, 0.0f, 5.0f);

        Output();
    } else if (data.Control_Status == Motor_DM_Status_DISABLE) {
        // 电机可能掉线, 使能电机
        CAN_Send_Enter();
    } else {
        // 电机错误, 发送清除错误帧
        CAN_Send_Clear_Error();
    }
}

/**
 * @brief 数据处理过程
 *
 */
void Class_Motor_DM_Normal::Data_Process(uint8_t *rx_data) {
    // 数据处理过程
    int32_t delta_encoder;
    uint16_t tmp_encoder, tmp_omega, tmp_torque;
    Struct_Motor_DM_CAN_Rx_Data_Normal *tmp_buffer = (Struct_Motor_DM_CAN_Rx_Data_Normal *) rx_data;

    // 电机ID不匹配, 则不进行处理
    if (tmp_buffer->CAN_ID != (DM_CAN_Tx_ID & 0x0f)) {
        return;
    }

    // 处理大小端
    Math_Endian_Reverse_16((void *) &tmp_buffer->Angle_Reverse, &tmp_encoder);
    tmp_omega = (tmp_buffer->Omega_11_4 << 4) | (tmp_buffer->Omega_3_0_Torque_11_8 >> 4);
    tmp_torque = ((tmp_buffer->Omega_3_0_Torque_11_8 & 0x0f) << 8) | (tmp_buffer->Torque_7_0);


    // 计算圈数与总角度值
    delta_encoder = tmp_encoder - data.Pre_Encoder;
    if (delta_encoder < -(1 << 15)) {
        // 正方向转过了一圈
        data.Total_Round++;
    } else if (delta_encoder > (1 << 15)) {
        // 反方向转过了一圈
        data.Total_Round--;
    }
    data.Total_Encoder = data.Total_Round * (1 << 16) + tmp_encoder - ((1 << 15) - 1);
    data.Control_Status = static_cast<Enum_Motor_DM_Control_Status_Normal>(tmp_buffer->Control_Status_Enum);
    // 计算电机本身信息
    data.Now_Rad = (float) (data.Total_Encoder) / (float) ((1 << 16) - 1) * Radian_Max ;
    data.Now_Omega = Math_Int_To_Float(tmp_omega, 0x7ff, (1 << 12) - 1, 0, Omega_Max);
    data.Now_Torque = Math_Int_To_Float(tmp_torque, 0x7ff, (1 << 12) - 1, 0, Torque_Max);
    data.Now_MOS_Temperature = tmp_buffer->MOS_Temperature + CELSIUS_TO_KELVIN;
    data.Now_Rotor_Temperature = tmp_buffer->Rotor_Temperature + CELSIUS_TO_KELVIN;

    // 存储预备信息
    data.Pre_Encoder = tmp_encoder;
}

/**
 * @brief 电机数据输出到CAN总线
 *
 */
void Class_Motor_DM_Normal::Output() {
    // 电机控制
    switch (Motor_DM_Control_Method) {
        case (Motor_DM_Control_Method_NORMAL_MIT): {
            Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *) Tx_Data;

            uint16_t tmp_angle, tmp_omega, tmp_torque, tmp_k_p, tmp_k_d;

            tmp_angle = Math_Float_To_Int(Control_Radian, 0, Radian_Max, 0x7fff, (1 << 16) - 1);
            tmp_omega = Math_Float_To_Int(Control_Omega, 0, Omega_Max, 0x7ff, (1 << 12) - 1);
            tmp_torque = Math_Float_To_Int(Control_Torque, 0, Torque_Max, 0x7ff, (1 << 12) - 1);
            tmp_k_p = Math_Float_To_Int(K_P, 0, 500.0f, 0, (1 << 12) - 1);
            tmp_k_d = Math_Float_To_Int(K_D, 0, 5.0f, 0, (1 << 12) - 1);

            tmp_buffer->Control_Angle_Reverse = Math_Endian_Reverse_16(&tmp_angle, nullptr);
            tmp_buffer->Control_Omega_11_4 = tmp_omega >> 4;
            tmp_buffer->Control_Omega_3_0_K_P_11_8 = ((tmp_omega & 0x0f) << 4) | (tmp_k_p >> 8);
            tmp_buffer->K_P_7_0 = tmp_k_p & 0xff;
            tmp_buffer->K_D_11_4 = tmp_k_d >> 4;
            tmp_buffer->K_D_3_0_Control_Torque_11_8 = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
            tmp_buffer->Control_Torque_7_0 = tmp_torque & 0xff;

    linkx_quick_can_send(LinkX_Handler, CAN_Channel, DM_CAN_Tx_ID, DM_Motor_CAN_Message_Save_Zero);

            break;
        }
        case (Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA): {
            Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega *) Tx_Data;

            tmp_buffer->Control_Angle = Control_Radian;
            tmp_buffer->Control_Omega = Control_Omega;

                linkx_quick_can_send(LinkX_Handler, CAN_Channel, DM_CAN_Tx_ID, Tx_Data);

            break;
        }
        case (Motor_DM_Control_Method_NORMAL_OMEGA): {
            Struct_Motor_DM_CAN_Tx_Data_Normal_Omega *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_Omega *) Tx_Data;

            tmp_buffer->Control_Omega = Control_Omega;

                linkx_quick_can_send(LinkX_Handler, CAN_Channel, DM_CAN_Tx_ID, Tx_Data);

            break;
        }
        case (Motor_DM_Control_Method_NORMAL_EMIT): {
            Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT *) Tx_Data;

            tmp_buffer->Control_Angle = Control_Radian;
            tmp_buffer->Control_Omega = (uint16_t) (Control_Omega * 100.0f);
            tmp_buffer->Control_Current = (uint16_t) (Control_Current / Current_Max * 10000.0f);

            linkx_quick_can_send(LinkX_Handler, CAN_Channel, DM_CAN_Tx_ID, Tx_Data);

            break;
        }
    }
}
