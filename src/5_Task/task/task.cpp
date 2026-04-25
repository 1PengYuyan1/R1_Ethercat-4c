#include "task.h"
#include "robot.h"
#include "ecat_manager.h"
#include "linkx4c_handler.h"
#include <chrono>
#include <thread>
#include <iostream>

// 全局机器人对象
Class_Robot robot;

// EtherCAT 相关句柄
ecat_master_t master;
linkx_t linkx_dev;

bool init_finished = false;

/**
 * @brief 核心控制任务逻辑
 */
void Robot_Control_Loop(const char *ifname)
{
    // 初始化 EtherCAT 主站
    if (!ecat_master_init(&master, ifname))
        return;
    // 初始化 LinkX 适配器 (假设 LinkX 是总线上的第 1 个从站)
    linkx_init(&linkx_dev, 1, &master.ctx);
    // 将 EtherCAT 网络带入运行状态 (OP)
    if (!ecat_master_bring_online(&master))
        return;
    // 唤醒 CAN 控制器并设置波特率
    linkx_hw_wakeup(&linkx_dev);

    // 为 4 个通道设置 1M 波特率 (非 FD 模式)
    for (int i = 0; i < 4; i++)
    {
        linkx_set_can_baudrate(&linkx_dev, i, 0, 1, 31, 8, 8, 1, 31, 8, 8);
    }

    // 初始化机器人逻辑类
    robot.Init(&linkx_dev);
    std::cout << "[TASK] Robot Logic Initialized." << std::endl;

    // 定时器变量
    auto next_wakeup = std::chrono::steady_clock::now();
    uint32_t tick_count = 0;

    // --- 实时控制循环 ---
    while (master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(1); // 1ms (1000Hz)

        // 同步过程数据
        // 将 EtherCAT 总线上的数据交换到本地内存
        ecat_master_sync(&master);

        // 接收与分发 (Read)
        // 立即处理所有刚收到的 CAN 帧，更新所有电机/编码器对象的状态
        can_msg_t recv_msg;
        for (uint8_t ch = 0; ch < 4; ch++)
        {
            while (linkx_quick_recv(&linkx_dev, ch, &recv_msg))
            {
                robot.CAN_Rx_Callback(ch, recv_msg.id, recv_msg.data);
            }
        }

        // 逻辑计算 (Process)
        // 此时所有对象的数据都是最新的，计算控制量
        // 1ms 任务
        robot.TIM_1ms_Calculate_Callback();
        // 2ms 任务
        if (tick_count % 2 == 0)
            robot.TIM_2ms_Calculate_PeriodElapsedCallback();
        // 100ms 任务 (存活检查等)
        if (tick_count % 100 == 0)
            robot.TIM_100ms_Alive_PeriodElapsedCallback();

        // 执行状态机和底盘解算
        robot.Loop();

        // 提交发送 (Write)
        // 将计算出的电机指令压入 LinkX 的发送缓冲区
        linkx_send_pdos(&linkx_dev);
        tick_count++;
        std::this_thread::sleep_until(next_wakeup);
        
    }
}
