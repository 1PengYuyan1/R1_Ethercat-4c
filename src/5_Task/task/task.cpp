#include "task.h"

#include "ecat_manager.h"
#include "linkx4c_handler.h"
#include "robot.h"

#include <chrono>
#include <iostream>
#include <thread>

// 全局机器人对象
Class_Robot robot;

// EtherCAT 相关句柄
ecat_master_t master;
linkx_t linkx_dev;

bool init_finished = false;

static constexpr uint32_t kCanStatPrintPeriodMs = 200;

static void Print_CAN_Stats(linkx_t *linkx)
{
    std::cout << "\n[CAN-TOTAL]\n";

    uint64_t sum_tx_f = 0;
    uint64_t sum_rx_f = 0;
    uint64_t sum_tx_b = 0;
    uint64_t sum_rx_b = 0;

    for (int ch = 0; ch < 4; ch++)
    {
        uint64_t tx_f = linkx->can_stats[ch].tx_frames;
        uint64_t rx_f = linkx->can_stats[ch].rx_frames;
        uint64_t tx_b = linkx->can_stats[ch].tx_bytes;
        uint64_t rx_b = linkx->can_stats[ch].rx_bytes;
        uint64_t loss_f = (tx_f > rx_f) ? (tx_f - rx_f) : 0;
        double loss_rate = (tx_f > 0) ? (100.0 * (double)loss_f / (double)tx_f) : 0.0;

        std::cout << "  CH" << ch
                  << " TX_TOTAL=" << tx_f << " frames (" << tx_b << " B)"
                  << " RX_TOTAL=" << rx_f << " frames (" << rx_b << " B)"
                  << " TOTAL_LOSS=" << loss_f
                  << " TOTAL_LOSS_RATE=" << loss_rate << "%"
                  << std::endl;

        sum_tx_f += tx_f;
        sum_rx_f += rx_f;
        sum_tx_b += tx_b;
        sum_rx_b += rx_b;
    }

    uint64_t sum_loss_f = (sum_tx_f > sum_rx_f) ? (sum_tx_f - sum_rx_f) : 0;
    double sum_loss_rate = (sum_tx_f > 0) ? (100.0 * (double)sum_loss_f / (double)sum_tx_f) : 0.0;
    std::cout << "  ALL TX_TOTAL=" << sum_tx_f << " frames (" << sum_tx_b << " B)"
              << " RX_TOTAL=" << sum_rx_f << " frames (" << sum_rx_b << " B)"
              << " TOTAL_LOSS=" << sum_loss_f
              << " TOTAL_LOSS_RATE=" << sum_loss_rate << "%"
              << std::endl;
}

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

    // 在 SAFE_OP 阶段先完成 LinkX 通道初始化
    linkx_hw_wakeup(&linkx_dev);

    // 为 4 个通道设置 1M 波特率 (非 FD 模式)
    for (int i = 0; i < 4; i++)
    {
        linkx_set_can_baudrate(&linkx_dev, i, 0, 1, 31, 8, 8, 1, 31, 8, 8);
    }

    // 将 EtherCAT 网络带入运行状态 (OP)
    if (!ecat_master_bring_online(&master))
        return;

    // 初始化机器人逻辑类（上位机版本）
    robot.Init(&linkx_dev);
    std::cout << "[TASK] Robot Logic Initialized." << std::endl;

    auto next_wakeup = std::chrono::steady_clock::now();
    uint32_t tick_count = 0;

    while (master.is_running)
    {
        next_wakeup += std::chrono::milliseconds(1); // 1ms (1000Hz)

        // EtherCAT 同步过程数据
        ecat_master_sync(&master);

        // 将 EtherCAT 输入PDO拷贝到 LinkX 接收缓存，供 linkx_quick_recv 读取
        linkx_recv_pdos(&linkx_dev);

        // 接收与分发 (Read)
        can_msg_t recv_msg;
        for (uint8_t ch = 0; ch < 4; ch++)
        {
            while (linkx_quick_recv(&linkx_dev, ch, &recv_msg))
            {
                robot.CAN_Rx_Callback(ch, recv_msg.id, recv_msg.data);
            }
        }

        // 与单片机任务节拍对齐：1ms / 2ms / 100ms
        robot.TIM_1ms_Calculate_Callback();

        if ((tick_count % 2) == 0)
            robot.TIM_2ms_Calculate_PeriodElapsedCallback();

        if ((tick_count % 100) == 0)
            robot.TIM_100ms_Alive_PeriodElapsedCallback();

        // 提交发送 (Write)
        linkx_send_pdos(&linkx_dev);

        if ((tick_count % kCanStatPrintPeriodMs) == 0 && tick_count != 0)
            Print_CAN_Stats(&linkx_dev);

        tick_count++;
        std::this_thread::sleep_until(next_wakeup);
    }
}
