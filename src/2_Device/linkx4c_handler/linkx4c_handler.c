
#include "linkx4c_handler.h"

#include "stdio.h"
#include "string.h"

#include "soem/osal.h"    // 提供基础操作系统抽象类型
#include "soem/ec_type.h" // 提供 uint16, uint8, ecx_contextt 等类型定义
#include "soem.h"         // 提供 SOEM 核心功能
#include "soem/ec_coe.h"  // 提供 SDO 读写函数 (ecx_SDOwrite)

#include "unistd.h"

// 封装 SDO 唤醒逻辑，屏蔽 0x8001 寄存器细节
bool linkx_hw_wakeup(linkx_t *linkx)
{
    uint8_t enable = 1;
    printf("[LinkX] Waking up CAN PHYs via SDO...\n");

    for (int ch = 0; ch < 4; ch++)
    {
        bool ok = linkx_switch_can_channel(linkx, ch, enable != 0);
        if (ok) printf("[LinkX] CAN Channel %d: WAKEUP SUCCESS\n", ch);
        else printf("[LinkX] CAN Channel %d: WAKEUP FAILED\n", ch);
    }
    printf("[LinkX] All hardware initialization commands sent.\n");
    return true;
}

// 封装发送：FDCAN自动处理指针强转和 linkx 参数顺序
void linkx_quick_FDcan_send(linkx_t *linkx, uint8_t ch, uint32_t id, uint8_t *data)
{
    // 参数顺序：linkx, channel, id, canfd, brs, ext, rtr, dlen, data
    linkx_send_can(linkx, ch, id, true, true, false, false, 8, (uint32_t *)data);
}

// 封装发送：经典CAN自动处理指针强转和 linkx 参数顺序
void linkx_quick_can_send(linkx_t *linkx, uint8_t ch, uint32_t id, uint8_t *data)
{
    // 参数顺序：linkx, channel, id, canfd, brs, ext, rtr, dlen, data
    linkx_send_can(linkx, ch, id, false, false, false, false, 8, (uint32_t *)data);
}

// 封装接收：内置硬件时间戳去重逻辑
bool linkx_quick_recv(linkx_t *linkx, uint8_t ch, can_msg_t *out_msg)
{
    static uint64_t last_timestamps[4] = {0};
    can_tx_pdo_t *rx_pdo = linkx_recv_can(linkx, ch); // 获取指向接收 PDO 的指针

    if (rx_pdo != NULL && rx_pdo->can_id != 0 && rx_pdo->timestamp != last_timestamps[ch])
    {
        out_msg->id = rx_pdo->can_id;               // CAN ID
        out_msg->dlen = rx_pdo->params.dlen;        // 数据长度
        out_msg->timestamp = rx_pdo->timestamp;     // 硬件时间戳
        memcpy(out_msg->data, rx_pdo->data_u32, 8); // 数据拷贝

        last_timestamps[ch] = rx_pdo->timestamp; // 更新时间戳
        linkx->can_stats[ch].rx_frames++;
        linkx->can_stats[ch].rx_bytes += out_msg->dlen;
        return true;
    }
    return false;
}

// 配置指定 CAN 通道的波特率 (仲裁段和数据段)
bool linkx_set_can_baudrate(linkx_t *linkx, uint8_t ch, uint8_t fd_en,
                          uint8_t n_pre, uint8_t n_seg1, uint8_t n_seg2, uint8_t n_sjw,
                          uint8_t d_pre, uint8_t d_seg1, uint8_t d_seg2, uint8_t d_sjw)
{
    if (ch >= 4) return false;

    uint16_t index = 0x8002;  // 统一的时序配置索引
    int wkc_count = 0;        // 记录成功的 SDO 写入次数
    uint8_t val;

    printf("[LinkX] Configuring CAN %d Timings...\n", ch);

    // 1. 指定要配置的通道号 (SubIndex 0x01)
    val = ch;
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x01, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);

    // 2. 使能 CANFD (SubIndex 0x02)
    val = fd_en;
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x02, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);

    // 3-6. 写入仲裁段 (Nominal) 经典 CAN 波特率参数
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x03, FALSE, sizeof(n_pre), &n_pre, EC_TIMEOUTRXM);
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x04, FALSE, sizeof(n_seg1), &n_seg1, EC_TIMEOUTRXM);
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x05, FALSE, sizeof(n_seg2), &n_seg2, EC_TIMEOUTRXM);
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x06, FALSE, sizeof(n_sjw), &n_sjw, EC_TIMEOUTRXM);

    // 7-10. 写入数据段 (Data) CAN-FD 高速波特率参数
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x07, FALSE, sizeof(d_pre), &d_pre, EC_TIMEOUTRXM);
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x08, FALSE, sizeof(d_seg1), &d_seg1, EC_TIMEOUTRXM);
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x09, FALSE, sizeof(d_seg2), &d_seg2, EC_TIMEOUTRXM);
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x0A, FALSE, sizeof(d_sjw), &d_sjw, EC_TIMEOUTRXM);

    // 11. 配置生效触发 (SubIndex 0x0B)
    val = 1;
    wkc_count += ecx_SDOwrite(linkx->master, linkx->slave_id, index, 0x0B, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);

    // 一共进行了 11 次 SDO 写入，全部成功才算配置成功
    if (wkc_count == 11) {
        printf("[LinkX] CAN Channel %d Timing configured successfully!\n", ch);
        return true;
    } else {
        printf("[LinkX] CAN Channel %d Timing config FAILED! (Success Count = %d/11)\n", ch, wkc_count);
        return false;
    }
}
