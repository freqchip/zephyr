#if 0
/*
 * Copyright (c) 2026 Freqchip
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * FR802X BLE Controller HCI driver (memory-based, in-chip HCI).
 *
 * This driver connects the Zephyr Bluetooth Host to the fr802x
 * BLE controller library (libbtdm_controller.a) running on the
 * same chip. All HCI data is exchanged in memory (no UART/SPI).
 *
 * Reference: zephyr/drivers/bluetooth/hci/hci_esp32.c
 */

#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/drivers/bluetooth.h>

#include "btdm_controller.h"

#define DT_DRV_COMPAT freqchip_fr802x_bt_hci

//#define FR802X_HCI_DEBUG_LOG

struct fr802x_hci_data {
    bt_hci_recv_t recv;   /* Zephyr Host 提供的接收回调 */
};

/*
 * 覆盖 Zephyr 默认的 z_arm_svc
 * 直接用 patch_svc_handler（C 函数，无 LR 判断）匹配 BLE ROM 补丁
 * - 匹配到：异常帧 PC 改为补丁函数地址，异常返回后执行
 * - 未匹配：恢复原始 PC，走 z_arm_svc_original
 */
__attribute__((naked)) void z_arm_svc(void)
{
    __asm volatile(
        /* 定位异常帧地址到 r2（用原始 LR=EXC_RETURN，此时还没 push） */
        "tst     lr, #4                      \n"
        "ite     eq                          \n"
        "mrseq   r2, MSP                     \n"
        "mrsne   r2, PSP                     \n"
        /* 压栈保存异常帧地址(r2)和EXC_RETURN(lr)，栈=[sp+0]r2, [sp+4]lr */
        "push    {r2, lr}                    \n"
        /* r3 = 原始 PC */
        "ldr     r3, [r2, #0x18]             \n"
        /* r0 = 原始 PC，调 patch_svc_handler */
        "mov     r0, r3                      \n"
        "bl      patch_svc_handler           \n"
        /* r0 = 补丁函数指针或 0 */
        "cbz     r0, 1f                      \n"
        /* 匹配成功：从栈恢复帧地址([sp+0])，写回补丁地址 */
        "ldr     r2, [sp, #0]                \n"
        "str     r0, [r2, #0x18]             \n"
        /* 恢复 EXC_RETURN 到 lr，异常返回 */
        "ldr     lr, [sp, #4]                \n"
        "add     sp, sp, #8                  \n"
        "bx      lr                          \n"
        "1:                                  \n"
        /* 未匹配：从栈恢复帧地址，恢复原始 PC */
        "ldr     r2, [sp, #0]                \n"
        "str     r3, [r2, #0x18]             \n"
        "ldr     lr, [sp, #4]                \n"
        "add     sp, sp, #8                  \n"
        "b       z_arm_svc_original          \n"
        :
        :
        : "r0", "r2", "r3"
    );
}

/* FreeRTOS 内存分配 → C 库 malloc/free（BLE 库需要） */
void *pvPortMalloc(size_t size)
{
    return malloc(size);
}

void vPortFree(void *ptr)
{
    free(ptr);
}

/* 控制器自定义参数（从原 controller.c 迁移） */
static const uint8_t controller_param[] = {
    0x80, 32,
    0x79, 0x87, 0x07, 0x5d, 0xa1, 0xa4, 0x11, 0x68, 0xe0, 0x65, 0x15, 0x06, 0x7c, 0xb8, 0x66, 0x47,
    0x1f, 0x94, 0xbd, 0x7f, 0x4f, 0x9b, 0xd3, 0x15, 0xf6, 0x71, 0x68, 0x13, 0xda, 0x91, 0x75, 0x30,

    0x81, 64,
    0x5e, 0xa6, 0xf6, 0x32, 0xe8, 0x4d, 0x6d, 0x22, 0xaa, 0x16, 0x95, 0x67, 0xad, 0x10, 0x4e, 0x9d,
    0xd3, 0xde, 0xef, 0xe3, 0x46, 0xfa, 0x68, 0xf6, 0x08, 0x17, 0x9d, 0xb6, 0x1a, 0x61, 0xe0, 0x0e,
    0x46, 0x5c, 0xfe, 0x4f, 0x21, 0x3c, 0x41, 0xfa, 0x9b, 0xe9, 0x27, 0xb2, 0xd3, 0x7a, 0x90, 0xcb,
    0x9d, 0x28, 0x16, 0x41, 0x4d, 0x63, 0xcd, 0x62, 0x95, 0x3e, 0xe1, 0x21, 0x9b, 0x0d, 0x3c, 0xb6,
};

/* 控制器 MAC 地址 */
static const uint8_t fr802x_bt_addr[6] = {0x32, 0x32, 0x32, 0x32, 0x32, 0x32};

/* 内存分配: C 库 malloc/free */
static void *fr802x_malloc(uint32_t size)
{
    return malloc(size);
}

static void fr802x_free(void *ptr)
{
    free(ptr);
}

/* ==================== 内存 HCI 传输（eif_api） ==================== */

/*
 * Host → Controller 方向（读请求）。
 * 控制器调用 read() 注册读请求，Host 通过 send() 发来命令后交付数据并回调。
 * 采用异步请求注册模式（与 UART HCI 版一致）：
 *   read() 立即返回，不阻塞控制器 schedule 循环。
 */
static uint8_t  fr802x_pending_cmd[512];
static int32_t  fr802x_pending_len;
static uint32_t fr802x_pending_cnt;

static uint8_t *fr802x_read_bufptr = NULL;
static uint32_t fr802x_read_size;
static btdm_controller_tl_callback fr802x_read_callback;
static void *fr802x_read_dummy;

static struct k_spinlock fr802x_hci_lock;

/*
 * 命令写入权信号量：保证上一条命令被控制器取走后，才允许 Host 写入下一条，
 * 避免覆盖未交付的命令。初始计数 1（允许第一条写入）。
 */
static K_SEM_DEFINE(fr802x_send_ready, 1, 1);

static K_SEM_DEFINE(fr802x_ble_control, 0, K_SEM_MAX_LIMIT);

/*
 * 控制器 → Host 的方向。
 * 控制器调用 write() 发出 HCI 事件/ACL，我们解析后通过 recv() 交给 Host。
 */
static bt_hci_recv_t fr802x_host_recv;
static const struct device *fr802x_dev;

/* 解析并上报 HCI 事件 */
static struct net_buf *fr802x_evt_recv(uint8_t *data, size_t remaining)
{
    struct bt_hci_evt_hdr hdr;
    struct net_buf *buf;
    size_t buf_tailroom;

    if (remaining < sizeof(hdr)) {
        return NULL;
    }

    memcpy(&hdr, data, sizeof(hdr));
    data += sizeof(hdr);
    remaining -= sizeof(hdr);

    if (remaining != hdr.len) {
        return NULL;
    }

    buf = bt_buf_get_evt(hdr.evt, false, K_NO_WAIT);
    if (!buf) {
        printk("EVT: get_evt FAILED!\n");   // ← 加这个
        return NULL;
    }

    net_buf_add_mem(buf, &hdr, sizeof(hdr));

    buf_tailroom = net_buf_tailroom(buf);
    if (buf_tailroom < remaining) {
        net_buf_unref(buf);
        return NULL;
    }

    net_buf_add_mem(buf, data, remaining);

    return buf;
}

/* 解析并上报 ACL 数据 */
static struct net_buf *fr802x_acl_recv(uint8_t *data, size_t remaining)
{
    struct bt_hci_acl_hdr hdr;
    struct net_buf *buf;
    size_t buf_tailroom;

    if (remaining < sizeof(hdr)) {
        return NULL;
    }

    buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_NO_WAIT);
    if (!buf) {
        return NULL;
    }

    memcpy(&hdr, data, sizeof(hdr));
    data += sizeof(hdr);
    remaining -= sizeof(hdr);

    if (remaining != sys_le16_to_cpu(hdr.len)) {
        net_buf_unref(buf);
        return NULL;
    }

    net_buf_add_mem(buf, &hdr, sizeof(hdr));

    buf_tailroom = net_buf_tailroom(buf);
    if (buf_tailroom < remaining) {
        net_buf_unref(buf);
        return NULL;
    }

    net_buf_add_mem(buf, data, remaining);

    return buf;
}

/*
 * 控制器通过 tl_api.write 上报数据（H4 帧格式: 类型字节 + 数据）
 * 我们在线程上下文调用，因此可以直接 recv() 给 Host。
 */
static void fr802x_hci_write(uint8_t *bufptr, uint32_t size, btdm_controller_tl_callback callback, void *dummy)
{
#ifdef FR802X_HCI_DEBUG_LOG
    printf("W: size=%u data: ", (unsigned int)size);
    for (uint32_t i = 0; i < size; i++) 
        printf("%02x ", bufptr[i]);
    printf("\n");
#endif

    uint8_t pkt_indicator;
    struct net_buf *buf = NULL;
    size_t remaining = size;

    pkt_indicator = *bufptr++;
    remaining -= 1;

    switch (pkt_indicator) 
    {
        case BT_HCI_H4_EVT: buf = fr802x_evt_recv(bufptr, remaining);break;
        case BT_HCI_H4_ACL: buf = fr802x_acl_recv(bufptr, remaining);break;
        default:break;
    }

    if (buf) {
        fr802x_host_recv(fr802x_dev, buf);
    }

    callback(dummy, 0);
}

static void fr802x_hci_read(uint8_t *bufptr, uint32_t size, btdm_controller_tl_callback callback, void *dummy)
{
    k_spinlock_key_t key = k_spin_lock(&fr802x_hci_lock);

    if (fr802x_read_size){
        printk("ERROR: fr802x_read_size > 0\n");
        while(1);
    }

    fr802x_read_bufptr   = bufptr;
    fr802x_read_size     = size;
    fr802x_read_callback = callback;
    fr802x_read_dummy    = dummy;

    k_spin_unlock(&fr802x_hci_lock, key);
}

static void fr802x_hci_flow_on(void)
{
}

static bool fr802x_hci_flow_off(void)
{
    return true;
}

static void fr802x_controller_notify(void)
{
    k_sem_give(&fr802x_ble_control);
}

static struct btdm_controller_tl_api fr802x_eif_api = {
    .read     = fr802x_hci_read,
    .write    = fr802x_hci_write,
    .flow_on  = fr802x_hci_flow_on,
    .flow_off = fr802x_hci_flow_off,
};

static const struct btdm_controller_itf fr802x_itf = {
    ._malloc = fr802x_malloc,
    ._free   = fr802x_free,
    .notify  = fr802x_controller_notify,
    .tl_api  = &fr802x_eif_api,
};

/* ==================== 控制器调度线程 ==================== */

#define FR802X_CONTROLLER_STACK_SIZE 4096
#define FR802X_CONTROLLER_PRIORITY   2

void BB_IRQHandler(void);
void BB_BLE_IRQHandler(void);
void BB_IRQHandler_dynamic(const void *dev)
{
#ifdef FR802X_HCI_DEBUG_LOG
    printf(".");
#endif
    BB_IRQHandler();
}
void BB_BLE_IRQHandler_dynamic(const void *dev)
{
#ifdef FR802X_HCI_DEBUG_LOG
    printf("-");
#endif
    BB_BLE_IRQHandler();
}

static K_THREAD_STACK_DEFINE(fr802x_controller_stack, FR802X_CONTROLLER_STACK_SIZE);
static struct k_thread fr802x_controller_thread;

/* 在线程上下文执行控制器初始化（与之前 UART HCI 手动测试环境一致）。
 * 不能在 POST_KERNEL device init 里做：ROM 补丁 SVC 在 init 阶段
 * 会把控制流劫持到 idle，导致 init 无法返回。 */
static void fr802x_controller_init_routine(void)
{
    struct btdm_controller_cfg *cfg;
    int BB_IP_IRQn_num, BB_BLE_IRQn_num, BB_IP_IRQn_prio, BB_BLE_IRQn_prio;

    cfg = malloc(sizeof(struct btdm_controller_cfg));
    if (!cfg) {
        return;
    }

    cfg->ble_public_addr    = fr802x_bt_addr;
    cfg->custom_param       = controller_param;
    cfg->custom_param_len   = sizeof(controller_param);
    cfg->release_rst_delay  = 2000;
    cfg->coded_phy_sel      = 0;
    cfg->conn_max           = 6;
    cfg->adv_max            = 2;
    cfg->adv_pkt_len_max    = 37;
    cfg->ble_tx_pkt_nb      = 8;
    cfg->ble_rx_pkt_nb      = 8;

    btdm_controller_init(cfg, &fr802x_itf);
    free(cfg);

    /* 按索引获取中断号和优先级 */
    BB_IP_IRQn_num  = DT_IRQ_BY_IDX(DT_DRV_INST(0), 0, irq);
    BB_IP_IRQn_prio = DT_IRQ_BY_IDX(DT_DRV_INST(0), 0, priority);
    BB_BLE_IRQn_num  = DT_IRQ_BY_IDX(DT_DRV_INST(0), 1, irq);
    BB_BLE_IRQn_prio = DT_IRQ_BY_IDX(DT_DRV_INST(0), 1, priority);

    /* 注册中断处理函数 */
    irq_connect_dynamic(BB_IP_IRQn_num, BB_IP_IRQn_prio, BB_IRQHandler_dynamic, fr802x_dev, 0);
    irq_connect_dynamic(BB_BLE_IRQn_num, BB_BLE_IRQn_prio, BB_BLE_IRQHandler_dynamic, fr802x_dev, 0);
    irq_enable(BB_IP_IRQn_num);
    irq_enable(BB_BLE_IRQn_num);
}

static void fr802x_controller_task(void *arg1, void *arg2, void *arg3)
{
    fr802x_controller_init_routine();

    while (1) 
    {
        k_spinlock_key_t key = k_spin_lock(&fr802x_hci_lock);

        /* 交付HCI命令 */
        uint32_t remaining = fr802x_pending_len - fr802x_pending_cnt;
        if (remaining > 0) {
            if (fr802x_read_size){
                memcpy(fr802x_read_bufptr, &fr802x_pending_cmd[fr802x_pending_cnt], fr802x_read_size);
                
#ifdef FR802X_HCI_DEBUG_LOG
                printf("S:");
                for (int i = 0; i < fr802x_read_size; i++)
                    printf("%02x ", fr802x_pending_cmd[fr802x_pending_cnt + i]);
                printf("\n");
#endif
                fr802x_pending_cnt += fr802x_read_size;
                fr802x_read_size = 0;
                fr802x_read_callback(fr802x_read_dummy, 0);
            }
        }
        /* 交付完成 */
        remaining = fr802x_pending_len - fr802x_pending_cnt;
        if (remaining == 0){
            if (fr802x_pending_len > 0){
                fr802x_pending_len = 0;
                fr802x_pending_cnt = 0;
                k_sem_give(&fr802x_send_ready);
            }
        }
        else if (remaining > 0){
            /* 还有未发送的数据 */
            k_sem_give(&fr802x_ble_control);
        }
        else{
            printk("ERROR: fr802x_pending_len < 0");
            while(1);
        }
        k_spin_unlock(&fr802x_hci_lock, key);
        
        k_sem_take(&fr802x_ble_control, K_FOREVER);
        btdm_controller_schedule();
    }
}

/* 启动控制器调度线程 */
static void fr802x_controller_start(void)
{
    k_thread_create(&fr802x_controller_thread,
                    fr802x_controller_stack, FR802X_CONTROLLER_STACK_SIZE,
                    fr802x_controller_task, NULL, NULL, NULL,
                    FR802X_CONTROLLER_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&fr802x_controller_thread, "BLE_ctrl");
}

/* ==================== bt_hci_driver_api ==================== */

static int fr802x_hci_open(const struct device *dev, bt_hci_recv_t recv)
{
    struct fr802x_hci_data *hci = dev->data;

    hci->recv = recv;

    fr802x_dev = dev;
    fr802x_host_recv = recv;

    /* 启动控制器调度线程 */
    fr802x_controller_start();

    return 0;
}

static int fr802x_hci_send(const struct device *dev, struct net_buf *buf)
{
    uint8_t *data = buf->data;
    size_t len = buf->len;
    int err = 0;
    uint8_t type;
    uint8_t h4_type;

    type = bt_buf_get_type(buf);

    /* 确定 H4 类型字节（标准 HCI 帧头），控制器按此解析 */
    switch (type)
    {
        case BT_BUF_CMD: h4_type = BT_HCI_H4_CMD; break;   /* 0x01 */
        case BT_BUF_ACL_OUT: h4_type = BT_HCI_H4_ACL; break;   /* 0x02 */
            
        default:err = -EINVAL; break;
    }

    /* 数据 + 1 字节 H4 类型不能超过待处理缓冲 */
    if (len + 1 > sizeof(fr802x_pending_cmd)) {
        err = -EINVAL;
    }

    if (err !=0 ){
        net_buf_unref(buf);
        return err;
    }

    /* 等待上一条命令被控制器取走（fr802x_send_ready 计数>0）后才可写入，
     * 避免覆盖未交付的命令。 */
    k_sem_take(&fr802x_send_ready, K_FOREVER);

    k_spinlock_key_t key = k_spin_lock(&fr802x_hci_lock);
    /* 构造带 H4 类型字节的标准 HCI 帧*/
    fr802x_pending_cmd[0] = h4_type;
    memcpy(&fr802x_pending_cmd[1], data, len);
    /* 长度,计数 */
    fr802x_pending_len = len + 1;
    fr802x_pending_cnt = 0;
    k_spin_unlock(&fr802x_hci_lock, key);

    k_sem_give(&fr802x_ble_control);

#ifdef FR802X_HCI_DEBUG_LOG
    printf("R:");
    for (int i = 0; i < fr802x_pending_len; i++)
        printf("%02x ", fr802x_pending_cmd[i]);
    printf("\n");
#endif

    net_buf_unref(buf);
    
    return err;
}

static int fr802x_hci_close(const struct device *dev)
{
    k_thread_abort(&fr802x_controller_thread);
    return 0;
}

static int fr802x_hci_init(const struct device *dev)
{
    ARG_UNUSED(dev);
    return 0;
}

static const struct bt_hci_driver_api drv = {
    .open  = fr802x_hci_open,
    .send  = fr802x_hci_send,
    .close = fr802x_hci_close,
};

#define FR802X_HCI_DEVICE_INIT(inst) \
    static struct fr802x_hci_data fr802x_hci_data_##inst; \
    DEVICE_DT_INST_DEFINE(inst, fr802x_hci_init, NULL, &fr802x_hci_data_##inst, NULL, \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &drv)

/* Only one instance supported */
FR802X_HCI_DEVICE_INIT(0)

#else

/*
 * Copyright (c) 2026 Freqchip
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * FR802X BLE Controller HCI driver (memory-based, in-chip HCI).
 *
 * This driver connects the Zephyr Bluetooth Host to the fr802x
 * BLE controller library (libbtdm_controller.a) running on the
 * same chip. All HCI data is exchanged in memory (no UART/SPI).
 *
 * Reference: zephyr/drivers/bluetooth/hci/hci_esp32.c
 */

#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/drivers/bluetooth.h>

#include "btdm_controller.h"

#define DT_DRV_COMPAT freqchip_fr802x_bt_hci

//#define FR802X_HCI_DEBUG_LOG

struct fr802x_hci_data {
    bt_hci_recv_t recv;   /* Zephyr Host 提供的接收回调 */
};

/*
 * 覆盖 Zephyr 默认的 z_arm_svc
 * 直接用 patch_svc_handler（C 函数，无 LR 判断）匹配 BLE ROM 补丁
 * - 匹配到：异常帧 PC 改为补丁函数地址，异常返回后执行
 * - 未匹配：恢复原始 PC，走 z_arm_svc_original
 */
__attribute__((naked)) void z_arm_svc(void)
{
    __asm volatile(
        /* 定位异常帧地址到 r2（用原始 LR=EXC_RETURN，此时还没 push） */
        "tst     lr, #4                      \n"
        "ite     eq                          \n"
        "mrseq   r2, MSP                     \n"
        "mrsne   r2, PSP                     \n"
        /* 压栈保存异常帧地址(r2)和EXC_RETURN(lr)，栈=[sp+0]r2, [sp+4]lr */
        "push    {r2, lr}                    \n"
        /* r3 = 原始 PC */
        "ldr     r3, [r2, #0x18]             \n"
        /* r0 = 原始 PC，调 patch_svc_handler */
        "mov     r0, r3                      \n"
        "bl      patch_svc_handler           \n"
        /* r0 = 补丁函数指针或 0 */
        "cbz     r0, 1f                      \n"
        /* 匹配成功：从栈恢复帧地址([sp+0])，写回补丁地址 */
        "ldr     r2, [sp, #0]                \n"
        "str     r0, [r2, #0x18]             \n"
        /* 恢复 EXC_RETURN 到 lr，异常返回 */
        "ldr     lr, [sp, #4]                \n"
        "add     sp, sp, #8                  \n"
        "bx      lr                          \n"
        "1:                                  \n"
        /* 未匹配：从栈恢复帧地址，恢复原始 PC */
        "ldr     r2, [sp, #0]                \n"
        "str     r3, [r2, #0x18]             \n"
        "ldr     lr, [sp, #4]                \n"
        "add     sp, sp, #8                  \n"
        "b       z_arm_svc_original          \n"
        :
        :
        : "r0", "r2", "r3"
    );
}

/* FreeRTOS 内存分配 → C 库 malloc/free（BLE 库需要） */
void *pvPortMalloc(size_t size)
{
    return malloc(size);
}

void vPortFree(void *ptr)
{
    free(ptr);
}

/* 控制器自定义参数（从原 controller.c 迁移） */
static const uint8_t controller_param[] = {
    0x80, 32,
    0x79, 0x87, 0x07, 0x5d, 0xa1, 0xa4, 0x11, 0x68, 0xe0, 0x65, 0x15, 0x06, 0x7c, 0xb8, 0x66, 0x47,
    0x1f, 0x94, 0xbd, 0x7f, 0x4f, 0x9b, 0xd3, 0x15, 0xf6, 0x71, 0x68, 0x13, 0xda, 0x91, 0x75, 0x30,

    0x81, 64,
    0x5e, 0xa6, 0xf6, 0x32, 0xe8, 0x4d, 0x6d, 0x22, 0xaa, 0x16, 0x95, 0x67, 0xad, 0x10, 0x4e, 0x9d,
    0xd3, 0xde, 0xef, 0xe3, 0x46, 0xfa, 0x68, 0xf6, 0x08, 0x17, 0x9d, 0xb6, 0x1a, 0x61, 0xe0, 0x0e,
    0x46, 0x5c, 0xfe, 0x4f, 0x21, 0x3c, 0x41, 0xfa, 0x9b, 0xe9, 0x27, 0xb2, 0xd3, 0x7a, 0x90, 0xcb,
    0x9d, 0x28, 0x16, 0x41, 0x4d, 0x63, 0xcd, 0x62, 0x95, 0x3e, 0xe1, 0x21, 0x9b, 0x0d, 0x3c, 0xb6,
};

/* 控制器 MAC 地址 */
static const uint8_t fr802x_bt_addr[6] = {0x32, 0x32, 0x32, 0x32, 0x32, 0x32};

/* 内存分配: C 库 malloc/free */
static void *fr802x_malloc(uint32_t size)
{
    return malloc(size);
}

static void fr802x_free(void *ptr)
{
    free(ptr);
}

/* ==================== 内存 HCI 传输（eif_api） ==================== */

/*
 * Host → Controller 方向（读请求）。
 * 控制器调用 read() 注册读请求，Host 通过 send() 发来命令后交付数据并回调。
 * 采用异步请求注册模式（与 UART HCI 版一致）：
 *   read() 立即返回，不阻塞控制器 schedule 循环。
 */
static uint8_t  fr802x_pending_cmd[512];
static int32_t  fr802x_pending_len;
static uint32_t fr802x_pending_cnt;

static uint8_t *fr802x_read_bufptr = NULL;
static uint32_t fr802x_read_size;
static btdm_controller_tl_callback fr802x_read_callback;
static void *fr802x_read_dummy;

static struct k_spinlock fr802x_hci_lock;

/*
 * 命令写入权信号量：保证上一条命令被控制器取走后，才允许 Host 写入下一条，
 * 避免覆盖未交付的命令。初始计数 1（允许第一条写入）。
 */
static K_SEM_DEFINE(fr802x_send_ready, 1, 1);

static K_SEM_DEFINE(fr802x_ble_control, 0, K_SEM_MAX_LIMIT);

static K_SEM_DEFINE(fr802x_ble_controller_rd_req, 0, K_SEM_MAX_LIMIT);

/*
 * 控制器 → Host 的方向。
 * 控制器调用 write() 发出 HCI 事件/ACL，我们解析后通过 recv() 交给 Host。
 */
static bt_hci_recv_t fr802x_host_recv;
static const struct device *fr802x_dev;

/* 解析并上报 HCI 事件 */
static struct net_buf *fr802x_evt_recv(uint8_t *data, size_t remaining)
{
    struct bt_hci_evt_hdr hdr;
    struct net_buf *buf;
    size_t buf_tailroom;

    if (remaining < sizeof(hdr)) {
        return NULL;
    }

    memcpy(&hdr, data, sizeof(hdr));
    data += sizeof(hdr);
    remaining -= sizeof(hdr);

    if (remaining != hdr.len) {
        return NULL;
    }

    buf = bt_buf_get_evt(hdr.evt, false, K_NO_WAIT);
    if (!buf) {
        printk("EVT: get_evt FAILED!\n");   // ← 加这个
        return NULL;
    }

    net_buf_add_mem(buf, &hdr, sizeof(hdr));

    buf_tailroom = net_buf_tailroom(buf);
    if (buf_tailroom < remaining) {
        net_buf_unref(buf);
        return NULL;
    }

    net_buf_add_mem(buf, data, remaining);

    return buf;
}

/* 解析并上报 ACL 数据 */
static struct net_buf *fr802x_acl_recv(uint8_t *data, size_t remaining)
{
    struct bt_hci_acl_hdr hdr;
    struct net_buf *buf;
    size_t buf_tailroom;

    if (remaining < sizeof(hdr)) {
        return NULL;
    }

    buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_NO_WAIT);
    if (!buf) {
        return NULL;
    }

    memcpy(&hdr, data, sizeof(hdr));
    data += sizeof(hdr);
    remaining -= sizeof(hdr);

    if (remaining != sys_le16_to_cpu(hdr.len)) {
        net_buf_unref(buf);
        return NULL;
    }

    net_buf_add_mem(buf, &hdr, sizeof(hdr));

    buf_tailroom = net_buf_tailroom(buf);
    if (buf_tailroom < remaining) {
        net_buf_unref(buf);
        return NULL;
    }

    net_buf_add_mem(buf, data, remaining);

    return buf;
}

/*
 * 控制器通过 tl_api.write 上报数据（H4 帧格式: 类型字节 + 数据）
 * 我们在线程上下文调用，因此可以直接 recv() 给 Host。
 */
static void fr802x_hci_write(uint8_t *bufptr, uint32_t size, btdm_controller_tl_callback callback, void *dummy)
{
#ifdef FR802X_HCI_DEBUG_LOG
    printf("W: size=%u data: ", (unsigned int)size);
    for (uint32_t i = 0; i < size; i++) 
        printf("%02x ", bufptr[i]);
    printf("\n");
#endif

    uint8_t pkt_indicator;
    struct net_buf *buf = NULL;
    size_t remaining = size;

    pkt_indicator = *bufptr++;
    remaining -= 1;

    switch (pkt_indicator) 
    {
        case BT_HCI_H4_EVT: buf = fr802x_evt_recv(bufptr, remaining);break;
        case BT_HCI_H4_ACL: buf = fr802x_acl_recv(bufptr, remaining);break;
        default:break;
    }

    if (buf) {
        fr802x_host_recv(fr802x_dev, buf);
    }

    callback(dummy, 0);
}

static void fr802x_hci_read(uint8_t *bufptr, uint32_t size, btdm_controller_tl_callback callback, void *dummy)
{
    if (fr802x_read_size){
        printk("ERROR: fr802x_read_size > 0\n");
        while(1);
    }

    fr802x_read_bufptr   = bufptr;
    fr802x_read_size     = size;
    fr802x_read_callback = callback;
    fr802x_read_dummy    = dummy;

    k_sem_give(&fr802x_ble_controller_rd_req);
}

static void fr802x_hci_flow_on(void)
{
}

static bool fr802x_hci_flow_off(void)
{
    return true;
}

static void fr802x_controller_notify(void)
{
    k_sem_give(&fr802x_ble_control);
}

static struct btdm_controller_tl_api fr802x_eif_api = {
    .read     = fr802x_hci_read,
    .write    = fr802x_hci_write,
    .flow_on  = fr802x_hci_flow_on,
    .flow_off = fr802x_hci_flow_off,
};

static const struct btdm_controller_itf fr802x_itf = {
    ._malloc = fr802x_malloc,
    ._free   = fr802x_free,
    .notify  = fr802x_controller_notify,
    .tl_api  = &fr802x_eif_api,
};

/* ==================== 控制器调度线程 ==================== */

#define FR802X_CONTROLLER_STACK_SIZE 4096
#define FR802X_CONTROLLER_PRIORITY   2

void BB_IRQHandler(void);
void BB_BLE_IRQHandler(void);
void BB_IRQHandler_dynamic(const void *dev)
{
#ifdef FR802X_HCI_DEBUG_LOG
    printf(".");
#endif
    BB_IRQHandler();
}
void BB_BLE_IRQHandler_dynamic(const void *dev)
{
#ifdef FR802X_HCI_DEBUG_LOG
    printf("-");
#endif
    BB_BLE_IRQHandler();
}

static K_THREAD_STACK_DEFINE(fr802x_controller_stack, FR802X_CONTROLLER_STACK_SIZE);
static struct k_thread fr802x_controller_thread;

/* 在线程上下文执行控制器初始化（与之前 UART HCI 手动测试环境一致）。
 * 不能在 POST_KERNEL device init 里做：ROM 补丁 SVC 在 init 阶段
 * 会把控制流劫持到 idle，导致 init 无法返回。 */
static void fr802x_controller_init_routine(void)
{
    struct btdm_controller_cfg *cfg;
    int BB_IP_IRQn_num, BB_BLE_IRQn_num, BB_IP_IRQn_prio, BB_BLE_IRQn_prio;

    cfg = malloc(sizeof(struct btdm_controller_cfg));
    if (!cfg) {
        return;
    }

    cfg->ble_public_addr    = fr802x_bt_addr;
    cfg->custom_param       = controller_param;
    cfg->custom_param_len   = sizeof(controller_param);
    cfg->release_rst_delay  = 2000;
    cfg->coded_phy_sel      = 0;
    cfg->conn_max           = 6;
    cfg->adv_max            = 2;
    cfg->adv_pkt_len_max    = 37;
    cfg->ble_tx_pkt_nb      = 8;
    cfg->ble_rx_pkt_nb      = 8;

    btdm_controller_init(cfg, &fr802x_itf);
    free(cfg);

    /* 按索引获取中断号和优先级 */
    BB_IP_IRQn_num  = DT_IRQ_BY_IDX(DT_DRV_INST(0), 0, irq);
    BB_IP_IRQn_prio = DT_IRQ_BY_IDX(DT_DRV_INST(0), 0, priority);
    BB_BLE_IRQn_num  = DT_IRQ_BY_IDX(DT_DRV_INST(0), 1, irq);
    BB_BLE_IRQn_prio = DT_IRQ_BY_IDX(DT_DRV_INST(0), 1, priority);

    /* 注册中断处理函数 */
    irq_connect_dynamic(BB_IP_IRQn_num, BB_IP_IRQn_prio, BB_IRQHandler_dynamic, fr802x_dev, 0);
    irq_connect_dynamic(BB_BLE_IRQn_num, BB_BLE_IRQn_prio, BB_BLE_IRQHandler_dynamic, fr802x_dev, 0);
    irq_enable(BB_IP_IRQn_num);
    irq_enable(BB_BLE_IRQn_num);
}

static void fr802x_controller_task(void *arg1, void *arg2, void *arg3)
{
    fr802x_controller_init_routine();

    while (1) 
    {
        k_sem_take(&fr802x_ble_control, K_FOREVER);
        btdm_controller_schedule();
    }
}

/* 启动控制器调度线程 */
static void fr802x_controller_start(void)
{
    k_thread_create(&fr802x_controller_thread,
                    fr802x_controller_stack, FR802X_CONTROLLER_STACK_SIZE,
                    fr802x_controller_task, NULL, NULL, NULL,
                    FR802X_CONTROLLER_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&fr802x_controller_thread, "BLE_ctrl");
}

/* ==================== bt_hci_driver_api ==================== */

static int fr802x_hci_open(const struct device *dev, bt_hci_recv_t recv)
{
    struct fr802x_hci_data *hci = dev->data;

    hci->recv = recv;

    fr802x_dev = dev;
    fr802x_host_recv = recv;

    /* 启动控制器调度线程 */
    fr802x_controller_start();

    return 0;
}

static int fr802x_hci_send(const struct device *dev, struct net_buf *buf)
{
    uint8_t *data = buf->data;
    size_t len = buf->len;
    uint8_t h4_type;
    uint8_t type;
    int err = 0;

    type = bt_buf_get_type(buf);
    switch(type)
    {
        case BT_BUF_CMD: h4_type = BT_HCI_H4_CMD; break;   /* 0x01 */
        case BT_BUF_ACL_OUT: h4_type = BT_HCI_H4_ACL; break;   /* 0x02 */
        default:err = -EINVAL; break;
    }

    k_sem_take(&fr802x_ble_controller_rd_req, K_FOREVER);
    if (fr802x_read_size != 1)
    {
        printk("ERROR: fr802x_read_size != 1\n");
        while(1);
    }
    fr802x_read_size = 0;
    *fr802x_read_bufptr = h4_type;
    fr802x_read_callback(fr802x_read_dummy, 0);

    k_sem_take(&fr802x_ble_controller_rd_req, K_FOREVER);
    switch (type) 
    {
        case BT_BUF_CMD:
            if (fr802x_read_size != 3)
            {
                printk("ERROR: fr802x_read_size != 3\n");
                while(1);
            }
            break;
        case BT_BUF_ACL_OUT:
            if (fr802x_read_size != 4)
            {
                printk("ERROR: fr802x_read_size != 4\n");
                while(1);
            }
            break;
        default: printk("ERROR: unknown type %d\n", h4_type); while(1);
    }
    if (len < fr802x_read_size)
    {
        printk("ERROR: len < fr802x_read_size\n");
        while(1);
    }
    memcpy(fr802x_read_bufptr, data, fr802x_read_size);
    data += fr802x_read_size;
    len -= fr802x_read_size;
    fr802x_read_size = 0;
    fr802x_read_callback(fr802x_read_dummy, 0);

    if (len > 0)
    {
        k_sem_take(&fr802x_ble_controller_rd_req, K_FOREVER);
        if (fr802x_read_size != len)
        {
            printk("ERROR: fr802x_read_size != len\n");
            while(1);
        }
        memcpy(fr802x_read_bufptr, data, len);
        fr802x_read_size = 0;
        fr802x_read_callback(fr802x_read_dummy, 0);
    }

    net_buf_unref(buf);
    
    return err;
}

static int fr802x_hci_close(const struct device *dev)
{
    k_thread_abort(&fr802x_controller_thread);
    return 0;
}

static int fr802x_hci_init(const struct device *dev)
{
    ARG_UNUSED(dev);
    return 0;
}

static const struct bt_hci_driver_api drv = {
    .open  = fr802x_hci_open,
    .send  = fr802x_hci_send,
    .close = fr802x_hci_close,
};

#define FR802X_HCI_DEVICE_INIT(inst) \
    static struct fr802x_hci_data fr802x_hci_data_##inst; \
    DEVICE_DT_INST_DEFINE(inst, fr802x_hci_init, NULL, &fr802x_hci_data_##inst, NULL, \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &drv)

/* Only one instance supported */
FR802X_HCI_DEVICE_INIT(0)

#endif
