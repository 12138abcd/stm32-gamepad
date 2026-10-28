/**
 * @file    param_store.h
 * @brief   Flash 参数存储：双备份 + CRC32（校准常量 / 配对 ID）
 *
 * 分区（F103C8T6：64KB = 64 页 × 1KB）：
 *   页 0~61  程序
 *   页 62    参数区 A   0x0800F800
 *   页 63    参数区 B   0x0800FC00
 *
 * 读写策略：
 *   读：两页都读 -> 校验 CRC -> 取"CRC 正确且 seq 大"的那页
 *   写：擦【旧】的那页 -> 写到另一页（seq = 旧+1）-> 读回校验
 *       ⚠️ 不能"先擦当前页再写"：擦完到写完之间掉电，两页就都没了。
 */

#ifndef PARAM_STORE_H
#define PARAM_STORE_H

#include <stdint.h>

/* ============================ Flash 布局 ============================ */
#define PARAM_PAGE_A_ADDR   0x0800F800u    /* 页 62 */
#define PARAM_PAGE_B_ADDR   0x0800FC00u    /* 页 63 */
#define PARAM_PAGE_SIZE     1024u

/* ============================ 结构与常量 ============================ */
#define PARAM_MAGIC         0x50414421u    /* "PAD!" */
#define PARAM_VERSION       1u

/* 参数结构：40 字节（远小于 1KB 一页）。
   packed 是保险 —— 防止编译器在 crc32 之前插对齐填充，导致读写偏移不一致。 */
typedef struct __attribute__((packed)) {
    uint32_t magic;          /* PARAM_MAGIC */
    uint32_t seq;            /* 每次写入 +1，大的更新 */
    uint16_t version;        /* 结构版本，改布局时 +1 */
    uint16_t dev_id;         /* 配对 ID；0 = 未配对 */

    /* 摇杆校准常量（12 位 ADC 原始值），默认值等于 analog_input.c 里实测的那套 */
    uint16_t js_lx_center;
    uint16_t js_ly_center;
    uint16_t js_lx_half_l;
    uint16_t js_lx_half_r;
    uint16_t js_ly_half_u;
    uint16_t js_ly_half_d;
    uint16_t js_rx_center;
    uint16_t js_rx_half_l;
    uint16_t js_rx_half_r;
    uint16_t js_ry_center;
    uint16_t js_ry_half_u;
    uint16_t js_ry_half_d;

    uint32_t crc32;          /* 覆盖前面所有字节（不含本字段）*/
} pad_params_t;

/* ============================ 返回值 ============================ */
#define PARAM_OK          0u
#define PARAM_ERR_NO_VALID 1u    /* 两页都无效（首次上电 / 都坏了），已加载默认值 */
#define PARAM_ERR_FLASH   2u     /* 擦除或编程失败 */

/* ============================ 诊断（Watch 可见）============================ */
/* 上电加载结果：0 = 从 Flash 读到了；1 = 两页都无效，用了默认值。
   首次烧写后应该是 1（Flash 是空的），之后每次上电都应该是 0。
   如果一直是 1，说明写入没成功（Flash 擦写失败 / 地址不对）。 */
extern volatile uint8_t param_boot_status;

/* ============================ API ============================ */

/**
 * @brief  上电初始化：从 Flash 加载参数（无效则装载默认值并写一次）
 * @return PARAM_OK = 从 Flash 成功加载；PARAM_ERR_NO_VALID = 用了默认值
 * @note   只在 app_init() 里调一次，之后用 param_get() 读。
 */
uint8_t param_store_init(void);

/**
 * @brief  读当前参数（只读，不要改它；要改走 param_set）
 */
const pad_params_t *param_get(void);

/**
 * @brief  写入参数（自动选备份页、seq+1、擦写、读回校验）
 * @return PARAM_OK 成功；PARAM_ERR_FLASH 失败（此时内存里的副本仍是旧值）
 */
uint8_t param_set(const pad_params_t *p);

/**
 * @brief  恢复出厂默认（校准常量回默认、dev_id = 0 未配对）
 */
uint8_t param_factory_reset(void);

/**
 * @brief  软件 CRC32（多项式 0xEDB88320，反射式，与 zlib 一致）
 */
uint32_t param_crc32(const void *data, uint32_t len);

#endif /* PARAM_STORE_H */
