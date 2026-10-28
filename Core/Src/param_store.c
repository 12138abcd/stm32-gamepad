/**
 * @file    param_store.c
 * @brief   Flash 参数存储：双备份 + CRC32
 *
 * 读写策略见 param_store.h 的注释。核心两条：
 *   1) 写新页之前只擦【旧】页 —— 任何时刻至少有一页是完整的
 *   2) 写完立刻读回校验 —— 不校验的话，写失败只能等下一次上电才发现
 */

#include "param_store.h"
#include "main.h"          /* HAL_FLASH_* / FLASH_EraseInitTypeDef */
#include <string.h>        /* memcpy / memset */
#include <stddef.h>        /* offsetof */

/* ============================ 内存里的当前副本 ============================ */
static pad_params_t g_params;

/* 上电加载结果（Watch 可见）：0 = 从 Flash 读到；1 = 用了默认值 */
volatile uint8_t param_boot_status = 1u;

/* ============================ 默认值 ============================
   与 analog_input.c 里的标定宏保持一致（2026-10-07 加 ADC 校准后实测，两平台取平均）。 */
static void param_set_defaults(pad_params_t *p)
{
    memset(p, 0, sizeof(*p));

    p->magic   = PARAM_MAGIC;
    p->seq     = 0u;
    p->version = PARAM_VERSION;
    p->dev_id  = 0u;          /* 0 = 未配对 */

    p->js_lx_center = 2060u;
    p->js_ly_center = 2059u;
    p->js_lx_half_l = 2059u;
    p->js_lx_half_r = 2035u;
    p->js_ly_half_u = 2058u;
    p->js_ly_half_d = 2036u;

    p->js_rx_center = 2052u;
    p->js_rx_half_l = 2051u;
    p->js_rx_half_r = 2043u;
    p->js_ry_center = 2083u;
    p->js_ry_half_u = 2082u;
    p->js_ry_half_d = 2012u;
}

/* ============================ CRC32（多项式 0xEDB88320）============================ */

uint32_t param_crc32(const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i;
    uint8_t  b;

    for (i = 0u; i < len; i++)
    {
        crc ^= (uint32_t)p[i];
        for (b = 0u; b < 8u; b++)
        {
            /* 位运算是为了不用 256×4 字节的查表（省 Flash）——
               40 字节数据也就 320 次循环，几微秒的事 */
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }

    return ~crc;
}

/* ============================ 读一页并校验 ============================ */

/**
 * @brief  从指定页读参数
 * @return 1 = 有效（magic + CRC 都对）；0 = 无效（空页 / 坏页）
 */
static uint8_t param_load_page(uint32_t addr, pad_params_t *out)
{
    const pad_params_t *p = (const pad_params_t *)addr;
    uint32_t crc;

    if (p->magic != PARAM_MAGIC) { return 0u; }
    if (p->version != PARAM_VERSION) { return 0u; }   /* 布局变了，旧数据不能用 */

    crc = param_crc32(p, (uint32_t)offsetof(pad_params_t, crc32));
    if (crc != p->crc32) { return 0u; }

    memcpy(out, p, sizeof(pad_params_t));
    return 1u;
}

/* ============================ 擦 + 写一页 ============================ */

static uint8_t param_write_page(uint32_t addr, const pad_params_t *p)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_err = 0u;
    uint32_t i;
    uint8_t  ok = PARAM_OK;

    HAL_FLASH_Unlock();

    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = addr;
    erase.NbPages     = 1u;
    if (HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return PARAM_ERR_FLASH;
    }

    /* ⚠️ STM32F1 的 Flash 编程粒度是【半字】（16 位），不是 32 位 ——
       用 FLASH_TYPEPROGRAM_HALFWORD，按 2 字节步进。
       （FLASH_TYPEPROGRAM_WORD 是 F4 才有的，在 F1 上根本不存在。）*/
    for (i = 0u; i < sizeof(pad_params_t); i += 2u)
    {
        uint16_t half;
        memcpy(&half, ((const uint8_t *)p) + i, 2u);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, (uint64_t)half) != HAL_OK)
        {
            ok = PARAM_ERR_FLASH;
            break;
        }
    }

    HAL_FLASH_Lock();
    return ok;
}

/* ============================ 对外 API ============================ */

uint8_t param_store_init(void)
{
    pad_params_t a;
    pad_params_t b;
    uint8_t ok_a;
    uint8_t ok_b;

    ok_a = param_load_page(PARAM_PAGE_A_ADDR, &a);
    ok_b = param_load_page(PARAM_PAGE_B_ADDR, &b);

    if (ok_a && ok_b)
    {
        /* 两页都有效：取 seq 大的（更新）*/
        memcpy(&g_params, (a.seq >= b.seq) ? &a : &b, sizeof(pad_params_t));
        param_boot_status = 0u;
        return PARAM_OK;
    }
    if (ok_a) { memcpy(&g_params, &a, sizeof(pad_params_t)); param_boot_status = 0u; return PARAM_OK; }
    if (ok_b) { memcpy(&g_params, &b, sizeof(pad_params_t)); param_boot_status = 0u; return PARAM_OK; }

    /* 两页都无效（首次上电 / 都坏了）→ 装载默认值并写一次，
       这样下次上电就能从 Flash 读到，而不是每次都用代码里的默认值。 */
    param_set_defaults(&g_params);
    (void)param_set(&g_params);
    return PARAM_ERR_NO_VALID;
}

const pad_params_t *param_get(void)
{
    return &g_params;
}

uint8_t param_set(const pad_params_t *p)
{
    pad_params_t tmp;
    pad_params_t a;
    pad_params_t b;
    uint8_t  ok_a;
    uint8_t  ok_b;
    uint32_t target;
    uint32_t new_seq = 0u;

    memcpy(&tmp, p, sizeof(pad_params_t));

    ok_a = param_load_page(PARAM_PAGE_A_ADDR, &a);
    ok_b = param_load_page(PARAM_PAGE_B_ADDR, &b);

    /* 目标页 = 当前【不是最新】的那页；seq 取两页里最大的 +1 */
    if (ok_a && ok_b)
    {
        if (a.seq >= b.seq) { target = PARAM_PAGE_B_ADDR; new_seq = a.seq; }
        else                { target = PARAM_PAGE_A_ADDR; new_seq = b.seq; }
    }
    else if (ok_a) { target = PARAM_PAGE_B_ADDR; new_seq = a.seq; }
    else if (ok_b) { target = PARAM_PAGE_A_ADDR; new_seq = b.seq; }
    else           { target = PARAM_PAGE_A_ADDR; new_seq = 0u; }

    tmp.magic   = PARAM_MAGIC;
    tmp.version = PARAM_VERSION;
    tmp.seq     = new_seq + 1u;
    tmp.crc32   = param_crc32(&tmp, (uint32_t)offsetof(pad_params_t, crc32));

    if (param_write_page(target, &tmp) != PARAM_OK)
    {
        return PARAM_ERR_FLASH;
    }

    /* 读回校验：写成功不等于写对了（电压不稳时可能写进错误的值）*/
    if (param_load_page(target, &a) == 0u)
    {
        return PARAM_ERR_FLASH;
    }

    memcpy(&g_params, &a, sizeof(pad_params_t));
    return PARAM_OK;
}

uint8_t param_factory_reset(void)
{
    pad_params_t def;

    param_set_defaults(&def);
    return param_set(&def);
}
