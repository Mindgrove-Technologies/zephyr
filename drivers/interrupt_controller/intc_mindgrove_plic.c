/*
 * Mindgrove PLIC Driver
 *
 * KEY FIX: Zephyr IRQ = PLIC source + CONFIG_2ND_LVL_ISR_TBL_OFFSET
 * All public functions receive Zephyr IRQ numbers and must subtract
 * the offset to get the raw PLIC source number.
 */

#define DT_DRV_COMPAT mindgrove_plic

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/interrupt_controller/riscv_plic.h>
#include <zephyr/sw_isr_table.h>

LOG_MODULE_REGISTER(intc_plic, LOG_LEVEL_DBG);

#ifndef PLIC_NODE
#define PLIC_NODE DT_NODELABEL(plic0)
#endif

#define PLIC_BASE        DT_REG_ADDR(PLIC_NODE)
#define PLIC_SIZE        DT_REG_SIZE(PLIC_NODE)
#define PLIC_CPU_IRQ     DT_IRQN(PLIC_NODE)
#define PLIC_MAX_SRC     DT_PROP(PLIC_NODE, riscv_ndev)
#define PLIC_MIN_SRC     1U
#define PLIC_OFFSET      CONFIG_2ND_LVL_ISR_TBL_OFFSET

#define PLIC_PRIO_BASE   0x0000U
#define PLIC_PEND_BASE   0x1000U
#define PLIC_EN_BASE     0x2000U
#define PLIC_THRESHOLD   0x200000U
#define PLIC_CLAIM       0x200004U
#define PLIC_PRIO_STRIDE 4U
#define PLIC_THRESHOLD_VAL 1U

static inline uint32_t plic_rd(uint32_t off)
{
    return sys_read32((mem_addr_t)(PLIC_BASE + off));
}

static inline void plic_wr(uint32_t off, uint32_t val)
{
    sys_write32(val, (mem_addr_t)(PLIC_BASE + off));
}

static uint32_t to_src(uint32_t zirq)
{
    if (zirq < PLIC_OFFSET) {
        printk("[PLIC] ERROR: zirq %u < offset %u\n", zirq, PLIC_OFFSET);
        return 0U;
    }
    uint32_t src = zirq - PLIC_OFFSET;
    if (src < PLIC_MIN_SRC || src > PLIC_MAX_SRC) {
        printk("[PLIC] ERROR: src %u out of range [%u..%u]\n",
               src, PLIC_MIN_SRC, PLIC_MAX_SRC);
        return 0U;
    }
    return src;
}

static void en_reg(uint32_t src, uint32_t *off, uint32_t *bit)
{
    *off = PLIC_EN_BASE + (src / 32U) * 4U;
    *bit = src % 32U;
}

uint32_t riscv_plic_get_irq(void)
{
    return plic_rd(PLIC_CLAIM);
}

void riscv_plic_complete(uint32_t src)
{
    printk("[PLIC] complete src=%u\n", src);
    plic_wr(PLIC_CLAIM, src);
    __asm__ volatile("fence iorw, iorw" ::: "memory");
}

void riscv_plic_set_priority(uint32_t irq, uint32_t prio)
{
    uint32_t src = (irq >= PLIC_OFFSET) ? to_src(irq) : irq;
    if (src < PLIC_MIN_SRC || src > PLIC_MAX_SRC) return;
    plic_wr(PLIC_PRIO_BASE + src * PLIC_PRIO_STRIDE, prio);
    __asm__ volatile("fence iorw, iorw" ::: "memory");
    printk("[PLIC] set_priority: src=%u prio=%u\n", src, prio);
}

void riscv_plic_irq_enable(uint32_t zirq)
{
    uint32_t src = to_src(zirq);
    if (src == 0U) return;
    uint32_t off, bit;
    en_reg(src, &off, &bit);
    uint32_t val = plic_rd(off) | BIT(bit);
    plic_wr(off, val);
    __asm__ volatile("fence iorw, iorw" ::: "memory");
    printk("[PLIC] irq_enable: zirq=%u src=%u en_off=0x%x bit=%u val=0x%08x\n",
           zirq, src, off, bit, val);
}

void riscv_plic_irq_disable(uint32_t zirq)
{
    uint32_t src = to_src(zirq);
    if (src == 0U) return;
    uint32_t off, bit;
    en_reg(src, &off, &bit);
    uint32_t val = plic_rd(off) & ~BIT(bit);
    plic_wr(off, val);
    __asm__ volatile("fence iorw, iorw" ::: "memory");
    printk("[PLIC] irq_disable: zirq=%u src=%u val=0x%08x\n",
           zirq, src, val);
}

uint32_t riscv_plic_get_pending(uint32_t src)
{
    if (src < PLIC_MIN_SRC || src > PLIC_MAX_SRC) return 0U;
    uint32_t off = PLIC_PEND_BASE + (src / 32U) * 4U;
    return (plic_rd(off) >> (src % 32U)) & 1U;
}

const struct device *riscv_plic_get_dev(void)
{
    return DEVICE_DT_INST_GET(0);
}

static void plic_ext_handler(const void *arg)
{
    ARG_UNUSED(arg);

    uint32_t src = riscv_plic_get_irq();
    printk("[PLIC] ext_handler: claim=%u\n", src);

    if (src == 0U) {
        printk("[PLIC] claim=0, returning\n");
        return;
    }

    if (src < PLIC_MIN_SRC || src > PLIC_MAX_SRC) {
        printk("[PLIC] invalid src=%u, completing\n", src);
        riscv_plic_complete(src);
        return;
    }

    uint32_t idx = src + CONFIG_2ND_LVL_ISR_TBL_OFFSET;
    struct _isr_table_entry *e = &_sw_isr_table[idx];

    if (e->isr != NULL) {
        printk("[PLIC] calling ISR for src=%u idx=%u\n", src, idx);
        e->isr(e->arg);
    } else {
        printk("[PLIC] WARNING: no ISR for src=%u idx=%u\n", src, idx);
    }

    riscv_plic_complete(src);
    printk("[PLIC] ext_handler done for src=%u\n", src);
}

struct plic_cfg { uintptr_t base; unsigned int irq; };
struct plic_dat { };
static const struct plic_cfg plic_cfg0 = { .base = PLIC_BASE, .irq = PLIC_CPU_IRQ };
static struct plic_dat plic_dat0;

static int plic_init(const struct device *dev)
{
    ARG_UNUSED(dev);

    printk("[PLIC] init: base=0x%lx cpu_irq=%u max_src=%u offset=%u\n",
           (unsigned long)PLIC_BASE, PLIC_CPU_IRQ, PLIC_MAX_SRC, PLIC_OFFSET);

    plic_wr(PLIC_EN_BASE,      0U);
    plic_wr(PLIC_EN_BASE + 4U, 0U);
    plic_wr(PLIC_EN_BASE + 8U, 0U);

    for (unsigned int i = PLIC_MIN_SRC; i <= PLIC_MAX_SRC; i++) {
        plic_wr(PLIC_PRIO_BASE + i * PLIC_PRIO_STRIDE, 0U);
    }

    plic_wr(PLIC_THRESHOLD, PLIC_THRESHOLD_VAL);

    IRQ_CONNECT(PLIC_CPU_IRQ, 0, plic_ext_handler, NULL, 0);
    irq_enable(PLIC_CPU_IRQ);

    printk("[PLIC] init done. thresh=%u\n", plic_rd(PLIC_THRESHOLD));
    return 0;
}

DEVICE_DT_INST_DEFINE(0, plic_init, NULL,
                      &plic_dat0, &plic_cfg0,
                      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                      NULL);