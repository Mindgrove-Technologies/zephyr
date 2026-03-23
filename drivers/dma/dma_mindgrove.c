#define DT_DRV_COMPAT mindgrove_dma

#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <errno.h>
#include <zephyr/drivers/interrupt_controller/riscv_plic.h>

#include "dma_mindgrove.h"

#define DMA_CHANNELS_COUNT 8U

/*
 * PLIC offset: number of CPU-level IRQ slots before PLIC sources start.
 * Confirmed: CONFIG_2ND_LVL_ISR_TBL_OFFSET = 41 on this SoC.
 * DT_INST_IRQN() is broken on this SoC (encodes priority bits).
 * Always use: zirq = DT_INST_IRQ_BY_IDX(n, 0, irq) + PLIC_OFFSET
 */
#define PLIC_OFFSET CONFIG_2ND_LVL_ISR_TBL_OFFSET

struct dma_custom_data {
    struct dma_context ctx;
    const struct dma_config *chan_cfgs[DMA_CHANNELS_COUNT];
};

struct dma_custom_config {
    uint32_t base_addr;
    uint32_t plic_src;   /* raw PLIC source from DT */
    uint32_t priority;
    void (*irq_config_func)(const struct device *dev);
};

static inline uint32_t get_zirq(uint32_t plic_src)
{
    return plic_src + PLIC_OFFSET;
}

/* -------------------------------------------------- */
/* CONFIGURE                                          */
/* -------------------------------------------------- */

static int dma_custom_configure(const struct device *dev, uint32_t channel,
                                struct dma_config *cfg)
{
    const struct dma_custom_config *config = dev->config;
    struct dma_custom_data *data = dev->data;

    uint32_t ccr = 0U;
    uint16_t request_select = 0U;
    uint8_t mode_flag = 0U;
    uint8_t request_shift = 0U;
    uint32_t pinc = 0U, minc = 0U, m2m = 0U, p2p = 0U, dir = 0U;
    volatile uint32_t *src_addr, *dest_addr, *temp;

    DMA_Type *regs = (DMA_Type *)(uintptr_t)config->base_addr;

    if (channel >= DMA_CHANNELS_COUNT) return -EINVAL;

    uint32_t src_size_index = (cfg->source_data_size == 8) ? 3
                            : (cfg->source_data_size == 4) ? 2
                            : (cfg->source_data_size == 2) ? 1 : 0;

    uint32_t dest_size_index = (cfg->dest_data_size == 8) ? 3
                             : (cfg->dest_data_size == 4) ? 2
                             : (cfg->dest_data_size == 2) ? 1 : 0;

    DMA_CHANNEL_Type *chan_reg = &regs->CH[channel];
    chan_reg->DMA_CNDTR = (uint16_t)(cfg->head_block->block_size);

    src_addr  = (volatile uint32_t *)cfg->head_block->source_address;
    dest_addr = (volatile uint32_t *)cfg->head_block->dest_address;


    
 /* Memory / Flash detection */
    if ((((uint32_t)src_addr >= DMA_RAM_START_ADDR) &&
         ((uint32_t)src_addr <= DMA_RAM_END_ADDR)) ||
        (((uint32_t)src_addr >= DMA_FLASH_START_ADDR) &&
         ((uint32_t)src_addr <= DMA_FLASH_END_ADDR))) {
        mode_flag = 1U;  /* Memory source */
    } else {
        switch ((uint32_t)src_addr) {
        case AES_OUT_REG_ADDR: /* AES output reg -> Fast */
            request_select = AES_OUTP_READY;
            mode_flag |= DMA_MODE_FAST_SOURCE;
            break;

        case SHA_OUT_REG_ADDR: /* SHA output reg -> Fast */
            request_select = SHA_OUTP_READY;
            mode_flag |= DMA_MODE_FAST_SOURCE;
            break;

        case RSA_OUT_REG_ADDR: /* RSA output reg -> Fast */
            request_select = RSA_OUTP_READY;
            mode_flag |= DMA_MODE_FAST_SOURCE;
            break;

        case QSPI0_DATA_REG_ADDR: /* QSPI0 data reg -> Fast */
            request_select = QSPI0_READY;
            mode_flag |= DMA_MODE_FAST_SOURCE;
            break;

        case QSPI1_DATA_REG_ADDR: /* QSPI1 data reg -> Fast */
            request_select = QSPI1_READY;
            mode_flag |= DMA_MODE_FAST_SOURCE;
            break;

        case UART0_RX_REG_ADDR:  /*UART0 RX reg - > Slow*/
            request_select = UART0_OUTP_READY;
            break;
        case UART1_RX_REG_ADDR:  /*UART1 RX reg - > Slow*/
            request_select = UART1_OUTP_READY;
            break;
        case UART2_RX_REG_ADDR:  /*UART2 RX reg - > Slow*/
            request_select = UART2_OUTP_READY;
            printk("UART2 RX reg selected\n");
            break;
        case UART3_RX_REG_ADDR:  /*UART3 RX reg - > Slow*/
            request_select = UART3_OUTP_READY;
            break;
        case UART4_RX_REG_ADDR:  /*UART4 RX reg - > Slow*/
            request_select = UART4_OUTP_READY;
            break;

        case SPI0_RX_REG_ADDR:  /*SPI0 RX reg - > Slow*/
            request_select = SPI0_OUTP_READY;
            break;
        case SPI1_RX_REG_ADDR:  /*SPI1 RX reg - > Slow*/
            request_select = SPI1_OUTP_READY;
            break;
        case SPI2_RX_REG_ADDR:  /*SPI2 RX reg - > Slow*/
            request_select = SPI2_OUTP_READY;
            break;
        case SPI3_RX_REG_ADDR:  /*SPI3 RX reg - > Slow*/
            request_select = SPI3_OUTP_READY;
            break;

        case ITRACE_DATA_REG_ADDR:  /*ITRACE Data reg - > Slow*/
            request_select = ITRACE_OUTP_READY;
            break;
        case ADC_DATA_REG_ADDR:  /*ADC Data reg - > Slow*/
            request_select = ADC_OUTP_READY;
            break;

        case PRO_IO_DUO_DATA_REG_ADDR:  /*PRO IO DUO Data Reg -> Slow*/
            request_select = PRO_IO_DUO_OUTP_READY;
            break;
        case PRO_IO_TETRA_DATA_REG_ADDR:  /*PRO IO TETRA Data Reg -> Slow*/
            request_select = PRO_IO_TETRA_OUTP_READY;
            break;
        case PRO_IO_OCTA_DATA_REG_ADDR:  /*PRO IO OCAT Data Reg -> Slow*/
            request_select = PRO_IO_OCTA_OUTP_READY;
            break;
        case PRO_IO_FUSION_DATA_REG_ADDR:  /*PRO IO FUSION Data Reg -> Slow*/
            request_select = PRO_IO_FUSION_OUTP_READY;
            break;

        default:
            return EINVAL;
        }
    }

    /* If source addr is memory don't left shift, else left shift 6 times */
    request_shift = (mode_flag == 1U) ? 0U : 6U;

    /* ================= Destination Detection ================= */

    /* Memory / Flash detection */
    if ((((uint32_t)dest_addr >= DMA_RAM_START_ADDR) &&
         ((uint32_t)dest_addr <= DMA_RAM_END_ADDR)) ||
        (((uint32_t)dest_addr >= DMA_FLASH_START_ADDR) &&
         ((uint32_t)dest_addr <= DMA_FLASH_END_ADDR))) {
        mode_flag |= (1U << 1);
    } else {
        switch ((uint32_t)dest_addr) {
        case AES_INP_REG_ADDR:  /*AES input reg -> Fast*/
            request_select |=
            ((uint16_t)AES_CAN_TAKE_INPUT << (uint16_t)request_shift);
            mode_flag |= DMA_MODE_FAST_DESTINATION;
            break;

        case SHA_INP_REG_ADDR:  /*SHA input reg -> Fast*/
            request_select |=
            ((uint16_t)SHA_CAN_TAKE_INPUT << (uint16_t)request_shift);
            mode_flag |= DMA_MODE_FAST_DESTINATION;
            break;

        case RSA_INP_REG_ADDR:  /*RSA input reg -> Fast*/
            request_select |=
            ((uint16_t)RSA_CAN_TAKE_INPUT << (uint16_t)request_shift);
            mode_flag |= DMA_MODE_FAST_DESTINATION;
            break;

        case QSPI0_DATA_REG_ADDR:  /*QSPI0 data reg -> Fast*/
            request_select |=
            ((uint16_t)QSPI0_READY << (uint16_t)request_shift);
            mode_flag |= DMA_MODE_FAST_DESTINATION;
            break;

        case QSPI1_DATA_REG_ADDR:  /*QSPI1 data reg -> Fast*/
            request_select |=
            ((uint16_t)QSPI1_READY << (uint16_t)request_shift);
            mode_flag |= DMA_MODE_FAST_DESTINATION;
            break;

        case UART0_TX_REG_ADDR:  /*UART0 TX reg -> Slow*/
            request_select |=
            ((uint16_t)UART0_CAN_TAKE_INPUT << (uint16_t)request_shift);
            break;

        case UART1_TX_REG_ADDR:  /*UART1 TX reg -> Slow*/
            request_select |=
            ((uint16_t)UART1_CAN_TAKE_INPUT << (uint16_t)request_shift);
            break;

        case UART2_TX_REG_ADDR:  /*UART2 TX reg -> Slow*/
            request_select |=
            ((uint16_t)UART2_CAN_TAKE_INPUT << (uint16_t)request_shift);
            break;

        case UART3_TX_REG_ADDR:  /*UART3 TX reg -> Slow*/
            request_select |=
            ((uint16_t)UART3_CAN_TAKE_INPUT << (uint16_t)request_shift);
            break;

        case UART4_TX_REG_ADDR:  /*UART4 TX reg -> Slow*/
            request_select |=
            ((uint16_t)UART4_CAN_TAKE_INPUT << (uint16_t)request_shift);
            break;

        case SPI0_TX_REG_ADDR:  /*SPI0 TX reg - > Slow*/
            request_select |=
            ((uint16_t)SPI0_CAN_TAKE_INPUT << (uint16_t)request_shift);
            break;

        case SPI1_TX_REG_ADDR:  /*SPI1 TX reg - > Slow*/
            request_select |=
            ((uint16_t)SPI1_CAN_TAKE_INPUT << (uint16_t)request_shift);
            break;

        case SPI2_TX_REG_ADDR:  /*SPI2 TX reg - > Slow*/
            request_select |=
            ((uint16_t)SPI2_CAN_TAKE_INPUT << (uint16_t)request_shift);
            break;

        case SPI3_TX_REG_ADDR:  /*SPI3 TX reg - > Slow*/
            request_select |=
            ((uint16_t)SPI3_CAN_TAKE_INPUT << (uint16_t)request_shift);
            break;

        case PRO_IO_DUO_DATA_REG_ADDR: /*PRO IO DUO Data Reg -> Slow*/
            request_select |=
            ((uint16_t)PRO_IO_DUO_CAN_TAKE_INP << (uint16_t)request_shift);
            break;

        case PRO_IO_TETRA_DATA_REG_ADDR:  /*PRO IO TETRA Data Reg -> Slow*/
            request_select |=
            ((uint16_t)PRO_IO_TETRA_CAN_TAKE_INP << (uint16_t)request_shift);
            break;

        case PRO_IO_OCTA_DATA_REG_ADDR:  /*PRO IO OCTA Data Reg -> Slow*/
            request_select |=
            ((uint16_t)PRO_IO_OCTA_CAN_TAKE_INP << (uint16_t)request_shift);
            break;

        case PRO_IO_FUSION_DATA_REG_ADDR:  /*PRO IO FUSION Data Reg -> Slow*/
            request_select |=
            ((uint16_t)PRO_IO_FUSION_CAN_TAKE_INP << (uint16_t)request_shift);
            break;

        default:
            return EINVAL;
        }
    }

    chan_reg->DMA_CSELR = request_select;

    /* ================= Mode Decode ================= */

    /**
     * mode_flag    Operation
     * 0            P2P
     * 1            M2P
     * 2            P2M
     * 3            M2M
     */

    switch (mode_flag & 0x03U) {
    case 0U: {
            /* P2P */
        p2p = DMA_CFG_PERIPH_TO_PERIPH;

        pinc = ((mode_flag & DMA_MODE_FAST_SOURCE) == 0U) ?
               DMA_SLOW_PERIPH_BURST : DMA_FAST_PERIPH_BURST;

        minc = ((mode_flag & DMA_MODE_FAST_DESTINATION) == 0U) ?
               DMA_SLOW_PERIPH_BURST : DMA_FAST_PERIPH_BURST;

        break;
    }

    case 1U: {
            /* M2P */
        temp = src_addr;
        src_addr  = dest_addr;
        dest_addr = temp;

        dir = DMA_CFG_TRANSFER_DIR;

        pinc = ((mode_flag & DMA_MODE_FAST_DESTINATION) == 0U) ?
               DMA_SLOW_PERIPH_BURST : DMA_FAST_PERIPH_BURST;

        minc = DMA_INC_ENABLE;

        break;
    }

    case 2U: {
            /* P2M */
        pinc = ((mode_flag & DMA_MODE_FAST_SOURCE) == 0U) ?
               DMA_SLOW_PERIPH_BURST : DMA_FAST_PERIPH_BURST;

        minc = DMA_INC_ENABLE;

        break;
    }

    case 3U: {
            /* M2M */
        temp = src_addr;
        src_addr  = dest_addr;
        dest_addr = temp;

        dir = DMA_CFG_TRANSFER_DIR;
        m2m = DMA_CFG_MEM_TO_MEM;

        pinc = DMA_INC_ENABLE;
        minc = DMA_INC_ENABLE;

        break;
    }

    default:
        return EINVAL;
    }

    chan_reg->DMA_CPAR = (uint32_t)src_addr;
    chan_reg->DMA_CMAR = (uint32_t)dest_addr;
    data->chan_cfgs[channel] = cfg;

    ccr = DMA_CCR_PL(cfg->channel_priority) | DMA_CCR_MSIZE(src_size_index) |
          DMA_CCR_PSIZE(dest_size_index) | DMA_CCR_MINC(minc) | DMA_CCR_PINC(pinc) |
          dir | m2m | p2p;

    if (cfg->dma_callback) {
        ccr |= (DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_HTIE);
    }

    chan_reg->DMA_CCR |= ccr;
    return 0;
}

/* -------------------------------------------------- */
/* START                                              */
/* -------------------------------------------------- */

static int dma_custom_start(const struct device *dev, uint32_t channel)
{
    const struct dma_custom_config *config = dev->config;
    DMA_Type *regs = (DMA_Type *)config->base_addr;

    if (channel >= DMA_CHANNELS_COUNT) return -EINVAL;

    /* Clear pending interrupts */
    if (regs->DMA_ISR) {
        regs->DMA_IFCR = regs->DMA_ISR;
    }

    /* Enable DMA channel */
    regs->CH[channel].DMA_CCR |= DMA_CCR_EN;

    /* Enable PLIC source — use Zephyr IRQ number */
    riscv_plic_irq_enable(get_zirq(config->plic_src));

    return 0;
}

/* -------------------------------------------------- */
/* STOP                                               */
/* -------------------------------------------------- */

static int dma_custom_stop(const struct device *dev, uint32_t channel)
{
    const struct dma_custom_config *config = dev->config;
    DMA_Type *regs = (DMA_Type *)config->base_addr;

    if (channel >= DMA_CHANNELS_COUNT) return -EINVAL;

    regs->CH[channel].DMA_CCR &= ~BIT(0);
    return 0;
}

/* -------------------------------------------------- */
/* STATUS                                             */
/* -------------------------------------------------- */

static int dma_custom_get_status(const struct device *dev, uint32_t channel,
                                 struct dma_status *stat)
{
    const struct dma_custom_config *config = dev->config;
    DMA_Type *regs = (DMA_Type *)config->base_addr;

    if (channel >= DMA_CHANNELS_COUNT || stat == NULL) return -EINVAL;

    uint32_t ccr = regs->CH[channel].DMA_CCR;
    stat->busy           = (ccr & BIT(0)) != 0U;
    stat->pending_length = regs->CH[channel].DMA_CNDTR;
    stat->dir            = (ccr & BIT(4)) ? MEMORY_TO_PERIPHERAL : PERIPHERAL_TO_MEMORY;
    return 0;
}

/* -------------------------------------------------- */
/* ISR                                                */
/* -------------------------------------------------- */

static void dma_custom_isr(const void *arg)
{
    const struct device *dev = arg;
    const struct dma_custom_config *config = dev->config;
    struct dma_custom_data *data = dev->data;
    DMA_Type *regs = (DMA_Type *)(uintptr_t)config->base_addr;

    uint32_t isr_status = regs->DMA_ISR;
    
    /* Print raw status for debugging */
    printk("[DMA ISR] Raw ISR status: 0x%08x\n", isr_status);

    for (uint32_t i = 0; i < DMA_CHANNELS_COUNT; i++) {
        uint32_t tc_flag = BIT(i * 4 + 1);
        uint32_t te_flag = BIT(i * 4 + 3);
        uint32_t ht_flag = BIT(i * 4 + 2);
        uint32_t ch_mask = tc_flag | te_flag | ht_flag;

        if (!(isr_status & ch_mask)) {
            continue;
        }

        /* Read remaining transfer count BEFORE clearing */
        uint16_t remaining = regs->CH[i].DMA_CNDTR;
        printk("[DMA ISR] Channel %d: TC=%d, HT=%d, TE=%d, Remaining=%d\n", 
               i, 
               (isr_status & tc_flag) ? 1 : 0,
               (isr_status & ht_flag) ? 1 : 0,
               (isr_status & te_flag) ? 1 : 0,
               remaining);

        /* Clear interrupt flags */
        regs->DMA_IFCR = ch_mask;

        const struct dma_config *cfg = data->chan_cfgs[i];
        if (cfg && cfg->dma_callback) {

            int status;

            if (isr_status & te_flag) {
                status = -EIO;
                printk("[DMA ISR] CH%u ERROR\n", i);
            } else if (isr_status & tc_flag) {
                status = 0;
                printk("[DMA ISR] CH%u COMPLETE\n", i);
            }  else if (isr_status & ht_flag) {
                status = 1;
                printk("[DMA ISR] CH%u HALF, Remaining: %d/%d\n", 
                       i, remaining, cfg->head_block->block_size);
            } else {
                continue;
            }

            cfg->dma_callback(dev, cfg->user_data, i, status);
        }
    }
}

/* -------------------------------------------------- */
/* API                                                */
/* -------------------------------------------------- */

static const struct dma_driver_api dma_custom_api = {
    .config     = dma_custom_configure,
    .start      = dma_custom_start,
    .stop       = dma_custom_stop,
    .get_status = dma_custom_get_status,
};

/* -------------------------------------------------- */
/* INIT                                               */
/* -------------------------------------------------- */

static int dma_custom_init(const struct device *dev)
{
    const struct dma_custom_config *config = dev->config;

    printk("[DMA] base=0x%x plic_src=%u zirq=%u prio=%u\n",
           config->base_addr, config->plic_src,
           get_zirq(config->plic_src), config->priority);

    config->irq_config_func(dev);
    return 0;
}

#define DMA_CUSTOM_DEVICE(inst)                                                     \
    static void dma_irq_config_##inst(const struct device *dev)                     \
    {                                                                               \
        uint32_t plic_src = DT_INST_IRQ_BY_IDX(inst, 0, irq);                       \
        uint32_t zirq = plic_src + PLIC_OFFSET;                                     \
        uint32_t prio = DT_INST_IRQ_BY_IDX(inst, 0, priority);                      \
                                                                                    \
                                                                                    \
        /* sanity checks */                                                         \
        if (zirq >= CONFIG_NUM_IRQS) {                                              \
            printk("  ERROR: zirq out of range!\n");                                \
        }                                                                           \
                                                                                    \
        riscv_plic_set_priority(zirq, 2U);                                          \
                                                                                    \
        printk("  priority set OK\n");                                              \
                                                                                    \
        irq_connect_dynamic(zirq, prio, dma_custom_isr,                             \
                            DEVICE_DT_INST_GET(inst), 0);                           \
                                                                                    \
        printk("  irq connected\n");                                                \
                                                                                    \
        irq_enable(zirq);                                                           \
                                                                                    \
        printk("  irq enabled\n");                                                  \
    }                                                                               \
    static const struct dma_custom_config dma_config_##inst = {                     \
        .base_addr       = DT_INST_REG_ADDR(inst),                                  \
        .plic_src        = DT_INST_IRQ_BY_IDX(inst, 0, irq),                        \
        .priority        = DT_INST_IRQ_BY_IDX(inst, 0, priority),                   \
        .irq_config_func = dma_irq_config_##inst,                                   \
    };                                                                              \
    static struct dma_custom_data dma_data_##inst = {                               \
        .ctx = {                                                                    \
            .magic        = DMA_MAGIC,                                              \
            .dma_channels = DMA_CHANNELS_COUNT,                                     \
            .atomic       = ATOMIC_INIT(0),                                         \
        },                                                                          \
    };                                                                              \
    DEVICE_DT_INST_DEFINE(inst, dma_custom_init, NULL,                              \
                          &dma_data_##inst, &dma_config_##inst,                     \
                          POST_KERNEL, CONFIG_DMA_INIT_PRIORITY,                    \
                          &dma_custom_api);

DT_INST_FOREACH_STATUS_OKAY(DMA_CUSTOM_DEVICE)