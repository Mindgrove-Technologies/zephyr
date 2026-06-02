#define DT_DRV_COMPAT mindgrove_dma

#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <zephyr/drivers/interrupt_controller/riscv_plic.h>
#include "dma_mindgrove.h"

#define DMA_CHANNELS_COUNT 8U

LOG_MODULE_REGISTER(dma_custom, LOG_LEVEL_DBG);

struct dma_custom_data {
    struct dma_context ctx;
    const struct dma_config *chan_cfgs[DMA_CHANNELS_COUNT];
};

struct dma_custom_config {
    uint32_t base_addr;
    uint32_t plic_src;       /* raw PLIC source from DT */
    uint32_t encoded_irqn;   /* DT_INST_IRQN — multilevel encoded, for irq_enable */
    uint32_t priority;
    void (*irq_config_func)(const struct device *dev);
};

/* -------------------------------------------------- */
/* CONFIGURE                                          */
/* -------------------------------------------------- */

static int dma_custom_configure(const struct device *dev, uint32_t channel,
                                struct dma_config *cfg)
{
    const struct dma_custom_config *config = dev->config;
    struct dma_custom_data *data = dev->data;

    uint32_t config_reg = 0U;
    uint16_t request_select = 0U;
    uint8_t mode_flag = 0U;
    uint8_t request_shift = 0U;
    uint32_t pinc = 0U, minc = 0U;
    volatile uint32_t *src_addr, *dest_addr, *temp;

    DMA_Type *regs = (DMA_Type *)(uintptr_t)config->base_addr;

    if (channel >= DMA_CHANNELS_COUNT) {
        return -EINVAL;
    }

    /* -------------------------------------------------------
     * FIX 1: Disable channel and wait before reconfiguring.
     * Matches baremetal DMA_Transfer_Configure which waits
     * for EN=0 before touching any channel registers.
     * ------------------------------------------------------- */
    regs->CHANNEL[channel].CONFIG_REG &= ~DMA_CFG_CHANNEL_ENABLE;
    while (regs->CHANNEL[channel].CONFIG_REG & DMA_CFG_CHANNEL_ENABLE) {
        k_busy_wait(1U);
    }

    uint32_t src_size_index = (cfg->source_data_size == 8U) ? 3U
                            : (cfg->source_data_size == 4U) ? 2U
                            : (cfg->source_data_size == 2U) ? 1U : 0U;

    uint32_t dest_size_index = (cfg->dest_data_size == 8U) ? 3U
                             : (cfg->dest_data_size == 4U) ? 2U
                             : (cfg->dest_data_size == 2U) ? 1U : 0U;

    DMA_CHANNEL_Type *chan_reg = &regs->CHANNEL[channel];

    chan_reg->TRANSFER_LENGTH_REG = (uint16_t)(cfg->head_block->block_size);

    src_addr  = (volatile uint32_t *)cfg->head_block->source_address;
    dest_addr = (volatile uint32_t *)cfg->head_block->dest_address;

    /* ================= Source Detection ================= */

    if ((((uint32_t)src_addr >= DMA_RAM_START_ADDR) &&
         ((uint32_t)src_addr <= DMA_RAM_END_ADDR)) ||
        (((uint32_t)src_addr >= DMA_FLASH_START_ADDR) &&
         ((uint32_t)src_addr <= DMA_FLASH_END_ADDR))) {
        mode_flag = 1U;  /* Memory source */
    } else {
        switch ((uint32_t)src_addr) {
        case AES_OUT_REG_ADDR:
            request_select = AES_OUTP_READY;
            mode_flag |= DMA_MODE_FAST_SOURCE;
            break;
        case SHA_OUT_REG_ADDR:
            request_select = SHA_OUTP_READY;
            mode_flag |= DMA_MODE_FAST_SOURCE;
            break;
        case RSA_OUT_REG_ADDR:
            request_select = RSA_OUTP_READY;
            mode_flag |= DMA_MODE_FAST_SOURCE;
            break;
        case QSPI0_DATA_REG_ADDR:
            request_select = QSPI0_READY;
            mode_flag |= DMA_MODE_FAST_SOURCE;
            break;
        case QSPI1_DATA_REG_ADDR:
            request_select = QSPI1_READY;
            mode_flag |= DMA_MODE_FAST_SOURCE;
            break;
        case UART0_RX_REG_ADDR:
            request_select = UART0_OUTP_READY;
            break;
        case UART1_RX_REG_ADDR:
            request_select = UART1_OUTP_READY;
            break;
        case UART2_RX_REG_ADDR:
            request_select = UART2_OUTP_READY;
            break;
        case UART3_RX_REG_ADDR:
            request_select = UART3_OUTP_READY;
            break;
        case UART4_RX_REG_ADDR:
            request_select = UART4_OUTP_READY;
            break;
        case SPI0_RX_REG_ADDR:
            request_select = SPI0_OUTP_READY;
            break;
        case SPI1_RX_REG_ADDR:
            request_select = SPI1_OUTP_READY;
            break;
        case SPI2_RX_REG_ADDR:
            request_select = SPI2_OUTP_READY;
            break;
        case SPI3_RX_REG_ADDR:
            request_select = SPI3_OUTP_READY;
            break;
        case ITRACE_DATA_REG_ADDR:
            request_select = ITRACE_OUTP_READY;
            break;
        case ADC_DATA_REG_ADDR:
            request_select = ADC_OUTP_READY;
            break;
        case PRO_IO_DUO_DATA_REG_ADDR:
            request_select = PRO_IO_DUO_OUTP_READY;
            break;
        case PRO_IO_TETRA_DATA_REG_ADDR:
            request_select = PRO_IO_TETRA_OUTP_READY;
            break;
        case PRO_IO_OCTA_DATA_REG_ADDR:
            request_select = PRO_IO_OCTA_OUTP_READY;
            break;
        case PRO_IO_FUSION_DATA_REG_ADDR:
            request_select = PRO_IO_FUSION_OUTP_READY;
            break;
        default:
            return -EINVAL;
        }
    }

    /* ================= Destination Detection ================= */

    if ((((uint32_t)dest_addr >= DMA_RAM_START_ADDR) &&
         ((uint32_t)dest_addr <= DMA_RAM_END_ADDR)) ||
        (((uint32_t)dest_addr >= DMA_FLASH_START_ADDR) &&
         ((uint32_t)dest_addr <= DMA_FLASH_END_ADDR))) {
        mode_flag |= (1U << 1);
    } else {
        /* -------------------------------------------------------
         * FIX 2: Compute request_shift AFTER source detection,
         * inside the destination peripheral block — matching the
         * baremetal driver which sets shift based on whether the
         * source was memory (shift=0) or peripheral (shift=6).
         * ------------------------------------------------------- */
        request_shift = (mode_flag == 1U) ? 0U : 6U;

        switch ((uint32_t)dest_addr) {
        case AES_INP_REG_ADDR:
            request_select |= ((uint16_t)AES_CAN_TAKE_INPUT << request_shift);
            mode_flag |= DMA_MODE_FAST_DESTINATION;
            break;
        case SHA_INP_REG_ADDR:
            request_select |= ((uint16_t)SHA_CAN_TAKE_INPUT << request_shift);
            mode_flag |= DMA_MODE_FAST_DESTINATION;
            break;
        case RSA_INP_REG_ADDR:
            request_select |= ((uint16_t)RSA_CAN_TAKE_INPUT << request_shift);
            mode_flag |= DMA_MODE_FAST_DESTINATION;
            break;
        case QSPI0_DATA_REG_ADDR:
            request_select |= ((uint16_t)QSPI0_READY << request_shift);
            mode_flag |= DMA_MODE_FAST_DESTINATION;
            break;
        case QSPI1_DATA_REG_ADDR:
            request_select |= ((uint16_t)QSPI1_READY << request_shift);
            mode_flag |= DMA_MODE_FAST_DESTINATION;
            break;
        case UART0_TX_REG_ADDR:
            request_select |= ((uint16_t)UART0_CAN_TAKE_INPUT << request_shift);
            break;
        case UART1_TX_REG_ADDR:
            request_select |= ((uint16_t)UART1_CAN_TAKE_INPUT << request_shift);
            break;
        case UART2_TX_REG_ADDR:
            request_select |= ((uint16_t)UART2_CAN_TAKE_INPUT << request_shift);
            break;
        case UART3_TX_REG_ADDR:
            request_select |= ((uint16_t)UART3_CAN_TAKE_INPUT << request_shift);
            break;
        case UART4_TX_REG_ADDR:
            request_select |= ((uint16_t)UART4_CAN_TAKE_INPUT << request_shift);
            break;
        case SPI0_TX_REG_ADDR:
            request_select |= ((uint16_t)SPI0_CAN_TAKE_INPUT << request_shift);
            break;
        case SPI1_TX_REG_ADDR:
            request_select |= ((uint16_t)SPI1_CAN_TAKE_INPUT << request_shift);
            break;
        case SPI2_TX_REG_ADDR:
            request_select |= ((uint16_t)SPI2_CAN_TAKE_INPUT << request_shift);
            break;
        case SPI3_TX_REG_ADDR:
            request_select |= ((uint16_t)SPI3_CAN_TAKE_INPUT << request_shift);
            break;
        case PRO_IO_DUO_DATA_REG_ADDR:
            request_select |= ((uint16_t)PRO_IO_DUO_CAN_TAKE_INP << request_shift);
            break;
        case PRO_IO_TETRA_DATA_REG_ADDR:
            request_select |= ((uint16_t)PRO_IO_TETRA_CAN_TAKE_INP << request_shift);
            break;
        case PRO_IO_OCTA_DATA_REG_ADDR:
            request_select |= ((uint16_t)PRO_IO_OCTA_CAN_TAKE_INP << request_shift);
            break;
        case PRO_IO_FUSION_DATA_REG_ADDR:
            request_select |= ((uint16_t)PRO_IO_FUSION_CAN_TAKE_INP << request_shift);
            break;
        default:
            return -EINVAL;
        }
    }

    chan_reg->REQUEST_SELECT_REG = request_select;

    /* ================= Read Current CCR ================= */
    config_reg = chan_reg->CONFIG_REG;

    /* ================= Clear Relevant Fields ================= */
    config_reg &= ~(DMA_CFG_PERIPH_TO_PERIPH_MASK |
                    DMA_CFG_TRANSFER_DIR_MASK      |
                    DMA_CFG_MEM_TO_MEM_MASK        |
                    DMA_CFG_PRIORITY_LEVEL_MASK    |
                    DMA_CFG_MEM_DATA_SIZE_MASK     |
                    DMA_CFG_PERIPH_DATA_SIZE_MASK  |
                    DMA_CFG_MEM_ADDR_INC_MASK      |
                    DMA_CFG_PERIPH_ADDR_INC_MASK);

    /* ================= Mode Decode ================= */
    /*
     * mode_flag & 0x03:
     *   0 = P2P, 1 = M2P, 2 = P2M, 3 = M2M
     */
    switch (mode_flag & 0x03U) {
    case 0U: /* P2P */
        config_reg |= DMA_CFG_PERIPH_TO_PERIPH;
        pinc = ((mode_flag & DMA_MODE_FAST_SOURCE) == 0U) ?
               DMA_SLOW_PERIPH_BURST : DMA_FAST_PERIPH_BURST;
        minc = ((mode_flag & DMA_MODE_FAST_DESTINATION) == 0U) ?
               DMA_SLOW_PERIPH_BURST : DMA_FAST_PERIPH_BURST;
        break;

    case 1U: /* M2P */
        temp      = src_addr;
        src_addr  = dest_addr;
        dest_addr = temp;
        config_reg |= DMA_CFG_TRANSFER_DIR;
        pinc = ((mode_flag & DMA_MODE_FAST_DESTINATION) == 0U) ?
               DMA_SLOW_PERIPH_BURST : DMA_FAST_PERIPH_BURST;
        minc = DMA_INC_ENABLE;
        break;

    case 2U: /* P2M */
        pinc = ((mode_flag & DMA_MODE_FAST_SOURCE) == 0U) ?
               DMA_SLOW_PERIPH_BURST : DMA_FAST_PERIPH_BURST;
        minc = DMA_INC_ENABLE;
        break;

    case 3U: /* M2M */
        config_reg |= DMA_CFG_MEM_TO_MEM;
        pinc = DMA_INC_ENABLE;
        minc = DMA_INC_ENABLE;
        break;

    default:
        return -EINVAL;
    }

    /* ================= Program Addresses ================= */
    chan_reg->PERIPH_ADDR_REG = (uint32_t)src_addr;
    chan_reg->MEM_ADDR_REG    = (uint32_t)dest_addr;

    /* ================= Set Required Fields ================= */
    config_reg |= DMA_CFG_PRIORITY_LEVEL(cfg->channel_priority) |
                  DMA_CFG_MEM_DATA_SIZE(src_size_index)          |
                  DMA_CFG_PERIPH_DATA_SIZE(dest_size_index)      |
                  DMA_CFG_MEM_ADDR_INC(minc)                     |
                  DMA_CFG_PERIPH_ADDR_INC(pinc);

    data->chan_cfgs[channel] = cfg;

    if (cfg->dma_callback) {
        config_reg |= (DMA_CFG_TC_INT_ENABLE |
                       DMA_CFG_ERR_INT_ENABLE );
     if (cfg->complete_callback_en) {
        config_reg |= DMA_CFG_HALF_INT_ENABLE;
    }
    }

    /* ================= Write Back ================= */
    chan_reg->CONFIG_REG = config_reg;

    return 0;
}

/* -------------------------------------------------- */
/* START                                              */
/* -------------------------------------------------- */

static int dma_custom_start(const struct device *dev, uint32_t channel)
{
    const struct dma_custom_config *config = dev->config;
    DMA_Type *regs = (DMA_Type *)config->base_addr;

    if (channel >= DMA_CHANNELS_COUNT) {
        return -EINVAL;
    }

    /* Clear any stale interrupt flags before enabling */
    if (regs->INTERRUPT_STATUS_REG) {
        uint32_t s = regs->INTERRUPT_STATUS_REG;
        regs->INT_FLAG_CLEAR_REG |=  s;
        regs->INT_FLAG_CLEAR_REG &= ~s;
    }
    __asm__ volatile("fence rw, rw" ::: "memory");

    /* Enable channel */
    regs->CHANNEL[channel].CONFIG_REG |= DMA_CFG_CHANNEL_ENABLE;

    /* Enable IRQ through Zephyr multilevel IRQ path */
    irq_enable(config->encoded_irqn);

    return 0;
}

/* -------------------------------------------------- */
/* STOP                                               */
/* -------------------------------------------------- */

static int dma_custom_stop(const struct device *dev, uint32_t channel)
{
    const struct dma_custom_config *config = dev->config;
    DMA_Type *regs = (DMA_Type *)config->base_addr;

    if (channel >= DMA_CHANNELS_COUNT) {
        return -EINVAL;
    }

    regs->CHANNEL[channel].CONFIG_REG &= ~DMA_CFG_CHANNEL_ENABLE;
    irq_disable(config->encoded_irqn);
    
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

    if (channel >= DMA_CHANNELS_COUNT || stat == NULL) {
        return -EINVAL;
    }

    uint32_t ccr     = regs->CHANNEL[channel].CONFIG_REG;
    stat->busy           = (ccr & BIT(0)) != 0U;
    stat->pending_length = regs->CHANNEL[channel].TRANSFER_LENGTH_REG;
    stat->dir            = (ccr & BIT(4)) ?
                           MEMORY_TO_PERIPHERAL : PERIPHERAL_TO_MEMORY;
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

    uint32_t isr_status = regs->INTERRUPT_STATUS_REG;

    LOG_DBG("[DMA ISR] Raw ISR status: 0x%08x", isr_status);

    for (uint32_t i = 0; i < DMA_CHANNELS_COUNT; i++) {
        uint32_t tc_flag = BIT(i * 4U + 1U);
        uint32_t ht_flag = BIT(i * 4U + 2U);
        uint32_t te_flag = BIT(i * 4U + 3U);
        uint32_t ch_mask = tc_flag | ht_flag | te_flag;

        if (!(isr_status & ch_mask)) {
            continue;
        }

        uint16_t remaining = regs->CHANNEL[i].TRANSFER_LENGTH_REG;
        LOG_DBG("[DMA ISR] Channel %u: TC=%d HT=%d TE=%d Remaining=%u",
                i,
                (isr_status & tc_flag) ? 1 : 0,
                (isr_status & ht_flag) ? 1 : 0,
                (isr_status & te_flag) ? 1 : 0,
                remaining);

        /* -------------------------------------------------------
         * FIX 3: Clear interrupt flags with a write-then-clear
         * pulse, matching the baremetal DMA_Clear_Interrupt_Flags
         * which does:  REG |= mask  then  REG &= ~mask
         * A single write is not sufficient to clear the flags.
         * ------------------------------------------------------- */
        regs->INT_FLAG_CLEAR_REG |=  ch_mask;
        regs->INT_FLAG_CLEAR_REG &= ~ch_mask;

        const struct dma_config *cfg = data->chan_cfgs[i];

        if (cfg != NULL && cfg->dma_callback != NULL) {
            int status;

            if (isr_status & te_flag) {
                status = -EIO;
                LOG_DBG("[DMA ISR] CH%u ERROR", i);
            } else if (isr_status & tc_flag) {
                status = 0;
                regs->CHANNEL[i].CONFIG_REG &= ~DMA_CFG_CHANNEL_ENABLE;
                LOG_DBG("[DMA ISR] CH%u COMPLETE", i);
            } else if (isr_status & ht_flag) {
                status = 1;
                LOG_DBG("[DMA ISR] CH%u HALF Remaining=%u/%u",
                        i, remaining, cfg->head_block->block_size);
            } else {
                continue;
            }

            cfg->dma_callback(dev, cfg->user_data, i, status);
        }
    }
}

static int dma_custom_suspend(const struct device *dev, uint32_t channel)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(channel);
    return -ENOSYS;  /* not supported */
}

static int dma_custom_resume(const struct device *dev, uint32_t channel)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(channel);
    return -ENOSYS;  /* not supported */
}

static int dma_custom_request_channel(const struct device *dev,
                                      void *filter_param)
{
    struct dma_custom_data *data = dev->data;
    ARG_UNUSED(filter_param);

    /*
     * dma_context.atomic is a bitmask of in-use channels.
     * Find the first bit that is 0 (free), set it, return the index.
     * This is exactly what dma_request_channel is supposed to do.
     */
    for (uint32_t i = 0; i < DMA_CHANNELS_COUNT; i++) {
        if (!atomic_test_and_set_bit(data->ctx.atomic, i)) {
            return (int)i;
        }
    }

    /* All channels busy */
    return -ENOSPC;
}

static int dma_custom_release_channel(const struct device *dev,
                                      uint32_t channel)
{
    struct dma_custom_data *data = dev->data;

    if (channel >= DMA_CHANNELS_COUNT) {
        return -EINVAL;
    }

    atomic_clear_bit(data->ctx.atomic, channel);
    return 0;
}

/* -------------------------------------------------- */
/* API                                                */
/* -------------------------------------------------- */


static const struct dma_driver_api dma_custom_api = {
    .config     = dma_custom_configure,
    .start      = dma_custom_start,
    .stop       = dma_custom_stop,
    .get_status = dma_custom_get_status,
    .suspend    = dma_custom_suspend,
    .resume     = dma_custom_resume,
};

/* -------------------------------------------------- */
/* INIT                                               */
/* -------------------------------------------------- */

static int dma_custom_init(const struct device *dev)
{
    const struct dma_custom_config *config = dev->config;

    LOG_DBG("[DMA] base=0x%x plic_src=%u irqn=0x%x prio=%u",
            config->base_addr, config->plic_src,
            config->encoded_irqn, config->priority);

    config->irq_config_func(dev);
    return 0;
}

#define DMA_CUSTOM_DEVICE(inst)                                              \
                                                                             \
    static void dma_irq_config_##inst(const struct device *dev)              \
    {                                                                        \
        IRQ_CONNECT(DT_INST_IRQN(inst),                                      \
                    DT_INST_IRQ_BY_IDX(inst, 0, priority),                   \
                    dma_custom_isr,                                           \
                    DEVICE_DT_INST_GET(inst),                                 \
                    0);                                                      \
        irq_enable(DT_INST_IRQN(inst));                                      \
        LOG_DBG("DMA inst=%d irqn=0x%x", inst, DT_INST_IRQN(inst));         \
    }                                                                        \
                                                                             \
    static const struct dma_custom_config dma_config_##inst = {              \
        .base_addr       = DT_INST_REG_ADDR(inst),                           \
        .plic_src        = DT_INST_IRQ_BY_IDX(inst, 0, irq),                 \
        .encoded_irqn    = DT_INST_IRQN(inst),                               \
        .priority        = DT_INST_IRQ_BY_IDX(inst, 0, priority),            \
        .irq_config_func = dma_irq_config_##inst,                            \
    };                                                                       \
                                                                             \
    static struct dma_custom_data dma_data_##inst = {                        \
        .ctx = {                                                             \
            .magic        = DMA_MAGIC,                                       \
            .dma_channels = DMA_CHANNELS_COUNT,                              \
            .atomic       = ATOMIC_INIT(0),                                  \
        },                                                                   \
    };                                                                       \
                                                                             \
    DEVICE_DT_INST_DEFINE(inst, dma_custom_init, NULL,                       \
                          &dma_data_##inst, &dma_config_##inst,              \
                          POST_KERNEL, CONFIG_DMA_INIT_PRIORITY,             \
                          &dma_custom_api);

DT_INST_FOREACH_STATUS_OKAY(DMA_CUSTOM_DEVICE)