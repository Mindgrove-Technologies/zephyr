/*
 * Copyright (c) Mindgrove Technologies Pvt. Ltd 2023. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mindgrove_spi

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_mindgrove);

#include <soc.h>
#include <stdbool.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include "spi_mindgrove.h"
#include "spi_context.h"
#include <stdio.h>
#include <zephyr/drivers/pinctrl.h>

sspi_struct *sspi_instance[SSPI_MAX_COUNT];

#define CLEAR_MASK 0x0000

/* Get SPI instance number from device config, not a global */
#define GET_SPI_NUM(dev) \
    ((int)(((struct spi_shakti_cfg *)(dev)->config)->spi_num))

static int spi_shakti_transceive(const struct device *dev,
                                  const struct spi_config *config,
                                  const struct spi_buf_set *tx_bufs,
                                  const struct spi_buf_set *rx_bufs)
{
    /* Get SPI number from device — fixes the global variable bug */
    int spi_num = GET_SPI_NUM(dev);
    uint32_t len;
    volatile uint32_t temp = 0;
    int pol, pha, lsb_first, comm_mode, spi_size, master_mode;
    int prescale = 30, setup_time = 0, hold_time = 0;

    if ((config->operation & 0x1) == 0) {
        master_mode = MASTER;
    } else {
        printk("Slave is not supported\n");
        return 1;
    }

    if ((config->operation & POL_AND_PHA) == POL_AND_PHA) {
        pol = 1; pha = 1;
    } else if ((config->operation & INV_POLANDPHA) == INV_POLANDPHA) {
        pol = 0; pha = 0;
    } else {
        printk("Invalid pol and pha combination\n");
        return -EINVAL;
    }

    if ((config->operation & SPI_TRANSFER_LSB) == SPI_TRANSFER_MSB) {
        lsb_first = MSB_FIRST;
    } else {
        lsb_first = LSB_FIRST;
    }

    if (config->operation & SPI_WORD_SET(8)) {
        spi_size = DATA_SIZE_8;
    } else if (config->operation & SPI_WORD_SET(16)) {
        spi_size = DATA_SIZE_16;
    } else if (config->operation & SPI_WORD_SET(32)) {
        spi_size = DATA_SIZE_32;
    } else {
        printk("Invalid data size\n");
        return -EINVAL;
    }

    if ((config->operation & HALFDUPLEX) == HALFDUPLEX) {
        comm_mode = HALF_DUPLEX;
    } else if ((config->operation & FULLDUPLEX) == FULLDUPLEX) {
        comm_mode = FULL_DUPLEX;
    } else if ((config->operation & SIMPLEX_TX) == SIMPLEX_TX) {
        comm_mode = SIMPLEX_TX;
    } else {
        comm_mode = FULL_DUPLEX; /* default */
    }

    if ((pol == 0 && pha == 1) || (pol == 1 && pha == 0)) {
        printk("Unsupported SPI mode (pol/pha mismatch)\n");
        return -EINVAL;
    }

    /* Local RX buffer fallback */
    uint8_t rx_fallback[16] = {0};
    struct spi_buf rx_loc_buf = {
        .buf = rx_fallback,
        .len = tx_bufs ? tx_bufs->buffers->len : 0
    };
    struct spi_buf_set rx_loc_set = { .buffers = &rx_loc_buf, .count = 1 };
    if (rx_bufs == NULL) {
        rx_bufs = &rx_loc_set;
    }

    /* Configure clock */
    sspi_instance[spi_num]->clk_control = CLEAR_MASK;
    sspi_instance[spi_num]->clk_control =
        SPI_TX2SS_DELAY(hold_time) | SPI_SS2TX_DELAY(setup_time) |
        SPI_PRESCALE(prescale) | SPI_CLK_POLARITY(pol) | SPI_CLK_PHASE(pha);

    /* Chip select */
    sspi_instance[spi_num]->ncs_ctrl = SPI_NCS_SW(1) | SPI_NCS_SELECT(0);

    /* Comm control */
    sspi_instance[spi_num]->comm_control = CLEAR_MASK;
    int out_en = (master_mode == MASTER)
        ? (SPI_OUT_EN_SCLK | SPI_OUT_EN_NCS | SPI_OUT_EN_MOSI)
        : SPI_OUT_EN_MISO;

    sspi_instance[spi_num]->comm_control =
        SPI_MODE(master_mode) | SPI_LSB_FIRST(lsb_first) |
        SPI_COMM_MODE(comm_mode) | SPI_TOTAL_BITS_TX(spi_size) |
        SPI_TOTAL_BITS_RX(spi_size) | out_en;

    /* Wait for not busy */
    uint32_t temp1 = sspi_instance[spi_num]->comm_control;
    while ((sspi_instance[spi_num]->comm_status & SPI_BUSY) == SPI_BUSY);

    sspi_instance[spi_num]->comm_control = temp1 | SPI_ENABLE(ENABLE);

    if (tx_bufs == NULL && rx_bufs == NULL) return 0;

    spi_context_buffers_setup(&SPI_DATA(dev)->ctx, tx_bufs, rx_bufs, 1);
    len = tx_bufs ? tx_bufs->buffers->len : rx_bufs->buffers->len;

    if (comm_mode == SIMPLEX_TX) {
        for (uint32_t i = 0; i < len; i++) {
            /* Wait for TX not full */
            while (sspi_instance[spi_num]->fifo_status & SPI_TX_FULL);

            if (spi_size == DATA_SIZE_8) {
                sspi_instance[spi_num]->data_tx.data_8 =
                    ((uint8_t *)(tx_bufs->buffers->buf))[i];
            } else if (spi_size == DATA_SIZE_16) {
                sspi_instance[spi_num]->data_tx.data_16 =
                    ((uint16_t *)(tx_bufs->buffers->buf))[i];
            } else {
                sspi_instance[spi_num]->data_tx.data_32 =
                    ((uint32_t *)(tx_bufs->buffers->buf))[i];
            }

            /* Wait for TX done */
            while ((sspi_instance[spi_num]->comm_status & SPI_BUSY) == SPI_BUSY);
        }
    } else if (comm_mode == SIMPLEX_RX) {
        for (uint32_t i = 0; i < len; i++) {
            /* Wait for RX not empty */
            while ((sspi_instance[spi_num]->fifo_status & SPI_RX_EMPTY) == SPI_RX_EMPTY);

            if (spi_size == DATA_SIZE_8) {
                ((uint8_t *)(rx_bufs->buffers->buf))[i] =
                    sspi_instance[spi_num]->data_rx.data_8;
            } else if (spi_size == DATA_SIZE_16) {
                ((uint16_t *)(rx_bufs->buffers->buf))[i] =
                    sspi_instance[spi_num]->data_rx.data_16;
            } else {
                ((uint32_t *)(rx_bufs->buffers->buf))[i] =
                    sspi_instance[spi_num]->data_rx.data_32;
            }
        }
    } else { /* FULL_DUPLEX or HALF_DUPLEX */
        for (uint32_t i = 0; i < len; i++) {
            /* Wait for TX not full, then write */
            while (sspi_instance[spi_num]->fifo_status & SPI_TX_FULL);

            if (spi_size == DATA_SIZE_8) {
                sspi_instance[spi_num]->data_tx.data_8 =
                    ((uint8_t *)(tx_bufs->buffers->buf))[i];
            } else if (spi_size == DATA_SIZE_16) {
                sspi_instance[spi_num]->data_tx.data_16 =
                    ((uint16_t *)(tx_bufs->buffers->buf))[i];
            } else {
                sspi_instance[spi_num]->data_tx.data_32 =
                    ((uint32_t *)(tx_bufs->buffers->buf))[i];
            }

            /* Wait for RX not empty — THIS was the bug, both branches broke */
            int timeout = 100000;
            while ((sspi_instance[spi_num]->fifo_status & SPI_RX_EMPTY) == SPI_RX_EMPTY
                   && timeout-- > 0) {
                k_busy_wait(1);
            }
            if (timeout <= 0) {
                printk("SPI RX timeout at byte %d\n", i);
                return -ETIMEDOUT;
            }

            if (spi_size == DATA_SIZE_8) {
                ((uint8_t *)(rx_bufs->buffers->buf))[i] =
                    sspi_instance[spi_num]->data_rx.data_8;
            } else if (spi_size == DATA_SIZE_16) {
                ((uint16_t *)(rx_bufs->buffers->buf))[i] =
                    sspi_instance[spi_num]->data_rx.data_16;
            } else {
                ((uint32_t *)(rx_bufs->buffers->buf))[i] =
                    sspi_instance[spi_num]->data_rx.data_32;
            }
        }
    }

    return 0;
}

int sspi_shakti_init(const struct device *dev)
{
    struct spi_shakti_cfg *cfg = (struct spi_shakti_cfg *)dev->config;
    int ret;

    /* Apply pinctrl if configured (spi2/spi3) */
    if (cfg->pcfg != NULL) {
        ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
        if (ret < 0 && ret != -ENOENT) {
            return ret;
        }
    }

    int spi_num = cfg->spi_num;
    printk("SPI%d init (base=0x%x)\n", spi_num, cfg->base);

    if (spi_num >= 0 && spi_num < SSPI_MAX_COUNT) {
        sspi_instance[spi_num] = (sspi_struct *)(SSPI0_BASE_ADDRESS +
                                  (spi_num * SSPI_BASE_OFFSET));
        return 0;
    }

    printk("Invalid SPI instance %d\n", spi_num);
    return -ENODEV;
}

static int spi_shakti_release(const struct device *dev,
                               const struct spi_config *config)
{
    return 0;
}

static struct spi_driver_api spi_shakti_api = {
    .transceive = spi_shakti_transceive,
    .release    = spi_shakti_release,
};

#define SPI_PINCTRL_DEFINE(n) \
    COND_CODE_1(DT_INST_PINCTRL_HAS_NAME(n, default), \
        (PINCTRL_DT_INST_DEFINE(n);), ())

#define SPI_PINCTRL_ASSIGN(n) \
    COND_CODE_1(DT_INST_PINCTRL_HAS_NAME(n, default), \
        (PINCTRL_DT_INST_DEV_CONFIG_GET(n)), \
        (NULL))

#define SPI_INIT(n)                                                     \
    SPI_PINCTRL_DEFINE(n)                                               \
    static struct spi_shakti_data spi_shakti_data_##n = {               \
        SPI_CONTEXT_INIT_LOCK(spi_shakti_data_##n, ctx),                \
        SPI_CONTEXT_INIT_SYNC(spi_shakti_data_##n, ctx),                \
    };                                                                  \
    static struct spi_shakti_cfg spi_shakti_cfg_##n = {                 \
        .ncs     = GPIO_DT_SPEC_INST_GET_OR(n, cs_gpios, {0}),         \
        .base    = DT_INST_REG_ADDR(n),                                 \
        .f_sys   = DT_INST_PROP(n, clock_frequency),                    \
        .pcfg    = SPI_PINCTRL_ASSIGN(n),                               \
        .spi_num = n,                                                   \
    };                                                                  \
    DEVICE_DT_INST_DEFINE(n,                                            \
        sspi_shakti_init,                                               \
        NULL,                                                           \
        &spi_shakti_data_##n,                                           \
        &spi_shakti_cfg_##n,                                            \
        POST_KERNEL,                                                    \
        CONFIG_SPI_INIT_PRIORITY,                                       \
        &spi_shakti_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_INIT)