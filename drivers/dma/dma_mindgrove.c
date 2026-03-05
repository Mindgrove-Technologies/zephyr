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

/* Driver Runtime Data */
struct dma_custom_data {
	struct dma_context ctx;
	const struct dma_config *chan_cfgs[DMA_CHANNELS_COUNT];
};

/* Driver Constant Configuration */
struct dma_custom_config {
	uint32_t base_addr;
	void (*irq_config)(const struct device *dev);
};

/* -------------------------------------------------- */
/* CONFIGURE */
/* -------------------------------------------------- */

static int dma_custom_configure(const struct device *dev, uint32_t channel, struct dma_config *cfg)
{
	printk("DMA CONFIGURE called: channel=%d\n", channel);

	const struct dma_custom_config *config = dev->config;
	struct dma_custom_data *data = dev->data;

	uint32_t ccr = 0U;
	uint16_t cselr = 0U;
	uint8_t mode_flag = 0U;
	uint8_t shift = 0U;

	uint32_t pinc = 0U;
	uint32_t minc = 0U;
	uint32_t m2m = 0U;
	uint32_t p2p = 0U;
	uint32_t dir = 0U;

	volatile uint32_t *src_addr;
	volatile uint32_t *dest_addr;
	volatile uint32_t *temp;

	DMA_Type *regs = (DMA_Type *)(uintptr_t)config->base_addr;

	printk("DMA base addr: 0x%x\n", config->base_addr);

	/* 1. Translate Zephyr Byte Count to Mindgrove Hardware Enum Index */
	uint32_t src_size_index;
	uint32_t dest_size_index;

	// Map: 1->0 (BYTE), 2->1 (TWOBYTE), 4->2 (FOURBYTE), 8->3 (EIGHTBYTE)
	src_size_index = (cfg->source_data_size == 8)   ? 3
			 : (cfg->source_data_size == 4) ? 2
			 : (cfg->source_data_size == 2) ? 1
							: 0;

	dest_size_index = (cfg->dest_data_size == 8)   ? 3
			  : (cfg->dest_data_size == 4) ? 2
			  : (cfg->dest_data_size == 2) ? 1
						       : 0;

	if (channel >= DMA_CHANNELS_COUNT) {
		printk("DMA ERROR: invalid channel\n");
		return -EINVAL;
	}

	DMA_CHANNEL_Type *chan_reg = &regs->CH[channel];

	chan_reg->DMA_CNDTR = (uint16_t)(cfg->head_block->block_size);

	printk("Block size = %d\n", cfg->head_block->block_size);

	src_addr = (volatile uint32_t *)cfg->head_block->source_address;
	dest_addr = (volatile uint32_t *)cfg->head_block->dest_address;

	printk("SRC = 0x%x DEST = 0x%x\n", (uint32_t)src_addr, (uint32_t)dest_addr);

	/* ---------------- Source Detection ---------------- */

	if ((((uint32_t)src_addr >= 0x80000000U) && ((uint32_t)src_addr <= 0x80020000U)) ||
	    (((uint32_t)src_addr >= 0x90000000U) && ((uint32_t)src_addr <= 0xD0000000U))) {

		printk("Source detected: MEMORY\n");
		mode_flag = 1U;

	} else {

		printk("Source detected: PERIPHERAL\n");

		switch ((uint32_t)src_addr) {

		case 0x04000040U:
			printk("AES output\n");
			cselr = AES_OUTP_READY;
			mode_flag |= MODE_FAST_SRC;
			break;

		case 0x03000080U:
			printk("SHA output\n");
			cselr = SHA_OUTP_READY;
			mode_flag |= MODE_FAST_SRC;
			break;

		case 0x05000080U:
			printk("RSA output\n");
			cselr = RSA_OUTP_READY;
			mode_flag |= MODE_FAST_SRC;
			break;

		default:
			printk("Unknown source peripheral\n");
			return EINVAL;
		}
	}

	shift = 0U;
	if (mode_flag == 0U) {
		shift = 6U;
	}

	/* ---------------- Destination Detection ---------------- */

	if ((((uint32_t)dest_addr >= 0x80000000U) && ((uint32_t)dest_addr <= 0x80020000U)) ||
	    (((uint32_t)dest_addr >= 0x90000000U) && ((uint32_t)dest_addr <= 0xD0000000U))) {

		printk("Destination detected: MEMORY\n");
		mode_flag |= (1U << 1);

	} else {

		printk("Destination detected: PERIPHERAL\n");

		switch ((uint32_t)dest_addr) {

		case 0x04000000U:
			printk("AES input\n");
			cselr |= (AES_CAN_TAKE_INPUT << shift);
			mode_flag |= MODE_FAST_DEST;
			break;

		default:
			printk("Unknown destination peripheral\n");
			return EINVAL;
		}
	}

	chan_reg->DMA_CSELR = cselr;

	printk("CSELR programmed: 0x%x\n", cselr);

	/* ---------------- Mode Decode ---------------- */

	switch (mode_flag & 0x03U) {

	case 0U:
		printk("DMA MODE: P2P\n");
		p2p = DMA_CCR_P2P;
		pinc = ((mode_flag & MODE_FAST_SRC) == 0U) ? DMA_SLOW_PERIPH_BURST
							   : DMA_FAST_PERIPH_BURST;

		minc = ((mode_flag & MODE_FAST_DEST) == 0U) ? DMA_SLOW_PERIPH_BURST
							    : DMA_FAST_PERIPH_BURST;

		break;

	case 1U:
		printk("DMA MODE: M2P\n");
		temp = src_addr;
		src_addr = dest_addr;
		dest_addr = temp;
		dir = DMA_CCR_DIR;
		pinc = ((mode_flag & MODE_FAST_DEST) == 0U) ? DMA_SLOW_PERIPH_BURST
							    : DMA_FAST_PERIPH_BURST;

		minc = DMA_INC_ENABLE;
		break;

	case 2U:
		printk("DMA MODE: P2M\n");
		pinc = ((mode_flag & MODE_FAST_SRC) == 0U) ? DMA_SLOW_PERIPH_BURST
							   : DMA_FAST_PERIPH_BURST;

		minc = DMA_INC_ENABLE;

		break;

	case 3U:
		printk("DMA MODE: M2M\n");
		temp = src_addr;
		src_addr = dest_addr;
		dest_addr = temp;
		dir = DMA_CCR_DIR;
		m2m = DMA_CCR_MEM2MEM;

		pinc = DMA_INC_ENABLE;
		minc = DMA_INC_ENABLE;
		break;
	}

	chan_reg->DMA_CPAR = (uint32_t)src_addr;
	chan_reg->DMA_CMAR = (uint32_t)dest_addr;

	printk("CPAR=0x%x CMAR=0x%x\n", chan_reg->DMA_CPAR, chan_reg->DMA_CMAR);

	data->chan_cfgs[channel] = cfg;

	ccr = DMA_CCR_PL(cfg->channel_priority) | DMA_CCR_MSIZE(src_size_index) |
	      DMA_CCR_PSIZE(dest_size_index) | DMA_CCR_MINC(minc) | DMA_CCR_PINC(pinc) | dir | m2m |
	      p2p;

	if (cfg->dma_callback) {
		printk("DMA callback enabled\n");
		ccr |= (DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_HTIE);
	}

	chan_reg->DMA_CCR = ccr;

	printk("CCR programmed: 0x%x\n", chan_reg->DMA_CCR);

	return 0;
}

/* -------------------------------------------------- */
/* START */
/* -------------------------------------------------- */

static int dma_custom_start(const struct device *dev, uint32_t channel)
{
	printk("DMA START channel=%d\n", channel);

	const struct dma_custom_config *config = dev->config;
	DMA_Type *regs = (DMA_Type *)config->base_addr;

	if (channel >= DMA_CHANNELS_COUNT) {
		printk("Invalid channel start\n");
		return -EINVAL;
	}

	/* Simple start - just enable and go */
	uint32_t reg_val = regs->CH[channel].DMA_CCR;
	printk("CCR before = 0x%x\n", reg_val);

	reg_val |= DMA_CCR_EN;
	regs->CH[channel].DMA_CCR = reg_val;

	/* Poll for completion instead of using interrupts */
	printk("Polling for completion...\n");

	uint32_t timeout = 1000000;
	while (timeout--) {
		uint32_t isr = regs->DMA_ISR;
		if (isr & BIT(channel * 4 + 1)) { /* TCIF flag */
			printk("Transfer complete! ISR=0x%x\n", isr);

			/* Clear the flag */
			regs->DMA_IFCR = BIT(channel * 4 + 1);

			/* Call callback manually */
			struct dma_custom_data *data = dev->data;
			const struct dma_config *cfg = data->chan_cfgs[channel];
			if (cfg && cfg->dma_callback) {
				cfg->dma_callback(dev, cfg->user_data, channel, 0);
			}
			break;
		}
	}

	if (timeout == 0) {
		printk("ERROR: DMA timeout!\n");
	}

	return 0;
}
/* -------------------------------------------------- */
/* STOP */
/* -------------------------------------------------- */

static int dma_custom_stop(const struct device *dev, uint32_t channel)
{
	printk("DMA STOP channel=%d\n", channel);

	const struct dma_custom_config *config = dev->config;
	DMA_Type *regs = (DMA_Type *)config->base_addr;

	if (channel >= DMA_CHANNELS_COUNT) {
		printk("Invalid channel stop\n");
		return -EINVAL;
	}

	regs->CH[channel].DMA_CCR &= ~BIT(0);

	return 0;
}

/* -------------------------------------------------- */
/* STATUS */
/* -------------------------------------------------- */

static int dma_custom_get_status(const struct device *dev, uint32_t channel,
				 struct dma_status *stat)
{
	const struct dma_custom_config *config = dev->config;
	DMA_Type *regs = (DMA_Type *)config->base_addr;

	if (channel >= DMA_CHANNELS_COUNT || stat == NULL) {
		printk("DMA STATUS error\n");
		return -EINVAL;
	}

	uint32_t ccr = regs->CH[channel].DMA_CCR;

	stat->busy = (ccr & BIT(0)) != 0U;
	stat->pending_length = regs->CH[channel].DMA_CNDTR;
	stat->dir = (ccr & BIT(4)) ? MEMORY_TO_PERIPHERAL : PERIPHERAL_TO_MEMORY;

	printk("STATUS: ch=%d busy=%d remaining=%d\n", channel, stat->busy, stat->pending_length);

	return 0;
}

/* -------------------------------------------------- */
/* ISR */
/* -------------------------------------------------- */

static void dma_custom_isr(const void *arg)
{
	const struct device *dev = arg;
	printk(">>> DMA ISR ENTERED <<<\n");

	const struct dma_custom_config *config = dev->config;
	struct dma_custom_data *data = dev->data;
	DMA_Type *regs = (DMA_Type *)(uintptr_t)config->base_addr;

	uint32_t isr_status = regs->DMA_ISR;
	printk("ISR status = 0x%x\n", isr_status);

	for (uint32_t i = 0; i < DMA_CHANNELS_COUNT; i++) {
		uint32_t tc_flag = BIT(i * 4 + 1);
		uint32_t te_flag = BIT(i * 4 + 3);
		uint32_t ht_flag = BIT(i * 4 + 2);

		if (isr_status & (tc_flag | te_flag | ht_flag)) {
			printk("Channel %d interrupt! flags=0x%x\n", i,
			       isr_status & (tc_flag | te_flag | ht_flag));

			// Clear interrupts for this channel
			regs->DMA_IFCR = (tc_flag | te_flag | ht_flag);

			const struct dma_config *cfg = data->chan_cfgs[i];
			if (cfg && cfg->dma_callback) {
				int status = (isr_status & te_flag) ? -EIO : 0;
				cfg->dma_callback(dev, cfg->user_data, i, status);
			}
		}
	}

	printk("<<< DMA ISR EXITING >>>\n");
}

/* -------------------------------------------------- */
/* INIT */
/* -------------------------------------------------- */

static const struct dma_driver_api dma_custom_api = {
	.config = dma_custom_configure,
	.start = dma_custom_start,
	.stop = dma_custom_stop,
	.get_status = dma_custom_get_status,
};

static int dma_custom_init(const struct device *dev)
{
	printk("Initializing Mindgrove DMA driver\n");

	const struct dma_custom_config *config = dev->config;

	config->irq_config(dev);

	/* Get both IRQ numbers */
	uint32_t plic_irq = DT_INST_IRQ_BY_IDX(0, 0, irq);
	uint32_t zephyr_irq = DT_INST_IRQN(0);
	uint32_t priority = DT_INST_IRQ_BY_IDX(0, 0, priority);

	printk("DMA IRQ config: PLIC=%d, Zephyr=%d, priority=%d\n", plic_irq, zephyr_irq, priority);

	/* Verify PLIC IRQ is valid */
	if (plic_irq > 1024) {
		printk("WARNING: PLIC IRQ %d seems invalid\n", plic_irq);
	}

	return 0;
}
/* -------------------------------------------------- */
/* DEVICE MACRO */
/* -------------------------------------------------- */
#define DMA_CUSTOM_DEVICE(inst)                                                                    \
                                                                                                   \
	static void dma_custom_irq_cfg_##inst(const struct device *dev)                            \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
                                                                                                   \
		printk("Configuring DMA inst %d\n", inst);                                         \
		printk("Zephyr IRQ = %d\n", DT_INST_IRQN(inst));                                   \
		printk("PLIC source ID = %d\n", DT_INST_IRQ_BY_IDX(inst, 0, irq));                 \
                                                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ_BY_IDX(inst, 0, priority),             \
			    dma_custom_isr, DEVICE_DT_INST_GET(inst), 0);                          \
                                                                                                   \
		/* Set priority high enough (0 is highest, 1 is next) */                           \
		riscv_plic_set_priority(DT_INST_IRQ_BY_IDX(inst, 0, irq), 0);                      \
                                                                                                   \
		/* Enable the interrupt in PLIC */                                                 \
		riscv_plic_irq_enable(DT_INST_IRQ_BY_IDX(inst, 0, irq));                           \
                                                                                                   \
		/* Also enable at CPU level */                                                     \
		irq_enable(DT_INST_IRQN(inst));                                                    \
                                                                                                   \
		printk("DMA IRQ %d fully enabled with priority 0\n",                               \
		       DT_INST_IRQ_BY_IDX(inst, 0, irq));                                          \
	}                                                                                          \
                                                                                                   \
	static const struct dma_custom_config dma_config_##inst = {                                \
		.base_addr = DT_INST_REG_ADDR(inst),                                               \
		.irq_config = dma_custom_irq_cfg_##inst,                                           \
	};                                                                                         \
                                                                                                   \
	static struct dma_custom_data dma_data_##inst = {                                          \
		.ctx =                                                                             \
			{                                                                          \
				.magic = DMA_MAGIC,                                                \
				.dma_channels = DMA_CHANNELS_COUNT,                                \
				.atomic = ATOMIC_INIT(0),                                          \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, dma_custom_init, NULL, &dma_data_##inst, &dma_config_##inst,   \
			      POST_KERNEL, CONFIG_DMA_INIT_PRIORITY, &dma_custom_api);

DT_INST_FOREACH_STATUS_OKAY(DMA_CUSTOM_DEVICE)
