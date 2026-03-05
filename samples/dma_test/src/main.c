#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/sys/printk.h>

#define DMA_NODE DT_NODELABEL(dma0)

static const struct device *dma_dev = DEVICE_DT_GET(DMA_NODE);

K_SEM_DEFINE(dma_sem, 0, 1);

static uint32_t tx_data[10] = {
    0xAAAAAAAA, 0xBBBBBBBB, 0xCCCCCCCC, 0xDDDDDDDD, 0xEEEEEEEE,
    0xFFFFFFFF, 0x11111111, 0x22222222, 0x33333333, 0x44444444
};

static uint32_t rx_data[10] = {0};

static void dma_callback(const struct device *dev, void *user_data,
                         uint32_t channel, int status)
{
    printk("DMA callback: channel=%d status=%d\n", channel, status);

    k_sem_give(&dma_sem);
}

int main(void)
{
    printk("DMA test start\n");

    if (!device_is_ready(dma_dev)) {
        printk("DMA device not ready\n");
        return 0;
    }

    struct dma_block_config block = {0};
    struct dma_config cfg = {0};

    block.source_address = (uint32_t)tx_data;
    block.dest_address = (uint32_t)rx_data;
    block.block_size = 40;

    cfg.channel_direction = MEMORY_TO_MEMORY;
    cfg.source_data_size = 4;
    cfg.dest_data_size = 4;
    cfg.source_burst_length = 1;
    cfg.dest_burst_length = 1;
    cfg.channel_priority = 2;
    cfg.head_block = &block;

    cfg.dma_callback = dma_callback;
    cfg.user_data = NULL;

    int channel = 0;

    printk("Configuring DMA\n");

    if (dma_config(dma_dev, channel, &cfg) != 0) {
        printk("DMA config failed\n");
        return 0;
    }

    printk("Starting DMA\n");

    if (dma_start(dma_dev, channel) != 0) {
        printk("DMA start failed\n");
        return 0;
    }

    printk("Waiting for DMA interrupt\n");
    k_sem_take(&dma_sem, K_FOREVER);
    printk("DMA transfer completed\n");

    for (int i = 0; i < 10; i++) {
        printk("rx[%d] = 0x%x\n", i, rx_data[i]);
    }

    return 0;
}