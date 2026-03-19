#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/sys/printk.h>

/*WORKING m2m DMA FULL TRANSFER*/
// #define DMA_NODE DT_NODELABEL(dma0)

// static const struct device *dma_dev = DEVICE_DT_GET(DMA_NODE);

// K_SEM_DEFINE(dma_sem, 0, 1);

// static uint32_t tx_data[10] = {
//     0xAAAAAAAA, 0xBBBBBBBB, 0xCCCCCCCC, 0xDDDDDDDD, 0xEEEEEEEE,
//     0xFFFFFFFF, 0x11111111, 0x22222222, 0x33333333, 0x44444444
// };

// static uint32_t rx_data[10] = {0};

// static void dma_callback(const struct device *dev, void *user_data,
//                          uint32_t channel, int status)
// {
//     printk("DMA callback: channel=%d status=%d\n", channel, status);

//     k_sem_give(&dma_sem);
// }

// int main(void)
// {
//     printk("DMA test start\n");

//     if (!device_is_ready(dma_dev)) {
//         printk("DMA device not ready\n");
//         return 0;
//     }

//     struct dma_block_config block = {0};
//     struct dma_config cfg = {0};

//     block.source_address = (uint32_t)tx_data;
//     block.dest_address = (uint32_t)rx_data;
//     block.block_size = 40;

//     cfg.channel_direction = MEMORY_TO_MEMORY;
//     cfg.source_data_size = 4;
//     cfg.dest_data_size = 4;
//     cfg.source_burst_length = 1;
//     cfg.dest_burst_length = 1;
//     cfg.channel_priority = 2;
//     cfg.head_block = &block;

//     cfg.dma_callback = dma_callback;
//     cfg.user_data = NULL;

//     int channel = 0;

//     printk("Configuring DMA\n");

//     if (dma_config(dma_dev, channel, &cfg) != 0) {
//         printk("DMA config failed\n");
//         return 0;
//     }

//     printk("Starting DMA\n");

//     if (dma_start(dma_dev, channel) != 0) {
//         printk("DMA start failed\n");
//         return 0;
//     }

//     printk("Waiting for DMA interrupt\n");
//     k_sem_take(&dma_sem, K_FOREVER);
//     printk("DMA transfer completed\n");

//     for (int i = 0; i < 10; i++) {
//         printk("rx[%d] = 0x%x\n", i, rx_data[i]);
//     }

//     return 0;
// }

/* WORKING m2m DMA HALF TRANSFER*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/sys/printk.h>

#define DMA_NODE DT_NODELABEL(dma0)
#define BUFFER_SIZE 10

static uint32_t tx_data[BUFFER_SIZE] = {
    0xAAAAAAAA, 0xBBBBBBBB, 0xCCCCCCCC, 0xDDDDDDDD,
    0xEEEEEEEE, 0xFFFFFFFF, 0x11111111, 0x22222222,
    0x33333333, 0x44444444
};

static uint32_t rx_data[BUFFER_SIZE] = {0};

static K_SEM_DEFINE(ht_sem, 0, 1);

static void dma_callback(const struct device *dev, void *user_data,
                         uint32_t channel, int status)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);
    printk("[CALLBACK] REACHED ISR");
    if (status == 1) {  // Half transfer
        printk("[CALLBACK] HALF TRANSFER (ch=%d)\n", channel);
        k_sem_give(&ht_sem);
    }
}

void main(void)
{
    const struct device *dma_dev = DEVICE_DT_GET(DMA_NODE);
    if (!device_is_ready(dma_dev)) {
        printk("DMA device not ready!\n");
        return;
    }

    printk("=== DMA HALF TRANSFER TEST START ===\n");

    struct dma_block_config block = {0};
    struct dma_config cfg = {0};

    block.source_address = (uint32_t)tx_data;
    block.dest_address   = (uint32_t)rx_data;
    block.block_size     = sizeof(tx_data);

    cfg.channel_direction = MEMORY_TO_MEMORY;
    cfg.source_data_size  = 4;
    cfg.dest_data_size    = 4;
    cfg.block_count       = 1;
    cfg.head_block        = &block;
    cfg.dma_callback      = dma_callback;
    cfg.user_data         = NULL;

    int channel = 0;

    printk("Configuring DMA...\n");
    if (dma_config(dma_dev, channel, &cfg) != 0) {
        printk("DMA config failed!\n");
        return;
    }

    printk("Starting DMA...\n");
    if (dma_start(dma_dev, channel) != 0) {
        printk("DMA start failed!\n");
        return;
    }

    /* Wait for half-transfer interrupt */
    k_sem_take(&ht_sem, K_FOREVER);
    printk("Half transfer reached!\n");

    /* Optionally verify first half of the buffer */
    for (int i = 0; i < BUFFER_SIZE ; i++) {
        printk("rx[%d] = 0x%x\n", i, rx_data[i]);
    }

    printk("=== DMA HALF TRANSFER TEST DONE ===\n");
}


/* p2m DMA FULL TRANSFER*/
// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/drivers/dma.h>
// #include <zephyr/drivers/uart.h>
// #include <zephyr/sys/printk.h>
// #include <string.h>

// #define DMA_NODE    DT_NODELABEL(dma0)
// #define UART_NODE   DT_NODELABEL(uart2)
// #define DMA_CHANNEL 0
// #define BUFFER_SIZE 16

// /* Test string to transmit */
// static const char tx_str[BUFFER_SIZE] = "MINDGROVE FACTS";

// /* RAM buffer to receive via DMA */
// static uint8_t rx_buffer[BUFFER_SIZE] __aligned(4);

// /* Semaphore to signal DMA completion */
// K_SEM_DEFINE(dma_sem, 0, 1);

// /* DMA callback */
// static void dma_callback(const struct device *dev, void *user_data,
//                          uint32_t channel, int status)
// {
//     ARG_UNUSED(dev);
//     ARG_UNUSED(user_data);

//     if (status == 0) {
//         printk("[DMA CALLBACK] Transfer complete (ch=%d)\n", channel);
//         k_sem_give(&dma_sem);
//     } else if (status == 1) {
//         printk("[DMA CALLBACK] Half-transfer (ch=%d)\n", channel);
//     } else {
//         printk("[DMA CALLBACK] ERROR (ch=%d)\n", channel);
//     }
// }

// void main(void)
// {
//     const struct device *dma_dev  = DEVICE_DT_GET(DMA_NODE);
//     const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);

//     if (!device_is_ready(dma_dev) || !device_is_ready(uart_dev)) {
//         printk("DMA or UART device not ready!\n");
//         return;
//     }

//     printk("=== UART2 → Memory DMA TEST START ===\n");

//     /* Transmit the test string over UART2 */
//     for (int i = 0; i < BUFFER_SIZE; i++) {
//         uart_poll_out(uart_dev, tx_str[i]);
//     }

//     /* Configure DMA block for P2M transfer */
//     struct dma_block_config block_cfg = {0};
//     block_cfg.source_address = 0x11508;   // Mindgrove UART2 RX register
//     block_cfg.dest_address   = (uint32_t)rx_buffer; // RAM buffer
//     block_cfg.block_size     = BUFFER_SIZE;

//     /* Configure DMA channel */
//     struct dma_config dma_cfg = {0};
//     dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
//     dma_cfg.source_data_size  = 1;  // 8-bit transfers
//     dma_cfg.dest_data_size    = 1;
//     dma_cfg.block_count       = 1;
//     dma_cfg.head_block        = &block_cfg;
//     dma_cfg.dma_callback      = dma_callback;
//     dma_cfg.user_data         = NULL;
//     dma_cfg.channel_priority  = 2;

//     if (dma_config(dma_dev, DMA_CHANNEL, &dma_cfg) != 0) {
//         printk("DMA configuration failed!\n");
//         return;
//     }

//     if (dma_start(dma_dev, DMA_CHANNEL) != 0) {
//         printk("DMA start failed!\n");
//         return;
//     }

//     /* Wait for DMA completion */
//     k_sem_take(&dma_sem, K_FOREVER);

//     printk("DMA transfer completed!\nReceived data: ");
//     for (int i = 0; i < BUFFER_SIZE; i++) {
//         printk("%c", rx_buffer[i]);
//         if (rx_buffer[i] != tx_str[i]) {
//             printk(" [MISMATCH at %d]", i);
//         }
//     }
//     printk("\n=== TEST DONE ===\n");
// }