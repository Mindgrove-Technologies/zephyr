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
//     printk("DMA M2M test start\n");

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

//     printk("DMA M2M test end\n");

//     return 0;
// }

/* WORKING m2m DMA HALF TRANSFER*/

// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/drivers/dma.h>
// #include <zephyr/sys/printk.h>

// #define DMA_NODE DT_NODELABEL(dma0)
// #define BUFFER_SIZE 10

// static uint32_t tx_data[BUFFER_SIZE] = {
//     0xAAAAAAAA, 0xBBBBBBBB, 0xCCCCCCCC, 0xDDDDDDDD,
//     0xEEEEEEEE, 0xFFFFFFFF, 0x11111111, 0x22222222,
//     0x33333333, 0x44444444
// };

// static uint32_t rx_data[BUFFER_SIZE] = {0};

// static K_SEM_DEFINE(ht_sem, 0, 1);

// static void dma_callback(const struct device *dev, void *user_data,
//                          uint32_t channel, int status)
// {
//     ARG_UNUSED(dev);
//     ARG_UNUSED(user_data);
//     printk("[CALLBACK] REACHED ISR");
//     if (status == 1) {  // Half transfer
//         printk("[CALLBACK] HALF TRANSFER (ch=%d)\n", channel);
//         k_sem_give(&ht_sem);
//     }
// }

// void main(void)
// {
//     const struct device *dma_dev = DEVICE_DT_GET(DMA_NODE);
//     if (!device_is_ready(dma_dev)) {
//         printk("DMA device not ready!\n");
//         return;
//     }

//     printk("=== DMA HALF TRANSFER TEST START ===\n");

//     struct dma_block_config block = {0};
//     struct dma_config cfg = {0};

//     block.source_address = (uint32_t)tx_data;
//     block.dest_address   = (uint32_t)rx_data;
//     block.block_size     = sizeof(tx_data);

//     cfg.channel_direction = MEMORY_TO_MEMORY;
//     cfg.source_data_size  = 4;
//     cfg.dest_data_size    = 4;
//     cfg.block_count       = 1;
//     cfg.head_block        = &block;
//     cfg.dma_callback      = dma_callback;
//     cfg.user_data         = NULL;

//     int channel = 0;

//     printk("Configuring DMA...\n");
//     if (dma_config(dma_dev, channel, &cfg) != 0) {
//         printk("DMA config failed!\n");
//         return;
//     }

//     printk("Starting DMA...\n");
//     if (dma_start(dma_dev, channel) != 0) {
//         printk("DMA start failed!\n");
//         return;
//     }

//     /* Wait for half-transfer interrupt */
//     k_sem_take(&ht_sem, K_FOREVER);
//     printk("Half transfer reached!\n");

//     /* Optionally verify first half of the buffer */
//     for (int i = 0; i < BUFFER_SIZE ; i++) {
//         printk("rx[%d] = 0x%x\n", i, rx_data[i]);
//     }

//     printk("=== DMA HALF TRANSFER TEST DONE ===\n");
// }

/*WORKING P2M*/

    // #define UART_STATUS_REG 0x0001150C
    // #define STS_TX_EMPTY    (1 << 0)
    // #define STS_RX_NOT_EMPTY    (1 << 2)

    // #include <zephyr/kernel.h>
    // #include <zephyr/device.h>
    // #include <zephyr/drivers/dma.h>
    // #include <zephyr/drivers/uart.h>
    // #include <zephyr/sys/printk.h>
    // #include <string.h>

    // /* Device tree nodes */
    // #define DMA_NODE    DT_NODELABEL(dma0)
    // #define UART_NODE   DT_NODELABEL(uart2)

    // /* Config */
    // #define DMA_CHANNEL 0
    // #define BUFFER_SIZE 16
    // #define UART_RX_REG_ADDR (0x00011508U)

    // /* Test data */
    // static const char tx_str[BUFFER_SIZE] = "MINDGROVE FACTS!";
    // static uint8_t rx_buffer[BUFFER_SIZE] __aligned(4);

    // /* Semaphore */
    // K_SEM_DEFINE(dma_sem, 0, 1);

    // /* DMA config structures (must be static) */
    // static struct dma_config dma_cfg;
    // static struct dma_block_config block_cfg;


    // /* ---------------- DMA CALLBACK ---------------- */
    // void dma_callback(const struct device *dev, void *user_data,
    //                   uint32_t channel, int status)
    // {
    //     printk("[DMA CALLBACK] status = 0x%x\n", status);
        
    //     if (status == 0) {
    //         /* ✅ Transfer COMPLETE - give semaphore */
    //         printk("[DMA] ✅ Transfer complete!\n");
    //         k_sem_give(&dma_sem);
    //     } 
    //     else if (status == 1) {
    //         /* ⏸️ Half-transfer - DO NOT give semaphore, wait for TC */
    //         printk("[DMA] ⏸️ Half-transfer (8/%d bytes) - continuing...\n", 
    //                BUFFER_SIZE);
    //         /* DO NOTHING - wait for TC interrupt */
    //     }
    //     else if (status < 0) {
    //         /* ❌ Error - give semaphore to unblock */
    //         printk("[DMA] ❌ Error: %d\n", status);
    //         k_sem_give(&dma_sem);
    //     }
    //     else {
    //         /* Unknown status */
    //         printk("[DMA] ⚠️ Unknown status: 0x%x\n", status);
    //     }
    // }


    // /* ---------------- WAIT ---------------- */
    // void dma_wait(void)
    // {
    //     int ret = k_sem_take(&dma_sem, K_MSEC(1000)); //Initially 5 seconds, reduced to 1 second for better feedback loop
    //     if (ret != 0) {
    //         printk("❌ DMA TIMEOUT! Transfer did not complete in 1 second\n");
            
    //     } else {
    //         printk("✅ DMA complete via interrupt\n");
    //     }
    // }

    // /* ---------------- VERIFY ---------------- */
    // void dma_verify(void)
    // {
    //     printk("\n\n=== DMA VERIFY ===\n");

    //     for (int i = 0; i < BUFFER_SIZE; i++) {

    //         uint8_t rx = rx_buffer[i];
    //         uint8_t tx = tx_str[i];

    //         printk("Index %2d | RX: 0x%02x (%c) | TX: 0x%02x (%c)",
    //                i,
    //                rx, (rx >= 32 && rx <= 126) ? rx : '.',
    //                tx, tx);

    //         if (rx != tx) {
    //             printk("  <-- MISMATCH");
    //         }

    //         printk("\n");
    //     }

    //     printk("====================\n");
    // }

    // void dma_uart_test(const struct device *uart_dev,
    //                    const struct device *dma_dev)
    // {
    //     printk("=== DMA UART P2M TEST START ===\n");

    //     k_sem_reset(&dma_sem);
    //      /* Clear buffer */
    //     memset(rx_buffer, 0, BUFFER_SIZE);

    //     /* Configure block */
    //     memset(&block_cfg, 0, sizeof(block_cfg));
    //     block_cfg.source_address = UART_RX_REG_ADDR;
    //     block_cfg.dest_address   = (uint32_t)rx_buffer;
    //     block_cfg.block_size     = BUFFER_SIZE;

    //     /* Configure DMA */
    //     memset(&dma_cfg, 0, sizeof(dma_cfg));
    //     dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
    //     dma_cfg.source_data_size  = 1;
    //     dma_cfg.dest_data_size    = 1;
    //     dma_cfg.block_count       = 8;  // Trigger half-transfer at 8 bytes
    //     dma_cfg.head_block        = &block_cfg;
    //     dma_cfg.dma_callback      = dma_callback;
    //     dma_cfg.user_data         = NULL;
    //     dma_cfg.channel_priority  = 2;

    //     if (dma_config(dma_dev, DMA_CHANNEL, &dma_cfg) < 0) {
    //     printk("DMA config failed\n");
    //     return;
    //     }

    //     if (dma_start(dma_dev, DMA_CHANNEL) < 0) {
    //         printk("DMA start failed\n");
    //         return;
    //     }


    //     for (int i = 0; i < BUFFER_SIZE; i++) {
    //         uart_poll_out(uart_dev, tx_str[i]);
            
    //         /* Wait for TX to physically complete */
    //         k_busy_wait(500);  // Increase delay to 500µs
    //     }
        

    //     /* 3. Wait for DMA completion with reasonable timeout */
    //     printk("[TEST] Waiting for DMA to complete...\n");
    //     dma_wait();  // With 5-second timeout - this is fine!

    //     /* 4. Verify */
    //     dma_verify();

    //     printk("=== DMA UART P2M TEST END ===\n");
    // }

    // /* ---------------- MAIN ---------------- */
    // void main(void)
    // {
    //     const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);
    //     const struct device *dma_dev  = DEVICE_DT_GET(DMA_NODE);

    //     if (!device_is_ready(uart_dev) || !device_is_ready(dma_dev)) {
    //         printk("Devices not ready!\n");
    //         return;
    //     }

    //     printk("System Start\n\n");

    //     dma_uart_test(uart_dev, dma_dev);
    // }


    /* WORKING M2P*/
    // #include <zephyr/kernel.h>
    // #include <zephyr/device.h>
    // #include <zephyr/drivers/dma.h>
    // #include <zephyr/drivers/uart.h>
    // #include <zephyr/sys/printk.h>
    // #include <string.h>

    // /* Device tree nodes */
    // #define DMA_NODE    DT_NODELABEL(dma0)
    // #define UART_NODE   DT_NODELABEL(uart2)

    // /* Config */
    // #define DMA_CHANNEL 0
    // #define BUFFER_SIZE 16
    // #define UART_RX_REG_ADDR (0x00011508U)

    // /* Test data */
    // static const char tx_str[BUFFER_SIZE] = "MINDGROVE FACTS!";
    // static uint8_t rx_buffer[BUFFER_SIZE] __aligned(4);

    // /* Sync */
    // K_SEM_DEFINE(dma_sem, 0, 1);

    // /* DMA config */
    // static struct dma_config dma_cfg;
    // static struct dma_block_config block_cfg;


    // /* ---------------- DMA CALLBACK ---------------- */
    // static void dma_callback(const struct device *dev, void *user_data,
    //                          uint32_t channel, int status)
    // {
    //     if (status == 0) {
    //         printk("[DMA] ✅ Transfer complete\n");
    //         k_sem_give(&dma_sem);
    //     } else if (status < 0) {
    //         printk("[DMA] ❌ Error: %d\n", status);
    //         k_sem_give(&dma_sem);
    //     }
    // }


    // /* ---------------- MAIN ---------------- */
    // void main(void)
    // {
    //     const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);
    //     const struct device *dma_dev  = DEVICE_DT_GET(DMA_NODE);

    //     if (!device_is_ready(uart_dev) || !device_is_ready(dma_dev)) {
    //         printk("Devices not ready!\n");
    //         return;
    //     }

    //     printk("=== DMA UART M2P SAMPLE START ===\n");

    //     /* Reset */
    //     k_sem_reset(&dma_sem);
    //     memset(rx_buffer, 0, BUFFER_SIZE);

    //     /* ---------------- DMA CONFIG ---------------- */
    //     memset(&block_cfg, 0, sizeof(block_cfg));
    //     block_cfg.source_address = UART_RX_REG_ADDR;
    //     block_cfg.dest_address   = (uint32_t)rx_buffer;
    //     block_cfg.block_size     = BUFFER_SIZE;

    //     memset(&dma_cfg, 0, sizeof(dma_cfg));
    //     dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
    //     dma_cfg.source_data_size  = 1;
    //     dma_cfg.dest_data_size    = 1;
    //     dma_cfg.block_count       = 1;   /* Only TC interrupt */
    //     dma_cfg.head_block        = &block_cfg;
    //     dma_cfg.dma_callback      = dma_callback;
    //     dma_cfg.channel_priority  = 2;

    //     if (dma_config(dma_dev, DMA_CHANNEL, &dma_cfg) < 0) {
    //         printk("DMA config failed\n");
    //         return;
    //     }

    //     if (dma_start(dma_dev, DMA_CHANNEL) < 0) {
    //         printk("DMA start failed\n");
    //         return;
    //     }

    //     /* ---------------- UART TX ---------------- */
    //     for (int i = 0; i < BUFFER_SIZE; i++) {
    //         uart_poll_out(uart_dev, tx_str[i]);
    //         k_busy_wait(500);  /* pacing so DMA doesn't miss bytes */
    //     }

    //     /* ---------------- WAIT ---------------- */
    //     printk("[APP] Waiting for DMA...\n");

    //     if (k_sem_take(&dma_sem, K_MSEC(1000)) != 0) {
    //         printk("❌ DMA TIMEOUT\n");
    //     } else {
    //         printk("✅ DMA DONE\n");
    //     }

    //     /* ---------------- VERIFY ---------------- */
    //     printk("\n=== VERIFY ===\n");

    //     for (int i = 0; i < BUFFER_SIZE; i++) {

    //         printk("Index %2d | RX: 0x%02x (%c) | TX: 0x%02x (%c)",
    //                i,
    //                rx_buffer[i],
    //                (rx_buffer[i] >= 32 && rx_buffer[i] <= 126) ? rx_buffer[i] : '.',
    //                tx_str[i],
    //                tx_str[i]);

    //         if (rx_buffer[i] != tx_str[i]) {
    //             printk("  <-- MISMATCH");
    //         }

    //         printk("\n");
    //     }

    //     printk("====================\n");
    //     printk("=== DMA UART M2P SAMPLE END ===\n");
    // }


/*P2P*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <string.h>

/* Device tree nodes */
#define DMA_NODE    DT_NODELABEL(dma0)
#define UART_SRC    DT_NODELABEL(uart2)
#define UART_DST    DT_NODELABEL(uart1)

/* Config */
#define DMA_CHANNEL 0
#define BUFFER_SIZE 15
#define UART_BAUD 115200

#define UART2_RX_ADDR (0x00011508U)
#define UART1_TX_ADDR (0x00011404U)

/* Test data */
static const char tx_str[BUFFER_SIZE] = "MINDGROVE FACTS";
static uint8_t rx_buffer[BUFFER_SIZE];

/* Semaphore */
K_SEM_DEFINE(dma_sem, 0, 1);

/* DMA config */
static struct dma_config dma_cfg;
static struct dma_block_config block_cfg;

/* DMA callback */
static void dma_callback(const struct device *dev, void *user_data,
                         uint32_t channel, int status)
{
    if (status == 0) {
        printk("DMA complete\n");
        k_sem_give(&dma_sem);
    }
}

void main(void)
{
    const struct device *uart_src = DEVICE_DT_GET(UART_SRC);
    const struct device *uart_dst = DEVICE_DT_GET(UART_DST);
    const struct device *dma_dev = DEVICE_DT_GET(DMA_NODE);

    /* Check devices */
    if (!device_is_ready(uart_src) || !device_is_ready(uart_dst) || !device_is_ready(dma_dev)) {
        printk("Devices not ready!\n");
        return;
    }

    /* Configure UARTs for DMA */
    struct uart_config uart_cfg = {
        .baudrate = UART_BAUD,
        .data_bits = UART_CFG_DATA_BITS_8,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };
    uart_configure(uart_src, &uart_cfg);
    uart_configure(uart_dst, &uart_cfg);

    /* Send data to UART2 */
    printk("Sending: %s\n", tx_str);
    for (int i = 0; i < BUFFER_SIZE; i++) {
        uart_poll_out(uart_src, tx_str[i]);
        k_busy_wait(100);
    }
    k_msleep(10);

    /* Configure DMA (UART2 RX → UART1 TX) */
    block_cfg.source_address = UART2_RX_ADDR;
    block_cfg.dest_address = UART1_TX_ADDR;
    block_cfg.block_size = BUFFER_SIZE;

    dma_cfg.channel_direction = PERIPHERAL_TO_PERIPHERAL;
    dma_cfg.source_data_size = 1;
    dma_cfg.dest_data_size = 1;
    dma_cfg.block_count = 1;
    dma_cfg.head_block = &block_cfg;
    dma_cfg.dma_callback = dma_callback;
    dma_cfg.channel_priority = 2;

    /* Start DMA */
    dma_config(dma_dev, DMA_CHANNEL, &dma_cfg);
    dma_start(dma_dev, DMA_CHANNEL);

    /* Wait for completion */
    k_sem_take(&dma_sem, K_SECONDS(10));

    /* Read from UART1 */
    printk("Received: ");
    for (int i = 0; i < BUFFER_SIZE; i++) {
        unsigned char c;
        if (uart_poll_in(uart_dst, &c) == 0) {
            rx_buffer[i] = c;
            printk("%c", c);
        }
    }
    printk("\n");

    /* Verify */
    if (memcmp(tx_str, rx_buffer, BUFFER_SIZE) == 0) {
        printk("✅ TEST PASSED\n");
    } else {
        printk("❌ TEST FAILED\n");
    }
}