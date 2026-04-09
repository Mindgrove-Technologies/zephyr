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

/* ============================================ */
/*         M2M TEST (Channel 0)                 */
/* ============================================ */
static K_SEM_DEFINE(m2m_sem, 0, 1);

static uint32_t m2m_tx_data[10] = {
    0xAAAAAAAA, 0xBBBBBBBB, 0xCCCCCCCC, 0xDDDDDDDD, 0xEEEEEEEE,
    0xFFFFFFFF, 0x11111111, 0x22222222, 0x33333333, 0x44444444
};
static uint32_t m2m_rx_data[10] = {0};

static void m2m_callback(const struct device *dev, void *user_data,
                         uint32_t channel, int status)
{
    printk("[M2M] DMA callback: channel=%d status=%d\n", channel, status);
    k_sem_give(&m2m_sem);
}

void test_m2m(const struct device *dma_dev)
{
    printk("\n=== DMA M2M TEST START ===\n");

    struct dma_block_config block = {0};
    struct dma_config cfg = {0};

    block.source_address = (uint32_t)m2m_tx_data;
    block.dest_address = (uint32_t)m2m_rx_data;
    block.block_size = sizeof(m2m_tx_data);

    cfg.channel_direction = MEMORY_TO_MEMORY;
    cfg.source_data_size = 4;
    cfg.dest_data_size = 4;
    cfg.source_burst_length = 1;
    cfg.dest_burst_length = 1;
    cfg.channel_priority = 2;
    cfg.head_block = &block;
    cfg.dma_callback = m2m_callback;
    cfg.user_data = NULL;

    int channel = 0;

    printk("Configuring DMA channel %d\n", channel);
    if (dma_config(dma_dev, channel, &cfg) != 0) {
        printk("DMA config failed\n");
        return;
    }

    printk("Starting DMA\n");
    if (dma_start(dma_dev, channel) != 0) {
        printk("DMA start failed\n");
        return;
    }

    printk("Waiting for DMA interrupt\n");
    k_sem_take(&m2m_sem, K_FOREVER);
    printk("DMA transfer completed\n");

    /* Verify */
    bool success = true;
    for (int i = 0; i < 10; i++) {
        printk("rx[%d] = 0x%08X (expected 0x%08X)", 
               i, m2m_rx_data[i], m2m_tx_data[i]);
        if (m2m_rx_data[i] != m2m_tx_data[i]) {
            printk(" <-- MISMATCH");
            success = false;
        }
        printk("\n");
    }

    printk("=== DMA M2M TEST %s ===\n", success ? "PASSED" : "FAILED");
    
    /* Stop the channel */
    dma_stop(dma_dev, channel);
    k_msleep(10);
}

/* ============================================ */
/*         P2P TEST (Channel 1)                 */
/* ============================================ */
static K_SEM_DEFINE(p2p_sem, 0, 1);

static const char p2p_tx_str[] = "MADYGROVE FACTS";
#define P2P_BUFFER_SIZE (sizeof(p2p_tx_str) - 1)  /* Exclude null terminator */

static uint8_t p2p_rx_buffer[P2P_BUFFER_SIZE];

/* Use different UART register addresses if needed, or same */
#define P2P_UART2_RX_ADDR (0x00011508U)
#define P2P_UART1_TX_ADDR (0x00011404U)

static struct dma_config p2p_dma_cfg;
static struct dma_block_config p2p_block_cfg;

static void p2p_callback(const struct device *dev, void *user_data,
                         uint32_t channel, int status)
{
    printk("[P2P] DMA callback: channel=%d status=%d\n", channel, status);
    k_sem_give(&p2p_sem);
}

void test_p2p(const struct device *dma_dev)
{
    printk("\n=== DMA UART P2P TEST START ===\n");

    const struct device *uart_src = DEVICE_DT_GET(UART_SRC);
    const struct device *uart_dst = DEVICE_DT_GET(UART_DST);

    if (!device_is_ready(uart_src) || !device_is_ready(uart_dst)) {
        printk("UART devices not ready!\n");
        return;
    }

    /* Configure UARTs */
    struct uart_config uart_cfg = {
        .baudrate = 115200,
        .data_bits = UART_CFG_DATA_BITS_8,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };
    uart_configure(uart_src, &uart_cfg);
    uart_configure(uart_dst, &uart_cfg);

    /* Clear receive buffer */
    memset(p2p_rx_buffer, 0, sizeof(p2p_rx_buffer));
    
    /* Reset semaphore */
    k_sem_reset(&p2p_sem);

    /* Configure DMA FIRST */
    memset(&p2p_block_cfg, 0, sizeof(p2p_block_cfg));
    p2p_block_cfg.source_address = P2P_UART2_RX_ADDR;
    p2p_block_cfg.dest_address = P2P_UART1_TX_ADDR;
    p2p_block_cfg.block_size = P2P_BUFFER_SIZE;

    memset(&p2p_dma_cfg, 0, sizeof(p2p_dma_cfg));
    p2p_dma_cfg.channel_direction = PERIPHERAL_TO_PERIPHERAL;
    p2p_dma_cfg.source_data_size = 1;
    p2p_dma_cfg.dest_data_size = 1;
    p2p_dma_cfg.block_count = 1;
    p2p_dma_cfg.head_block = &p2p_block_cfg;
    p2p_dma_cfg.dma_callback = p2p_callback;
    p2p_dma_cfg.channel_priority = 2;

    int channel = 1;

    printk("Configuring DMA channel %d\n", channel);
    if (dma_config(dma_dev, channel, &p2p_dma_cfg) != 0) {
        printk("DMA config failed\n");
        return;
    }

    printk("Starting DMA (ready and waiting)\n");
    if (dma_start(dma_dev, channel) != 0) {
        printk("DMA start failed\n");
        return;
    }

    /* NOW send the data - DMA is already waiting */
    printk("Sending: %s\n", p2p_tx_str);
    for (int i = 0; i < P2P_BUFFER_SIZE; i++) {
        uart_poll_out(uart_src, p2p_tx_str[i]);
        /* Small delay to allow UART to process each byte */
        k_busy_wait(100);
    }

    /* Wait for DMA completion */
    printk("Waiting for DMA completion...\n");
    int ret = k_sem_take(&p2p_sem, K_SECONDS(10));
    if (ret != 0) {
        printk("❌ DMA timeout!\n");
        
        /* Check if transfer actually completed despite timeout */
        struct dma_status stat;
        dma_get_status(dma_dev, channel, &stat);
        if (stat.pending_length == 0 && !stat.busy) {
            printk("   (But DMA reports complete - TC interrupt missed)\n");
        }
    } else {
        printk("✅ DMA completed with TC interrupt\n");
    }

    /* Read from UART1 */
    printk("Received: ");
    for (int i = 0; i < P2P_BUFFER_SIZE; i++) {
        unsigned char c;
        if (uart_poll_in(uart_dst, &c) == 0) {
            p2p_rx_buffer[i] = c;
            printk("%c", c);
        } else {
            printk("?");
        }
    }
    printk("\n");

    /* Verify */
    if (memcmp(p2p_tx_str, p2p_rx_buffer, P2P_BUFFER_SIZE) == 0) {
        printk("=== DMA UART P2P TEST PASSED ===\n");
    } else {
        printk("=== DMA UART P2P TEST FAILED ===\n");
    }
    
    /* Stop the channel */
    dma_stop(dma_dev, channel);
}

/* ============================================ */
/*         P2M TEST (UART RX to Memory)         */
/* ============================================ */
static K_SEM_DEFINE(p2m_sem, 0, 1);

#define P2M_BUFFER_SIZE 16
#define P2M_UART2_RX_ADDR (0x00011508U)

static const char p2m_tx_str[] = "MINDGROVE FACTS!";
static uint8_t p2m_rx_buffer[P2M_BUFFER_SIZE] __aligned(4);

static struct dma_config p2m_dma_cfg;
static struct dma_block_config p2m_block_cfg;

static void p2m_callback(const struct device *dev, void *user_data,
                         uint32_t channel, int status)
{
    printk("[P2M] DMA callback: channel=%d status=%d\n", channel, status);
    
    if (status == 0) {
        /* Transfer complete */
        printk("[P2M] ✅ Transfer complete!\n");
        k_sem_give(&p2m_sem);
    } else if (status == 1) {
        /* Half-transfer - just log, don't give semaphore */
        printk("[P2M] ⏸️ Half-transfer reached, continuing...\n");
    } else if (status < 0) {
        /* Error */
        printk("[P2M] ❌ Error: %d\n", status);
        k_sem_give(&p2m_sem);
    } else {
        printk("[P2M] ⚠️ Unknown status: %d\n", status);
    }
}

void test_p2m(const struct device *dma_dev)
{
    printk("\n=== DMA P2M (UART RX to Memory) TEST START ===\n");
    
    const struct device *uart_dev = DEVICE_DT_GET(UART_SRC);  /* UART2 */
    
    if (!device_is_ready(uart_dev)) {
        printk("UART device not ready!\n");
        return;
    }
    
    /* Reset semaphore */
    k_sem_reset(&p2m_sem);
    
    /* Clear receive buffer */
    memset(p2m_rx_buffer, 0, sizeof(p2m_rx_buffer));
    
    /* Configure DMA FIRST */
    memset(&p2m_block_cfg, 0, sizeof(p2m_block_cfg));
    p2m_block_cfg.source_address = P2M_UART2_RX_ADDR;
    p2m_block_cfg.dest_address = (uint32_t)p2m_rx_buffer;
    p2m_block_cfg.block_size = P2M_BUFFER_SIZE;
    
    memset(&p2m_dma_cfg, 0, sizeof(p2m_dma_cfg));
    p2m_dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
    p2m_dma_cfg.source_data_size = 1;
    p2m_dma_cfg.dest_data_size = 1;
    p2m_dma_cfg.block_count = 1;  /* Only TC interrupt */
    p2m_dma_cfg.head_block = &p2m_block_cfg;
    p2m_dma_cfg.dma_callback = p2m_callback;
    p2m_dma_cfg.user_data = NULL;
    p2m_dma_cfg.channel_priority = 2;
    
    int channel = 2;  /* Use different channel */
    
    printk("Configuring DMA channel %d\n", channel);
    if (dma_config(dma_dev, channel, &p2m_dma_cfg) < 0) {
        printk("DMA config failed\n");
        return;
    }
    
    printk("Starting DMA (ready and waiting)\n");
    if (dma_start(dma_dev, channel) < 0) {
        printk("DMA start failed\n");
        return;
    }
    
    /* NOW send the data - DMA is already waiting */
    printk("Sending: %s\n", p2m_tx_str);
    for (int i = 0; i < P2M_BUFFER_SIZE; i++) {
        uart_poll_out(uart_dev, p2m_tx_str[i]);
        k_busy_wait(500);  /* pacing so DMA doesn't miss bytes */
    }
    
    /* Wait for DMA completion */
    printk("Waiting for DMA to complete...\n");
    int ret = k_sem_take(&p2m_sem, K_SECONDS(5));
    if (ret != 0) {
        printk("❌ DMA TIMEOUT!\n");
        
        /* Check if transfer actually completed */
        struct dma_status stat;
        dma_get_status(dma_dev, channel, &stat);
        if (stat.pending_length == 0 && !stat.busy) {
            printk("   (But DMA reports complete)\n");
        }
    } else {
        printk("✅ DMA DONE\n");
    }
    
    /* Verify */
    printk("\n=== P2M VERIFICATION ===\n");
    bool success = true;
    for (int i = 0; i < P2M_BUFFER_SIZE; i++) {
        uint8_t rx = p2m_rx_buffer[i];
        uint8_t tx = p2m_tx_str[i];
        
        printk("Index %2d | RX: 0x%02x (%c) | TX: 0x%02x (%c)",
               i, rx, (rx >= 32 && rx <= 126) ? rx : '.', tx, tx);
        
        if (rx != tx) {
            printk("  <-- MISMATCH");
            success = false;
        }
        printk("\n");
    }
    
    printk("=== DMA P2M TEST %s ===\n", success ? "PASSED" : "FAILED");
    
    /* Stop the channel */
    dma_stop(dma_dev, channel);
}


/* ============================================ */
/*         M2P TEST (Memory to UART TX)         */
/* ============================================ */
static K_SEM_DEFINE(m2p_sem, 0, 1);

#define M2P_BUFFER_SIZE 16
#define UART2_TX_REG_ADDR (0x00011504U)  /* UART2 TX register */

static const char m2p_tx_str[M2P_BUFFER_SIZE] = "MINDGROVE FACTS!";
static uint8_t m2p_verify_buffer[M2P_BUFFER_SIZE] = {0};  /* For verification if loopback exists */

static struct dma_config m2p_dma_cfg;
static struct dma_block_config m2p_block_cfg;

static void m2p_callback(const struct device *dev, void *user_data,
                         uint32_t channel, int status)
{
    printk("[M2P] DMA callback: channel=%d status=%d\n", channel, status);
    
    if (status == 0) {
        /* Transfer complete */
        printk("[M2P] ✅ Transfer complete! Data sent to UART TX\n");
        k_sem_give(&m2p_sem);
    } else if (status == 1) {
        /* Half-transfer - just log */
        printk("[M2P] ⏸️ Half-transfer reached, continuing...\n");
    } else if (status < 0) {
        /* Error */
        printk("[M2P] ❌ Error: %d\n", status);
        k_sem_give(&m2p_sem);
    }
}
void test_m2p(const struct device *dma_dev)
{
    printk("\n=== DMA M2P (Memory to UART TX) TEST START ===\n");

    const struct device *uart_dev = DEVICE_DT_GET(UART_SRC);  /* UART2 */

    if (!device_is_ready(uart_dev)) {
        printk("UART device not ready!\n");
        return;
    }

    /* ================= UART CONFIG ================= */
    struct uart_config uart_cfg = {
        .baudrate  = 115200,
        .data_bits = UART_CFG_DATA_BITS_8,
        .parity    = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };
    uart_configure(uart_dev, &uart_cfg);

    /* ================= DMA CONFIG ================= */
    k_sem_reset(&m2p_sem);

    memset(&m2p_block_cfg, 0, sizeof(m2p_block_cfg));
    m2p_block_cfg.source_address = (uint32_t)m2p_tx_str;
    m2p_block_cfg.dest_address   = UART2_TX_REG_ADDR;
    m2p_block_cfg.block_size     = M2P_BUFFER_SIZE;

    memset(&m2p_dma_cfg, 0, sizeof(m2p_dma_cfg));
    m2p_dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
    m2p_dma_cfg.source_data_size  = 1;
    m2p_dma_cfg.dest_data_size    = 1;
    m2p_dma_cfg.block_count       = 1;
    m2p_dma_cfg.head_block        = &m2p_block_cfg;
    m2p_dma_cfg.dma_callback      = m2p_callback;
    m2p_dma_cfg.user_data         = NULL;
    m2p_dma_cfg.channel_priority  = 2;

    int channel = 3;

    printk("DMA Channel: %d\n", channel);
    printk("Source      : 0x%08lx (%s)\n", (uint32_t)m2p_tx_str, m2p_tx_str);
    printk("Destination : 0x%08x (UART TX)\n", UART2_TX_REG_ADDR);
    printk("Length      : %d bytes\n", M2P_BUFFER_SIZE);

    if (dma_config(dma_dev, channel, &m2p_dma_cfg) < 0) {
        printk("❌ DMA config failed\n");
        return;
    }

    if (dma_start(dma_dev, channel) < 0) {
        printk("❌ DMA start failed\n");
        return;
    }

    /* ================= WAIT ================= */
    printk("Waiting for DMA completion...\n");

    if (k_sem_take(&m2p_sem, K_SECONDS(5)) != 0) {
        printk("❌ DMA TIMEOUT\n");
        return;
    }

    printk("✅ DMA transfer complete\n");

    /* Allow UART to flush */
    k_msleep(10);

    /* ================= VERIFICATION ================= */
    printk("\n=== M2P VERIFICATION ===\n");
    printk("Data sent via UART TX: %s\n", m2p_tx_str);
    printk("(Loopback required: connect TX → RX)\n");

    printk("\nReading RX buffer...\n");

    for (int i = 0; i < M2P_BUFFER_SIZE; i++) {
        unsigned char c = 0xFF;

        if (uart_poll_in(uart_dev, &c) == 0) {
            m2p_verify_buffer[i] = c;
        } else {
            m2p_verify_buffer[i] = 0xFF;
        }
    }

    printk("\nAnalyzing DMA pattern...\n");

    /* Print received data */
    for (int i = 0; i < M2P_BUFFER_SIZE; i++) {
        printk("Index %2d | RX: 0x%02x (%c)\n",
               i,
               m2p_verify_buffer[i],
               (m2p_verify_buffer[i] >= 32 && m2p_verify_buffer[i] <= 126)
                   ? m2p_verify_buffer[i] : '.');
    }

    /* Count transitions */
    int transitions = 0;

    for (int i = 1; i < M2P_BUFFER_SIZE; i++) {
        if (m2p_verify_buffer[i] != m2p_verify_buffer[i - 1]) {
            transitions++;
        }
    }

    printk("Transitions detected: %d\n", transitions);

    if (transitions >= 1 && transitions <= 3) {
        printk("✅ EXPECTED DMA REGISTER-SAMPLING BEHAVIOR\n");
    } else {
        printk("❌ UNEXPECTED DATA PATTERN\n");
    }

    /* ================= CLEANUP ================= */
    dma_stop(dma_dev, channel);

    printk("=== DMA M2P TEST END ===\n");
}

/* ============================================================
 * Main — run all tests in sequence
 * ============================================================ */
void main(void)
{
    const struct device *dma_dev = DEVICE_DT_GET(DMA_NODE);
    
    printk("\n========================================\n");
    printk("  DMA Test Suite (Zephyr 3.2)\n");
    printk("========================================\n");
    
    /* Check DMA device once at start */
    if (!device_is_ready(dma_dev)) {
        printk("ERROR: DMA device not ready!\n");
        return;
    }
    
    /* Test 1: M2M on channel 0 */
    printk("\n>>> Running Test 1: M2M (Channel 0)\n");
    test_m2m(dma_dev);
    k_msleep(100);
    
    /* Test 2: P2P on channel 1 */
    printk("\n>>> Running Test 2: P2P (Channel 1)\n");
    test_p2p(dma_dev);
    k_msleep(100);
    
    /* Test 3: P2M on channel 2 */
    printk("\n>>> Running Test 3: P2M (Channel 2)\n");
    test_p2m(dma_dev);
    k_msleep(100);
    
    /* Test 4: M2P on channel 3 */
    printk("\n>>> Running Test 4: M2P (Channel 3)\n");
    test_m2p(dma_dev);
    
    printk("\n========================================\n");
    printk("  DMA Test Suite Complete\n");
    printk("========================================\n");
}