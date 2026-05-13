/*
 * Copyright (c) Mindgrove Technologies Pvt. Ltd 2025.
 * SPDX-License-Identifier: Apache-2.0
 *
 * uart_irq_test.c — optimized for memory and speed
 *
 *   Phase 1 — Static API checks        (no loopback needed)
 *   Phase 2 — TX path                  (loopback TX->RX wire required)
 *   Phase 3 — RX loopback path         (loopback TX->RX wire required)
 *   Phase 4 — Callback identity        (loopback TX->RX wire required)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Configuration                                                        */
/* ------------------------------------------------------------------ */

#define UART_NODE       DT_NODELABEL(uart1)
#define TEST_LEN        8
#define RX_TIMEOUT_MS   2000

static const uint8_t TX_PATTERN[TEST_LEN] = {
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF
};

/* ------------------------------------------------------------------ */
/* Shared state — single set reused across all phases                   */
/* ------------------------------------------------------------------ */

static const struct device *uart_dev;

/*
 * One buffer + counter + semaphore used by every RX ISR.
 * Phases run sequentially so there is no overlap.
 */
static uint8_t          rx_buf[TEST_LEN];
static volatile int     rx_count;

/* Semaphore replaces all k_sleep polling loops.
 * ISR gives it when rx_count reaches the target;
 * test thread takes it with a timeout.             */
static K_SEM_DEFINE(rx_sem, 0, 1);

/* Phase 4 counters — small, kept separate for clarity */
static volatile uint32_t cb_a_hits;
static volatile uint32_t cb_b_hits;
static K_SEM_DEFINE(cb_sem, 0, 1);

/* ------------------------------------------------------------------ */
/* Test result helpers                                                  */
/* ------------------------------------------------------------------ */

static uint8_t total_pass;
static uint8_t total_fail;

static void report(const char *name, bool pass)
{
    if (pass) {
        printk("  [PASS] %s\n", name);
        total_pass++;
    } else {
        printk("  [FAIL] %s\n", name);
        total_fail++;
    }
}

/* ------------------------------------------------------------------ */
/* Generic RX ISR — used by Phase 2, 3, and 4                          */
/*                                                                      */
/* user_data = pointer to the expected byte count (uint8_t *).         */
/* When rx_count reaches *target, disables RX and gives rx_sem.        */
/* ------------------------------------------------------------------ */

static void generic_rx_isr(const struct device *dev, void *user_data)
{
    if (!uart_irq_update(dev))
        return;

    if (!uart_irq_rx_ready(dev))
        return;

    const uint8_t target = *(const uint8_t *)user_data;

    while (rx_count < target) {
        uint8_t byte;
        if (uart_fifo_read(dev, &byte, 1) <= 0)
            break;
        rx_buf[rx_count++] = byte;
    }

    if (rx_count >= target) {
        uart_irq_rx_disable(dev);
        k_sem_give(&rx_sem);
    }
}

/* Phase 4 callback A */
static void cb_tag_a(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);
    uint8_t dummy;
    uart_fifo_read(dev, &dummy, 1);
    cb_a_hits++;
    k_sem_give(&cb_sem);
}

/* Phase 4 callback B */
static void cb_tag_b(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);
    uint8_t dummy;
    uart_fifo_read(dev, &dummy, 1);
    cb_b_hits++;
    k_sem_give(&cb_sem);
}

/* ------------------------------------------------------------------ */
/* Helper: reset RX state and arm the generic ISR                       */
/* ------------------------------------------------------------------ */

static void arm_rx_isr(uint8_t *target_len)
{
    rx_count = 0;
    memset(rx_buf, 0, TEST_LEN);
    k_sem_reset(&rx_sem);
    uart_irq_callback_user_data_set(uart_dev, generic_rx_isr, target_len);
    uart_irq_rx_enable(uart_dev);
}

/* ------------------------------------------------------------------ */
/* Phase 1 — Static API checks                                          */
/* ------------------------------------------------------------------ */

static void phase1_static_api(void)
{
    printk("\n--- Phase 1: Static API checks ---\n");

    /* T1.4 — err enable/disable must not crash */
    uart_irq_err_enable(uart_dev);
    uart_irq_err_disable(uart_dev);
    report("T1.4  irq_err_enable/disable no-op (no crash)", true);

    /* T1.3 — irq_update must return 1 */
    report("T1.3  irq_update returns 1",
           uart_irq_update(uart_dev) == 1);

    /* T1.1 — TX enable/disable toggles irq_tx_ready */
    uart_irq_tx_enable(uart_dev);
    bool tx_en  = uart_irq_tx_ready(uart_dev);
    uart_irq_tx_disable(uart_dev);
    bool tx_dis = !uart_irq_tx_ready(uart_dev);
    report("T1.1a irq_tx_enable  → irq_tx_ready = 1", tx_en);
    report("T1.1b irq_tx_disable → irq_tx_ready = 0", tx_dis);

    /* T1.2 — RX enable/disable; rx_ready = 0 after disable */
    uart_irq_rx_enable(uart_dev);
    uart_irq_rx_disable(uart_dev);
    report("T1.2  irq_rx_disable → irq_rx_ready = 0",
           !uart_irq_rx_ready(uart_dev));
}

/* ------------------------------------------------------------------ */
/* Phase 2 — TX path with loopback RX verification                     */
/* ------------------------------------------------------------------ */

static void phase2_tx_path(void)
{
    printk("\n--- Phase 2: TX path (loopback TX->RX) ---\n");

    /* T2.1 — irq_tx_ready after enable */
    uart_irq_tx_enable(uart_dev);
    report("T2.1  irq_tx_ready = 1 after enable",
           uart_irq_tx_ready(uart_dev));

    /* T2.2 — irq_is_pending while TX enabled */
    report("T2.2  irq_is_pending = 1 with TX enabled",
           uart_irq_is_pending(uart_dev));

    uart_irq_tx_disable(uart_dev);

    /* T2.5 — irq_tx_ready after disable */
    report("T2.5  irq_tx_ready = 0 after disable",
           !uart_irq_tx_ready(uart_dev));

    /* T2.3 — fifo_fill: arm RX ISR first so no loopback bytes are missed */
    static uint8_t target = TEST_LEN;
    arm_rx_isr(&target);

    int accepted = uart_fifo_fill(uart_dev, TX_PATTERN, TEST_LEN);
    report("T2.3  fifo_fill accepts > 0 bytes", accepted > 0);
    printk("         fifo_fill accepted %d / %d bytes\n", accepted, TEST_LEN);

    /* T2.3b — wait for loopback bytes via semaphore (no polling) */
    bool lb_done = k_sem_take(&rx_sem, K_MSEC(RX_TIMEOUT_MS)) == 0;
    report("T2.3b TX bytes received via loopback RX ISR", lb_done);

    /* T2.3c — verify data integrity */
    bool match = lb_done &&
                 (rx_count == TEST_LEN) &&
                 (memcmp(rx_buf, TX_PATTERN, TEST_LEN) == 0);
    report("T2.3c Loopback data matches TX pattern", match);

    if (!match) {
        for (int i = 0; i < TEST_LEN; i++)
            printk("        [%d] TX=0x%02X RX=0x%02X %s\n",
                   i, TX_PATTERN[i], rx_buf[i],
                   TX_PATTERN[i] == rx_buf[i] ? "OK" : "MISMATCH");
    }

    /* T2.4 — irq_tx_complete: semaphore already taken, just poll status */
    report("T2.4  irq_tx_complete = 1 after FIFO drains",
           uart_irq_tx_complete(uart_dev));

    uart_irq_rx_disable(uart_dev);
}

/* ------------------------------------------------------------------ */
/* Phase 3 — RX loopback path                                           */
/* ------------------------------------------------------------------ */

static void phase3_rx_loopback(void)
{
    printk("\n--- Phase 3: RX loopback (TX->RX wire) ---\n");

    static uint8_t target = TEST_LEN;
    arm_rx_isr(&target);

    /* T3.1 — rx_ready = 0 before any data */
    report("T3.1  irq_rx_ready = 0 before TX",
           !uart_irq_rx_ready(uart_dev));

    /* Send pattern via poll_out */
    printk("      Sending %d bytes via poll_out ...\n", TEST_LEN);
    for (int i = 0; i < TEST_LEN; i++)
        uart_poll_out(uart_dev, TX_PATTERN[i]);

    /* T3.2 — wait for ISR to collect all bytes */
    bool done = k_sem_take(&rx_sem, K_MSEC(RX_TIMEOUT_MS)) == 0;
    report("T3.2a RX ISR received all bytes within timeout", done);

    bool match = done &&
                 (rx_count == TEST_LEN) &&
                 (memcmp(rx_buf, TX_PATTERN, TEST_LEN) == 0);
    report("T3.2b fifo_read data matches TX pattern", match);

    if (!match) {
        for (int i = 0; i < TEST_LEN; i++)
            printk("        [%d] TX=0x%02X RX=0x%02X %s\n",
                   i, TX_PATTERN[i], rx_buf[i],
                   TX_PATTERN[i] == rx_buf[i] ? "OK" : "MISMATCH");
    }

    /* T3.3 — irq_is_pending = 0 after drain (ISR already disabled RX) */
    report("T3.3  irq_is_pending = 0 after full RX drain",
           !uart_irq_is_pending(uart_dev));

    /* T3.4 — no further callbacks after irq_rx_disable */
    int snap = rx_count;
    uart_poll_out(uart_dev, 0xBE);
    /* Use semaphore with short timeout — if it fires, ISR ran (bad) */
    bool extra_fired = k_sem_take(&rx_sem, K_MSEC(50)) == 0;
    report("T3.4  No ISR callback after irq_rx_disable", !extra_fired);

    /* Consume the stray byte */
    unsigned char dummy;
    uart_poll_in(uart_dev, &dummy);

    ARG_UNUSED(snap);
}

/* ------------------------------------------------------------------ */
/* Phase 4 — Callback identity                                          */
/* ------------------------------------------------------------------ */

static void phase4_callback_identity(void)
{
    printk("\n--- Phase 4: Callback identity ---\n");

    cb_a_hits = 0;
    cb_b_hits = 0;

    /* T4.1 — callback A fires, B does not */
    k_sem_reset(&cb_sem);
    uart_irq_callback_user_data_set(uart_dev, cb_tag_a, NULL);
    uart_irq_rx_enable(uart_dev);
    uart_poll_out(uart_dev, 0xA1);
    k_sem_take(&cb_sem, K_MSEC(200));   /* wait for ISR, not a fixed sleep */
    uart_irq_rx_disable(uart_dev);

    report("T4.1  Callback A fired after irq_callback_set(A)",
           cb_a_hits > 0);
    report("T4.1b Callback B did NOT fire",
           cb_b_hits == 0);

    /* T4.2 — replace with B; A count must not change */
    uint32_t a_snap = cb_a_hits;
    k_sem_reset(&cb_sem);
    uart_irq_callback_user_data_set(uart_dev, cb_tag_b, NULL);
    uart_irq_rx_enable(uart_dev);
    uart_poll_out(uart_dev, 0xB2);
    k_sem_take(&cb_sem, K_MSEC(200));
    uart_irq_rx_disable(uart_dev);

    report("T4.2  Callback B fired after irq_callback_set(B)",
           cb_b_hits > 0);
    report("T4.2b Callback A count unchanged after replacing with B",
           cb_a_hits == a_snap);
}

/* ------------------------------------------------------------------ */
/* Main                                                                 */
/* ------------------------------------------------------------------ */

int main(void)
{
    printk("\n");
    printk("============================================\n");
    printk("  Mindgrove UART Interrupt Driver Test\n");
    printk("============================================\n");

    uart_dev = DEVICE_DT_GET(UART_NODE);
    if (!device_is_ready(uart_dev)) {
        printk("FATAL: UART device not ready\n");
        return -1;
    }
    printk("UART device: %s — ready\n", uart_dev->name);

    phase1_static_api();
    phase2_tx_path();
    phase3_rx_loopback();
    phase4_callback_identity();

    printk("\n============================================\n");
    printk("  Results: %d PASS / %d FAIL / %d TOTAL\n",
           total_pass, total_fail, total_pass + total_fail);
    printk("============================================\n");
    printk(total_fail == 0 ? "  *** ALL TESTS PASSED ***\n"
                           : "  *** %d TEST(S) FAILED ***\n", total_fail);
    printk("============================================\n");

    while (1)
        k_sleep(K_FOREVER);

    return 0;
}