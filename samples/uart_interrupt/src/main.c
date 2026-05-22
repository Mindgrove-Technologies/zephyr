/*
 * UART0 interrupt-driven echo test
 * Mindgrove SoC — Zephyr 4.4
 *
 * Open a terminal at the UART0 baud rate.
 * Every character you type is echoed back.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>

/* ------------------------------------------------------------------ */
/* Config                                                              */
/* ------------------------------------------------------------------ */

#define UART_NODE  DT_NODELABEL(uart0)

#define RX_BUF_SIZE 256

/* ------------------------------------------------------------------ */
/* Ring buffer (ISR producer, main consumer)                          */
/* ------------------------------------------------------------------ */

static uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint32_t rx_head;  /* written by ISR  */
static volatile uint32_t rx_tail;  /* read  by main() */

static inline bool rb_empty(void)
{
    return rx_head == rx_tail;
}

static inline void rb_push(uint8_t c)
{
    uint32_t next = (rx_head + 1U) % RX_BUF_SIZE;

    if (next != rx_tail) {   /* silently drop on overflow */
        rx_buf[rx_head] = c;
        rx_head = next;
    }
}

static inline uint8_t rb_pop(void)
{
    uint8_t c = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1U) % RX_BUF_SIZE;
    return c;
}

/* ------------------------------------------------------------------ */
/* UART ISR                                                            */
/* ------------------------------------------------------------------ */

static void uart0_isr(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    if (!uart_irq_update(dev)) {
        return;
    }

    while (uart_irq_rx_ready(dev)) {
        uint8_t c;

        if (uart_fifo_read(dev, &c, 1) == 1) {
            rb_push(c);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
    const struct device *uart0 = DEVICE_DT_GET(UART_NODE);

    if (!device_is_ready(uart0)) {
        printk("uart0 not ready\n");
        return -ENODEV;
    }

    uart_irq_callback_set(uart0, uart0_isr);
    uart_irq_rx_enable(uart0);

    printk("=== UART0 echo test ready — type something! ===\n");

    while (1) {
        while (!rb_empty()) {
            uint8_t c = rb_pop();

            /* Echo back */
            uart_poll_out(uart0, c);

            /* On Enter, also send \r\n for clean terminal display */
            if (c == '\r' || c == '\n') {
                uart_poll_out(uart0, '\n');
                printk("[RX] newline\n");
            } else {
                printk("[RX] 0x%02X '%c'\n", c, (c >= 32 && c < 127) ? c : '.');
            }
        }

        k_sleep(K_MSEC(1));
    }

    return 0;
}