#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(uart2));
void uart_loopback_test(void)
{
    const char *msg = "Hello from UART\n";
    char recv_char;

    // Send message
    for (int i = 0; msg[i] != '\0'; i++) {
        uart_poll_out(dev, msg[i]);
    }

	k_busy_wait(1000); // 1 ms delay (1000 microseconds)

    // Receive message
    printk("Trying to read back:\n");
    for (int i = 0; msg[i] != '\0'; i++) {
        if (uart_poll_in(dev, &recv_char) == 0) {
            printk("%c", recv_char);
        }
    }
    printk("\n");
}

void main(void)
{
	printf("Hello world!\n");
    if (!device_is_ready(dev)) {
        printk("UART device is not ready\n");
        return;
    }

	uart_loopback_test();
}