#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

void main(void)
{
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(uart1));

    if (!device_is_ready(dev)) {
        printk("UART1 device is not ready\n");
        return;
    }

    const char *msg = "Hello from UART1\n";
    for (int i = 0; msg[i] != '\0'; i++) {
        uart_poll_out(dev, msg[i]);
    }
}
