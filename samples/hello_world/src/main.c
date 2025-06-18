#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/devicetree.h>

int main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);
	return 0;
}