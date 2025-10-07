#include <stdio.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

int main(void)
{
	printk("hello world\n");

	return 0;
}