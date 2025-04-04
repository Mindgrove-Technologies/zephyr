/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#define SectorErase_addr 0x00000000
#define FLASH_4K_ERASE             4096
#define FLASH_32K_ERASE            32768
#define FLASH_CHIP_ERASE           134217728    

int main(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(qspi0));
	printf("Hello World! %s\n", CONFIG_BOARD);

	const uint8_t expected[] = { 0x55, 0xaa, 0x66, 0x99 };
	const size_t len = sizeof(expected);
	uint8_t buf[sizeof(expected)];
	int rc;

	/*
	printk("\nTest 1: Flash erase\n");
	
	rc = flash_erase(dev, SectorErase_addr, FLASH_32K_ERASE);
	if(rc != 0) {
		printk("Flash erase failed %d\n", rc);
	} else {
		printk("Flash erase succeeded!\n");
	}

	printk("\n Test 2: Flash write\n");
	rc = flash_write(dev, 0x00000400, expected, len);
	if (rc == 0) {
		printf("Flash write successfull! %d\n", rc);
		// return;
	}

	printk("\n Test 3: Flash read\n");
	rc = flash_read(dev, 0x00000400, buf, len);
	if (rc == 0) {
		printf("Flash read successfull! %d\n", rc);
		// return;
	}
	*/

	return 0;
}
