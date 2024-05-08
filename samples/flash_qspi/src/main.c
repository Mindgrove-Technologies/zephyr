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

#define SectorErase 0x00000000
#define SPI_FLASH_TEST_REGION_OFFSET 0xff000
#define SPI_FLASH_SECTOR_SIZE        4096

int main(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(qspi0));
	printf("Hello World! %s\n", CONFIG_BOARD);
	int rc;
	rc = flash_erase(dev, SectorErase, SPI_FLASH_SECTOR_SIZE);
	return 0;
}
