/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/drivers/flash.h>

int main(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(qspi0));
	printf("Hello World! %s\n", CONFIG_BOARD);
	return 0;
}
