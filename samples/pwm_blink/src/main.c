/*
 * Copyright (c) 2025 Mindgrove Technologies Pvt Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/pwm.h>

void main() 
{
    const struct device *dev0 = DEVICE_DT_GET(DT_NODELABEL(pwm0));
    const struct device *dev1 = DEVICE_DT_GET(DT_NODELABEL(pwm1));
    uint64_t cycle0,cycle1;

    pwm_set_cycles(dev0,0, 1000, 500, PWM_POLARITY_NORMAL);
    pwm_set_cycles(dev1,0, 100, 50, PWM_POLARITY_NORMAL);
    
	pwm_get_cycles_per_sec(dev0, 0, &cycle0);
	pwm_get_cycles_per_sec(dev1, 0, &cycle1);

    printf("Frequency of PWM0 : %uHz\n",cycle0);
    printf("Frequency of PWM1 : %uHz\n",cycle1);
}
