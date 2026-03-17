/*
 * @copyright Copyright (c) Mindgrove Technologies Pvt. Ltd 2025. All rights reserved.
 * 
 * Global interrupt data maintenance structure
*/
#ifndef __INTC_PLIC_H__
#define __INTC_PLIC_H__

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/interrupt_controller/riscv_plic.h>

#ifdef __cplusplus
extern "C" {
#endif

enum{
   PLIC_PRIORITY_1,
   PLIC_PRIORITY_2,
   PLIC_PRIORITY_3,
   PLIC_PRIORITY_4,
   PLIC_PRIORITY_5,
   PLIC_PRIORITY_6,
   PLIC_PRIORITY_7 
};


#ifdef __cplusplus
}
#endif

#endif /* __INTC_PLIC_H__ */
