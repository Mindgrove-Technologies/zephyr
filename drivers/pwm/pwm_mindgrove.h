#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/slist.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/device.h>
#include <zephyr/dt-bindings/pwm/pwm.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Macros */

/* pwmcfg Bit Offsets */
#define PWM_0 0
#define PWM_1 1
#define PWM_2 2
#define PWM_3 3
#define PWM_4 4
#define PWM_5 5
#define PWM_6 6
#define PWM_7 7

// Control Register Individual Bits
#define PWM_ENABLE                     	(1<<0)
#define PWM_START                      	(1<<1)
#define PWM_OUTPUT_ENABLE              	(1<<2)
#define PWM_OUTPUT_POLARITY            	(1<<3)
#define PWM_HALFPERIOD_INTERRUPT_ENABLE	(1<<6)
#define PWM_FALL_INTERRUPT_ENABLE      	(1<<7)
#define PWM_RISE_INTERRUPT_ENABLE      	(1<<8)
#define PWM_UPDATE_ENABLE              	(1<<12)


#define CLOCK_FREQUENCY CONFIG_PWM_CLOCK_FREQUENCY

/*!Pulse Width Modulation Start Offsets */
#define PWM_MAX_COUNT 8				/*Maximum number of PWM*/

/*pinmux*/
#define PINMUX_START 0x40300 			/*Pinmux start address*/
#define PINMUX_CONFIGURE_REG 0x40300	/*Pinmux configuration register*/

#define PWM_BASE                   0x00030000UL
#define PWM_OFFSET                 0x00000100UL

// #define PWM_REG(z_config, _offset) ((mem_addr_t) ((z_config)->base + _offset))

#define PWM_REG(x)      ((PWM_Type*)(PWM_BASE + (x)*PWM_OFFSET))

volatile unsigned int* pinmux_config_reg = (volatile unsigned int* ) PINMUX_CONFIGURE_REG;

/*Interrupt enum*/
typedef enum
{
	rise_interrupt,				//Enable interrupt only on rise
	fall_interrupt,				//Enable interrupt only on fall
	halfperiod_interrupt,		//Enable interrupt only on halfperiod
	no_interrupt				//Disable interrupts
}pwm_interrupt_modes;

/* Structure Declarations */
/* PWM registers*/
typedef struct
{
  uint16_t clock;       	/*pwm clock register 16 bits*/
  uint16_t reserved0;		/*reserved for future use and currently has no defined purpose 16 bits*/
  uint16_t control;     	/*pwm control register 16 bits*/
  uint16_t reserved1;   	/*reserved for future use and currently has no defined purpose 16 bits*/
  uint32_t period;      	/*pwm period register 32 bits */
  uint32_t duty;        	/*pwm duty cycle register 32 bits*/
  uint16_t deadband_delay;	/*pwm deadband delay register 16 bits */
  uint16_t reserved2;		/*reserved for future use and currently has no defined purpose 16 bits*/
}PWM_Type;

/*Data structure*/
/*Zephyr configuration structure*/
struct pwm_mindgrove_cfg {
	uint32_t base;
	uint32_t f_sys;
	uint32_t cmpwidth;
	const struct pinctrl_dev_config *pcfg;
};

#ifdef __cplusplus
}
#endif
