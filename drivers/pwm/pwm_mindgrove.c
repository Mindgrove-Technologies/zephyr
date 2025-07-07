/**
 * Project                           : Secure IoT SoC
 * Name of the file                  : pwm_mindgrove.c
 * Brief Description of file         : This is a zephyr rtos PWM Driver file for Mindgrove Silicon's PWM Peripheral.
 * Name of Author                    : Harini Sree.S
 * Email ID                          : harini@mindgrovetech.in
 * 
 * @file pwm_mindgrove.c
 * @author Harini Sree.S (harini@mindgrovetech.in)
 * @brief This is a zephyr rtos PWM Driver file for Mindgrove Silicon's PWM Peripheral.
 * @version 0.1
 * @date 2024-05-03
 * 
 * @copyright Copyright (c) Mindgrove Technologies Pvt. Ltd 2023. All rights reserved.
 */

#include "pwm_mindgrove.h"


#define DT_DRV_COMPAT mindgrove_pwm

/* Functions */
/** @fn  pwm_init
 * @brief Function to initialize all pwm modules
 * @details This function will be called to initialize all pwm modules
 * @param[in] dev The device declared in devicetree
 * @param[Out] No output parameter
 */
static int pwm_mindgrove_init(const struct device *dev)
{
	const struct pwm_mindgrove_cfg *cfg = dev->config;
	char *pwm_no;
	pwm_no = dev->name;
	int pwm_number = pwm_no[6] - '0';
}

/** @fn  configure_control_mode
 * @brief Function to set value of control register based on parameteres
 * @details This function will set value of control register based on parameters
 * @param[in]  bool          (update                      - specifies if the module is to be updated)
 *           interrupt_mode  (interrupt_mode              - it specifes the mode of the interrupt)
 *           bool            (change_output_polarity      - it specifies if the output polarity should be changed) 
 * @param[Out] uint32_t (returns value to be set in the control register.)
 */
inline int configure_control(bool update, pwm_interrupt_modes interrupt_mode, bool change_output_polarity)
{
	int value = 0x0;

	if(update==1)
	{
		value |=PWM_UPDATE_ENABLE;
	}

	if(interrupt_mode==0)
	{
		value |= PWM_RISE_INTERRUPT_ENABLE;
	}

	if(interrupt_mode==1)
	{
		value |= PWM_FALL_INTERRUPT_ENABLE;
	}

	if(interrupt_mode==2)
	{
		value |= PWM_HALFPERIOD_INTERRUPT_ENABLE;
	}

	if(change_output_polarity)
	{
		value |= PWM_OUTPUT_POLARITY;
	}

	return value;
}

/** @fn pinmux_enable_pwm
 * @brief Function to configure the pin as pwm module
 * @details This function will be called to configure the pin as pwm module
 * @param[in] uint32_t (num - specifies the pwm module to be selected)
 * @param[Out] No output parameter
 */
void pinmux_enable_pwm(int num)
{
	if (num < 8)
	{
		*(pinmux_config_reg + num) = 1;
	}
	else
		printf("Max pinmuxed PWMs are 8");
}

/** @fn pwm_mindgrove_set_cycles
 * @brief Set the period and pulse width for a specific pwm module
 * @details This function will set the period and pulse width for a specific pwm module
 * @param[in] dev The device declared in devicetree
 * 			uint32_t (channel-  the pwm module to be selected)
 * 			uint32_t (period_cycles- value of periodic cycle to be used)
 * 			uint32_t (pulse_cycles- value of duty cycle)
 * 			pwm_flags_t (flags- polarity of a PWM channel.)
 * @param[Out] No output parameter
 */
static int pwm_mindgrove_set_cycles(const struct device *dev, uint32_t channel,uint32_t period_cycles, uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_mindgrove_cfg *cfg = dev->config;

	char *pwm_inst; 
	pwm_inst = dev->name;
	int pwm_number = pwm_inst[6] - '0';
	uint32_t deadband_delay;	/*32bit field to store deadband delay value*/ 
	uint16_t prescale;			/*32bit field to store prescale value*/

	/*Switch case to obtain deadband delay, control register value and prescale from the devicetree for the given pwm_number*/
	switch (pwm_number)
	{
	case PWM_0:
		int db_config0[] = DT_PROP(DT_NODELABEL(pwm0), db_configure);
		deadband_delay = db_config0[0];
		prescale = db_config0[1];
		break;
	case PWM_1:
		int db_config1[] = DT_PROP(DT_NODELABEL(pwm1), db_configure);
		deadband_delay = db_config1[0];
		prescale = db_config1[1];
		break;
	case PWM_2:
		int db_config2[] = DT_PROP(DT_NODELABEL(pwm2), db_configure);
		deadband_delay = db_config2[0];
		prescale = db_config2[1];
		break;
	case PWM_3:
		int db_config3[] = DT_PROP(DT_NODELABEL(pwm3), db_configure);
		deadband_delay = db_config3[0];
		prescale = db_config3[1];
		break;
	case PWM_4:
		int db_config4[] = DT_PROP(DT_NODELABEL(pwm4), db_configure);
		deadband_delay = db_config4[0];
		prescale = db_config4[1];
		break;
	case PWM_5:
		int db_config5[] = DT_PROP(DT_NODELABEL(pwm5), db_configure);
		deadband_delay = db_config5[0];
		prescale = db_config5[1];
		break;
	case PWM_6:
		int db_config6[] = DT_PROP(DT_NODELABEL(pwm6), db_configure);
		deadband_delay = db_config6[0];
		prescale = db_config6[1];
		break;
	case PWM_7:
		int db_config7[] = DT_PROP(DT_NODELABEL(pwm7), db_configure);
		deadband_delay = db_config7[0];
		prescale = db_config7[1];
		break;
	default:
		break;
	}

	pinmux_enable_pwm(pwm_number);

    if( prescale > 32768)
		return 0;

	PWM_REG(pwm_number)->clock = (prescale << 1);

	if(pulse_cycles == period_cycles)//Period is equal to Duty, use GPIO pin to avoid triggering interrupt in PWM
		return 0;
	else
		PWM_REG(pwm_number)->duty=pulse_cycles;

	PWM_REG(pwm_number)->period=period_cycles;
	PWM_REG(pwm_number)->deadband_delay = deadband_delay;
	int control = configure_control(false,no_interrupt,flags);
	PWM_REG(pwm_number)->control=control;
	PWM_REG(pwm_number)->control= control | (PWM_UPDATE_ENABLE | PWM_ENABLE | PWM_START | PWM_OUTPUT_ENABLE);

	return 0;
}

static int pwm_mindgrove_get_cycles_per_sec(const struct device *dev,
					 uint32_t channel, uint64_t *cycles)
{
	const struct pwm_mindgrove_cfg *cfg = dev->config;

	char *pwm_inst; 
	pwm_inst = dev->name;
	int pwm_number = pwm_inst[6] - '0';

	int period, prescale, freq;
	period = PWM_REG(pwm_number)->period;
	prescale = PWM_REG(pwm_number)->clock;
	freq = CLOCK_FREQUENCY / (period * prescale);

	*cycles = freq;
	return 0;
}

/* Device Instantiation */

static const struct pwm_driver_api pwm_mindgrove_api = {
	.set_cycles = pwm_mindgrove_set_cycles,
	.get_cycles_per_sec = pwm_mindgrove_get_cycles_per_sec,
};
// .cmpwidth = DT_INST_PROP(n, mindgrove_compare_width), \

#define PWM_MINDGROVE_INIT(n)	\
	static const struct pwm_mindgrove_cfg pwm_mindgrove_cfg_##n = {	\
			.base = DT_INST_REG_ADDR(n) ,	\
			.f_sys = CLOCK_FREQUENCY,  \
		};	\
	DEVICE_DT_INST_DEFINE(n,	\
			    pwm_mindgrove_init,	\
			    NULL,	\
			    NULL,	\
			    &pwm_mindgrove_cfg_##n,	\
			    POST_KERNEL,	\
			    CONFIG_PWM_INIT_PRIORITY,	\
			    &pwm_mindgrove_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_MINDGROVE_INIT)
