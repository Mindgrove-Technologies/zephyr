/* 
 * Copyright (c) Mindgrove Technologies Pvt. Ltd 2023.
 */

/**
 * @brief UART driver for the Mindgrove Processor
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/uart.h>

#define DT_DRV_COMPAT mindgrove_uart

#ifdef CONFIG_BOARD_SHAKTI_VAJRA

#define SHAKTI_UART_0_CLK_FREQUENCY 50000000
#define SHAKTI_UART_1_CLK_FREQUENCY 50000000
#define SHAKTI_UART_BAUD 			19200
#define SHAKTI_VCU_UART_BAUD 		115200

#endif

#ifdef CONFIG_BOARD_SECURE_IOT

#define SHAKTI_UART_0_CLK_FREQUENCY 700000000 // Change to 40000000 for nexys video board and 100 * 10^6 for vcu118 FPGA
#define SHAKTI_UART_1_CLK_FREQUENCY 40000000
#define SECIOT_NEXYS_UART_BAUD 19200
#define SECIOT_VCU118_UART_BAUD 115200

#endif

#define RXDATA_EMPTY   (1 << 31)   /* Receive FIFO Empty */
#define RXDATA_MASK    0xFF        /* Receive Data Mask */

#define TXDATA_FULL    (1 << 31)   /* Transmit FIFO Full */

#define TXCTRL_TXEN    (1 << 0)    /* Activate Tx Channel */

#define RXCTRL_RXEN    (1 << 0)    /* Activate Rx Channel */

#define IE_TXWM        (1 << 0)    /* TX Interrupt Enable/Pending */
#define IE_RXWM        (1 << 1)    /* RX Interrupt Enable/Pending */

#define UART_TX_OFFSET        0x04
#define UART_RX_OFFSET        0x08
#define UART_STATUS_OFFSET      0x0c
#define UART_EV_ENABLE_OFFSET   0x18
#define UART_BAUD_OFFSET        0x00

#define BREAK_ERROR	    1 << 7
#define FRAME_ERROR	    1 << 6
#define OVERRUN        	    1 << 5
#define PARITY_ERROR        1 << 4
#define STS_RX_FULL 	    1 << 3
#define STS_RX_NOT_EMPTY    1 << 2
#define STS_TX_FULL 	    1 << 1
#define STS_TX_EMPTY 	    1 << 0

/*
 * RX/TX Threshold count to generate TX/RX Interrupts.
 * Used by txctrl and rxctrl registers
 */
#define CTRL_CNT(x)    (((x) & 0x07) << 16)

struct uart_mindgrove_regs_t {
    uint16_t div;
    uint16_t reserv0;
    uint32_t tx;
    uint32_t rx;
    unsigned short  status;
    uint16_t reserv2;
    uint16_t delay;
    uint16_t reserv3;
    uint16_t control;
    uint16_t reserv4;
    uint8_t  ie; 
    uint8_t  reserv5;
    uint16_t reserv6;
    uint8_t  iqcycles;
    uint8_t  reserv7;
    uint16_t reserv8;
    uint8_t  rx_threshold;
    uint8_t  reserv9;
    uint16_t reserv10;
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
typedef void (*irq_cfg_func_t)(void);
#endif

struct uart_mindgrove_config {
	uint32_t       port;
	uint32_t       sys_clk_freq;
	uint32_t       baud_rate;
	uint32_t       rxcnt_irq;
	uint32_t       txcnt_irq;
	const struct	pinctrl_dev_config *pcfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	irq_cfg_func_t cfg_func;
#endif
};

struct uart_mindgrove_data {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *cb_data;
#endif
};

#define DEV_CFG(dev) ((struct uart_mindgrove_config * const)(dev)->config)

#define DEV_UART(dev) ((struct uart_mindgrove_regs_t *)(uintptr_t)(DEV_CFG(dev)->port))

#define DEV_DATA(dev) ((struct uart_mindgrove_data * const)(dev)->data)

/**
 * @brief Output a character in polled mode.
 *
 * Writes data to tx register if transmitter is not full.
 *
 * @param dev UART device struct
 * @param c Character to send
 *
 * @return Sent character
 */
static void uart_mindgrove_poll_out(const struct device *dev,
					 unsigned char c)
{
  	volatile struct uart_mindgrove_regs_t *uart = DEV_UART(dev);

	// Wait while TX FIFO is full
	while (uart->status & STS_TX_FULL)
		;

	uart->tx = (int)c;
}

/**
 * @brief Poll the device for input.
 *
 * @param dev UART device struct
 * @param c Pointer to character
 *
 * @return 0 if a character arrived, -1 if the input buffer if empty.
 */
static int uart_mindgrove_poll_in(const struct device *dev, unsigned char *c)
{
	volatile struct uart_mindgrove_regs_t *uart = DEV_UART(dev);

	while ((uart->status & STS_RX_NOT_EMPTY) == 0);
	volatile uint32_t read_val = uart->rx;
	*c = (unsigned char)(read_val & RXDATA_MASK);

	return 0;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

/**
 * @brief Fill FIFO with data
 *
 * @param dev UART device struct
 * @param tx_data Data to transmit
 * @param size Number of bytes to send
 *
 * @return Number of bytes sent
 */
static int uart_mindgrove_fifo_fill(const struct device *dev,
				const uint8_t *tx_data,
				int size)
{
	volatile struct uart_mindgrove_regs_t *uart = DEV_UART(dev);
	int i;

	for (i = 0; i < size && !(uart->tx & TXDATA_FULL); i++)
		uart->tx = (int)tx_data[i];

	return i;
}

/**
 * @brief Read data from FIFO
 *
 * @param dev UART device struct
 * @param rxData Data container
 * @param size Container size
 *
 * @return Number of bytes read
 */
static int uart_mindgrove_fifo_read(const struct device *dev,
				uint8_t *rx_data,
				const int size)
{
	volatile struct uart_mindgrove_regs_t *uart = DEV_UART(dev);
	int i;
	uint32_t val;

	for (i = 0; i < size; i++) {
		val = uart->rx;

		if (val & RXDATA_EMPTY)
			break;

		rx_data[i] = (uint8_t)(val & RXDATA_MASK);
	}

	return i;
}

/**
 * @brief Enable TX interrupt in ie register
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_mindgrove_irq_tx_enable(const struct device *dev)
{
	volatile struct uart_mindgrove_regs_t *uart = DEV_UART(dev);

	uart->ie |= IE_TXWM;
}

/**
 * @brief Disable TX interrupt in ie register
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_mindgrove_irq_tx_disable(const struct device *dev)
{
	volatile struct uart_mindgrove_regs_t *uart = DEV_UART(dev);

	uart->ie &= ~IE_TXWM;
}

/**
 * @brief Check if Tx IRQ has been raised
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is ready, 0 otherwise
 */
static int uart_mindgrove_irq_tx_ready(const struct device *dev)
{
	volatile struct uart_mindgrove_regs_t *uart = DEV_UART(dev);

	return !!(uart->ie & IE_TXWM);
}

/**
 * @brief Check if nothing remains to be transmitted
 *
 * @param dev UART device struct
 *
 * @return 1 if nothing remains to be transmitted, 0 otherwise
 */
static int uart_mindgrove_irq_tx_complete(const struct device *dev)
{
	volatile struct uart_mindgrove_regs_t *uart = DEV_UART(dev);

	/*
	 * No TX EMTPY flag for this controller,
	 * just check if TX FIFO is not full
	 */
	return !(uart->tx & TXDATA_FULL);
}

/**
 * @brief Enable RX interrupt in ie register
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_mindgrove_irq_rx_enable(const struct device *dev)
{
	volatile struct uart_mindgrove_regs_t *uart = DEV_UART(dev);

	uart->ie |= IE_RXWM;
}

/**
 * @brief Disable RX interrupt in ie register
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_mindgrove_irq_rx_disable(const struct device *dev)
{
	volatile struct uart_mindgrove_regs_t *uart = DEV_UART(dev);

	uart->ie &= ~IE_RXWM;
}

/**
 * @brief Check if Rx IRQ has been raised
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is ready, 0 otherwise
 */
static int uart_mindgrove_irq_rx_ready(const struct device *dev)
{
	volatile struct uart_mindgrove_regs_t *uart = DEV_UART(dev);

	return !!(uart->ie & IE_RXWM);
}

/* No error interrupt for this controller */
static void uart_mindgrove_irq_err_enable(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static void uart_mindgrove_irq_err_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
}

/**
 * @brief Check if any IRQ is pending
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is pending, 0 otherwise
 */
static int uart_mindgrove_irq_is_pending(const struct device *dev)
{
	volatile struct uart_mindgrove_regs_t *uart = DEV_UART(dev);

	return !!(uart->ie & (IE_RXWM | IE_TXWM));
}

static int uart_mindgrove_irq_update(const struct device *dev)
{
	return 1;
}

/**
 * @brief Set the callback function pointer for IRQ.
 *
 * @param dev UART device struct
 * @param cb Callback function pointer.
 *
 * @return N/A
 */
static void uart_mindgrove_irq_callback_set(const struct device *dev,
					uart_irq_callback_user_data_t cb,
					void *cb_data)
{
	struct uart_mindgrove_data *data = DEV_DATA(dev);

	data->callback = cb;
	data->cb_data = cb_data;
}

static void uart_mindgrove_irq_handler(void *arg)
{
	struct device *dev = (struct device *)arg;
	struct uart_mindgrove_data *data = DEV_DATA(dev);

	if (data->callback)
		data->callback(dev,data->cb_data);
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */


static int uart_mindgrove_init(const struct device *dev)
{
	struct uart_mindgrove_config * const cfg = DEV_CFG(dev);
	volatile struct uart_mindgrove_regs_t *uart = DEV_UART(dev);

	/* Set baud rate */
	uart->div = (cfg->sys_clk_freq / cfg->baud_rate) / 16;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	/* Ensure that uart IRQ is disabled initially */
	uart->ie = 0;

	/* Setup IRQ handler */
	cfg->cfg_func();

#endif

	return 0;
}

static const struct uart_driver_api uart_mindgrove_driver_api = {
	.poll_in          = uart_mindgrove_poll_in,
	.poll_out         = uart_mindgrove_poll_out,
	.err_check        = NULL,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill        = uart_mindgrove_fifo_fill,
	.fifo_read        = uart_mindgrove_fifo_read,
	.irq_tx_enable    = uart_mindgrove_irq_tx_enable,
	.irq_tx_disable   = uart_mindgrove_irq_tx_disable,
	.irq_tx_ready     = uart_mindgrove_irq_tx_ready,
	.irq_tx_complete  = uart_mindgrove_irq_tx_complete,
	.irq_rx_enable    = uart_mindgrove_irq_rx_enable,
	.irq_rx_disable   = uart_mindgrove_irq_rx_disable,
	.irq_rx_ready     = uart_mindgrove_irq_rx_ready,
	.irq_err_enable   = uart_mindgrove_irq_err_enable,
	.irq_err_disable  = uart_mindgrove_irq_err_disable,
	.irq_is_pending   = uart_mindgrove_irq_is_pending,
	.irq_update       = uart_mindgrove_irq_update,
	.irq_callback_set = uart_mindgrove_irq_callback_set,
#endif
};

#ifdef CONFIG_UART_MINDGROVE_PORT

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_MINDGROVE_CFG_FUNC(n) .cfg_func = uart_mindgrove_irq_cfg_func_##n,
#define UART_MINDGROVE_IRQ_CONFIG_FUNC(n) \
    static void uart_mindgrove_irq_cfg_func_##n(void) { \
        IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), \
                    uart_mindgrove_irq_handler, DEVICE_DT_INST_GET(n), 0); \
        irq_enable(DT_INST_IRQN(n)); \
    }
#else
#define UART_MINDGROVE_CFG_FUNC(n)
#define UART_MINDGROVE_IRQ_CONFIG_FUNC(n)
#endif // CONFIG_UART_INTERRUPT_DRIVEN

#endif // CONFIG_UART_MINDGROVE_PORT

#define UART_MINDGROVE_INIT(n) \
    UART_MINDGROVE_IRQ_CONFIG_FUNC(n) \
    static struct uart_mindgrove_config uart_mindgrove_config_##n = { \
        .port = DT_INST_REG_ADDR(n), \
        .sys_clk_freq = DT_INST_PROP(n, clock_frequency), \
        .baud_rate = DT_INST_PROP(n, current_speed), \
        .rxcnt_irq = 0, \
        .txcnt_irq = 0, \
        UART_MINDGROVE_CFG_FUNC(n) \
    }; \
    static struct uart_mindgrove_data uart_mindgrove_data_##n; \
    DEVICE_DT_INST_DEFINE(n, \
        uart_mindgrove_init, \
        NULL, \
        &uart_mindgrove_data_##n, \
        &uart_mindgrove_config_##n, \
        PRE_KERNEL_1, \
        CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
        &uart_mindgrove_driver_api, \
        NULL);
	
DT_INST_FOREACH_STATUS_OKAY(UART_MINDGROVE_INIT)