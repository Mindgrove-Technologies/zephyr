/*
 * Copyright (c) 2024 Mindgrove Technologies
 * SPDX-License-Identifier: Apache-2.0
 *
 * pinctrl_soc.h – Mindgrove MGS2401
 *
 * Drop this file in soc/riscv/mindgrove/mgs2401/ (already in the compiler
 * include path via the SoC CMakeLists).  The Zephyr pinctrl core includes
 * it by name: #include <pinctrl_soc.h>
 *
 * -----------------------------------------------------------------------
 * Register semantics  (verified against pinmux_driver.c)
 * -----------------------------------------------------------------------
 *
 *  MUX reg   bit=0 (PRIMARY)      bit=1 (ALT)
 *  --------  -------------------  -------------------
 *  MUX0–13   GPIO 0–7, 17–22      PWM 0–13
 *  MUX14–16  SPI2 MOSI/MISO/NCS   GPIO 32–34
 *  MUX17–19  SPI3 MOSI/MISO/NCS   GPIO 35–37
 *  MUX20–21  GPIO 8–9             UART3 TX/RX
 *  MUX22–23  GPIO 11, 15          UART4 TX/RX
 *  MUX24–27  GPTIMER 0–3          GPIO 38–41
 *  MUX28–30  JTAG TDI/TMS/TDO     GPIO 42–44
 *
 * The bit value to write depends on which function you want, NOT on a
 * uniform "primary=0, alt=1" rule.  The MINDGROVE_PINMUX macro encodes
 * the REGISTER BIT VALUE directly so the driver just writes it verbatim.
 *
 * -----------------------------------------------------------------------
 * Encoding  (32-bit word used as pinctrl_soc_pin_t)
 * -----------------------------------------------------------------------
 *
 *   bits [4:0]  – MUX register index, 0–30
 *   bit  [8]    – bit value to write to the register (0 or 1)
 *
 * Use the named macros below; do not construct values by hand.
 */

#ifndef ZEPHYR_SOC_RISCV_MINDGROVE_MGS2401_PINCTRL_SOC_H_
#define ZEPHYR_SOC_RISCV_MINDGROVE_MGS2401_PINCTRL_SOC_H_

#include <zephyr/types.h>
#include <zephyr/devicetree.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * Low-level encoding  (mux index + raw bit value)
 * --------------------------------------------------------------------- */

/**
 * @brief Encode a pin configuration.
 *
 * @param mux_idx   MUX register index, 0–30.
 * @param bit_val   Bit value to write: 0 or 1.
 */
#define MINDGROVE_PINMUX(mux_idx, bit_val) \
	((uint32_t)(((mux_idx) & 0x1FU) | (((bit_val) & 0x1U) << 8U)))

/** Extract the MUX register index. */
#define MINDGROVE_PIN_MUX_IDX(val)  ((uint8_t)((val) & 0x1FU))
/** Extract the bit value to write. */
#define MINDGROVE_PIN_BIT(val)      ((uint8_t)(((val) >> 8U) & 0x1U))

/* -----------------------------------------------------------------------
 * Named pin-function macros
 * (each expands to a ready-to-use MINDGROVE_PINMUX() call)
 *
 * Naming convention:  <PERIPHERAL>_<SIGNAL>
 * Where the peripheral is what you are ENABLING.
 * -----------------------------------------------------------------------*/

/* ── GPIO 0–7  (MUX0–7,  bit=0) ──────────────────────────────────────── */
#define MG_GPIO0   MINDGROVE_PINMUX(0,  0)
#define MG_GPIO1   MINDGROVE_PINMUX(1,  0)
#define MG_GPIO2   MINDGROVE_PINMUX(2,  0)
#define MG_GPIO3   MINDGROVE_PINMUX(3,  0)
#define MG_GPIO4   MINDGROVE_PINMUX(4,  0)
#define MG_GPIO5   MINDGROVE_PINMUX(5,  0)
#define MG_GPIO6   MINDGROVE_PINMUX(6,  0)
#define MG_GPIO7   MINDGROVE_PINMUX(7,  0)

/* ── PWM 0–7   (MUX0–7,  bit=1) ──────────────────────────────────────── */
#define MG_PWM0    MINDGROVE_PINMUX(0,  1)
#define MG_PWM1    MINDGROVE_PINMUX(1,  1)
#define MG_PWM2    MINDGROVE_PINMUX(2,  1)
#define MG_PWM3    MINDGROVE_PINMUX(3,  1)
#define MG_PWM4    MINDGROVE_PINMUX(4,  1)
#define MG_PWM5    MINDGROVE_PINMUX(5,  1)
#define MG_PWM6    MINDGROVE_PINMUX(6,  1)
#define MG_PWM7    MINDGROVE_PINMUX(7,  1)

/* ── GPIO 17–22 (MUX8–13, bit=0) ─────────────────────────────────────── */
#define MG_GPIO17  MINDGROVE_PINMUX(8,  0)
#define MG_GPIO18  MINDGROVE_PINMUX(9,  0)
#define MG_GPIO19  MINDGROVE_PINMUX(10, 0)
#define MG_GPIO20  MINDGROVE_PINMUX(11, 0)
#define MG_GPIO21  MINDGROVE_PINMUX(12, 0)
#define MG_GPIO22  MINDGROVE_PINMUX(13, 0)

/* ── PWM 8–13  (MUX8–13, bit=1) ──────────────────────────────────────── */
#define MG_PWM8    MINDGROVE_PINMUX(8,  1)
#define MG_PWM9    MINDGROVE_PINMUX(9,  1)
#define MG_PWM10   MINDGROVE_PINMUX(10, 1)
#define MG_PWM11   MINDGROVE_PINMUX(11, 1)
#define MG_PWM12   MINDGROVE_PINMUX(12, 1)
#define MG_PWM13   MINDGROVE_PINMUX(13, 1)

/* ── SPI2  (MUX14–16, bit=0) ─────────────────────────────────────────── */
#define MG_SPI2_MOSI  MINDGROVE_PINMUX(14, 0)
#define MG_SPI2_MISO  MINDGROVE_PINMUX(15, 0)
#define MG_SPI2_NCS   MINDGROVE_PINMUX(16, 0)

/* ── GPIO32–34 (MUX14–16, bit=1) ─────────────────────────────────────── */
#define MG_GPIO32  MINDGROVE_PINMUX(14, 1)
#define MG_GPIO33  MINDGROVE_PINMUX(15, 1)
#define MG_GPIO34  MINDGROVE_PINMUX(16, 1)

/* ── SPI3  (MUX17–19, bit=0) ─────────────────────────────────────────── */
#define MG_SPI3_MOSI  MINDGROVE_PINMUX(17, 0)
#define MG_SPI3_MISO  MINDGROVE_PINMUX(18, 0)
#define MG_SPI3_NCS   MINDGROVE_PINMUX(19, 0)

/* ── GPIO35–37 (MUX17–19, bit=1) ─────────────────────────────────────── */
#define MG_GPIO35  MINDGROVE_PINMUX(17, 1)
#define MG_GPIO36  MINDGROVE_PINMUX(18, 1)
#define MG_GPIO37  MINDGROVE_PINMUX(19, 1)

/* ── GPIO8–9  (MUX20–21, bit=0) ──────────────────────────────────────── */
#define MG_GPIO8   MINDGROVE_PINMUX(20, 0)
#define MG_GPIO9   MINDGROVE_PINMUX(21, 0)

/* ── UART3    (MUX20–21, bit=1) ──────────────────────────────────────── */
#define MG_UART3_TX  MINDGROVE_PINMUX(20, 1)
#define MG_UART3_RX  MINDGROVE_PINMUX(21, 1)

/* ── GPIO11, GPIO15 (MUX22–23, bit=0) ────────────────────────────────── */
#define MG_GPIO11  MINDGROVE_PINMUX(22, 0)
#define MG_GPIO15  MINDGROVE_PINMUX(23, 0)

/* ── UART4    (MUX22–23, bit=1) ──────────────────────────────────────── */
#define MG_UART4_TX  MINDGROVE_PINMUX(22, 1)
#define MG_UART4_RX  MINDGROVE_PINMUX(23, 1)

/* ── GPTIMER0–3 (MUX24–27, bit=0) ───────────────────────────────────── */
#define MG_GPTIMER0  MINDGROVE_PINMUX(24, 0)
#define MG_GPTIMER1  MINDGROVE_PINMUX(25, 0)
#define MG_GPTIMER2  MINDGROVE_PINMUX(26, 0)
#define MG_GPTIMER3  MINDGROVE_PINMUX(27, 0)

/* ── GPIO38–41 (MUX24–27, bit=1) ─────────────────────────────────────── */
#define MG_GPIO38  MINDGROVE_PINMUX(24, 1)
#define MG_GPIO39  MINDGROVE_PINMUX(25, 1)
#define MG_GPIO40  MINDGROVE_PINMUX(26, 1)
#define MG_GPIO41  MINDGROVE_PINMUX(27, 1)

/* ── JTAG     (MUX28–30, bit=0) ──────────────────────────────────────── */
#define MG_JTAG_TDI  MINDGROVE_PINMUX(28, 0)
#define MG_JTAG_TMS  MINDGROVE_PINMUX(29, 0)
#define MG_JTAG_TDO  MINDGROVE_PINMUX(30, 0)

/* ── GPIO42–44 (MUX28–30, bit=1) ─────────────────────────────────────── */
#define MG_GPIO42  MINDGROVE_PINMUX(28, 1)
#define MG_GPIO43  MINDGROVE_PINMUX(29, 1)
#define MG_GPIO44  MINDGROVE_PINMUX(30, 1)

/* -----------------------------------------------------------------------
 * Zephyr-required type and macro
 * --------------------------------------------------------------------- */

/** Opaque SoC pin type: a 32-bit encoded word from MINDGROVE_PINMUX(). */
typedef uint32_t pinctrl_soc_pin_t;

/**
 * @brief Build the pin array initialiser for one pinctrl state.
 *
 * The Zephyr pinctrl core calls this from PINCTRL_DT_INST_DEFINE().
 * It iterates every child group node of the state node and expands each
 * element of the `pinmux` property.
 *
 * @param node_id  DTS node identifier of the pinctrl state node.
 * @param prop     Property name passed by the core (pinctrl-0, etc.).
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx) \
    ((pinctrl_soc_pin_t)DT_PROP_BY_IDX(node_id, prop, idx)),

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                    \
    { DT_FOREACH_PROP_ELEM(                                         \
        DT_PHANDLE(node_id, prop),                                  \
        pinmux,                                                      \
        Z_PINCTRL_STATE_PIN_INIT) }
#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_RISCV_MINDGROVE_MGS2401_PINCTRL_SOC_H_ */