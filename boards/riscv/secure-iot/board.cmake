# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BOARD_SECURE_IOT)
  board_runner_args(openocd "--use-elf" "--config=${ZEPHYR_BASE}/boards/riscv/secure-iot/support/FT2232H-JTAG.cfg")
  set(OPENOCD "/usr/local/bin/openocd")

  include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)

endif()
