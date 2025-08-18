# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BOARD_SECURE_IOT)
  board_runner_args(openocd "--use-elf" "--config=${CMAKE_CURRENT_SOURCE_DIR}/secure-iot_200t.cfg")

  include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)

endif()
