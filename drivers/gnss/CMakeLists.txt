# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/gnss)
zephyr_library_sources_ifdef(CONFIG_GNSS_SIMCOM7X00 simcom7x00_gnss.c)
