# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_MODEM_SIMCOM7X00 simcom7x00_modem.c)
