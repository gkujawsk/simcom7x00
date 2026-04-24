/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/kernel.h>

int main(void)
{
	ARG_UNUSED(device_is_ready);
	ARG_UNUSED(gnss_get_supported_systems);

	while (1) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
