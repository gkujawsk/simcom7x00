/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/modem/simcom7x00.h>

int main(void)
{
	const struct device *gnss =
		DEVICE_DT_GET(DT_PATH(test, uart_55556666, modem, gnss));
	uint32_t interval_ms;
	struct simcom7x00_gnss_snapshot snapshot;

	(void)gnss_get_fix_rate(gnss, &interval_ms);
	(void)gnss_set_fix_rate(gnss, 0U);
	(void)gnss_set_fix_rate(gnss, 1000U);
	(void)simcom7x00_gnss_get_snapshot(gnss, &snapshot);

	return 0;
}
