/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/modem/simcom7x00.h>
#include <zephyr/drivers/cellular.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

int main(void)
{
	const struct device *modem = DEVICE_DT_GET(DT_PATH(test, uart_55556666, modem));
	uint32_t interval_ms;
	enum simcom7x00_ppp_state ppp_state;
	enum simcom7x00_rat rat;
	enum simcom7x00_rat_mode rat_mode;
	char apn[32];
	char serial[32];
	char fw[80];
	struct simcom7x00_operator op;
	struct simcom7x00_network_status net_status;
	struct simcom7x00_rat_policy rat_policy = {
		.priorities = {
			SIMCOM7X00_RAT_LTE,
			SIMCOM7X00_RAT_UMTS,
			SIMCOM7X00_RAT_GSM,
		},
		.size = 3,
	};
	struct simcom7x00_control_diagnostics diagnostics;
	const struct cellular_network *supported_networks;
	uint8_t supported_networks_size;
	const struct cellular_network network_cfg[] = {
		{ .technology = CELLULAR_ACCESS_TECHNOLOGY_LTE, .bands = NULL, .size = 0 },
		{ .technology = CELLULAR_ACCESS_TECHNOLOGY_UMTS, .bands = NULL, .size = 0 },
	};

	(void)simcom7x00_get_signal_poll_interval(modem, &interval_ms);
	(void)simcom7x00_set_signal_poll_interval(modem, 0U);
	(void)simcom7x00_set_signal_poll_interval(modem, 2000U);
	(void)simcom7x00_get_ppp_state(modem, &ppp_state);
	(void)simcom7x00_ppp_connect(modem);
	(void)simcom7x00_ppp_disconnect(modem);
	(void)simcom7x00_set_apn(modem, "internet");
	(void)simcom7x00_get_apn(modem, apn, sizeof(apn));
	(void)simcom7x00_get_network_status(modem, &net_status);
	(void)simcom7x00_get_operator(modem, &op);
	(void)simcom7x00_get_current_rat(modem, &rat);
	(void)simcom7x00_set_rat_mode(modem, SIMCOM7X00_RAT_MODE_4G_ONLY);
	(void)simcom7x00_get_rat_mode(modem, &rat_mode);
	(void)simcom7x00_set_rat_policy(modem, &rat_policy);
	(void)simcom7x00_get_rat_policy(modem, &rat_policy);
	(void)simcom7x00_get_control_diagnostics(modem, &diagnostics);
	(void)simcom7x00_get_serial_number(modem, serial, sizeof(serial));
	(void)simcom7x00_get_firmware_version(modem, fw, sizeof(fw));
	(void)cellular_get_supported_networks(modem, &supported_networks, &supported_networks_size);
	(void)cellular_configure_networks(modem, network_cfg, ARRAY_SIZE(network_cfg));

	return 0;
}
