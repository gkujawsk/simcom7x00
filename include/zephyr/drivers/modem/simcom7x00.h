/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MODEM_SIMCOM7X00_H_
#define ZEPHYR_INCLUDE_DRIVERS_MODEM_SIMCOM7X00_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/cellular.h>
#include <zephyr/drivers/gnss.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIMCOM7X00_OPERATOR_LONG_MAX_LEN 64
#define SIMCOM7X00_OPERATOR_SHORT_MAX_LEN 32
#define SIMCOM7X00_OPERATOR_PLMN_MAX_LEN 7
#define SIMCOM7X00_RAT_POLICY_MAX_PRIORITIES 3
#define SIMCOM7X00_CONTROL_COMMAND_MAX_LEN 64
#define SIMCOM7X00_CONTROL_FAILURE_MAX_LEN 64

enum simcom7x00_ppp_state {
	SIMCOM7X00_PPP_STATE_IDLE = 0,
	SIMCOM7X00_PPP_STATE_CONNECTING,
	SIMCOM7X00_PPP_STATE_WAITING_REGISTRATION,
	SIMCOM7X00_PPP_STATE_CONNECTED,
};

enum simcom7x00_rat {
	SIMCOM7X00_RAT_UNKNOWN = 0,
	SIMCOM7X00_RAT_GSM,
	SIMCOM7X00_RAT_EDGE,
	SIMCOM7X00_RAT_UMTS,
	SIMCOM7X00_RAT_LTE,
};

enum simcom7x00_rat_mode {
	SIMCOM7X00_RAT_MODE_AUTO = 0,
	SIMCOM7X00_RAT_MODE_2G_ONLY,
	SIMCOM7X00_RAT_MODE_3G_ONLY,
	SIMCOM7X00_RAT_MODE_4G_ONLY,
};

enum simcom7x00_control_result {
	SIMCOM7X00_CONTROL_RESULT_NONE = 0,
	SIMCOM7X00_CONTROL_RESULT_SUCCESS,
	SIMCOM7X00_CONTROL_RESULT_FAILED,
	SIMCOM7X00_CONTROL_RESULT_TIMEOUT,
	SIMCOM7X00_CONTROL_RESULT_BUSY,
};

struct simcom7x00_operator {
	char long_name[SIMCOM7X00_OPERATOR_LONG_MAX_LEN + 1];
	char short_name[SIMCOM7X00_OPERATOR_SHORT_MAX_LEN + 1];
	char plmn[SIMCOM7X00_OPERATOR_PLMN_MAX_LEN + 1];
};

struct simcom7x00_network_status {
	enum cellular_registration_status registration_status;
	bool roaming;
	enum simcom7x00_rat current_rat;
};

struct simcom7x00_rat_policy {
	enum simcom7x00_rat priorities[SIMCOM7X00_RAT_POLICY_MAX_PRIORITIES];
	uint8_t size;
};

struct simcom7x00_control_diagnostics {
	char last_command[SIMCOM7X00_CONTROL_COMMAND_MAX_LEN + 1];
	char last_failure[SIMCOM7X00_CONTROL_FAILURE_MAX_LEN + 1];
	uint32_t transaction_id;
	uint32_t completed_transaction_id;
	enum simcom7x00_control_result last_result;
	int last_cme_error;
	int last_cms_error;
	bool transaction_in_progress;
	bool has_cme_error;
	bool has_cms_error;
};

struct simcom7x00_gnss_snapshot {
	struct gnss_data data;
	uint32_t fix_interval_ms;
	/* True only while the cached fix is still current and usable. */
	bool data_valid;
	bool active;
	bool pipe_connected;
};

int simcom7x00_set_signal_poll_interval(const struct device *dev, uint32_t interval_ms);

int simcom7x00_get_signal_poll_interval(const struct device *dev, uint32_t *interval_ms);

int simcom7x00_ppp_connect(const struct device *dev);

int simcom7x00_ppp_disconnect(const struct device *dev);

int simcom7x00_get_ppp_state(const struct device *dev, enum simcom7x00_ppp_state *state);

int simcom7x00_set_apn(const struct device *dev, const char *apn);

int simcom7x00_get_apn(const struct device *dev, char *buf, size_t len);

int simcom7x00_get_network_status(const struct device *dev,
				  struct simcom7x00_network_status *status);

int simcom7x00_get_operator(const struct device *dev, struct simcom7x00_operator *op);

int simcom7x00_get_current_rat(const struct device *dev, enum simcom7x00_rat *rat);

int simcom7x00_set_rat_mode(const struct device *dev, enum simcom7x00_rat_mode mode);

int simcom7x00_get_rat_mode(const struct device *dev, enum simcom7x00_rat_mode *mode);

int simcom7x00_set_rat_policy(const struct device *dev,
			      const struct simcom7x00_rat_policy *policy);

int simcom7x00_get_rat_policy(const struct device *dev, struct simcom7x00_rat_policy *policy);

int simcom7x00_get_control_diagnostics(const struct device *dev,
				       struct simcom7x00_control_diagnostics *diagnostics);

int simcom7x00_gnss_get_snapshot(const struct device *dev,
				 struct simcom7x00_gnss_snapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MODEM_SIMCOM7X00_H_ */
