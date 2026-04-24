/*
 * Copyright (c) 2023 Bjarki Arge Andreasen
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/cellular.h>
#include <zephyr/modem/chat.h>
#include <zephyr/modem/cmux.h>
#include <zephyr/modem/pipe.h>
#include <zephyr/modem/pipelink.h>
#include <zephyr/modem/ppp.h>
#include <zephyr/modem/backend/uart.h>
#include <zephyr/net/ppp.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/modem/simcom7x00.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(simcom7x00_modem, CONFIG_MODEM_SIMCOM7X00_LOG_LEVEL);

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MODEM_CELLULAR_PERIODIC_SCRIPT_TIMEOUT \
	K_MSEC(CONFIG_MODEM_SIMCOM7X00_PERIODIC_SCRIPT_MS)

#define MODEM_CELLULAR_DATA_IMEI_LEN         (16)
#define MODEM_CELLULAR_DATA_MODEL_ID_LEN     (65)
#define MODEM_CELLULAR_DATA_IMSI_LEN         (23)
#define MODEM_CELLULAR_DATA_ICCID_LEN        (22)
#define MODEM_CELLULAR_DATA_MANUFACTURER_LEN (65)
#define MODEM_CELLULAR_DATA_FW_VERSION_LEN   (65)
#define MODEM_CELLULAR_DATA_APN_LEN          (63)
#define MODEM_CELLULAR_DATA_OPERATOR_LONG_LEN  SIMCOM7X00_OPERATOR_LONG_MAX_LEN
#define MODEM_CELLULAR_DATA_OPERATOR_SHORT_LEN SIMCOM7X00_OPERATOR_SHORT_MAX_LEN
#define MODEM_CELLULAR_DATA_OPERATOR_PLMN_LEN  SIMCOM7X00_OPERATOR_PLMN_MAX_LEN
#define MODEM_CELLULAR_CGDCONT_CMD_LEN       (96)
#define MODEM_CELLULAR_SIMCOM_DIAL_CHAT_COUNT (5)
#define MODEM_CELLULAR_CNAOP_CMD_LEN         (48)
#define MODEM_CELLULAR_CNMP_CMD_LEN          (24)
#define MODEM_CELLULAR_CPSI_PLMN_LEN         (7)
#define MODEM_CELLULAR_CONTROL_COMMAND_LEN   SIMCOM7X00_CONTROL_COMMAND_MAX_LEN
#define MODEM_CELLULAR_CONTROL_FAILURE_LEN   SIMCOM7X00_CONTROL_FAILURE_MAX_LEN
#define MODEM_CELLULAR_PLMN_CACHE_SIZE       (4)

#define MODEM_CELLULAR_RESERVED_DLCIS        (2)

/* Magic constants */
#define CSQ_RSSI_UNKNOWN		     (99)
#define CESQ_RSRP_UNKNOWN		     (255)
#define CESQ_RSRQ_UNKNOWN		     (255)

/* Magic numbers to units conversions */
#define CSQ_RSSI_TO_DB(v) (-113 + (2 * (rssi)))
#define CESQ_RSRP_TO_DB(v) (-140 + (v))
#define CESQ_RSRQ_TO_DB(v) (-20 + ((v) / 2))

enum modem_cellular_state {
	MODEM_CELLULAR_STATE_IDLE = 0,
	MODEM_CELLULAR_STATE_RESET_PULSE,
	MODEM_CELLULAR_STATE_POWER_ON_PULSE,
	MODEM_CELLULAR_STATE_AWAIT_POWER_ON,
	MODEM_CELLULAR_STATE_RUN_INIT_SCRIPT,
	MODEM_CELLULAR_STATE_CONNECT_CMUX,
	MODEM_CELLULAR_STATE_OPEN_DLCI1,
	MODEM_CELLULAR_STATE_OPEN_DLCI2,
	MODEM_CELLULAR_STATE_READY,
	MODEM_CELLULAR_STATE_RUN_DIAL_SCRIPT,
	MODEM_CELLULAR_STATE_AWAIT_REGISTERED,
	MODEM_CELLULAR_STATE_CARRIER_ON,
	MODEM_CELLULAR_STATE_INIT_POWER_OFF,
	MODEM_CELLULAR_STATE_POWER_OFF_PULSE,
	MODEM_CELLULAR_STATE_AWAIT_POWER_OFF,
};

enum modem_cellular_event {
	MODEM_CELLULAR_EVENT_RESUME = 0,
	MODEM_CELLULAR_EVENT_SUSPEND,
	MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS,
	MODEM_CELLULAR_EVENT_SCRIPT_FAILED,
	MODEM_CELLULAR_EVENT_CMUX_CONNECTED,
	MODEM_CELLULAR_EVENT_DLCI1_OPENED,
	MODEM_CELLULAR_EVENT_DLCI2_OPENED,
	MODEM_CELLULAR_EVENT_TIMEOUT,
	MODEM_CELLULAR_EVENT_REGISTERED,
	MODEM_CELLULAR_EVENT_DEREGISTERED,
	MODEM_CELLULAR_EVENT_BUS_OPENED,
	MODEM_CELLULAR_EVENT_BUS_CLOSED,
	MODEM_CELLULAR_EVENT_PERIODIC_SCRIPT_INTERVAL_CHANGED,
	MODEM_CELLULAR_EVENT_PPP_CONNECT_REQUEST,
	MODEM_CELLULAR_EVENT_PPP_DISCONNECT_REQUEST,
};

enum modem_cellular_control_state {
	MODEM_CELLULAR_CONTROL_STATE_IDLE = 0,
	MODEM_CELLULAR_CONTROL_STATE_BUSY_SYNC,
	MODEM_CELLULAR_CONTROL_STATE_BUSY_ASYNC,
};

struct modem_cellular_operator_cache_entry {
	char plmn[MODEM_CELLULAR_DATA_OPERATOR_PLMN_LEN + 1];
	char long_name[MODEM_CELLULAR_DATA_OPERATOR_LONG_LEN + 1];
	char short_name[MODEM_CELLULAR_DATA_OPERATOR_SHORT_LEN + 1];
	bool valid;
};

struct modem_cellular_data {
	/* UART backend */
	struct modem_pipe *uart_pipe;
	struct modem_backend_uart uart_backend;
	uint8_t uart_backend_receive_buf[CONFIG_MODEM_SIMCOM7X00_UART_BUFFER_SIZES];
	uint8_t uart_backend_transmit_buf[CONFIG_MODEM_SIMCOM7X00_UART_BUFFER_SIZES];

	/* CMUX */
	struct modem_cmux cmux;
	uint8_t cmux_receive_buf[CONFIG_MODEM_SIMCOM7X00_CMUX_MAX_FRAME_SIZE];
	uint8_t cmux_transmit_buf[2 * CONFIG_MODEM_SIMCOM7X00_CMUX_MAX_FRAME_SIZE];

	struct modem_cmux_dlci dlci1;
	struct modem_cmux_dlci dlci2;
	struct modem_pipe *dlci1_pipe;
	struct modem_pipe *dlci2_pipe;
	uint8_t dlci1_receive_buf[CONFIG_MODEM_SIMCOM7X00_CMUX_MAX_FRAME_SIZE];
	/* DLCI 2 is only used for chat scripts. */
	uint8_t dlci2_receive_buf[CONFIG_MODEM_SIMCOM7X00_CHAT_BUFFER_SIZES];

	/* Modem chat */
	struct modem_chat chat;
	uint8_t chat_receive_buf[CONFIG_MODEM_SIMCOM7X00_CHAT_BUFFER_SIZES];
	uint8_t *chat_delimiter;
	uint8_t *chat_filter;
	uint8_t *chat_argv[32];

	/* Status */
	enum cellular_registration_status registration_status_gsm;
	enum cellular_registration_status registration_status_gprs;
	enum cellular_registration_status registration_status_lte;
	uint8_t rssi;
	uint8_t rsrp;
	uint8_t rsrq;
	uint8_t imei[MODEM_CELLULAR_DATA_IMEI_LEN];
	uint8_t model_id[MODEM_CELLULAR_DATA_MODEL_ID_LEN];
	uint8_t imsi[MODEM_CELLULAR_DATA_IMSI_LEN];
	uint8_t iccid[MODEM_CELLULAR_DATA_ICCID_LEN];
	uint8_t manufacturer[MODEM_CELLULAR_DATA_MANUFACTURER_LEN];
	uint8_t fw_version[MODEM_CELLULAR_DATA_FW_VERSION_LEN];
	char operator_long[MODEM_CELLULAR_DATA_OPERATOR_LONG_LEN + 1];
	char operator_short[MODEM_CELLULAR_DATA_OPERATOR_SHORT_LEN + 1];
	char operator_plmn[MODEM_CELLULAR_DATA_OPERATOR_PLMN_LEN + 1];
	enum simcom7x00_rat current_rat;
	enum cellular_registration_status current_registration_status;
	bool roaming;
	bool operator_valid;
	bool current_rat_valid;

	/* PPP */
	struct modem_ppp *ppp;

	enum modem_cellular_state state;
	const struct device *dev;
	struct k_work_delayable timeout_work;

	/* Power management */
	struct k_sem suspended_sem;

	/* Event dispatcher */
	struct k_work event_dispatch_work;
	uint8_t event_buf[8];
	struct ring_buf event_rb;
	struct k_mutex event_rb_lock;
	atomic_t periodic_script_interval_ms;
	struct k_sem ready_sem;
	bool ppp_requested;
	bool force_ready_only_on_resume;

	struct k_mutex apn_lock;
	char apn[MODEM_CELLULAR_DATA_APN_LEN + 1];
	size_t apn_len;
	char staged_apn_command[MODEM_CELLULAR_CGDCONT_CMD_LEN];
	struct modem_chat_script_chat simcom_dial_script_chats[MODEM_CELLULAR_SIMCOM_DIAL_CHAT_COUNT];
	struct modem_chat_script simcom_dial_script;
	struct k_mutex network_lock;
	struct k_mutex control_lock;
	struct k_mutex diagnostics_lock;
	enum modem_cellular_control_state control_state;
	enum simcom7x00_rat_mode rat_mode;
	struct simcom7x00_rat_policy rat_policy;
	bool rat_policy_enabled;
	bool rat_config_dirty;
	char staged_cnmp_command[MODEM_CELLULAR_CNMP_CMD_LEN];
	char staged_cnaop_command[MODEM_CELLULAR_CNAOP_CMD_LEN];
	char operator_lookup_plmn[MODEM_CELLULAR_DATA_OPERATOR_PLMN_LEN + 1];
	struct modem_cellular_operator_cache_entry operator_cache[MODEM_CELLULAR_PLMN_CACHE_SIZE];
	uint8_t operator_cache_next;
	char last_control_command[MODEM_CELLULAR_CONTROL_COMMAND_LEN + 1];
	char active_control_script[MODEM_CELLULAR_CONTROL_COMMAND_LEN + 1];
	char last_control_failure[MODEM_CELLULAR_CONTROL_FAILURE_LEN + 1];
	uint32_t control_transaction_id;
	uint32_t control_completed_transaction_id;
	enum simcom7x00_control_result control_last_result;
	bool control_transaction_in_progress;
	int last_cme_error;
	int last_cms_error;
	bool has_last_cme_error;
	bool has_last_cms_error;
	bool operator_lookup_found;
};

struct modem_cellular_user_pipe {
	struct modem_cmux_dlci dlci;
	uint8_t dlci_address;
	uint8_t *dlci_receive_buf;
	uint16_t dlci_receive_buf_size;
	struct modem_pipe *pipe;
	struct modem_pipelink *pipelink;
};

struct modem_cellular_config {
	const struct device *uart;
	struct gpio_dt_spec power_gpio;
	struct gpio_dt_spec reset_gpio;
	uint16_t power_pulse_duration_ms;
	uint16_t reset_pulse_duration_ms;
	uint16_t startup_time_ms;
	uint16_t shutdown_time_ms;
	bool autostarts;
	bool default_ppp_autostart;
	bool runtime_apn;
	const struct modem_chat_script *init_chat_script;
	const struct modem_chat_script *dial_chat_script;
	const struct modem_chat_script *periodic_chat_script;
	struct modem_cellular_user_pipe *user_pipes;
	uint8_t user_pipes_size;
};

static const char *modem_cellular_state_str(enum modem_cellular_state state)
{
	switch (state) {
	case MODEM_CELLULAR_STATE_IDLE:
		return "idle";
	case MODEM_CELLULAR_STATE_RESET_PULSE:
		return "reset pulse";
	case MODEM_CELLULAR_STATE_POWER_ON_PULSE:
		return "power pulse";
	case MODEM_CELLULAR_STATE_AWAIT_POWER_ON:
		return "await power on";
	case MODEM_CELLULAR_STATE_RUN_INIT_SCRIPT:
		return "run init script";
	case MODEM_CELLULAR_STATE_CONNECT_CMUX:
		return "connect cmux";
	case MODEM_CELLULAR_STATE_OPEN_DLCI1:
		return "open dlci1";
	case MODEM_CELLULAR_STATE_OPEN_DLCI2:
		return "open dlci2";
	case MODEM_CELLULAR_STATE_READY:
		return "ready";
	case MODEM_CELLULAR_STATE_AWAIT_REGISTERED:
		return "await registered";
	case MODEM_CELLULAR_STATE_RUN_DIAL_SCRIPT:
		return "run dial script";
	case MODEM_CELLULAR_STATE_CARRIER_ON:
		return "carrier on";
	case MODEM_CELLULAR_STATE_INIT_POWER_OFF:
		return "init power off";
	case MODEM_CELLULAR_STATE_POWER_OFF_PULSE:
		return "power off pulse";
	case MODEM_CELLULAR_STATE_AWAIT_POWER_OFF:
		return "await power off";
	}

	return "";
}

static const char *modem_cellular_event_str(enum modem_cellular_event event)
{
	switch (event) {
	case MODEM_CELLULAR_EVENT_RESUME:
		return "resume";
	case MODEM_CELLULAR_EVENT_SUSPEND:
		return "suspend";
	case MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS:
		return "script success";
	case MODEM_CELLULAR_EVENT_SCRIPT_FAILED:
		return "script failed";
	case MODEM_CELLULAR_EVENT_CMUX_CONNECTED:
		return "cmux connected";
	case MODEM_CELLULAR_EVENT_DLCI1_OPENED:
		return "dlci1 opened";
	case MODEM_CELLULAR_EVENT_DLCI2_OPENED:
		return "dlci2 opened";
	case MODEM_CELLULAR_EVENT_TIMEOUT:
		return "timeout";
	case MODEM_CELLULAR_EVENT_REGISTERED:
		return "registered";
	case MODEM_CELLULAR_EVENT_DEREGISTERED:
		return "deregistered";
	case MODEM_CELLULAR_EVENT_BUS_OPENED:
		return "bus opened";
	case MODEM_CELLULAR_EVENT_BUS_CLOSED:
		return "bus closed";
	case MODEM_CELLULAR_EVENT_PERIODIC_SCRIPT_INTERVAL_CHANGED:
		return "periodic script interval changed";
	case MODEM_CELLULAR_EVENT_PPP_CONNECT_REQUEST:
		return "ppp connect request";
	case MODEM_CELLULAR_EVENT_PPP_DISCONNECT_REQUEST:
		return "ppp disconnect request";
	}

	return "";
}

static bool modem_cellular_gpio_is_enabled(const struct gpio_dt_spec *gpio)
{
	return gpio->port != NULL;
}

static bool modem_cellular_state_is_ready(enum modem_cellular_state state)
{
	return (state == MODEM_CELLULAR_STATE_READY) ||
	       (state == MODEM_CELLULAR_STATE_RUN_DIAL_SCRIPT) ||
	       (state == MODEM_CELLULAR_STATE_AWAIT_REGISTERED) ||
	       (state == MODEM_CELLULAR_STATE_CARRIER_ON);
}

static void modem_cellular_copy_stripped(char *dst, size_t dst_size, const char *src)
{
	size_t len;

	if ((dst == NULL) || (dst_size == 0U)) {
		return;
	}

	dst[0] = '\0';
	if (src == NULL) {
		return;
	}

	while ((*src == ' ') || (*src == '"')) {
		src++;
	}

	len = strlen(src);
	while ((len > 0U) &&
	       ((src[len - 1] == ' ') || (src[len - 1] == '"') || (src[len - 1] == '\r') ||
		(src[len - 1] == '\n'))) {
		len--;
	}

	len = MIN(len, dst_size - 1U);
	memcpy(dst, src, len);
	dst[len] = '\0';
}

static void modem_cellular_copy_digits(char *dst, size_t dst_size, const char *src)
{
	size_t len = 0U;

	if ((dst == NULL) || (dst_size == 0U)) {
		return;
	}

	dst[0] = '\0';
	if (src == NULL) {
		return;
	}

	while ((*src != '\0') && (len < (dst_size - 1U))) {
		if ((*src >= '0') && (*src <= '9')) {
			dst[len++] = *src;
		}
		src++;
	}

	dst[len] = '\0';
}

static void modem_cellular_set_control_command(struct modem_cellular_data *data, const char *cmd)
{
	k_mutex_lock(&data->diagnostics_lock, K_FOREVER);
	modem_cellular_copy_stripped(data->last_control_command, sizeof(data->last_control_command),
				     cmd);
	k_mutex_unlock(&data->diagnostics_lock);
}

static void modem_cellular_set_active_script(struct modem_cellular_data *data, const char *name)
{
	k_mutex_lock(&data->diagnostics_lock, K_FOREVER);
	modem_cellular_copy_stripped(data->active_control_script, sizeof(data->active_control_script),
				     name);
	k_mutex_unlock(&data->diagnostics_lock);
}

static void modem_cellular_set_control_failure(struct modem_cellular_data *data, const char *reason)
{
	k_mutex_lock(&data->diagnostics_lock, K_FOREVER);
	modem_cellular_copy_stripped(data->last_control_failure, sizeof(data->last_control_failure),
				     reason);
	k_mutex_unlock(&data->diagnostics_lock);
}

static void modem_cellular_begin_control_transaction(struct modem_cellular_data *data)
{
	k_mutex_lock(&data->diagnostics_lock, K_FOREVER);
	data->control_transaction_id++;
	data->control_transaction_in_progress = true;
	data->control_last_result = SIMCOM7X00_CONTROL_RESULT_NONE;
	data->last_control_failure[0] = '\0';
	data->last_cme_error = 0;
	data->last_cms_error = 0;
	data->has_last_cme_error = false;
	data->has_last_cms_error = false;
	k_mutex_unlock(&data->diagnostics_lock);
}

static void modem_cellular_finish_control_transaction(struct modem_cellular_data *data,
						      enum simcom7x00_control_result result)
{
	k_mutex_lock(&data->diagnostics_lock, K_FOREVER);
	data->control_completed_transaction_id = data->control_transaction_id;
	data->control_transaction_in_progress = false;
	data->control_last_result = result;
	if (result == SIMCOM7X00_CONTROL_RESULT_SUCCESS) {
		data->last_control_failure[0] = '\0';
		data->last_cme_error = 0;
		data->last_cms_error = 0;
		data->has_last_cme_error = false;
		data->has_last_cms_error = false;
	}
	k_mutex_unlock(&data->diagnostics_lock);
}

static void modem_cellular_format_control_failure(struct modem_cellular_data *data,
						  const char *prefix, int code)
{
	char failure[MODEM_CELLULAR_CONTROL_FAILURE_LEN + 1];
	int ret;

	ret = snprintf(failure, sizeof(failure), "%s: %d", prefix, code);
	if ((ret < 0) || (ret >= (int)sizeof(failure))) {
		failure[0] = '\0';
		modem_cellular_copy_stripped(failure, sizeof(failure), prefix);
	}

	modem_cellular_set_control_failure(data, failure);
}

static void modem_cellular_set_control_script_failure(struct modem_cellular_data *data,
						      const char *suffix)
{
	char script_name[MODEM_CELLULAR_CONTROL_COMMAND_LEN + 1];
	char failure[MODEM_CELLULAR_CONTROL_FAILURE_LEN + 1];
	int ret;

	k_mutex_lock(&data->diagnostics_lock, K_FOREVER);
	memcpy(script_name, data->active_control_script, sizeof(script_name));
	k_mutex_unlock(&data->diagnostics_lock);

	if (script_name[0] == '\0') {
		modem_cellular_copy_stripped(script_name, sizeof(script_name), "control script");
	}

	ret = snprintf(failure, sizeof(failure), "%s %s", script_name, suffix);
	if ((ret < 0) || (ret >= (int)sizeof(failure))) {
		modem_cellular_copy_stripped(failure, sizeof(failure), script_name);
	}

	modem_cellular_set_control_failure(data, failure);
}

static int modem_cellular_control_acquire(struct modem_cellular_data *data,
					  enum modem_cellular_control_state state)
{
	int ret = 0;

	k_mutex_lock(&data->control_lock, K_FOREVER);
	if (data->control_state != MODEM_CELLULAR_CONTROL_STATE_IDLE) {
		ret = -EBUSY;
	} else {
		data->control_state = state;
		modem_cellular_begin_control_transaction(data);
	}
	k_mutex_unlock(&data->control_lock);

	if (ret < 0) {
		LOG_DBG("control plane busy for state %d", state);
		modem_cellular_set_control_failure(data, "control plane busy");
		k_mutex_lock(&data->diagnostics_lock, K_FOREVER);
		data->control_last_result = SIMCOM7X00_CONTROL_RESULT_BUSY;
		k_mutex_unlock(&data->diagnostics_lock);
	}

	return ret;
}

static void modem_cellular_control_release(struct modem_cellular_data *data,
					   enum simcom7x00_control_result result)
{
	bool finish = false;

	k_mutex_lock(&data->control_lock, K_FOREVER);
	if (data->control_state != MODEM_CELLULAR_CONTROL_STATE_IDLE) {
		data->control_state = MODEM_CELLULAR_CONTROL_STATE_IDLE;
		finish = true;
	}
	k_mutex_unlock(&data->control_lock);

	if (finish) {
		modem_cellular_finish_control_transaction(data, result);
	}
}

static void modem_cellular_track_script_request(struct modem_cellular_data *data,
						const struct modem_chat_script *script)
{
	if ((script == NULL) || (script->script_chats == NULL)) {
		return;
	}

	modem_cellular_set_active_script(data, script->name);

	for (uint16_t i = 0; i < script->script_chats_size; i++) {
		const struct modem_chat_script_chat *chat = &script->script_chats[i];

		if ((chat->request == NULL) || (chat->request_size == 0U) ||
		    (((const char *)chat->request)[0] == '\0')) {
			continue;
		}

		modem_cellular_set_control_command(data, (const char *)chat->request);
		return;
	}
}

static struct modem_cellular_operator_cache_entry *modem_cellular_find_cached_operator(
	struct modem_cellular_data *data, const char *plmn)
{
	for (size_t i = 0; i < ARRAY_SIZE(data->operator_cache); i++) {
		if (data->operator_cache[i].valid && (strcmp(data->operator_cache[i].plmn, plmn) == 0)) {
			return &data->operator_cache[i];
		}
	}

	return NULL;
}

static bool modem_cellular_apply_cached_operator(struct modem_cellular_data *data, const char *plmn)
{
	struct modem_cellular_operator_cache_entry *entry;

	k_mutex_lock(&data->network_lock, K_FOREVER);
	entry = modem_cellular_find_cached_operator(data, plmn);
	if (entry != NULL) {
		memcpy(data->operator_long, entry->long_name, sizeof(data->operator_long));
		memcpy(data->operator_short, entry->short_name, sizeof(data->operator_short));
		k_mutex_unlock(&data->network_lock);
		return true;
	}
	k_mutex_unlock(&data->network_lock);

	return false;
}

static void modem_cellular_cache_operator_name(struct modem_cellular_data *data, const char *plmn,
					       const char *long_name, const char *short_name)
{
	struct modem_cellular_operator_cache_entry *entry;

	k_mutex_lock(&data->network_lock, K_FOREVER);
	entry = modem_cellular_find_cached_operator(data, plmn);
	if (entry == NULL) {
		entry = &data->operator_cache[data->operator_cache_next];
		data->operator_cache_next =
			(uint8_t)((data->operator_cache_next + 1U) % ARRAY_SIZE(data->operator_cache));
	}

	modem_cellular_copy_stripped(entry->plmn, sizeof(entry->plmn), plmn);
	modem_cellular_copy_stripped(entry->long_name, sizeof(entry->long_name), long_name);
	modem_cellular_copy_stripped(entry->short_name, sizeof(entry->short_name), short_name);
	entry->valid = (entry->plmn[0] != '\0') && (entry->long_name[0] != '\0');
	k_mutex_unlock(&data->network_lock);
}

static enum simcom7x00_rat modem_cellular_cnsmod_to_rat(uint8_t stat)
{
	switch (stat) {
	case 1:
	case 2:
		return SIMCOM7X00_RAT_GSM;
	case 3:
		return SIMCOM7X00_RAT_EDGE;
	case 4:
	case 5:
	case 6:
	case 7:
		return SIMCOM7X00_RAT_UMTS;
	case 8:
		return SIMCOM7X00_RAT_LTE;
	default:
		return SIMCOM7X00_RAT_UNKNOWN;
	}
}

static enum cellular_registration_status modem_cellular_get_current_registration_status(
	const struct modem_cellular_data *data)
{
	switch (data->current_rat) {
	case SIMCOM7X00_RAT_GSM:
		return data->registration_status_gsm;
	case SIMCOM7X00_RAT_EDGE:
	case SIMCOM7X00_RAT_UMTS:
		return data->registration_status_gprs;
	case SIMCOM7X00_RAT_LTE:
		return data->registration_status_lte;
	default:
		break;
	}

	if (data->registration_status_lte != CELLULAR_REGISTRATION_UNKNOWN) {
		return data->registration_status_lte;
	}

	if (data->registration_status_gprs != CELLULAR_REGISTRATION_UNKNOWN) {
		return data->registration_status_gprs;
	}

	return data->registration_status_gsm;
}

static void modem_cellular_update_registration_snapshot(struct modem_cellular_data *data)
{
	data->current_registration_status = modem_cellular_get_current_registration_status(data);
	data->roaming = (data->current_registration_status ==
			 CELLULAR_REGISTRATION_REGISTERED_ROAMING);
}

static int modem_cellular_run_script_locked(struct modem_cellular_data *data,
					    const struct modem_chat_script *script)
{
	int ret;

	ret = modem_cellular_control_acquire(data, MODEM_CELLULAR_CONTROL_STATE_BUSY_SYNC);
	if (ret < 0) {
		return ret;
	}

	modem_cellular_track_script_request(data, script);
	ret = modem_chat_run_script(&data->chat, script);
	if (ret < 0) {
		modem_cellular_format_control_failure(data, "script start failed", ret);
		modem_cellular_control_release(data, SIMCOM7X00_CONTROL_RESULT_FAILED);
		return ret;
	}

	modem_cellular_control_release(data, SIMCOM7X00_CONTROL_RESULT_SUCCESS);
	return ret;
}

static int modem_cellular_run_script_async_locked(struct modem_cellular_data *data,
						  const struct modem_chat_script *script)
{
	int ret;

	ret = modem_cellular_control_acquire(data, MODEM_CELLULAR_CONTROL_STATE_BUSY_ASYNC);
	if (ret < 0) {
		return ret;
	}

	modem_cellular_track_script_request(data, script);
	ret = modem_chat_run_script_async(&data->chat, script);
	if (ret < 0) {
		modem_cellular_format_control_failure(data, "script async start failed", ret);
		modem_cellular_control_release(data, SIMCOM7X00_CONTROL_RESULT_FAILED);
	}

	return ret;
}

static int modem_cellular_attach_chat_locked(struct modem_cellular_data *data,
					     struct modem_pipe *pipe)
{
	int ret;

	k_mutex_lock(&data->control_lock, K_FOREVER);
	ret = modem_chat_attach(&data->chat, pipe);
	k_mutex_unlock(&data->control_lock);
	if (ret < 0) {
		modem_cellular_format_control_failure(data, "chat attach failed", ret);
	}

	return ret;
}

static void modem_cellular_release_chat_locked(struct modem_cellular_data *data)
{
	k_mutex_lock(&data->control_lock, K_FOREVER);
	if (data->control_state == MODEM_CELLULAR_CONTROL_STATE_BUSY_ASYNC) {
		data->control_state = MODEM_CELLULAR_CONTROL_STATE_IDLE;
		k_mutex_unlock(&data->control_lock);
		modem_cellular_set_control_failure(data, "control script cancelled");
		modem_cellular_finish_control_transaction(data, SIMCOM7X00_CONTROL_RESULT_FAILED);
		k_mutex_lock(&data->control_lock, K_FOREVER);
	}
	modem_chat_release(&data->chat);
	k_mutex_unlock(&data->control_lock);
}

static int modem_cellular_rat_to_sys_mode(enum simcom7x00_rat rat)
{
	switch (rat) {
	case SIMCOM7X00_RAT_GSM:
	case SIMCOM7X00_RAT_EDGE:
		return 3;
	case SIMCOM7X00_RAT_UMTS:
		return 5;
	case SIMCOM7X00_RAT_LTE:
		return 9;
	default:
		return -EINVAL;
	}
}

static int modem_cellular_mode_to_cnmp(enum simcom7x00_rat_mode mode)
{
	switch (mode) {
	case SIMCOM7X00_RAT_MODE_AUTO:
		return 2;
	case SIMCOM7X00_RAT_MODE_2G_ONLY:
		return 13;
	case SIMCOM7X00_RAT_MODE_3G_ONLY:
		return 14;
	case SIMCOM7X00_RAT_MODE_4G_ONLY:
		return 38;
	default:
		return -EINVAL;
	}
}

static int modem_cellular_policy_to_cnmp(const struct simcom7x00_rat_policy *policy)
{
	bool has_gsm = false;
	bool has_umts = false;
	bool has_lte = false;

	if ((policy == NULL) || (policy->size == 0U) ||
	    (policy->size > SIMCOM7X00_RAT_POLICY_MAX_PRIORITIES)) {
		return -EINVAL;
	}

	for (uint8_t i = 0; i < policy->size; i++) {
		switch (policy->priorities[i]) {
		case SIMCOM7X00_RAT_GSM:
		case SIMCOM7X00_RAT_EDGE:
			has_gsm = true;
			break;
		case SIMCOM7X00_RAT_UMTS:
			has_umts = true;
			break;
		case SIMCOM7X00_RAT_LTE:
			has_lte = true;
			break;
		default:
			return -EINVAL;
		}
	}

	if (has_gsm && has_umts && has_lte) {
		return 39;
	}

	if (has_gsm && has_umts) {
		return 19;
	}

	if (has_gsm && has_lte) {
		return 51;
	}

	if (has_umts && has_lte) {
		return 54;
	}

	if (has_lte) {
		return 38;
	}

	if (has_umts) {
		return 14;
	}

	if (has_gsm) {
		return 13;
	}

	return -EINVAL;
}

static enum simcom7x00_rat_mode modem_cellular_cnmp_to_mode(int cnmp)
{
	switch (cnmp) {
	case 13:
		return SIMCOM7X00_RAT_MODE_2G_ONLY;
	case 14:
		return SIMCOM7X00_RAT_MODE_3G_ONLY;
	case 38:
		return SIMCOM7X00_RAT_MODE_4G_ONLY;
	default:
		return SIMCOM7X00_RAT_MODE_AUTO;
	}
}

static int modem_cellular_validate_rat_policy(const struct simcom7x00_rat_policy *policy)
{
	int sys_modes[SIMCOM7X00_RAT_POLICY_MAX_PRIORITIES];

	if ((policy == NULL) || (policy->size == 0U) ||
	    (policy->size > SIMCOM7X00_RAT_POLICY_MAX_PRIORITIES)) {
		return -EINVAL;
	}

	for (uint8_t i = 0; i < policy->size; i++) {
		int sys_mode = modem_cellular_rat_to_sys_mode(policy->priorities[i]);

		if (sys_mode < 0) {
			return -EINVAL;
		}

		for (uint8_t j = 0; j < i; j++) {
			if (sys_modes[j] == sys_mode) {
				return -EINVAL;
			}
		}

		sys_modes[i] = sys_mode;
	}

	return modem_cellular_policy_to_cnmp(policy);
}

static void modem_cellular_set_policy_for_mode(struct modem_cellular_data *data,
					       enum simcom7x00_rat_mode mode)
{
	memset(&data->rat_policy, 0, sizeof(data->rat_policy));

	switch (mode) {
	case SIMCOM7X00_RAT_MODE_AUTO:
		data->rat_policy.priorities[0] = SIMCOM7X00_RAT_LTE;
		data->rat_policy.priorities[1] = SIMCOM7X00_RAT_UMTS;
		data->rat_policy.priorities[2] = SIMCOM7X00_RAT_GSM;
		data->rat_policy.size = 3;
		break;
	case SIMCOM7X00_RAT_MODE_2G_ONLY:
		data->rat_policy.priorities[0] = SIMCOM7X00_RAT_GSM;
		data->rat_policy.size = 1;
		break;
	case SIMCOM7X00_RAT_MODE_3G_ONLY:
		data->rat_policy.priorities[0] = SIMCOM7X00_RAT_UMTS;
		data->rat_policy.size = 1;
		break;
	case SIMCOM7X00_RAT_MODE_4G_ONLY:
		data->rat_policy.priorities[0] = SIMCOM7X00_RAT_LTE;
		data->rat_policy.size = 1;
		break;
	default:
		break;
	}
}

static void modem_cellular_notify_user_pipes_connected(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;
	struct modem_cellular_user_pipe *user_pipe;
	struct modem_pipelink *pipelink;

	for (uint8_t i = 0; i < config->user_pipes_size; i++) {
		user_pipe = &config->user_pipes[i];
		pipelink = user_pipe->pipelink;
		modem_pipelink_notify_connected(pipelink);
	}
}

static void modem_cellular_notify_user_pipes_disconnected(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;
	struct modem_cellular_user_pipe *user_pipe;
	struct modem_pipelink *pipelink;

	for (uint8_t i = 0; i < config->user_pipes_size; i++) {
		user_pipe = &config->user_pipes[i];
		pipelink = user_pipe->pipelink;
		modem_pipelink_notify_disconnected(pipelink);
	}
}

static void modem_cellular_enter_state(struct modem_cellular_data *data,
				       enum modem_cellular_state state);
static enum simcom7x00_rat_mode modem_cellular_cnmp_to_mode(int cnmp);
static void modem_cellular_set_policy_for_mode(struct modem_cellular_data *data,
					       enum simcom7x00_rat_mode mode);
static int modem_cellular_refresh_network_snapshot(struct modem_cellular_data *data);

static void modem_cellular_delegate_event(struct modem_cellular_data *data,
					  enum modem_cellular_event evt);

static void modem_cellular_event_handler(struct modem_cellular_data *data,
					 enum modem_cellular_event evt);

static void modem_cellular_bus_pipe_handler(struct modem_pipe *pipe,
					    enum modem_pipe_event event,
					    void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	switch (event) {
	case MODEM_PIPE_EVENT_OPENED:
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_BUS_OPENED);
		break;

	case MODEM_PIPE_EVENT_CLOSED:
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_BUS_CLOSED);
		break;

	default:
		break;
	}
}

static void modem_cellular_dlci1_pipe_handler(struct modem_pipe *pipe,
					      enum modem_pipe_event event,
					      void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	switch (event) {
	case MODEM_PIPE_EVENT_OPENED:
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_DLCI1_OPENED);
		break;

	default:
		break;
	}
}

static void modem_cellular_dlci2_pipe_handler(struct modem_pipe *pipe,
					      enum modem_pipe_event event,
					      void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	switch (event) {
	case MODEM_PIPE_EVENT_OPENED:
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_DLCI2_OPENED);
		break;

	default:
		break;
	}
}

static void modem_cellular_chat_callback_handler(struct modem_chat *chat,
						 enum modem_chat_script_result result,
						 void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (result == MODEM_CHAT_SCRIPT_RESULT_SUCCESS) {
		modem_cellular_control_release(data, SIMCOM7X00_CONTROL_RESULT_SUCCESS);
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS);
	} else if (result == MODEM_CHAT_SCRIPT_RESULT_TIMEOUT) {
		modem_cellular_set_control_script_failure(data, "timed out");
		modem_cellular_control_release(data, SIMCOM7X00_CONTROL_RESULT_TIMEOUT);
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_SCRIPT_FAILED);
	} else {
		modem_cellular_set_control_script_failure(data, "aborted");
		modem_cellular_control_release(data, SIMCOM7X00_CONTROL_RESULT_FAILED);
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_SCRIPT_FAILED);
	}
}

static void modem_cellular_chat_on_error(struct modem_chat *chat, char **argv, uint16_t argc,
					 void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	ARG_UNUSED(chat);
	ARG_UNUSED(argv);
	ARG_UNUSED(argc);

	modem_cellular_set_control_failure(data, "generic ERROR");
}

static void modem_cellular_chat_on_cme_error(struct modem_chat *chat, char **argv, uint16_t argc,
					     void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	ARG_UNUSED(chat);

	if (argc != 2) {
		return;
	}

	k_mutex_lock(&data->diagnostics_lock, K_FOREVER);
	data->last_cme_error = atoi(argv[1]);
	data->has_last_cme_error = true;
	k_mutex_unlock(&data->diagnostics_lock);
	modem_cellular_format_control_failure(data, "CME ERROR", atoi(argv[1]));
}

static void modem_cellular_chat_on_cms_error(struct modem_chat *chat, char **argv, uint16_t argc,
					     void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	ARG_UNUSED(chat);

	if (argc != 2) {
		return;
	}

	k_mutex_lock(&data->diagnostics_lock, K_FOREVER);
	data->last_cms_error = atoi(argv[1]);
	data->has_last_cms_error = true;
	k_mutex_unlock(&data->diagnostics_lock);
	modem_cellular_format_control_failure(data, "CMS ERROR", atoi(argv[1]));
}

static void modem_cellular_chat_on_imei(struct modem_chat *chat, char **argv, uint16_t argc,
					void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (argc != 2) {
		return;
	}

	strncpy(data->imei, argv[1], sizeof(data->imei) - 1);
}

static void modem_cellular_chat_on_cgmm(struct modem_chat *chat, char **argv, uint16_t argc,
					void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (argc != 2) {
		return;
	}

	strncpy(data->model_id, argv[1], sizeof(data->model_id) - 1);
}

static void modem_cellular_chat_on_cgmi(struct modem_chat *chat, char **argv, uint16_t argc,
					void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (argc != 2) {
		return;
	}

	strncpy(data->manufacturer, argv[1], sizeof(data->manufacturer) - 1);
}

static void modem_cellular_chat_on_cgmr(struct modem_chat *chat, char **argv, uint16_t argc,
					void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (argc != 2) {
		return;
	}

	strncpy(data->fw_version, argv[1], sizeof(data->fw_version) - 1);
}

static void modem_cellular_chat_on_csq(struct modem_chat *chat, char **argv, uint16_t argc,
				       void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (argc != 3) {
		return;
	}

	data->rssi = (uint8_t)atoi(argv[1]);
}

static void modem_cellular_chat_on_cesq(struct modem_chat *chat, char **argv, uint16_t argc,
					void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (argc != 7) {
		return;
	}

	data->rsrq = (uint8_t)atoi(argv[5]);
	data->rsrp = (uint8_t)atoi(argv[6]);
}

static void modem_cellular_chat_on_iccid(struct modem_chat *chat, char **argv, uint16_t argc,
					void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (argc != 2) {
		return;
	}

	strncpy(data->iccid, argv[1], sizeof(data->iccid) - 1);
}

static void modem_cellular_chat_on_imsi(struct modem_chat *chat, char **argv, uint16_t argc,
					void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (argc != 2) {
		return;
	}

	strncpy(data->imsi, argv[1], sizeof(data->imsi) - 1);
}

static bool modem_cellular_is_registered(struct modem_cellular_data *data)
{
	return (data->registration_status_gsm == CELLULAR_REGISTRATION_REGISTERED_HOME)
		|| (data->registration_status_gsm == CELLULAR_REGISTRATION_REGISTERED_ROAMING)
		|| (data->registration_status_gprs == CELLULAR_REGISTRATION_REGISTERED_HOME)
		|| (data->registration_status_gprs == CELLULAR_REGISTRATION_REGISTERED_ROAMING)
		|| (data->registration_status_lte == CELLULAR_REGISTRATION_REGISTERED_HOME)
		|| (data->registration_status_lte == CELLULAR_REGISTRATION_REGISTERED_ROAMING);
}

static void modem_cellular_chat_on_cxreg(struct modem_chat *chat, char **argv, uint16_t argc,
					void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;
	enum cellular_registration_status registration_status = 0;

	if (argc == 2) {
		registration_status = atoi(argv[1]);
	} else if (argc == 3 || argc == 6) {
		registration_status = atoi(argv[2]);
	} else {
		return;
	}

	if (strcmp(argv[0], "+CREG: ") == 0) {
		data->registration_status_gsm = registration_status;
	} else if (strcmp(argv[0], "+CGREG: ") == 0) {
		data->registration_status_gprs = registration_status;
	} else {
		data->registration_status_lte = registration_status;
	}

	if (modem_cellular_is_registered(data)) {
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_REGISTERED);
	} else {
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_DEREGISTERED);
	}

	modem_cellular_update_registration_snapshot(data);
}

static void modem_cellular_chat_on_copn(struct modem_chat *chat, char **argv, uint16_t argc,
					void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;
	char plmn[MODEM_CELLULAR_DATA_OPERATOR_PLMN_LEN + 1];
	char operator_name[MODEM_CELLULAR_DATA_OPERATOR_LONG_LEN + 1];

	if (argc != 3) {
		return;
	}

	modem_cellular_copy_digits(plmn, sizeof(plmn), argv[1]);
	if (strcmp(plmn, data->operator_lookup_plmn) != 0) {
		return;
	}

	modem_cellular_copy_stripped(operator_name, sizeof(operator_name), argv[2]);
	k_mutex_lock(&data->network_lock, K_FOREVER);
	modem_cellular_copy_stripped(data->operator_long, sizeof(data->operator_long), operator_name);
	modem_cellular_copy_stripped(data->operator_short, sizeof(data->operator_short), operator_name);
	data->operator_lookup_found = true;
	k_mutex_unlock(&data->network_lock);
	modem_cellular_cache_operator_name(data, plmn, operator_name, operator_name);
}

static enum simcom7x00_rat modem_cellular_parse_cpsi_rat(const char *system_mode)
{
	if (system_mode == NULL) {
		return SIMCOM7X00_RAT_UNKNOWN;
	}

	if ((strncmp(system_mode, "LTE", 3) == 0) || (strncmp(system_mode, "CAT-M", 5) == 0) ||
	    (strncmp(system_mode, "NB-IOT", 6) == 0)) {
		return SIMCOM7X00_RAT_LTE;
	}

	if ((strncmp(system_mode, "WCDMA", 5) == 0) || (strncmp(system_mode, "HSDPA", 5) == 0) ||
	    (strncmp(system_mode, "HSUPA", 5) == 0) || (strncmp(system_mode, "HSPA", 4) == 0) ||
	    (strncmp(system_mode, "TD-SCDMA", 8) == 0)) {
		return SIMCOM7X00_RAT_UMTS;
	}

	if ((strncmp(system_mode, "EDGE", 4) == 0) || (strncmp(system_mode, "EGPRS", 5) == 0)) {
		return SIMCOM7X00_RAT_EDGE;
	}

	if (strncmp(system_mode, "GSM", 3) == 0) {
		return SIMCOM7X00_RAT_GSM;
	}

	return SIMCOM7X00_RAT_UNKNOWN;
}

static void modem_cellular_chat_on_cpsi(struct modem_chat *chat, char **argv, uint16_t argc,
					void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;
	enum simcom7x00_rat rat;

	if (argc < 4) {
		return;
	}

	rat = modem_cellular_parse_cpsi_rat(argv[1]);

	k_mutex_lock(&data->network_lock, K_FOREVER);
	if (rat != SIMCOM7X00_RAT_UNKNOWN) {
		data->current_rat = rat;
		data->current_rat_valid = true;
	}

	modem_cellular_copy_digits(data->operator_plmn, sizeof(data->operator_plmn), argv[3]);
	if (data->operator_plmn[0] != '\0') {
		data->operator_valid = true;
	}

	modem_cellular_update_registration_snapshot(data);
	k_mutex_unlock(&data->network_lock);
}

static void modem_cellular_chat_on_cnmp(struct modem_chat *chat, char **argv, uint16_t argc,
					void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	if (argc != 2) {
		return;
	}

	k_mutex_lock(&data->network_lock, K_FOREVER);
	data->rat_mode = modem_cellular_cnmp_to_mode(atoi(argv[1]));
	k_mutex_unlock(&data->network_lock);
}

static void modem_cellular_chat_on_cnaop(struct modem_chat *chat, char **argv, uint16_t argc,
					 void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;
	struct simcom7x00_rat_policy policy = { 0 };

	if (argc < 2) {
		return;
	}

	for (uint16_t i = 2; (i < argc) && (policy.size < SIMCOM7X00_RAT_POLICY_MAX_PRIORITIES); i++) {
		switch (atoi(argv[i])) {
		case 3:
			policy.priorities[policy.size++] = SIMCOM7X00_RAT_GSM;
			break;
		case 5:
			policy.priorities[policy.size++] = SIMCOM7X00_RAT_UMTS;
			break;
		case 9:
			policy.priorities[policy.size++] = SIMCOM7X00_RAT_LTE;
			break;
		default:
			break;
		}
	}

	k_mutex_lock(&data->network_lock, K_FOREVER);
	if (policy.size > 0U) {
		data->rat_policy = policy;
		data->rat_policy_enabled = true;
	} else {
		modem_cellular_set_policy_for_mode(data, data->rat_mode);
		data->rat_policy_enabled = false;
	}
	k_mutex_unlock(&data->network_lock);
}

static void modem_cellular_chat_on_cnsmod(struct modem_chat *chat, char **argv, uint16_t argc,
					  void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;
	enum simcom7x00_rat rat;

	if (argc != 3) {
		return;
	}

	rat = modem_cellular_cnsmod_to_rat((uint8_t)atoi(argv[2]));

	k_mutex_lock(&data->network_lock, K_FOREVER);
	data->current_rat = rat;
	data->current_rat_valid = (rat != SIMCOM7X00_RAT_UNKNOWN);
	modem_cellular_update_registration_snapshot(data);
	k_mutex_unlock(&data->network_lock);
}

MODEM_CHAT_MATCH_DEFINE(ok_match, "OK", "", NULL);
MODEM_CHAT_MATCHES_DEFINE(allow_match,
			  MODEM_CHAT_MATCH("OK", "", NULL),
			  MODEM_CHAT_MATCH("ERROR", "", NULL));

MODEM_CHAT_MATCH_DEFINE(imei_match, "", "", modem_cellular_chat_on_imei);
MODEM_CHAT_MATCH_DEFINE(cgmm_match, "", "", modem_cellular_chat_on_cgmm);
MODEM_CHAT_MATCH_DEFINE(csq_match, "+CSQ: ", ",", modem_cellular_chat_on_csq);
MODEM_CHAT_MATCH_DEFINE(cesq_match, "+CESQ: ", ",", modem_cellular_chat_on_cesq);
MODEM_CHAT_MATCH_DEFINE(qccid_match __maybe_unused, "+QCCID: ", "", modem_cellular_chat_on_iccid);
MODEM_CHAT_MATCH_DEFINE(iccid_match __maybe_unused, "+ICCID: ", "", modem_cellular_chat_on_iccid);
MODEM_CHAT_MATCH_DEFINE(cimi_match __maybe_unused, "", "", modem_cellular_chat_on_imsi);
MODEM_CHAT_MATCH_DEFINE(cgmi_match __maybe_unused, "", "", modem_cellular_chat_on_cgmi);
MODEM_CHAT_MATCH_DEFINE(cgmr_match __maybe_unused, "", "", modem_cellular_chat_on_cgmr);
MODEM_CHAT_MATCH_DEFINE(copn_match, "+COPN: ", ",", modem_cellular_chat_on_copn);
MODEM_CHAT_MATCH_DEFINE(cpsi_match, "+CPSI: ", ",", modem_cellular_chat_on_cpsi);
MODEM_CHAT_MATCH_DEFINE(cnmp_match, "+CNMP: ", "", modem_cellular_chat_on_cnmp);
MODEM_CHAT_MATCH_DEFINE(cnaop_match, "+CNAOP: ", ",", modem_cellular_chat_on_cnaop);
MODEM_CHAT_MATCH_DEFINE(cnsmod_match, "+CNSMOD: ", ",", modem_cellular_chat_on_cnsmod);
MODEM_CHAT_MATCH_DEFINE(cme_error_match, "+CME ERROR: ", "", modem_cellular_chat_on_cme_error);
MODEM_CHAT_MATCH_DEFINE(cms_error_match, "+CMS ERROR: ", "", modem_cellular_chat_on_cms_error);
MODEM_CHAT_MATCH_DEFINE(error_match, "ERROR", "", modem_cellular_chat_on_error);

MODEM_CHAT_MATCHES_DEFINE(unsol_matches,
			  MODEM_CHAT_MATCH("+CREG: ", ",", modem_cellular_chat_on_cxreg),
			  MODEM_CHAT_MATCH("+CEREG: ", ",", modem_cellular_chat_on_cxreg),
			  MODEM_CHAT_MATCH("+CGREG: ", ",", modem_cellular_chat_on_cxreg));

MODEM_CHAT_MATCHES_DEFINE(abort_matches, error_match, cme_error_match, cms_error_match);

MODEM_CHAT_MATCHES_DEFINE(dial_abort_matches,
			  error_match,
			  cme_error_match,
			  cms_error_match,
			  MODEM_CHAT_MATCH("BUSY", "", NULL),
			  MODEM_CHAT_MATCH("NO ANSWER", "", NULL),
			  MODEM_CHAT_MATCH("NO CARRIER", "", NULL),
			  MODEM_CHAT_MATCH("NO DIALTONE", "", NULL));

#if DT_HAS_COMPAT_STATUS_OKAY(swir_hl7800) || DT_HAS_COMPAT_STATUS_OKAY(sqn_gm02s)
MODEM_CHAT_MATCH_DEFINE(connect_match, "CONNECT", "", NULL);
#endif

static bool modem_cellular_apn_char_is_valid(char c)
{
	return (c >= 33) && (c <= 126) && (c != '"') && (c != '\\');
}

static int modem_cellular_validate_apn(const char *apn, size_t *apn_len)
{
	size_t len;

	if (apn == NULL) {
		return -EINVAL;
	}

	len = strlen(apn);
	if ((len == 0U) || (len > MODEM_CELLULAR_DATA_APN_LEN)) {
		return -EINVAL;
	}

	for (size_t i = 0; i < len; i++) {
		if (!modem_cellular_apn_char_is_valid(apn[i])) {
			return -EINVAL;
		}
	}

	if (apn_len != NULL) {
		*apn_len = len;
	}

	return 0;
}

static int modem_cellular_stage_apn_command(struct modem_cellular_data *data)
{
	int ret;

	k_mutex_lock(&data->apn_lock, K_FOREVER);
	ret = snprintf(data->staged_apn_command, sizeof(data->staged_apn_command),
		       "AT+CGDCONT=1,\"IP\",\"%s\"", data->apn);
	k_mutex_unlock(&data->apn_lock);

	if ((ret < 0) || (ret >= (int)sizeof(data->staged_apn_command))) {
		return -EINVAL;
	}

	return modem_chat_script_chat_set_request(&data->simcom_dial_script_chats[1],
						  data->staged_apn_command);
}

static const struct modem_chat_script *modem_cellular_get_dial_chat_script(
	const struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;

	if (config->runtime_apn) {
		return &data->simcom_dial_script;
	}

	return config->dial_chat_script;
}

static int modem_cellular_init_runtime_apn(struct modem_cellular_data *data)
{
	int ret;

	ret = modem_cellular_validate_apn(CONFIG_MODEM_SIMCOM7X00_APN, &data->apn_len);
	if (ret < 0) {
		return ret;
	}

	memcpy(data->apn, CONFIG_MODEM_SIMCOM7X00_APN, data->apn_len + 1);

	modem_chat_script_chat_init(&data->simcom_dial_script_chats[0]);
	ret = modem_chat_script_chat_set_request(&data->simcom_dial_script_chats[0],
						 "AT+CGACT=0,1");
	if (ret < 0) {
		return ret;
	}
	ret = modem_chat_script_chat_set_response_matches(&data->simcom_dial_script_chats[0],
							  allow_match,
							  ARRAY_SIZE(allow_match));
	if (ret < 0) {
		return ret;
	}

	modem_chat_script_chat_init(&data->simcom_dial_script_chats[1]);
	ret = modem_chat_script_chat_set_response_matches(&data->simcom_dial_script_chats[1],
							  &ok_match, 1);
	if (ret < 0) {
		return ret;
	}

	modem_chat_script_chat_init(&data->simcom_dial_script_chats[2]);
	ret = modem_chat_script_chat_set_request(&data->simcom_dial_script_chats[2], "AT+CFUN=1");
	if (ret < 0) {
		return ret;
	}
	ret = modem_chat_script_chat_set_response_matches(&data->simcom_dial_script_chats[2],
							  &ok_match, 1);
	if (ret < 0) {
		return ret;
	}

	modem_chat_script_chat_init(&data->simcom_dial_script_chats[3]);
	ret = modem_chat_script_chat_set_request(&data->simcom_dial_script_chats[3], "AT+CGATT=1");
	if (ret < 0) {
		return ret;
	}
	ret = modem_chat_script_chat_set_response_matches(&data->simcom_dial_script_chats[3],
							  &ok_match, 1);
	if (ret < 0) {
		return ret;
	}

	modem_chat_script_chat_init(&data->simcom_dial_script_chats[4]);
	ret = modem_chat_script_chat_set_request(&data->simcom_dial_script_chats[4],
						 "ATD*99***1#");
	if (ret < 0) {
		return ret;
	}
	modem_chat_script_chat_set_timeout(&data->simcom_dial_script_chats[4], 0);

	modem_chat_script_init(&data->simcom_dial_script);
	modem_chat_script_set_name(&data->simcom_dial_script, "simcom_sim7x00_dial_dynamic");
	ret = modem_chat_script_set_script_chats(&data->simcom_dial_script,
						 data->simcom_dial_script_chats,
						 ARRAY_SIZE(data->simcom_dial_script_chats));
	if (ret < 0) {
		return ret;
	}
	ret = modem_chat_script_set_abort_matches(&data->simcom_dial_script, dial_abort_matches,
						  ARRAY_SIZE(dial_abort_matches));
	if (ret < 0) {
		return ret;
	}
	modem_chat_script_set_callback(&data->simcom_dial_script,
				       modem_cellular_chat_callback_handler);
	modem_chat_script_set_timeout(&data->simcom_dial_script, 15);

	return modem_cellular_stage_apn_command(data);
}

MODEM_CHAT_SCRIPT_CMDS_DEFINE(simcom7x00_current_rat_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CPSI?", cpsi_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CNSMOD?", cnsmod_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(simcom7x00_current_rat_chat_script,
			 simcom7x00_current_rat_chat_script_cmds, abort_matches,
			 modem_cellular_chat_callback_handler, 2);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(simcom7x00_network_refresh_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CPSI?", cpsi_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CNSMOD?", cnsmod_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(simcom7x00_network_refresh_chat_script,
			 simcom7x00_network_refresh_chat_script_cmds, abort_matches,
			 modem_cellular_chat_callback_handler, 2);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(simcom7x00_rat_readback_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CNMP?", cnmp_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CNAOP?", cnaop_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CPSI?", cpsi_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CNSMOD?", cnsmod_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(simcom7x00_rat_readback_chat_script,
			 simcom7x00_rat_readback_chat_script_cmds, abort_matches,
			 modem_cellular_chat_callback_handler, 5);

static int modem_cellular_refresh_operator(struct modem_cellular_data *data)
{
	int ret;
	struct modem_chat_script_chat script_chats[1];
	struct modem_chat_script script;
	char current_plmn[MODEM_CELLULAR_DATA_OPERATOR_PLMN_LEN + 1];

	ret = modem_cellular_refresh_network_snapshot(data);
	if (ret < 0) {
		return ret;
	}

	k_mutex_lock(&data->network_lock, K_FOREVER);
	memcpy(data->operator_lookup_plmn, data->operator_plmn, sizeof(data->operator_lookup_plmn));
	memcpy(current_plmn, data->operator_plmn, sizeof(current_plmn));
	data->operator_lookup_found = false;
	k_mutex_unlock(&data->network_lock);

	if (current_plmn[0] == '\0') {
		k_mutex_lock(&data->network_lock, K_FOREVER);
		data->operator_long[0] = '\0';
		data->operator_short[0] = '\0';
		k_mutex_unlock(&data->network_lock);
		return -ENODATA;
	}

	if (modem_cellular_apply_cached_operator(data, current_plmn)) {
		return 0;
	}

	modem_chat_script_chat_init(&script_chats[0]);
	ret = modem_chat_script_chat_set_request(&script_chats[0], "AT+COPN");
	if (ret < 0) {
		return ret;
	}
	ret = modem_chat_script_chat_set_response_matches(&script_chats[0], &copn_match, 1);
	if (ret < 0) {
		return ret;
	}

	modem_chat_script_init(&script);
	modem_chat_script_set_name(&script, "simcom7x00_operator_lookup");
	ret = modem_chat_script_set_script_chats(&script, script_chats, ARRAY_SIZE(script_chats));
	if (ret < 0) {
		return ret;
	}
	ret = modem_chat_script_set_abort_matches(&script, abort_matches, ARRAY_SIZE(abort_matches));
	if (ret < 0) {
		return ret;
	}
	modem_chat_script_set_callback(&script, modem_cellular_chat_callback_handler);
	modem_chat_script_set_timeout(&script, 10);

	ret = modem_cellular_run_script_locked(data, &script);
	if (ret < 0) {
		k_mutex_lock(&data->network_lock, K_FOREVER);
		if (strcmp(data->operator_plmn, current_plmn) == 0) {
			data->operator_long[0] = '\0';
			data->operator_short[0] = '\0';
		}
		k_mutex_unlock(&data->network_lock);
		return -ENODATA;
	}

	k_mutex_lock(&data->network_lock, K_FOREVER);
	ret = data->operator_lookup_found ? 0 : -ENODATA;
	if ((ret < 0) && (strcmp(data->operator_plmn, current_plmn) == 0)) {
		data->operator_long[0] = '\0';
		data->operator_short[0] = '\0';
	}
	k_mutex_unlock(&data->network_lock);

	return ret;
}

static int modem_cellular_refresh_network_snapshot(struct modem_cellular_data *data)
{
	return modem_cellular_run_script_locked(data, &simcom7x00_network_refresh_chat_script);
}

static int modem_cellular_apply_rat_config(struct modem_cellular_data *data)
{
	struct modem_chat_script_chat script_chats[2];
	struct modem_chat_script script;
	int cnmp;
	int expected_cnmp;
	int ret;
	int offset;

	expected_cnmp = data->rat_policy_enabled ? modem_cellular_policy_to_cnmp(&data->rat_policy) :
						  modem_cellular_mode_to_cnmp(data->rat_mode);
	if (expected_cnmp < 0) {
		return expected_cnmp;
	}
	cnmp = expected_cnmp;

	ret = snprintf(data->staged_cnmp_command, sizeof(data->staged_cnmp_command), "AT+CNMP=%d",
		       cnmp);
	if ((ret < 0) || (ret >= (int)sizeof(data->staged_cnmp_command))) {
		return -EINVAL;
	}

	modem_chat_script_chat_init(&script_chats[0]);
	ret = modem_chat_script_chat_set_request(&script_chats[0], data->staged_cnmp_command);
	if (ret < 0) {
		return ret;
	}
	ret = modem_chat_script_chat_set_response_matches(&script_chats[0], &ok_match, 1);
	if (ret < 0) {
		return ret;
	}

	offset = snprintf(data->staged_cnaop_command, sizeof(data->staged_cnaop_command), "AT+CNAOP=7");
	if ((offset < 0) || (offset >= (int)sizeof(data->staged_cnaop_command))) {
		return -EINVAL;
	}

	for (uint8_t i = 0; i < data->rat_policy.size; i++) {
		int sys_mode = modem_cellular_rat_to_sys_mode(data->rat_policy.priorities[i]);

		if (sys_mode < 0) {
			return -EINVAL;
		}

		offset += snprintf(data->staged_cnaop_command + offset,
				   sizeof(data->staged_cnaop_command) - (size_t)offset, ",%d",
				   sys_mode);
		if ((offset < 0) || (offset >= (int)sizeof(data->staged_cnaop_command))) {
			return -EINVAL;
		}
	}

	modem_chat_script_chat_init(&script_chats[1]);
	ret = modem_chat_script_chat_set_request(&script_chats[1], data->staged_cnaop_command);
	if (ret < 0) {
		return ret;
	}
	ret = modem_chat_script_chat_set_response_matches(&script_chats[1], &ok_match, 1);
	if (ret < 0) {
		return ret;
	}

	modem_chat_script_init(&script);
	modem_chat_script_set_name(&script, "simcom7x00_rat_apply");
	ret = modem_chat_script_set_script_chats(&script, script_chats, ARRAY_SIZE(script_chats));
	if (ret < 0) {
		return ret;
	}
	ret = modem_chat_script_set_abort_matches(&script, abort_matches, ARRAY_SIZE(abort_matches));
	if (ret < 0) {
		return ret;
	}
	modem_chat_script_set_callback(&script, modem_cellular_chat_callback_handler);
	modem_chat_script_set_timeout(&script, 10);

	ret = modem_cellular_run_script_locked(data, &script);
	if (ret < 0) {
		return ret;
	}

	ret = modem_cellular_run_script_locked(data, &simcom7x00_rat_readback_chat_script);
	if (ret < 0) {
		data->rat_config_dirty = true;
		return ret;
	}

	data->rat_config_dirty = false;
	return ret;
}

static void modem_cellular_log_state_changed(enum modem_cellular_state last_state,
					     enum modem_cellular_state new_state)
{
	LOG_DBG("switch from %s to %s", modem_cellular_state_str(last_state),
		modem_cellular_state_str(new_state));
}

static void modem_cellular_log_event(enum modem_cellular_event evt)
{
	LOG_DBG("event %s", modem_cellular_event_str(evt));
}

static void modem_cellular_start_timer(struct modem_cellular_data *data, k_timeout_t timeout)
{
	k_work_schedule(&data->timeout_work, timeout);
}

static void modem_cellular_stop_timer(struct modem_cellular_data *data)
{
	k_work_cancel_delayable(&data->timeout_work);
}

static uint32_t modem_cellular_get_periodic_script_interval_ms(struct modem_cellular_data *data)
{
	return (uint32_t)atomic_get(&data->periodic_script_interval_ms);
}

static void modem_cellular_schedule_periodic_script(struct modem_cellular_data *data)
{
	uint32_t interval_ms = modem_cellular_get_periodic_script_interval_ms(data);

	if (interval_ms == 0U) {
		modem_cellular_stop_timer(data);
		return;
	}

	modem_cellular_start_timer(data, K_MSEC(interval_ms));
}

static void modem_cellular_handle_periodic_timeout(
	struct modem_cellular_data *data, const struct modem_chat_script *script)
{
	int ret;

	ret = modem_cellular_run_script_async_locked(data, script);
	if (ret == -EBUSY) {
		LOG_DBG("periodic script deferred because control plane is busy");
		modem_cellular_schedule_periodic_script(data);
		return;
	}

	if (ret < 0) {
		LOG_WRN("periodic script start failed: %d", ret);
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_SCRIPT_FAILED);
	}
}

int simcom7x00_modem_ensure_ready(const struct device *dev, bool ppp_requested);

static void modem_cellular_timeout_handler(struct k_work *item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(item);
	struct modem_cellular_data *data =
		CONTAINER_OF(dwork, struct modem_cellular_data, timeout_work);

	modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_TIMEOUT);
}

static void modem_cellular_event_dispatch_handler(struct k_work *item)
{
	struct modem_cellular_data *data =
		CONTAINER_OF(item, struct modem_cellular_data, event_dispatch_work);

	uint8_t events[sizeof(data->event_buf)];
	uint8_t events_cnt;

	k_mutex_lock(&data->event_rb_lock, K_FOREVER);

	events_cnt = (uint8_t)ring_buf_get(&data->event_rb, events, sizeof(data->event_buf));

	k_mutex_unlock(&data->event_rb_lock);

	for (uint8_t i = 0; i < events_cnt; i++) {
		modem_cellular_event_handler(data, (enum modem_cellular_event)events[i]);
	}
}

static void modem_cellular_delegate_event(struct modem_cellular_data *data,
					  enum modem_cellular_event evt)
{
	k_mutex_lock(&data->event_rb_lock, K_FOREVER);
	ring_buf_put(&data->event_rb, (uint8_t *)&evt, 1);
	k_mutex_unlock(&data->event_rb_lock);
	k_work_submit(&data->event_dispatch_work);
}

static int modem_cellular_on_idle_state_enter(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;

	if (modem_cellular_gpio_is_enabled(&config->reset_gpio)) {
		gpio_pin_set_dt(&config->reset_gpio, 1);
	}

	modem_cellular_notify_user_pipes_disconnected(data);
	modem_cellular_release_chat_locked(data);
	modem_ppp_release(data->ppp);
	modem_cmux_release(&data->cmux);
	modem_pipe_close_async(data->uart_pipe);
	k_sem_give(&data->suspended_sem);
	return 0;
}

static void modem_cellular_idle_event_handler(struct modem_cellular_data *data,
					      enum modem_cellular_event evt)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;

	switch (evt) {
	case MODEM_CELLULAR_EVENT_RESUME:
		data->ppp_requested = data->force_ready_only_on_resume ?
			false : (data->ppp_requested || config->default_ppp_autostart);
		data->force_ready_only_on_resume = false;
		if (config->autostarts) {
			modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_AWAIT_POWER_ON);
			break;
		}

		if (modem_cellular_gpio_is_enabled(&config->power_gpio)) {
			modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_POWER_ON_PULSE);
			break;
		}

		if (modem_cellular_gpio_is_enabled(&config->reset_gpio)) {
			modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_AWAIT_POWER_ON);
			break;
		}

		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_RUN_INIT_SCRIPT);
		break;

	case MODEM_CELLULAR_EVENT_SUSPEND:
		k_sem_give(&data->suspended_sem);
		break;

	default:
		break;
	}
}

static int modem_cellular_on_idle_state_leave(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;

	k_sem_take(&data->suspended_sem, K_NO_WAIT);

	if (modem_cellular_gpio_is_enabled(&config->reset_gpio)) {
		gpio_pin_set_dt(&config->reset_gpio, 0);
	}

	return 0;
}

static int modem_cellular_on_reset_pulse_state_enter(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;

	gpio_pin_set_dt(&config->reset_gpio, 1);
	modem_cellular_start_timer(data, K_MSEC(config->reset_pulse_duration_ms));
	return 0;
}

static void modem_cellular_reset_pulse_event_handler(struct modem_cellular_data *data,
							enum modem_cellular_event evt)
{
	switch (evt) {
	case MODEM_CELLULAR_EVENT_TIMEOUT:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_AWAIT_POWER_ON);
		break;

	case MODEM_CELLULAR_EVENT_SUSPEND:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_IDLE);
		break;

	default:
		break;
	}
}

static int modem_cellular_on_reset_pulse_state_leave(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;

	gpio_pin_set_dt(&config->reset_gpio, 0);
	modem_cellular_stop_timer(data);
	return 0;
}

static int modem_cellular_on_power_on_pulse_state_enter(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;

	gpio_pin_set_dt(&config->power_gpio, 1);
	modem_cellular_start_timer(data, K_MSEC(config->power_pulse_duration_ms));
	return 0;
}

static void modem_cellular_power_on_pulse_event_handler(struct modem_cellular_data *data,
							enum modem_cellular_event evt)
{
	switch (evt) {
	case MODEM_CELLULAR_EVENT_TIMEOUT:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_AWAIT_POWER_ON);
		break;

	case MODEM_CELLULAR_EVENT_SUSPEND:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_IDLE);
		break;

	default:
		break;
	}
}

static int modem_cellular_on_power_on_pulse_state_leave(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;

	gpio_pin_set_dt(&config->power_gpio, 0);
	modem_cellular_stop_timer(data);
	return 0;
}

static int modem_cellular_on_await_power_on_state_enter(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;

	modem_cellular_start_timer(data, K_MSEC(config->startup_time_ms));
	return 0;
}

static void modem_cellular_await_power_on_event_handler(struct modem_cellular_data *data,
							enum modem_cellular_event evt)
{
	switch (evt) {
	case MODEM_CELLULAR_EVENT_TIMEOUT:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_RUN_INIT_SCRIPT);
		break;

	case MODEM_CELLULAR_EVENT_SUSPEND:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_IDLE);
		break;

	default:
		break;
	}
}

static int modem_cellular_on_run_init_script_state_enter(struct modem_cellular_data *data)
{
	modem_pipe_attach(data->uart_pipe, modem_cellular_bus_pipe_handler, data);
	return modem_pipe_open_async(data->uart_pipe);
}

static void modem_cellular_run_init_script_event_handler(struct modem_cellular_data *data,
							 enum modem_cellular_event evt)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;

	switch (evt) {
	case MODEM_CELLULAR_EVENT_BUS_OPENED:
		modem_cellular_attach_chat_locked(data, data->uart_pipe);
		modem_cellular_run_script_async_locked(data, config->init_chat_script);
		break;

	case MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS:
		net_if_set_link_addr(modem_ppp_get_iface(data->ppp), data->imei,
				     ARRAY_SIZE(data->imei), NET_LINK_UNKNOWN);

		modem_cellular_release_chat_locked(data);
		modem_pipe_attach(data->uart_pipe, modem_cellular_bus_pipe_handler, data);
		modem_pipe_close_async(data->uart_pipe);
		break;

	case MODEM_CELLULAR_EVENT_BUS_CLOSED:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_CONNECT_CMUX);
		break;

	case MODEM_CELLULAR_EVENT_SUSPEND:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_IDLE);
		break;

	case MODEM_CELLULAR_EVENT_SCRIPT_FAILED:
		if (modem_cellular_gpio_is_enabled(&config->power_gpio)) {
			modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_POWER_ON_PULSE);
			break;
		}

		if (modem_cellular_gpio_is_enabled(&config->reset_gpio)) {
			modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_RESET_PULSE);
			break;
		}

		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_IDLE);
		break;

	default:
		break;
	}
}

static int modem_cellular_on_connect_cmux_state_enter(struct modem_cellular_data *data)
{
	/*
	 * Allow modem to switch bus into CMUX mode. Some modems disable UART RX while
	 * switching, resulting in UART RX errors as bus is no longer pulled up by modem.
	 */
	modem_cellular_start_timer(data, K_MSEC(100));
	return 0;
}

static void modem_cellular_connect_cmux_event_handler(struct modem_cellular_data *data,
						      enum modem_cellular_event evt)
{
	switch (evt) {
	case MODEM_CELLULAR_EVENT_TIMEOUT:
		modem_pipe_attach(data->uart_pipe, modem_cellular_bus_pipe_handler, data);
		modem_pipe_open_async(data->uart_pipe);
		break;

	case MODEM_CELLULAR_EVENT_BUS_OPENED:
		modem_cmux_attach(&data->cmux, data->uart_pipe);
		modem_cmux_connect_async(&data->cmux);
		break;

	case MODEM_CELLULAR_EVENT_CMUX_CONNECTED:
		modem_cellular_notify_user_pipes_connected(data);
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_OPEN_DLCI1);
		break;

	case MODEM_CELLULAR_EVENT_SUSPEND:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_INIT_POWER_OFF);
		break;

	default:
		break;
	}
}

static int modem_cellular_on_open_dlci1_state_enter(struct modem_cellular_data *data)
{
	modem_pipe_attach(data->dlci1_pipe, modem_cellular_dlci1_pipe_handler, data);
	return modem_pipe_open_async(data->dlci1_pipe);
}

static void modem_cellular_open_dlci1_event_handler(struct modem_cellular_data *data,
						    enum modem_cellular_event evt)
{
	switch (evt) {
	case MODEM_CELLULAR_EVENT_DLCI1_OPENED:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_OPEN_DLCI2);
		break;

	case MODEM_CELLULAR_EVENT_SUSPEND:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_INIT_POWER_OFF);
		break;

	default:
		break;
	}
}

static int modem_cellular_on_open_dlci1_state_leave(struct modem_cellular_data *data)
{
	modem_pipe_release(data->dlci1_pipe);
	return 0;
}

static int modem_cellular_on_open_dlci2_state_enter(struct modem_cellular_data *data)
{
	modem_pipe_attach(data->dlci2_pipe, modem_cellular_dlci2_pipe_handler, data);
	return modem_pipe_open_async(data->dlci2_pipe);
}

static void modem_cellular_open_dlci2_event_handler(struct modem_cellular_data *data,
						    enum modem_cellular_event evt)
{
	switch (evt) {
	case MODEM_CELLULAR_EVENT_DLCI2_OPENED:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_READY);
		break;

	case MODEM_CELLULAR_EVENT_SUSPEND:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_INIT_POWER_OFF);
		break;

	default:
		break;
	}
}

static int modem_cellular_on_open_dlci2_state_leave(struct modem_cellular_data *data)
{
	modem_pipe_release(data->dlci2_pipe);
	return 0;
}

static int modem_cellular_on_ready_state_enter(struct modem_cellular_data *data)
{
	int ret;

	modem_cellular_release_chat_locked(data);
	modem_ppp_release(data->ppp);
	modem_cellular_attach_chat_locked(data, data->dlci2_pipe);

	if (data->rat_config_dirty) {
		ret = modem_cellular_apply_rat_config(data);
		if (ret < 0) {
			LOG_WRN("failed to apply RAT configuration: %d", ret);
		}
	}

	modem_cellular_schedule_periodic_script(data);
	k_sem_give(&data->ready_sem);

	if (data->ppp_requested) {
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_PPP_CONNECT_REQUEST);
	}

	return 0;
}

static void modem_cellular_ready_event_handler(struct modem_cellular_data *data,
					       enum modem_cellular_event evt)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;

	switch (evt) {
	case MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS:
	case MODEM_CELLULAR_EVENT_SCRIPT_FAILED:
	case MODEM_CELLULAR_EVENT_PERIODIC_SCRIPT_INTERVAL_CHANGED:
		modem_cellular_schedule_periodic_script(data);
		break;

	case MODEM_CELLULAR_EVENT_TIMEOUT:
		modem_cellular_handle_periodic_timeout(data, config->periodic_chat_script);
		break;

	case MODEM_CELLULAR_EVENT_PPP_CONNECT_REQUEST:
		if (data->ppp_requested) {
			modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_RUN_DIAL_SCRIPT);
		}
		break;

	case MODEM_CELLULAR_EVENT_SUSPEND:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_INIT_POWER_OFF);
		break;

	default:
		break;
	}
}

static int modem_cellular_on_ready_state_leave(struct modem_cellular_data *data)
{
	modem_cellular_stop_timer(data);
	modem_cellular_release_chat_locked(data);
	return 0;
}

static int modem_cellular_on_run_dial_script_state_enter(struct modem_cellular_data *data)
{
	/* Allow modem time to enter command mode before running dial script */
	modem_cellular_start_timer(data, K_MSEC(100));
	return 0;
}

static void modem_cellular_run_dial_script_event_handler(struct modem_cellular_data *data,
							 enum modem_cellular_event evt)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;
	const struct modem_chat_script *dial_script;
	int ret;

	switch (evt) {
	case MODEM_CELLULAR_EVENT_TIMEOUT:
		if (config->runtime_apn) {
			ret = modem_cellular_stage_apn_command(data);
			if (ret < 0) {
				data->ppp_requested = false;
				modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_READY);
				break;
			}
		}

		dial_script = modem_cellular_get_dial_chat_script(data);
		modem_cellular_attach_chat_locked(data, data->dlci1_pipe);
		ret = modem_cellular_run_script_async_locked(data, dial_script);
		if (ret < 0) {
			data->ppp_requested = false;
			modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_READY);
		}
		break;

	case MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_AWAIT_REGISTERED);
		break;

	case MODEM_CELLULAR_EVENT_PPP_DISCONNECT_REQUEST:
		data->ppp_requested = false;
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_READY);
		break;

	case MODEM_CELLULAR_EVENT_SUSPEND:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_INIT_POWER_OFF);
		break;

	default:
		break;
	}
}

static int modem_cellular_on_run_dial_script_state_leave(struct modem_cellular_data *data)
{
	modem_cellular_release_chat_locked(data);
	return 0;
}

static int modem_cellular_on_await_registered_state_enter(struct modem_cellular_data *data)
{
	if (modem_ppp_attach(data->ppp, data->dlci1_pipe) < 0) {
		return -EAGAIN;
	}

	modem_cellular_schedule_periodic_script(data);
	return modem_cellular_attach_chat_locked(data, data->dlci2_pipe);
}

static void modem_cellular_await_registered_event_handler(struct modem_cellular_data *data,
						  enum modem_cellular_event evt)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;

	switch (evt) {
	case MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS:
	case MODEM_CELLULAR_EVENT_SCRIPT_FAILED:
	case MODEM_CELLULAR_EVENT_PERIODIC_SCRIPT_INTERVAL_CHANGED:
		modem_cellular_schedule_periodic_script(data);
		break;

	case MODEM_CELLULAR_EVENT_TIMEOUT:
		modem_cellular_handle_periodic_timeout(data, config->periodic_chat_script);
		break;

	case MODEM_CELLULAR_EVENT_REGISTERED:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_CARRIER_ON);
		break;

	case MODEM_CELLULAR_EVENT_PPP_DISCONNECT_REQUEST:
		data->ppp_requested = false;
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_READY);
		break;

	case MODEM_CELLULAR_EVENT_SUSPEND:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_INIT_POWER_OFF);
		break;

	default:
		break;
	}
}

static int modem_cellular_on_await_registered_state_leave(struct modem_cellular_data *data)
{
	modem_cellular_stop_timer(data);
	return 0;
}

static int modem_cellular_on_carrier_on_state_enter(struct modem_cellular_data *data)
{
	net_if_carrier_on(modem_ppp_get_iface(data->ppp));
	modem_cellular_schedule_periodic_script(data);
	return 0;
}

static void modem_cellular_carrier_on_event_handler(struct modem_cellular_data *data,
						    enum modem_cellular_event evt)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;

	switch (evt) {
	case MODEM_CELLULAR_EVENT_SCRIPT_SUCCESS:
	case MODEM_CELLULAR_EVENT_SCRIPT_FAILED:
	case MODEM_CELLULAR_EVENT_PERIODIC_SCRIPT_INTERVAL_CHANGED:
		modem_cellular_schedule_periodic_script(data);
		break;

	case MODEM_CELLULAR_EVENT_TIMEOUT:
		modem_cellular_handle_periodic_timeout(data, config->periodic_chat_script);
		break;

	case MODEM_CELLULAR_EVENT_DEREGISTERED:
		if (data->ppp_requested) {
			modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_RUN_DIAL_SCRIPT);
		} else {
			modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_READY);
		}
		break;

	case MODEM_CELLULAR_EVENT_PPP_DISCONNECT_REQUEST:
		data->ppp_requested = false;
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_READY);
		break;

	case MODEM_CELLULAR_EVENT_SUSPEND:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_INIT_POWER_OFF);
		break;

	default:
		break;
	}
}

static int modem_cellular_on_carrier_on_state_leave(struct modem_cellular_data *data)
{
	modem_cellular_stop_timer(data);
	net_if_carrier_off(modem_ppp_get_iface(data->ppp));
	modem_cellular_release_chat_locked(data);
	modem_ppp_release(data->ppp);
	return 0;
}

static int modem_cellular_on_init_power_off_state_enter(struct modem_cellular_data *data)
{
	modem_pipe_close_async(data->uart_pipe);
	modem_cellular_start_timer(data, K_MSEC(2000));
	return 0;
}

static void modem_cellular_init_power_off_event_handler(struct modem_cellular_data *data,
							enum modem_cellular_event evt)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;

	switch (evt) {
	case MODEM_CELLULAR_EVENT_TIMEOUT:
		if (modem_cellular_gpio_is_enabled(&config->power_gpio)) {
			modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_POWER_OFF_PULSE);
			break;
		}

		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_IDLE);
		break;

	default:
		break;
	}
}

static int modem_cellular_on_init_power_off_state_leave(struct modem_cellular_data *data)
{
	modem_cellular_notify_user_pipes_disconnected(data);
	modem_cellular_release_chat_locked(data);
	modem_ppp_release(data->ppp);
	return 0;
}

static int modem_cellular_on_power_off_pulse_state_enter(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;

	gpio_pin_set_dt(&config->power_gpio, 1);
	modem_cellular_start_timer(data, K_MSEC(config->power_pulse_duration_ms));
	return 0;
}

static void modem_cellular_power_off_pulse_event_handler(struct modem_cellular_data *data,
							enum modem_cellular_event evt)
{
	switch (evt) {
	case MODEM_CELLULAR_EVENT_TIMEOUT:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_AWAIT_POWER_OFF);
		break;

	default:
		break;
	}
}

static int modem_cellular_on_power_off_pulse_state_leave(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;

	gpio_pin_set_dt(&config->power_gpio, 0);
	modem_cellular_stop_timer(data);
	return 0;
}

static int modem_cellular_on_await_power_off_state_enter(struct modem_cellular_data *data)
{
	const struct modem_cellular_config *config =
		(const struct modem_cellular_config *)data->dev->config;

	modem_cellular_start_timer(data, K_MSEC(config->shutdown_time_ms));
	return 0;
}

static void modem_cellular_await_power_off_event_handler(struct modem_cellular_data *data,
							enum modem_cellular_event evt)
{
	switch (evt) {
	case MODEM_CELLULAR_EVENT_TIMEOUT:
		modem_cellular_enter_state(data, MODEM_CELLULAR_STATE_IDLE);
		break;

	default:
		break;
	}
}

static int modem_cellular_on_state_enter(struct modem_cellular_data *data)
{
	int ret;

	switch (data->state) {
	case MODEM_CELLULAR_STATE_IDLE:
		ret = modem_cellular_on_idle_state_enter(data);
		break;

	case MODEM_CELLULAR_STATE_RESET_PULSE:
		ret = modem_cellular_on_reset_pulse_state_enter(data);
		break;

	case MODEM_CELLULAR_STATE_POWER_ON_PULSE:
		ret = modem_cellular_on_power_on_pulse_state_enter(data);
		break;

	case MODEM_CELLULAR_STATE_AWAIT_POWER_ON:
		ret = modem_cellular_on_await_power_on_state_enter(data);
		break;

	case MODEM_CELLULAR_STATE_RUN_INIT_SCRIPT:
		ret = modem_cellular_on_run_init_script_state_enter(data);
		break;

	case MODEM_CELLULAR_STATE_CONNECT_CMUX:
		ret = modem_cellular_on_connect_cmux_state_enter(data);
		break;

	case MODEM_CELLULAR_STATE_OPEN_DLCI1:
		ret = modem_cellular_on_open_dlci1_state_enter(data);
		break;

	case MODEM_CELLULAR_STATE_OPEN_DLCI2:
		ret = modem_cellular_on_open_dlci2_state_enter(data);
		break;

	case MODEM_CELLULAR_STATE_READY:
		ret = modem_cellular_on_ready_state_enter(data);
		break;

	case MODEM_CELLULAR_STATE_RUN_DIAL_SCRIPT:
		ret = modem_cellular_on_run_dial_script_state_enter(data);
		break;

	case MODEM_CELLULAR_STATE_AWAIT_REGISTERED:
		ret = modem_cellular_on_await_registered_state_enter(data);
		break;

	case MODEM_CELLULAR_STATE_CARRIER_ON:
		ret = modem_cellular_on_carrier_on_state_enter(data);
		break;

	case MODEM_CELLULAR_STATE_INIT_POWER_OFF:
		ret = modem_cellular_on_init_power_off_state_enter(data);
		break;

	case MODEM_CELLULAR_STATE_POWER_OFF_PULSE:
		ret = modem_cellular_on_power_off_pulse_state_enter(data);
		break;

	case MODEM_CELLULAR_STATE_AWAIT_POWER_OFF:
		ret = modem_cellular_on_await_power_off_state_enter(data);
		break;

	default:
		ret = 0;
		break;
	}

	return ret;
}

static int modem_cellular_on_state_leave(struct modem_cellular_data *data)
{
	int ret;

	switch (data->state) {
	case MODEM_CELLULAR_STATE_IDLE:
		ret = modem_cellular_on_idle_state_leave(data);
		break;

	case MODEM_CELLULAR_STATE_RESET_PULSE:
		ret = modem_cellular_on_reset_pulse_state_leave(data);
		break;

	case MODEM_CELLULAR_STATE_POWER_ON_PULSE:
		ret = modem_cellular_on_power_on_pulse_state_leave(data);
		break;

	case MODEM_CELLULAR_STATE_OPEN_DLCI1:
		ret = modem_cellular_on_open_dlci1_state_leave(data);
		break;

	case MODEM_CELLULAR_STATE_OPEN_DLCI2:
		ret = modem_cellular_on_open_dlci2_state_leave(data);
		break;

	case MODEM_CELLULAR_STATE_READY:
		ret = modem_cellular_on_ready_state_leave(data);
		break;

	case MODEM_CELLULAR_STATE_RUN_DIAL_SCRIPT:
		ret = modem_cellular_on_run_dial_script_state_leave(data);
		break;

	case MODEM_CELLULAR_STATE_AWAIT_REGISTERED:
		ret = modem_cellular_on_await_registered_state_leave(data);
		break;

	case MODEM_CELLULAR_STATE_CARRIER_ON:
		ret = modem_cellular_on_carrier_on_state_leave(data);
		break;

	case MODEM_CELLULAR_STATE_INIT_POWER_OFF:
		ret = modem_cellular_on_init_power_off_state_leave(data);
		break;

	case MODEM_CELLULAR_STATE_POWER_OFF_PULSE:
		ret = modem_cellular_on_power_off_pulse_state_leave(data);
		break;

	default:
		ret = 0;
		break;
	}

	return ret;
}

static void modem_cellular_enter_state(struct modem_cellular_data *data,
				       enum modem_cellular_state state)
{
	int ret;

	ret = modem_cellular_on_state_leave(data);

	if (ret < 0) {
		LOG_WRN("failed to leave state, error: %i", ret);

		return;
	}

	data->state = state;
	ret = modem_cellular_on_state_enter(data);

	if (ret < 0) {
		LOG_WRN("failed to enter state error: %i", ret);
	}
}

static void modem_cellular_event_handler(struct modem_cellular_data *data,
					 enum modem_cellular_event evt)
{
	enum modem_cellular_state state;

	state = data->state;

	modem_cellular_log_event(evt);

	switch (data->state) {
	case MODEM_CELLULAR_STATE_IDLE:
		modem_cellular_idle_event_handler(data, evt);
		break;

	case MODEM_CELLULAR_STATE_RESET_PULSE:
		modem_cellular_reset_pulse_event_handler(data, evt);
		break;

	case MODEM_CELLULAR_STATE_POWER_ON_PULSE:
		modem_cellular_power_on_pulse_event_handler(data, evt);
		break;

	case MODEM_CELLULAR_STATE_AWAIT_POWER_ON:
		modem_cellular_await_power_on_event_handler(data, evt);
		break;

	case MODEM_CELLULAR_STATE_RUN_INIT_SCRIPT:
		modem_cellular_run_init_script_event_handler(data, evt);
		break;

	case MODEM_CELLULAR_STATE_CONNECT_CMUX:
		modem_cellular_connect_cmux_event_handler(data, evt);
		break;

	case MODEM_CELLULAR_STATE_OPEN_DLCI1:
		modem_cellular_open_dlci1_event_handler(data, evt);
		break;

	case MODEM_CELLULAR_STATE_OPEN_DLCI2:
		modem_cellular_open_dlci2_event_handler(data, evt);
		break;

	case MODEM_CELLULAR_STATE_READY:
		modem_cellular_ready_event_handler(data, evt);
		break;

	case MODEM_CELLULAR_STATE_RUN_DIAL_SCRIPT:
		modem_cellular_run_dial_script_event_handler(data, evt);
		break;

	case MODEM_CELLULAR_STATE_AWAIT_REGISTERED:
		modem_cellular_await_registered_event_handler(data, evt);
		break;

	case MODEM_CELLULAR_STATE_CARRIER_ON:
		modem_cellular_carrier_on_event_handler(data, evt);
		break;

	case MODEM_CELLULAR_STATE_INIT_POWER_OFF:
		modem_cellular_init_power_off_event_handler(data, evt);
		break;

	case MODEM_CELLULAR_STATE_POWER_OFF_PULSE:
		modem_cellular_power_off_pulse_event_handler(data, evt);
		break;

	case MODEM_CELLULAR_STATE_AWAIT_POWER_OFF:
		modem_cellular_await_power_off_event_handler(data, evt);
		break;
	}

	if (state != data->state) {
		modem_cellular_log_state_changed(state, data->state);
	}
}

static void modem_cellular_cmux_handler(struct modem_cmux *cmux, enum modem_cmux_event event,
					void *user_data)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)user_data;

	switch (event) {
	case MODEM_CMUX_EVENT_CONNECTED:
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_CMUX_CONNECTED);
		break;

	default:
		break;
	}
}

MODEM_CHAT_SCRIPT_CMDS_DEFINE(get_signal_csq_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CSQ", csq_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(get_signal_csq_chat_script, get_signal_csq_chat_script_cmds,
			 abort_matches, modem_cellular_chat_callback_handler, 2);

static inline int modem_cellular_csq_parse_rssi(uint8_t rssi, int16_t *value)
{
	/* AT+CSQ returns a response +CSQ: <rssi>,<ber> where:
	 * - rssi is a integer from 0 to 31 whose values describes a signal strength
	 *   between -113 dBm for 0 and -51dbM for 31 or unknown for 99
	 * - ber is an integer from 0 to 7 that describes the error rate, it can also
	 *   be 99 for an unknown error rate
	 */
	if (rssi == CSQ_RSSI_UNKNOWN) {
		return -EINVAL;
	}

	*value = (int16_t)CSQ_RSSI_TO_DB(rssi);
	return 0;
}

MODEM_CHAT_SCRIPT_CMDS_DEFINE(get_signal_cesq_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CESQ", cesq_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(get_signal_cesq_chat_script, get_signal_cesq_chat_script_cmds,
			 abort_matches, modem_cellular_chat_callback_handler, 2);

/* AT+CESQ returns a response +CESQ: <rxlev>,<ber>,<rscp>,<ecn0>,<rsrq>,<rsrp> where:
 * - rsrq is a integer from 0 to 34 whose values describes the Reference Signal Receive
 *   Quality between -20 dB for 0 and -3 dB for 34 (0.5 dB steps), or unknown for 255
 * - rsrp is an integer from 0 to 97 that describes the Reference Signal Receive Power
 *   between -140 dBm for 0 and -44 dBm for 97 (1 dBm steps), or unknown for 255
 */
static inline int modem_cellular_cesq_parse_rsrp(uint8_t rsrp, int16_t *value)
{
	if (rsrp == CESQ_RSRP_UNKNOWN) {
		return -EINVAL;
	}

	*value = (int16_t)CESQ_RSRP_TO_DB(rsrp);
	return 0;
}

static inline int modem_cellular_cesq_parse_rsrq(uint8_t rsrq, int16_t *value)
{
	if (rsrq == CESQ_RSRQ_UNKNOWN) {
		return -EINVAL;
	}

	*value = (int16_t)CESQ_RSRQ_TO_DB(rsrq);
	return 0;
}

static int modem_cellular_get_signal(const struct device *dev,
				     const enum cellular_signal_type type,
				     int16_t *value)
{
	int ret = -ENOTSUP;
	struct modem_cellular_data *data = (struct modem_cellular_data *)dev->data;

	if ((data->state != MODEM_CELLULAR_STATE_READY) &&
	    (data->state != MODEM_CELLULAR_STATE_AWAIT_REGISTERED) &&
	    (data->state != MODEM_CELLULAR_STATE_CARRIER_ON)) {
		return -ENODATA;
	}

	/* Run chat script */
	switch (type) {
	case CELLULAR_SIGNAL_RSSI:
		ret = modem_cellular_run_script_locked(data, &get_signal_csq_chat_script);
		break;

	case CELLULAR_SIGNAL_RSRP:
	case CELLULAR_SIGNAL_RSRQ:
		ret = modem_cellular_run_script_locked(data, &get_signal_cesq_chat_script);
		break;

	default:
		ret = -ENOTSUP;
		break;
	}

	/* Verify chat script ran successfully */
	if (ret < 0) {
		return ret;
	}

	/* Parse received value */
	switch (type) {
	case CELLULAR_SIGNAL_RSSI:
		ret = modem_cellular_csq_parse_rssi(data->rssi, value);
		break;

	case CELLULAR_SIGNAL_RSRP:
		ret = modem_cellular_cesq_parse_rsrp(data->rsrp, value);
		break;

	case CELLULAR_SIGNAL_RSRQ:
		ret = modem_cellular_cesq_parse_rsrq(data->rsrq, value);
		break;

	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static enum simcom7x00_rat modem_cellular_normalize_cellular_rat(
	enum cellular_access_technology tech)
{
	switch (tech) {
	case CELLULAR_ACCESS_TECHNOLOGY_GSM:
	case CELLULAR_ACCESS_TECHNOLOGY_GPRS:
	case CELLULAR_ACCESS_TECHNOLOGY_EDGE:
		return SIMCOM7X00_RAT_GSM;
	case CELLULAR_ACCESS_TECHNOLOGY_UMTS:
		return SIMCOM7X00_RAT_UMTS;
	case CELLULAR_ACCESS_TECHNOLOGY_LTE:
	case CELLULAR_ACCESS_TECHNOLOGY_LTE_CAT_M1:
	case CELLULAR_ACCESS_TECHNOLOGY_LTE_CAT_M2:
	case CELLULAR_ACCESS_TECHNOLOGY_NB_IOT:
		return SIMCOM7X00_RAT_LTE;
	default:
		return SIMCOM7X00_RAT_UNKNOWN;
	}
}

static const struct cellular_network modem_cellular_supported_networks[] = {
	{ .technology = CELLULAR_ACCESS_TECHNOLOGY_GSM, .bands = NULL, .size = 0 },
	{ .technology = CELLULAR_ACCESS_TECHNOLOGY_EDGE, .bands = NULL, .size = 0 },
	{ .technology = CELLULAR_ACCESS_TECHNOLOGY_UMTS, .bands = NULL, .size = 0 },
	{ .technology = CELLULAR_ACCESS_TECHNOLOGY_LTE, .bands = NULL, .size = 0 },
};

static int modem_cellular_get_supported_networks(const struct device *dev,
						 const struct cellular_network **networks,
						 uint8_t *size)
{
	ARG_UNUSED(dev);

	if ((networks == NULL) || (size == NULL)) {
		return -EINVAL;
	}

	*networks = modem_cellular_supported_networks;
	*size = ARRAY_SIZE(modem_cellular_supported_networks);

	return 0;
}

static int modem_cellular_configure_networks(const struct device *dev,
					     const struct cellular_network *networks, uint8_t size)
{
	struct simcom7x00_rat_policy policy = { 0 };
	enum simcom7x00_rat normalized[SIMCOM7X00_RAT_POLICY_MAX_PRIORITIES];

	if ((dev == NULL) || (networks == NULL) || (size == 0U) ||
	    (size > SIMCOM7X00_RAT_POLICY_MAX_PRIORITIES)) {
		return -EINVAL;
	}

	for (uint8_t i = 0; i < size; i++) {
		enum simcom7x00_rat rat;

		if ((networks[i].bands != NULL) || (networks[i].size != 0U)) {
			return -ENOTSUP;
		}

		rat = modem_cellular_normalize_cellular_rat(networks[i].technology);
		if (rat == SIMCOM7X00_RAT_UNKNOWN) {
			return -ENOTSUP;
		}

		for (uint8_t j = 0; j < policy.size; j++) {
			if (normalized[j] == rat) {
				rat = SIMCOM7X00_RAT_UNKNOWN;
				break;
			}
		}

		if (rat == SIMCOM7X00_RAT_UNKNOWN) {
			continue;
		}

		normalized[policy.size] = rat;
		policy.priorities[policy.size++] = rat;
	}

	if (policy.size == 0U) {
		return -EINVAL;
	}

	if (policy.size == 1U) {
		switch (policy.priorities[0]) {
		case SIMCOM7X00_RAT_GSM:
			return simcom7x00_set_rat_mode(dev, SIMCOM7X00_RAT_MODE_2G_ONLY);
		case SIMCOM7X00_RAT_UMTS:
			return simcom7x00_set_rat_mode(dev, SIMCOM7X00_RAT_MODE_3G_ONLY);
		case SIMCOM7X00_RAT_LTE:
			return simcom7x00_set_rat_mode(dev, SIMCOM7X00_RAT_MODE_4G_ONLY);
		default:
			return -EINVAL;
		}
	}

	return simcom7x00_set_rat_policy(dev, &policy);
}

static int modem_cellular_get_modem_info(const struct device *dev,
					 enum cellular_modem_info_type type,
					 char *info, size_t size)
{
	int ret = 0;
	struct modem_cellular_data *data = (struct modem_cellular_data *)dev->data;

	switch (type) {
	case CELLULAR_MODEM_INFO_IMEI:
		strncpy(info, &data->imei[0], MIN(size, sizeof(data->imei)));
		break;
	case CELLULAR_MODEM_INFO_SIM_IMSI:
		strncpy(info, &data->imsi[0], MIN(size, sizeof(data->imsi)));
		break;
	case CELLULAR_MODEM_INFO_MANUFACTURER:
		strncpy(info, &data->manufacturer[0], MIN(size, sizeof(data->manufacturer)));
		break;
	case CELLULAR_MODEM_INFO_FW_VERSION:
		strncpy(info, &data->fw_version[0], MIN(size, sizeof(data->fw_version)));
		break;
	case CELLULAR_MODEM_INFO_MODEL_ID:
		strncpy(info, &data->model_id[0], MIN(size, sizeof(data->model_id)));
		break;
	case CELLULAR_MODEM_INFO_SIM_ICCID:
		strncpy(info, &data->iccid[0], MIN(size, sizeof(data->iccid)));
		break;
	default:
		ret = -ENODATA;
		break;
	}

	return ret;
}
static int modem_cellular_get_registration_status(const struct device *dev,
						  enum cellular_access_technology tech,
						  enum cellular_registration_status *status)
{
	int ret = 0;
	struct modem_cellular_data *data = (struct modem_cellular_data *)dev->data;

	switch (tech) {
	case CELLULAR_ACCESS_TECHNOLOGY_GSM:
		*status = data->registration_status_gsm;
		break;
	case CELLULAR_ACCESS_TECHNOLOGY_GPRS:
	case CELLULAR_ACCESS_TECHNOLOGY_UMTS:
	case CELLULAR_ACCESS_TECHNOLOGY_EDGE:
		*status = data->registration_status_gprs;
		break;
	case CELLULAR_ACCESS_TECHNOLOGY_LTE:
	case CELLULAR_ACCESS_TECHNOLOGY_LTE_CAT_M1:
	case CELLULAR_ACCESS_TECHNOLOGY_LTE_CAT_M2:
	case CELLULAR_ACCESS_TECHNOLOGY_NB_IOT:
		*status = data->registration_status_lte;
		break;
	default:
		ret = -ENODATA;
		break;
	}

	return ret;
}

const static struct cellular_driver_api modem_cellular_api = {
	.configure_networks = modem_cellular_configure_networks,
	.get_supported_networks = modem_cellular_get_supported_networks,
	.get_signal = modem_cellular_get_signal,
	.get_modem_info = modem_cellular_get_modem_info,
	.get_registration_status = modem_cellular_get_registration_status,
};

int simcom7x00_set_signal_poll_interval(const struct device *dev, uint32_t interval_ms)
{
	struct modem_cellular_data *data;

	if (dev == NULL) {
		return -EINVAL;
	}

	if ((interval_ms != 0U) && (interval_ms < 1000U)) {
		return -EINVAL;
	}

	data = dev->data;
	atomic_set(&data->periodic_script_interval_ms, (atomic_val_t)interval_ms);
	LOG_INF("signal/status poll interval set to %u ms", interval_ms);
	modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_PERIODIC_SCRIPT_INTERVAL_CHANGED);

	return 0;
}

int simcom7x00_get_signal_poll_interval(const struct device *dev, uint32_t *interval_ms)
{
	struct modem_cellular_data *data;

	if ((dev == NULL) || (interval_ms == NULL)) {
		return -EINVAL;
	}

	data = dev->data;
	*interval_ms = modem_cellular_get_periodic_script_interval_ms(data);

	return 0;
}

int simcom7x00_ppp_connect(const struct device *dev)
{
	struct modem_cellular_data *data;

	if (dev == NULL) {
		return -EINVAL;
	}

	data = dev->data;

	switch (data->state) {
	case MODEM_CELLULAR_STATE_CARRIER_ON:
		return -EALREADY;
	case MODEM_CELLULAR_STATE_RUN_DIAL_SCRIPT:
	case MODEM_CELLULAR_STATE_AWAIT_REGISTERED:
		return -EINPROGRESS;
	default:
		break;
	}

	data->ppp_requested = true;
	LOG_INF("PPP connect requested in state %s", modem_cellular_state_str(data->state));

	if (!modem_cellular_state_is_ready(data->state)) {
		return simcom7x00_modem_ensure_ready(dev, true);
	}

	modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_PPP_CONNECT_REQUEST);
	return 0;
}

int simcom7x00_ppp_disconnect(const struct device *dev)
{
	struct modem_cellular_data *data;

	if (dev == NULL) {
		return -EINVAL;
	}

	data = dev->data;

	if (!modem_cellular_state_is_ready(data->state)) {
		return -ENODEV;
	}

	if ((data->state == MODEM_CELLULAR_STATE_READY) && !data->ppp_requested) {
		return -EALREADY;
	}

	data->ppp_requested = false;
	LOG_INF("PPP disconnect requested in state %s", modem_cellular_state_str(data->state));
	modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_PPP_DISCONNECT_REQUEST);
	return 0;
}

int simcom7x00_get_ppp_state(const struct device *dev, enum simcom7x00_ppp_state *state)
{
	struct modem_cellular_data *data;

	if ((dev == NULL) || (state == NULL)) {
		return -EINVAL;
	}

	data = dev->data;

	switch (data->state) {
	case MODEM_CELLULAR_STATE_RUN_DIAL_SCRIPT:
		*state = SIMCOM7X00_PPP_STATE_CONNECTING;
		break;
	case MODEM_CELLULAR_STATE_AWAIT_REGISTERED:
		*state = SIMCOM7X00_PPP_STATE_WAITING_REGISTRATION;
		break;
	case MODEM_CELLULAR_STATE_CARRIER_ON:
		*state = SIMCOM7X00_PPP_STATE_CONNECTED;
		break;
	default:
		*state = SIMCOM7X00_PPP_STATE_IDLE;
		break;
	}

	return 0;
}

int simcom7x00_set_apn(const struct device *dev, const char *apn)
{
	struct modem_cellular_data *data;
	const struct modem_cellular_config *config;
	size_t apn_len;
	int ret;

	if (dev == NULL) {
		return -EINVAL;
	}

	ret = modem_cellular_validate_apn(apn, &apn_len);
	if (ret < 0) {
		return ret;
	}

	data = dev->data;
	config = dev->config;

	if (!config->runtime_apn) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->apn_lock, K_FOREVER);
	memcpy(data->apn, apn, apn_len + 1);
	data->apn_len = apn_len;
	k_mutex_unlock(&data->apn_lock);

	return 0;
}

int simcom7x00_get_apn(const struct device *dev, char *buf, size_t len)
{
	struct modem_cellular_data *data;
	const struct modem_cellular_config *config;

	if ((dev == NULL) || (buf == NULL) || (len == 0U)) {
		return -EINVAL;
	}

	data = dev->data;
	config = dev->config;

	if (!config->runtime_apn) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->apn_lock, K_FOREVER);
	if (len <= data->apn_len) {
		k_mutex_unlock(&data->apn_lock);
		return -ENOSPC;
	}

	memcpy(buf, data->apn, data->apn_len + 1);
	k_mutex_unlock(&data->apn_lock);

	return 0;
}

static bool modem_cellular_can_query_network(const struct modem_cellular_data *data)
{
	return (data->state == MODEM_CELLULAR_STATE_READY) ||
	       (data->state == MODEM_CELLULAR_STATE_AWAIT_REGISTERED) ||
	       (data->state == MODEM_CELLULAR_STATE_CARRIER_ON);
}

int simcom7x00_get_network_status(const struct device *dev, struct simcom7x00_network_status *status)
{
	struct modem_cellular_data *data;
	int ret;

	if ((dev == NULL) || (status == NULL)) {
		return -EINVAL;
	}

	data = dev->data;
	if (!modem_cellular_can_query_network(data)) {
		return -ENODATA;
	}

	ret = modem_cellular_refresh_network_snapshot(data);
	if (ret < 0) {
		return ret;
	}

	k_mutex_lock(&data->network_lock, K_FOREVER);
	status->registration_status = data->current_registration_status;
	status->roaming = data->roaming;
	status->current_rat = data->current_rat;
	k_mutex_unlock(&data->network_lock);

	return 0;
}

int simcom7x00_get_operator(const struct device *dev, struct simcom7x00_operator *op)
{
	struct modem_cellular_data *data;
	int ret;

	if ((dev == NULL) || (op == NULL)) {
		return -EINVAL;
	}

	data = dev->data;
	if (!modem_cellular_can_query_network(data)) {
		return -ENODATA;
	}

	ret = modem_cellular_refresh_operator(data);
	k_mutex_lock(&data->network_lock, K_FOREVER);
	memcpy(op->long_name, data->operator_long, sizeof(op->long_name));
	memcpy(op->short_name, data->operator_short, sizeof(op->short_name));
	memcpy(op->plmn, data->operator_plmn, sizeof(op->plmn));
	k_mutex_unlock(&data->network_lock);

	return ret;
}

int simcom7x00_get_current_rat(const struct device *dev, enum simcom7x00_rat *rat)
{
	struct modem_cellular_data *data;
	int ret;

	if ((dev == NULL) || (rat == NULL)) {
		return -EINVAL;
	}

	data = dev->data;
	if (!modem_cellular_can_query_network(data)) {
		return -ENODATA;
	}

	ret = modem_cellular_run_script_locked(data, &simcom7x00_current_rat_chat_script);
	if (ret < 0) {
		return ret;
	}

	k_mutex_lock(&data->network_lock, K_FOREVER);
	*rat = data->current_rat;
	k_mutex_unlock(&data->network_lock);

	return 0;
}

int simcom7x00_set_rat_mode(const struct device *dev, enum simcom7x00_rat_mode mode)
{
	struct modem_cellular_data *data;
	int ret;

	if (dev == NULL) {
		return -EINVAL;
	}

	ret = modem_cellular_mode_to_cnmp(mode);
	if (ret < 0) {
		return ret;
	}

	data = dev->data;

	k_mutex_lock(&data->network_lock, K_FOREVER);
	data->rat_mode = mode;
	data->rat_policy_enabled = false;
	modem_cellular_set_policy_for_mode(data, mode);
	data->rat_config_dirty = true;
	k_mutex_unlock(&data->network_lock);

	if (modem_cellular_can_query_network(data)) {
		return modem_cellular_apply_rat_config(data);
	}

	return 0;
}

int simcom7x00_get_rat_mode(const struct device *dev, enum simcom7x00_rat_mode *mode)
{
	struct modem_cellular_data *data;

	if ((dev == NULL) || (mode == NULL)) {
		return -EINVAL;
	}

	data = dev->data;

	k_mutex_lock(&data->network_lock, K_FOREVER);
	*mode = data->rat_mode;
	k_mutex_unlock(&data->network_lock);

	return 0;
}

int simcom7x00_set_rat_policy(const struct device *dev, const struct simcom7x00_rat_policy *policy)
{
	struct modem_cellular_data *data;
	int ret;

	if (dev == NULL) {
		return -EINVAL;
	}

	ret = modem_cellular_validate_rat_policy(policy);
	if (ret < 0) {
		return ret;
	}

	data = dev->data;

	k_mutex_lock(&data->network_lock, K_FOREVER);
	memcpy(&data->rat_policy, policy, sizeof(data->rat_policy));
	data->rat_mode = modem_cellular_cnmp_to_mode(ret);
	data->rat_policy_enabled = true;
	data->rat_config_dirty = true;
	k_mutex_unlock(&data->network_lock);

	if (modem_cellular_can_query_network(data)) {
		return modem_cellular_apply_rat_config(data);
	}

	return 0;
}

int simcom7x00_get_rat_policy(const struct device *dev, struct simcom7x00_rat_policy *policy)
{
	struct modem_cellular_data *data;

	if ((dev == NULL) || (policy == NULL)) {
		return -EINVAL;
	}

	data = dev->data;

	k_mutex_lock(&data->network_lock, K_FOREVER);
	memcpy(policy, &data->rat_policy, sizeof(*policy));
	k_mutex_unlock(&data->network_lock);

	return 0;
}

int simcom7x00_get_control_diagnostics(const struct device *dev,
				       struct simcom7x00_control_diagnostics *diagnostics)
{
	struct modem_cellular_data *data;

	if ((dev == NULL) || (diagnostics == NULL)) {
		return -EINVAL;
	}

	data = dev->data;

	k_mutex_lock(&data->diagnostics_lock, K_FOREVER);
	memcpy(diagnostics->last_command, data->last_control_command,
	       sizeof(diagnostics->last_command));
	memcpy(diagnostics->last_failure, data->last_control_failure,
	       sizeof(diagnostics->last_failure));
	diagnostics->transaction_id = data->control_transaction_id;
	diagnostics->completed_transaction_id = data->control_completed_transaction_id;
	diagnostics->last_result = data->control_last_result;
	diagnostics->last_cme_error = data->last_cme_error;
	diagnostics->last_cms_error = data->last_cms_error;
	diagnostics->transaction_in_progress = data->control_transaction_in_progress;
	diagnostics->has_cme_error = data->has_last_cme_error;
	diagnostics->has_cms_error = data->has_last_cms_error;
	k_mutex_unlock(&data->diagnostics_lock);

	return 0;
}

int simcom7x00_modem_ensure_ready(const struct device *dev, bool ppp_requested)
{
	struct modem_cellular_data *data;
	int ret = 0;

	if (dev == NULL) {
		return -EINVAL;
	}

	data = dev->data;

	if (modem_cellular_state_is_ready(data->state)) {
		LOG_DBG("ensure_ready satisfied immediately in state %s (ppp_requested=%d)",
			modem_cellular_state_str(data->state), ppp_requested);
		if (ppp_requested && (data->state == MODEM_CELLULAR_STATE_READY)) {
			data->ppp_requested = true;
			modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_PPP_CONNECT_REQUEST);
		}
		return 0;
	}

	data->ppp_requested = ppp_requested;
	data->force_ready_only_on_resume = !ppp_requested;
	k_sem_reset(&data->ready_sem);
	LOG_INF("bringing modem to ready state (ppp_requested=%d)", ppp_requested);

#ifdef CONFIG_PM_DEVICE
	{
		enum pm_device_state pm_state;

			if ((pm_device_state_get(dev, &pm_state) == 0) &&
			    (pm_state == PM_DEVICE_STATE_SUSPENDED)) {
				LOG_DBG("resuming modem from PM suspended state");
				ret = pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME);
				if (ret < 0) {
					LOG_WRN("modem resume action failed: %d", ret);
					return ret;
				}
			} else if (data->state == MODEM_CELLULAR_STATE_IDLE) {
				LOG_DBG("waking idle modem via RESUME event");
				modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_RESUME);
			}
		}
#else
	if (data->state == MODEM_CELLULAR_STATE_IDLE) {
		LOG_DBG("waking idle modem via RESUME event");
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_RESUME);
	}
#endif

	ret = k_sem_take(&data->ready_sem, K_SECONDS(30));
	if (ret < 0) {
		LOG_WRN("timed out waiting for modem ready: %d", ret);
		return ret;
	}

	LOG_INF("modem ready (ppp_requested=%d)", ppp_requested);
	return 0;
}

#if defined(CONFIG_ZTEST)
enum simcom7x00_rat simcom7x00_test_parse_cpsi_rat(const char *system_mode)
{
	return modem_cellular_parse_cpsi_rat(system_mode);
}

enum cellular_registration_status simcom7x00_test_registration_snapshot(
	enum simcom7x00_rat current_rat, enum cellular_registration_status gsm,
	enum cellular_registration_status gprs, enum cellular_registration_status lte)
{
	struct modem_cellular_data data = { 0 };

	data.current_rat = current_rat;
	data.registration_status_gsm = gsm;
	data.registration_status_gprs = gprs;
	data.registration_status_lte = lte;

	return modem_cellular_get_current_registration_status(&data);
}

int simcom7x00_test_operator_cache(void)
{
	struct modem_cellular_data data = { 0 };
	bool cache_hit;
	bool cache_miss;

	k_mutex_init(&data.network_lock);

	modem_cellular_cache_operator_name(&data, "26001", "Test Operator", "Test Operator");
	cache_hit = modem_cellular_apply_cached_operator(&data, "26001");
	cache_miss = modem_cellular_apply_cached_operator(&data, "99999");

	if (!cache_hit) {
		return -EINVAL;
	}

	if (strcmp(data.operator_long, "Test Operator") != 0) {
		return -EIO;
	}

	if (cache_miss) {
		return -EALREADY;
	}

	return 0;
}

int simcom7x00_test_control_arbitration(void)
{
	struct modem_cellular_data data = { 0 };
	int ret;

	k_mutex_init(&data.control_lock);
	k_mutex_init(&data.diagnostics_lock);

	ret = modem_cellular_control_acquire(&data, MODEM_CELLULAR_CONTROL_STATE_BUSY_ASYNC);
	if (ret < 0) {
		return ret;
	}

	ret = modem_cellular_control_acquire(&data, MODEM_CELLULAR_CONTROL_STATE_BUSY_SYNC);
	if (ret != -EBUSY) {
		return -EINVAL;
	}

	modem_cellular_control_release(&data, SIMCOM7X00_CONTROL_RESULT_SUCCESS);

	ret = modem_cellular_control_acquire(&data, MODEM_CELLULAR_CONTROL_STATE_BUSY_SYNC);
	if (ret < 0) {
		return ret;
	}

	modem_cellular_control_release(&data, SIMCOM7X00_CONTROL_RESULT_SUCCESS);

	return 0;
}

int simcom7x00_test_diagnostics_sequence(struct simcom7x00_control_diagnostics *failed,
					 struct simcom7x00_control_diagnostics *succeeded)
{
	struct modem_cellular_data data = { 0 };
	char *cme_argv[] = { "+CME ERROR: ", "515" };

	if ((failed == NULL) || (succeeded == NULL)) {
		return -EINVAL;
	}

	k_mutex_init(&data.control_lock);
	k_mutex_init(&data.diagnostics_lock);

	if (modem_cellular_control_acquire(&data, MODEM_CELLULAR_CONTROL_STATE_BUSY_SYNC) < 0) {
		return -EIO;
	}

	modem_cellular_set_control_command(&data, "AT+FAIL");
	modem_cellular_chat_on_cme_error(NULL, cme_argv, 2, &data);
	modem_cellular_control_release(&data, SIMCOM7X00_CONTROL_RESULT_FAILED);

	failed->transaction_id = data.control_transaction_id;
	failed->completed_transaction_id = data.control_completed_transaction_id;
	failed->last_result = data.control_last_result;
	failed->last_cme_error = data.last_cme_error;
	failed->has_cme_error = data.has_last_cme_error;
	failed->transaction_in_progress = data.control_transaction_in_progress;

	if (modem_cellular_control_acquire(&data, MODEM_CELLULAR_CONTROL_STATE_BUSY_SYNC) < 0) {
		return -EIO;
	}

	modem_cellular_set_control_command(&data, "AT+OK");
	modem_cellular_control_release(&data, SIMCOM7X00_CONTROL_RESULT_SUCCESS);

	succeeded->transaction_id = data.control_transaction_id;
	succeeded->completed_transaction_id = data.control_completed_transaction_id;
	succeeded->last_result = data.control_last_result;
	succeeded->last_cme_error = data.last_cme_error;
	succeeded->has_cme_error = data.has_last_cme_error;
	succeeded->transaction_in_progress = data.control_transaction_in_progress;

	return 0;
}

int simcom7x00_test_periodic_busy_retry(void)
{
	struct modem_cellular_data data = { 0 };

	k_mutex_init(&data.control_lock);
	k_mutex_init(&data.diagnostics_lock);
	k_work_init_delayable(&data.timeout_work, modem_cellular_timeout_handler);
	atomic_set(&data.periodic_script_interval_ms, 1000);

	if (modem_cellular_control_acquire(&data, MODEM_CELLULAR_CONTROL_STATE_BUSY_SYNC) < 0) {
		return -EIO;
	}

	modem_cellular_handle_periodic_timeout(&data, NULL);

	if (!k_work_delayable_is_pending(&data.timeout_work)) {
		modem_cellular_control_release(&data, SIMCOM7X00_CONTROL_RESULT_SUCCESS);
		return -ETIMEDOUT;
	}

	modem_cellular_control_release(&data, SIMCOM7X00_CONTROL_RESULT_SUCCESS);
	k_work_cancel_delayable(&data.timeout_work);

	return 0;
}
#endif

#ifdef CONFIG_PM_DEVICE
static int modem_cellular_pm_action(const struct device *dev, enum pm_device_action action)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)dev->data;
	int ret;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_RESUME);
		ret = 0;
		break;

	case PM_DEVICE_ACTION_SUSPEND:
		modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_SUSPEND);
		ret = k_sem_take(&data->suspended_sem, K_SECONDS(30));
		break;

	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

static int modem_cellular_init(const struct device *dev)
{
	struct modem_cellular_data *data = (struct modem_cellular_data *)dev->data;
	struct modem_cellular_config *config = (struct modem_cellular_config *)dev->config;

	data->dev = dev;

	k_work_init_delayable(&data->timeout_work, modem_cellular_timeout_handler);

	k_work_init(&data->event_dispatch_work, modem_cellular_event_dispatch_handler);
	ring_buf_init(&data->event_rb, sizeof(data->event_buf), data->event_buf);
	k_mutex_init(&data->event_rb_lock);
	k_mutex_init(&data->apn_lock);
	k_mutex_init(&data->network_lock);
	k_mutex_init(&data->control_lock);
	k_mutex_init(&data->diagnostics_lock);

	k_sem_init(&data->suspended_sem, 0, 1);
	k_sem_init(&data->ready_sem, 0, 1);
	atomic_set(&data->periodic_script_interval_ms, CONFIG_MODEM_SIMCOM7X00_PERIODIC_SCRIPT_MS);
	data->ppp_requested = false;
	data->force_ready_only_on_resume = false;
	data->current_rat = SIMCOM7X00_RAT_UNKNOWN;
	data->current_registration_status = CELLULAR_REGISTRATION_UNKNOWN;
	data->roaming = false;
	data->operator_valid = false;
	data->current_rat_valid = false;
	data->rat_mode = SIMCOM7X00_RAT_MODE_AUTO;
	modem_cellular_set_policy_for_mode(data, data->rat_mode);
	if (IS_ENABLED(CONFIG_MODEM_SIMCOM7X00_DEFAULT_MAX_RELIABILITY)) {
		data->rat_policy_enabled = true;
		data->rat_config_dirty = true;
		LOG_INF("using maximum reliability radio defaults");
	} else {
		data->rat_policy_enabled = false;
		data->rat_config_dirty = false;
	}

	if (config->runtime_apn) {
		int ret = modem_cellular_init_runtime_apn(data);

		if (ret < 0) {
			return ret;
		}
	}

	if (modem_cellular_gpio_is_enabled(&config->power_gpio)) {
		gpio_pin_configure_dt(&config->power_gpio, GPIO_OUTPUT_INACTIVE);
	}

	if (modem_cellular_gpio_is_enabled(&config->reset_gpio)) {
		gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
	}

	{
		const struct modem_backend_uart_config uart_backend_config = {
			.uart = config->uart,
			.receive_buf = data->uart_backend_receive_buf,
			.receive_buf_size = ARRAY_SIZE(data->uart_backend_receive_buf),
			.transmit_buf = data->uart_backend_transmit_buf,
			.transmit_buf_size = ARRAY_SIZE(data->uart_backend_transmit_buf),
		};

		data->uart_pipe = modem_backend_uart_init(&data->uart_backend,
							  &uart_backend_config);
	}

	{
		const struct modem_cmux_config cmux_config = {
			.callback = modem_cellular_cmux_handler,
			.user_data = data,
			.receive_buf = data->cmux_receive_buf,
			.receive_buf_size = ARRAY_SIZE(data->cmux_receive_buf),
			.transmit_buf = data->cmux_transmit_buf,
			.transmit_buf_size = ARRAY_SIZE(data->cmux_transmit_buf),
		};

		modem_cmux_init(&data->cmux, &cmux_config);
	}

	{
		const struct modem_cmux_dlci_config dlci1_config = {
			.dlci_address = 1,
			.receive_buf = data->dlci1_receive_buf,
			.receive_buf_size = ARRAY_SIZE(data->dlci1_receive_buf),
		};

		data->dlci1_pipe = modem_cmux_dlci_init(&data->cmux, &data->dlci1,
							&dlci1_config);
	}

	{
		const struct modem_cmux_dlci_config dlci2_config = {
			.dlci_address = 2,
			.receive_buf = data->dlci2_receive_buf,
			.receive_buf_size = ARRAY_SIZE(data->dlci2_receive_buf),
		};

		data->dlci2_pipe = modem_cmux_dlci_init(&data->cmux, &data->dlci2,
							&dlci2_config);
	}

	for (uint8_t i = 0; i < config->user_pipes_size; i++) {
		struct modem_cellular_user_pipe *user_pipe = &config->user_pipes[i];
		const struct modem_cmux_dlci_config user_dlci_config = {
			.dlci_address = user_pipe->dlci_address,
			.receive_buf = user_pipe->dlci_receive_buf,
			.receive_buf_size = user_pipe->dlci_receive_buf_size,
		};

		user_pipe->pipe = modem_cmux_dlci_init(&data->cmux, &user_pipe->dlci,
						       &user_dlci_config);

		modem_pipelink_init(user_pipe->pipelink, user_pipe->pipe);
	}

	{
		const struct modem_chat_config chat_config = {
			.user_data = data,
			.receive_buf = data->chat_receive_buf,
			.receive_buf_size = ARRAY_SIZE(data->chat_receive_buf),
			.delimiter = data->chat_delimiter,
			.delimiter_size = strlen(data->chat_delimiter),
			.filter = data->chat_filter,
			.filter_size = data->chat_filter ? strlen(data->chat_filter) : 0,
			.argv = data->chat_argv,
			.argv_size = ARRAY_SIZE(data->chat_argv),
			.unsol_matches = unsol_matches,
			.unsol_matches_size = ARRAY_SIZE(unsol_matches),
		};

		modem_chat_init(&data->chat, &chat_config);
	}

#ifndef CONFIG_PM_DEVICE
	modem_cellular_delegate_event(data, MODEM_CELLULAR_EVENT_RESUME);
#else
	pm_device_init_suspended(dev);
#endif /* CONFIG_PM_DEVICE */

	return 0;
}

/*
 * Every modem uses two custom scripts to initialize the modem and dial out.
 *
 * The first script is named <dt driver compatible>_init_chat_script, with its
 * script commands named <dt driver compatible>_init_chat_script_cmds. This
 * script is sent to the modem after it has started up, and must configure the
 * modem to use CMUX.
 *
 * The second script is named <dt driver compatible>_dial_chat_script, with its
 * script commands named <dt driver compatible>_dial_chat_script_cmds. This
 * script is sent on a DLCI channel in command mode, and must request the modem
 * dial out and put the DLCI channel into data mode.
 */

#if DT_HAS_COMPAT_STATUS_OKAY(quectel_bg95)
MODEM_CHAT_SCRIPT_CMDS_DEFINE(quectel_bg95_init_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("ATE0", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=4", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CMEE=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CNSMOD=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGSN", imei_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMM", cgmm_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMI", cgmi_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMR", cgmr_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CIMI", cimi_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+QCCID", qccid_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT+CMUX=0,0,5,127", 300));

MODEM_CHAT_SCRIPT_DEFINE(quectel_bg95_init_chat_script, quectel_bg95_init_chat_script_cmds,
			 abort_matches, modem_cellular_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(quectel_bg95_dial_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CGACT=0,1", allow_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGDCONT=1,\"IP\","
							 "\"" CONFIG_MODEM_SIMCOM7X00_APN "\"",
							 ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("ATD*99***1#", 0),);

MODEM_CHAT_SCRIPT_DEFINE(quectel_bg95_dial_chat_script, quectel_bg95_dial_chat_script_cmds,
			 dial_abort_matches, modem_cellular_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(quectel_bg95_periodic_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG?", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(quectel_bg95_periodic_chat_script,
			 quectel_bg95_periodic_chat_script_cmds, abort_matches,
			 modem_cellular_chat_callback_handler, 4);
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(quectel_eg25_g)
MODEM_CHAT_SCRIPT_CMDS_DEFINE(
	quectel_eg25_g_init_chat_script_cmds, MODEM_CHAT_SCRIPT_CMD_RESP("ATE0", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=4", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CMEE=1", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG=1", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG=1", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG=1", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG?", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG?", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGSN", imei_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMM", cgmm_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMI", cgmi_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMR", cgmr_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("AT+CIMI", cimi_match),
	MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
	MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT+CMUX=0,0,5,127,10,3,30,10,2", 100));

MODEM_CHAT_SCRIPT_DEFINE(quectel_eg25_g_init_chat_script, quectel_eg25_g_init_chat_script_cmds,
			 abort_matches, modem_cellular_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(quectel_eg25_g_dial_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CGACT=0,1", allow_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGDCONT=1,\"IP\","
							 "\"" CONFIG_MODEM_SIMCOM7X00_APN "\"",
							 ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("ATD*99***1#", 0),);

MODEM_CHAT_SCRIPT_DEFINE(quectel_eg25_g_dial_chat_script, quectel_eg25_g_dial_chat_script_cmds,
			 dial_abort_matches, modem_cellular_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(quectel_eg25_g_periodic_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CSQ", csq_match));

MODEM_CHAT_SCRIPT_DEFINE(quectel_eg25_g_periodic_chat_script,
			 quectel_eg25_g_periodic_chat_script_cmds, abort_matches,
			 modem_cellular_chat_callback_handler, 4);
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(simcom_sim7x00)
#define SIMCOM7X00_CMUX_PORT_SPEED 5
#define SIMCOM7X00_CMUX_T2_MS 600
#define SIMCOM7X00_URC_DEST_CMUX_VIRTUAL_PORT2 5

/*
 * SIM7500/SIM7600 exposes AT+CICCID for SIM ICCID reads and AT+CATR for URC
 * routing. Keep unsolicited registration updates on CMUX virtual port 2 so
 * they land on the dedicated chat channel instead of leaking onto PPP or the
 * GNSS side pipe.
 */
MODEM_CHAT_SCRIPT_CMDS_DEFINE(simcom_sim7x00_init_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
			      MODEM_CHAT_SCRIPT_CMD_RESP("ATE0", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CMEE=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CSCLK=0", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=4", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGSN", imei_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMM", cgmm_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMI", cgmi_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMR", cgmr_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CIMI", cimi_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CICCID", iccid_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CATR="
							 STRINGIFY(SIMCOM7X00_URC_DEST_CMUX_VIRTUAL_PORT2),
							 ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT+CMUX=0,0,"
							      STRINGIFY(SIMCOM7X00_CMUX_PORT_SPEED)
							      ","
							      STRINGIFY(CONFIG_MODEM_SIMCOM7X00_CMUX_MAX_FRAME_SIZE)
							      ",0,0,"
							      STRINGIFY(SIMCOM7X00_CMUX_T2_MS),
							      300));

MODEM_CHAT_SCRIPT_DEFINE(simcom_sim7x00_init_chat_script, simcom_sim7x00_init_chat_script_cmds,
			 abort_matches, modem_cellular_chat_callback_handler, 15);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(simcom_sim7x00_dial_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CGACT=0,1", allow_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGDCONT=1,\"IP\","
							 "\"" CONFIG_MODEM_SIMCOM7X00_APN "\"",
							 ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGATT=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("ATD*99***1#", 0),);

MODEM_CHAT_SCRIPT_DEFINE(simcom_sim7x00_dial_chat_script, simcom_sim7x00_dial_chat_script_cmds,
			 dial_abort_matches, modem_cellular_chat_callback_handler, 15);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(simcom_sim7x00_periodic_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CSQ", csq_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CESQ", cesq_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CPSI?", cpsi_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CNSMOD?", cnsmod_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(simcom_sim7x00_periodic_chat_script,
			 simcom_sim7x00_periodic_chat_script_cmds, abort_matches,
			 modem_cellular_chat_callback_handler, 4);
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(u_blox_sara_r4)
MODEM_CHAT_SCRIPT_CMDS_DEFINE(u_blox_sara_r4_init_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
			      MODEM_CHAT_SCRIPT_CMD_RESP("ATE0", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=4", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CMEE=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGSN", imei_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMM", cgmm_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CMUX=0,0,5,127", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(u_blox_sara_r4_init_chat_script, u_blox_sara_r4_init_chat_script_cmds,
			 abort_matches, modem_cellular_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(u_blox_sara_r4_dial_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CGACT=0,1", allow_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGDCONT=1,\"IP\","
							 "\"" CONFIG_MODEM_SIMCOM7X00_APN "\"",
							 ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("ATD*99***1#", 0),);

MODEM_CHAT_SCRIPT_DEFINE(u_blox_sara_r4_dial_chat_script, u_blox_sara_r4_dial_chat_script_cmds,
			 dial_abort_matches, modem_cellular_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(u_blox_sara_r4_periodic_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG?", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(u_blox_sara_r4_periodic_chat_script,
			 u_blox_sara_r4_periodic_chat_script_cmds, abort_matches,
			 modem_cellular_chat_callback_handler, 4);
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(u_blox_sara_r5)
MODEM_CHAT_SCRIPT_CMDS_DEFINE(u_blox_sara_r5_init_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
			      MODEM_CHAT_SCRIPT_CMD_RESP("ATE0", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=4", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CMEE=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGSN", imei_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMM", cgmm_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMI", cgmi_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMR", cgmr_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CIMI", cimi_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CMUX=0,0,5,127", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(u_blox_sara_r5_init_chat_script, u_blox_sara_r5_init_chat_script_cmds,
			 abort_matches, modem_cellular_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(u_blox_sara_r5_dial_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CGACT=0,1", allow_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGDCONT=1,\"IP\","
							 "\"" CONFIG_MODEM_SIMCOM7X00_APN "\"",
							 ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("ATD*99***1#", 0),);

MODEM_CHAT_SCRIPT_DEFINE(u_blox_sara_r5_dial_chat_script, u_blox_sara_r5_dial_chat_script_cmds,
			 dial_abort_matches, modem_cellular_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(u_blox_sara_r5_periodic_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG?", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(u_blox_sara_r5_periodic_chat_script,
			 u_blox_sara_r5_periodic_chat_script_cmds, abort_matches,
			 modem_cellular_chat_callback_handler, 4);
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(swir_hl7800)
MODEM_CHAT_SCRIPT_CMDS_DEFINE(swir_hl7800_init_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
			      MODEM_CHAT_SCRIPT_CMD_RESP("ATE0", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CGACT=0", allow_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=4", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CMEE=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGSN", imei_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMM", cgmm_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMI", cgmi_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMR", cgmr_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CIMI", cimi_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT+CMUX=0,0,5,127", 0));

MODEM_CHAT_SCRIPT_DEFINE(swir_hl7800_init_chat_script, swir_hl7800_init_chat_script_cmds,
			 abort_matches, modem_cellular_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(swir_hl7800_dial_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGDCONT=1,\"IP\","
							 "\"" CONFIG_MODEM_SIMCOM7X00_APN "\"",
							 ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+KCNXCFG=1,\"GPRS\",\""
							 CONFIG_MODEM_SIMCOM7X00_APN
							 "\",,,\"IPV4\"",
							 ok_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("AT+WPPP=0", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("ATD*99***1#", connect_match));

MODEM_CHAT_SCRIPT_CMDS_DEFINE(swir_hl7800_periodic_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(swir_hl7800_periodic_chat_script,
			 swir_hl7800_periodic_chat_script_cmds, abort_matches,
			 modem_cellular_chat_callback_handler, 4);

MODEM_CHAT_SCRIPT_DEFINE(swir_hl7800_dial_chat_script, swir_hl7800_dial_chat_script_cmds,
			 dial_abort_matches, modem_cellular_chat_callback_handler, 10);
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(telit_me910g1)
MODEM_CHAT_SCRIPT_CMDS_DEFINE(telit_me910g1_init_chat_script_cmds,
				  MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
				  MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
				  MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
				  MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT", 100),
				  MODEM_CHAT_SCRIPT_CMD_RESP("ATE0", ok_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("AT+ICCID", iccid_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("AT+CIMI", cimi_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
				  /* The Telit me910g1 often has an error trying
				   * to set the PDP context. The radio must be on to set
				   * the context, and this step must be successful.
				   * It is moved to the init script to allow retries.
				   */
				  MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGDCONT=1,\"IP\","
							 "\"" CONFIG_MODEM_SIMCOM7X00_APN "\"",
							 ok_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=4", ok_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("AT+CMEE=1", ok_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG=1", ok_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG=1", ok_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG=1", ok_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG?", ok_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG?", ok_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGSN", imei_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMM", cgmm_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMI", cgmi_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMR", cgmr_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=1", ok_match),
				  MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT+CMUX=0,0,5,127,10,3,30,10,2",
								  300));

MODEM_CHAT_SCRIPT_DEFINE(telit_me910g1_init_chat_script, telit_me910g1_init_chat_script_cmds,
			 abort_matches, modem_cellular_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(telit_me910g1_dial_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("ATD*99***1#", 0));

MODEM_CHAT_SCRIPT_DEFINE(telit_me910g1_dial_chat_script, telit_me910g1_dial_chat_script_cmds,
			 dial_abort_matches, modem_cellular_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(telit_me910g1_periodic_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(telit_me910g1_periodic_chat_script,
			 telit_me910g1_periodic_chat_script_cmds, abort_matches,
			 modem_cellular_chat_callback_handler, 4);
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf91_slm)
MODEM_CHAT_SCRIPT_CMDS_DEFINE(nordic_nrf91_slm_init_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT", allow_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CMEE=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGSN", imei_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMM", cgmm_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMI", cgmi_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMR", cgmr_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT#XCMUX=1", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(nordic_nrf91_slm_init_chat_script, nordic_nrf91_slm_init_chat_script_cmds,
			 abort_matches, modem_cellular_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(nordic_nrf91_slm_dial_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=4", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT#XCMUX=2", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(nordic_nrf91_slm_dial_chat_script, nordic_nrf91_slm_dial_chat_script_cmds,
			 dial_abort_matches, modem_cellular_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(nordic_nrf91_slm_periodic_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(nordic_nrf91_slm_periodic_chat_script,
			 nordic_nrf91_slm_periodic_chat_script_cmds, abort_matches,
			 modem_cellular_chat_callback_handler, 4);
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(sqn_gm02s)
MODEM_CHAT_SCRIPT_CMDS_DEFINE(sqn_gm02s_init_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("ATE0", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=4", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CMEE=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG=1", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGSN", imei_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMM", cgmm_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMI", cgmi_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMR", cgmr_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("", ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CMUX=0,0,5,127", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(sqn_gm02s_init_chat_script, sqn_gm02s_init_chat_script_cmds,
			 abort_matches, modem_cellular_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(sqn_gm02s_dial_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CGACT=0,1", allow_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGDCONT=1,\"IP\","
							 "\"" CONFIG_MODEM_SIMCOM7X00_APN "\"",
							 ok_match),
			      MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT+CFUN=1", 10000),
			      MODEM_CHAT_SCRIPT_CMD_RESP("ATD*99***1#", connect_match));

MODEM_CHAT_SCRIPT_DEFINE(sqn_gm02s_dial_chat_script, sqn_gm02s_dial_chat_script_cmds,
			 dial_abort_matches, modem_cellular_chat_callback_handler, 15);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(sqn_gm02s_periodic_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", ok_match));

MODEM_CHAT_SCRIPT_DEFINE(sqn_gm02s_periodic_chat_script,
			 sqn_gm02s_periodic_chat_script_cmds, abort_matches,
			 modem_cellular_chat_callback_handler, 4);
#endif

#define MODEM_CELLULAR_INST_NAME(name, inst) \
	_CONCAT_4(name, _, DT_DRV_COMPAT, inst)

#define MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(inst, name, size)                                     \
	MODEM_PIPELINK_DT_INST_DEFINE(inst, name);                                                 \
	static uint8_t MODEM_CELLULAR_INST_NAME(name, inst)[size]                                  \

#define MODEM_CELLULAR_INIT_USER_PIPE(_inst, _name, _dlci_address)                                 \
	{                                                                                          \
		.dlci_address = _dlci_address,                                                     \
		.dlci_receive_buf = MODEM_CELLULAR_INST_NAME(_name, _inst),                        \
		.dlci_receive_buf_size = sizeof(MODEM_CELLULAR_INST_NAME(_name, _inst)),           \
		.pipelink = MODEM_PIPELINK_DT_INST_GET(_inst, _name),                              \
	}

#define MODEM_CELLULAR_DEFINE_USER_PIPES(inst, ...)                                                \
	static struct modem_cellular_user_pipe MODEM_CELLULAR_INST_NAME(user_pipes, inst)[] = {    \
		__VA_ARGS__                                                                        \
	}

#define MODEM_CELLULAR_GET_USER_PIPES(inst) \
	MODEM_CELLULAR_INST_NAME(user_pipes, inst)

#define MODEM_CELLULAR_DEVICE_QUECTEL_BG95(inst)                                                   \
	MODEM_PPP_DEFINE(MODEM_CELLULAR_INST_NAME(ppp, inst), NULL, 98, 1500, 64);                 \
                                                                                                   \
	static struct modem_cellular_data MODEM_CELLULAR_INST_NAME(data, inst) = {                 \
		.chat_delimiter = "\r",                                                            \
		.chat_filter = "\n",                                                               \
		.ppp = &MODEM_CELLULAR_INST_NAME(ppp, inst),                                       \
	};                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(                                                      \
		inst,                                                                              \
		user_pipe_0,                                                                       \
		CONFIG_MODEM_SIMCOM7X00_USER_PIPE_BUFFER_SIZES                                     \
	);                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(                                                      \
		inst,                                                                              \
		user_pipe_1,                                                                       \
		CONFIG_MODEM_SIMCOM7X00_USER_PIPE_BUFFER_SIZES                                     \
	);                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPES(                                                          \
		inst,                                                                              \
		MODEM_CELLULAR_INIT_USER_PIPE(inst, user_pipe_0, 3),                               \
		MODEM_CELLULAR_INIT_USER_PIPE(inst, user_pipe_1, 4),                               \
	);                                                                                         \
                                                                                                   \
	static const struct modem_cellular_config MODEM_CELLULAR_INST_NAME(config, inst) = {       \
		.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                                          \
		.power_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_power_gpios, {}),                 \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_reset_gpios, {}),                 \
		.power_pulse_duration_ms = 1500,                                                   \
		.reset_pulse_duration_ms = 100,                                                    \
		.startup_time_ms = 10000,                                                          \
		.shutdown_time_ms = 5000,                                                          \
		.init_chat_script = &quectel_bg95_init_chat_script,                                \
		.dial_chat_script = &quectel_bg95_dial_chat_script,                                \
		.periodic_chat_script = &_CONCAT(DT_DRV_COMPAT, _periodic_chat_script),            \
		.user_pipes = MODEM_CELLULAR_GET_USER_PIPES(inst),                                 \
		.user_pipes_size = ARRAY_SIZE(MODEM_CELLULAR_GET_USER_PIPES(inst)),                \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, modem_cellular_pm_action);                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, modem_cellular_init, PM_DEVICE_DT_INST_GET(inst),              \
			      &MODEM_CELLULAR_INST_NAME(data, inst),                               \
			      &MODEM_CELLULAR_INST_NAME(config, inst), POST_KERNEL, 70,            \
			      &modem_cellular_api);

#define MODEM_CELLULAR_DEVICE_QUECTEL_EG25_G(inst)                                                 \
	MODEM_PPP_DEFINE(MODEM_CELLULAR_INST_NAME(ppp, inst), NULL, 98, 1500, 64);                 \
                                                                                                   \
	static struct modem_cellular_data MODEM_CELLULAR_INST_NAME(data, inst) = {                 \
		.chat_delimiter = "\r",                                                            \
		.chat_filter = "\n",                                                               \
		.ppp = &MODEM_CELLULAR_INST_NAME(ppp, inst),                                       \
	};                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(                                                      \
		inst,                                                                              \
		user_pipe_0,                                                                       \
		CONFIG_MODEM_SIMCOM7X00_USER_PIPE_BUFFER_SIZES                                     \
	);                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(                                                      \
		inst,                                                                              \
		user_pipe_1,                                                                       \
		CONFIG_MODEM_SIMCOM7X00_USER_PIPE_BUFFER_SIZES                                     \
	);                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPES(                                                          \
		inst,                                                                              \
		MODEM_CELLULAR_INIT_USER_PIPE(inst, user_pipe_0, 3),                               \
		MODEM_CELLULAR_INIT_USER_PIPE(inst, user_pipe_1, 4),                               \
	);                                                                                         \
                                                                                                   \
	static const struct modem_cellular_config MODEM_CELLULAR_INST_NAME(config, inst) = {       \
		.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                                          \
		.power_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_power_gpios, {}),                 \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_reset_gpios, {}),                 \
		.power_pulse_duration_ms = 1500,                                                   \
		.reset_pulse_duration_ms = 500,                                                    \
		.startup_time_ms = 15000,                                                          \
		.shutdown_time_ms = 5000,                                                          \
		.init_chat_script = &quectel_eg25_g_init_chat_script,                              \
		.dial_chat_script = &quectel_eg25_g_dial_chat_script,                              \
		.periodic_chat_script = &_CONCAT(DT_DRV_COMPAT, _periodic_chat_script),            \
		.user_pipes = MODEM_CELLULAR_GET_USER_PIPES(inst),                                 \
		.user_pipes_size = ARRAY_SIZE(MODEM_CELLULAR_GET_USER_PIPES(inst)),                \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, modem_cellular_pm_action);                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, modem_cellular_init, PM_DEVICE_DT_INST_GET(inst),              \
			      &MODEM_CELLULAR_INST_NAME(data, inst),                               \
			      &MODEM_CELLULAR_INST_NAME(config, inst), POST_KERNEL, 70,            \
			      &modem_cellular_api);

#define MODEM_CELLULAR_DEVICE_SIMCOM_SIM7X00(inst)                                                 \
	MODEM_PPP_DEFINE(MODEM_CELLULAR_INST_NAME(ppp, inst), NULL, 98, 1500, 64);                 \
                                                                                                   \
	static struct modem_cellular_data MODEM_CELLULAR_INST_NAME(data, inst) = {                 \
		.chat_delimiter = "\r",                                                            \
		.chat_filter = "\n",                                                               \
		.ppp = &MODEM_CELLULAR_INST_NAME(ppp, inst),                                       \
	};                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(                                                      \
		inst,                                                                              \
		gnss_pipe,                                                                         \
		CONFIG_MODEM_SIMCOM7X00_USER_PIPE_BUFFER_SIZES                                     \
	);                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(                                                      \
		inst,                                                                              \
		user_pipe_0,                                                                       \
		CONFIG_MODEM_SIMCOM7X00_USER_PIPE_BUFFER_SIZES                                     \
	);                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPES(                                                          \
		inst,                                                                              \
		MODEM_CELLULAR_INIT_USER_PIPE(inst, gnss_pipe, 3),                                 \
		MODEM_CELLULAR_INIT_USER_PIPE(inst, user_pipe_0, 4),                               \
	);                                                                                         \
                                                                                                   \
	static const struct modem_cellular_config MODEM_CELLULAR_INST_NAME(config, inst) = {       \
		.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                                          \
		.power_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_power_gpios, {}),                 \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_reset_gpios, {}),                 \
		.autostarts = true,                                                                \
		.default_ppp_autostart = CONFIG_MODEM_SIMCOM7X00_DEFAULT_PPP_AUTOSTART,           \
		.runtime_apn = true,                                                               \
		.power_pulse_duration_ms = 1500,                                                   \
		.reset_pulse_duration_ms = 100,                                                    \
		.startup_time_ms = 15000,                                                          \
		.shutdown_time_ms = 5000,                                                          \
		.init_chat_script = &simcom_sim7x00_init_chat_script,                              \
		.dial_chat_script = &simcom_sim7x00_dial_chat_script,                              \
		.periodic_chat_script = &simcom_sim7x00_periodic_chat_script,                      \
		.user_pipes = MODEM_CELLULAR_GET_USER_PIPES(inst),                                 \
		.user_pipes_size = ARRAY_SIZE(MODEM_CELLULAR_GET_USER_PIPES(inst)),                \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, modem_cellular_pm_action);                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, modem_cellular_init, PM_DEVICE_DT_INST_GET(inst),              \
			      &MODEM_CELLULAR_INST_NAME(data, inst),                               \
			      &MODEM_CELLULAR_INST_NAME(config, inst), POST_KERNEL, 99,            \
			      &modem_cellular_api);

#define MODEM_CELLULAR_DEVICE_U_BLOX_SARA_R4(inst)                                                 \
	MODEM_PPP_DEFINE(MODEM_CELLULAR_INST_NAME(ppp, inst), NULL, 98, 1500, 64);                 \
                                                                                                   \
	static struct modem_cellular_data MODEM_CELLULAR_INST_NAME(data, inst) = {                 \
		.chat_delimiter = "\r",                                                            \
		.chat_filter = "\n",                                                               \
		.ppp = &MODEM_CELLULAR_INST_NAME(ppp, inst),                                       \
	};                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(                                                      \
		inst,                                                                              \
		gnss_pipe,                                                                         \
		CONFIG_MODEM_SIMCOM7X00_USER_PIPE_BUFFER_SIZES                                     \
	);                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(                                                      \
		inst,                                                                              \
		user_pipe_0,                                                                       \
		CONFIG_MODEM_SIMCOM7X00_USER_PIPE_BUFFER_SIZES                                     \
	);                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPES(                                                          \
		inst,                                                                              \
		MODEM_CELLULAR_INIT_USER_PIPE(inst, gnss_pipe, 3),                                 \
		MODEM_CELLULAR_INIT_USER_PIPE(inst, user_pipe_0, 4),                               \
	);                                                                                         \
                                                                                                   \
	static const struct modem_cellular_config MODEM_CELLULAR_INST_NAME(config, inst) = {       \
		.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                                          \
		.power_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_power_gpios, {}),                 \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_reset_gpios, {}),                 \
		.power_pulse_duration_ms = 1500,                                                   \
		.reset_pulse_duration_ms = 100,                                                    \
		.startup_time_ms = 10000,                                                          \
		.shutdown_time_ms = 5000,                                                          \
		.init_chat_script = &u_blox_sara_r4_init_chat_script,                              \
		.dial_chat_script = &u_blox_sara_r4_dial_chat_script,                              \
		.periodic_chat_script = &u_blox_sara_r4_periodic_chat_script,                      \
		.user_pipes = MODEM_CELLULAR_GET_USER_PIPES(inst),                                 \
		.user_pipes_size = ARRAY_SIZE(MODEM_CELLULAR_GET_USER_PIPES(inst)),                \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, modem_cellular_pm_action);                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, modem_cellular_init, PM_DEVICE_DT_INST_GET(inst),              \
			      &MODEM_CELLULAR_INST_NAME(data, inst),                               \
			      &MODEM_CELLULAR_INST_NAME(config, inst), POST_KERNEL, 99,            \
			      &modem_cellular_api);

#define MODEM_CELLULAR_DEVICE_U_BLOX_SARA_R5(inst)                                                 \
	MODEM_PPP_DEFINE(MODEM_CELLULAR_INST_NAME(ppp, inst), NULL, 98, 1500, 64);                 \
                                                                                                   \
	static struct modem_cellular_data MODEM_CELLULAR_INST_NAME(data, inst) = {                 \
		.chat_delimiter = "\r",                                                            \
		.chat_filter = "\n",                                                               \
		.ppp = &MODEM_CELLULAR_INST_NAME(ppp, inst),                                       \
	};                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(                                                      \
		inst,                                                                              \
		gnss_pipe,                                                                         \
		CONFIG_MODEM_SIMCOM7X00_USER_PIPE_BUFFER_SIZES                                     \
	);                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(                                                      \
		inst,                                                                              \
		user_pipe_0,                                                                       \
		CONFIG_MODEM_SIMCOM7X00_USER_PIPE_BUFFER_SIZES                                     \
	);                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPES(                                                          \
		inst,                                                                              \
		MODEM_CELLULAR_INIT_USER_PIPE(inst, gnss_pipe, 4),                                 \
		MODEM_CELLULAR_INIT_USER_PIPE(inst, user_pipe_0, 3),                               \
	);                                                                                         \
                                                                                                   \
	static const struct modem_cellular_config MODEM_CELLULAR_INST_NAME(config, inst) = {       \
		.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                                          \
		.power_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_power_gpios, {}),                 \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_reset_gpios, {}),                 \
		.autostarts = true,                                                                \
		.power_pulse_duration_ms = 1500,                                                   \
		.reset_pulse_duration_ms = 100,                                                    \
		.startup_time_ms = 1500,                                                           \
		.shutdown_time_ms = 13000,                                                         \
		.init_chat_script = &u_blox_sara_r5_init_chat_script,                              \
		.dial_chat_script = &u_blox_sara_r5_dial_chat_script,                              \
		.periodic_chat_script = &u_blox_sara_r5_periodic_chat_script,                      \
		.user_pipes = MODEM_CELLULAR_GET_USER_PIPES(inst),                                 \
		.user_pipes_size = ARRAY_SIZE(MODEM_CELLULAR_GET_USER_PIPES(inst)),                \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, modem_cellular_pm_action);                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, modem_cellular_init, PM_DEVICE_DT_INST_GET(inst),              \
			      &MODEM_CELLULAR_INST_NAME(data, inst),                               \
			      &MODEM_CELLULAR_INST_NAME(config, inst), POST_KERNEL, 99,            \
			      &modem_cellular_api);

#define MODEM_CELLULAR_DEVICE_SWIR_HL7800(inst)                                                    \
	MODEM_PPP_DEFINE(MODEM_CELLULAR_INST_NAME(ppp, inst), NULL, 98, 1500, 64);                 \
                                                                                                   \
	static struct modem_cellular_data MODEM_CELLULAR_INST_NAME(data, inst) = {                 \
		.chat_delimiter = "\r",                                                            \
		.chat_filter = "\n",                                                               \
		.ppp = &MODEM_CELLULAR_INST_NAME(ppp, inst),                                       \
	};                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(                                                      \
		inst,                                                                              \
		user_pipe_0,                                                                       \
		CONFIG_MODEM_SIMCOM7X00_USER_PIPE_BUFFER_SIZES                                     \
	);                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(                                                      \
		inst,                                                                              \
		user_pipe_1,                                                                       \
		CONFIG_MODEM_SIMCOM7X00_USER_PIPE_BUFFER_SIZES                                     \
	);                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPES(                                                          \
		inst,                                                                              \
		MODEM_CELLULAR_INIT_USER_PIPE(inst, user_pipe_0, 3),                               \
		MODEM_CELLULAR_INIT_USER_PIPE(inst, user_pipe_1, 4),                               \
	);                                                                                         \
                                                                                                   \
	static const struct modem_cellular_config MODEM_CELLULAR_INST_NAME(config, inst) = {       \
		.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                                          \
		.power_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_power_gpios, {}),                 \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_reset_gpios, {}),                 \
		.power_pulse_duration_ms = 1500,                                                   \
		.reset_pulse_duration_ms = 100,                                                    \
		.startup_time_ms = 10000,                                                          \
		.shutdown_time_ms = 5000,                                                          \
		.init_chat_script = &swir_hl7800_init_chat_script,                                 \
		.dial_chat_script = &swir_hl7800_dial_chat_script,                                 \
		.periodic_chat_script = &swir_hl7800_periodic_chat_script,                         \
		.user_pipes = MODEM_CELLULAR_GET_USER_PIPES(inst),                                 \
		.user_pipes_size = ARRAY_SIZE(MODEM_CELLULAR_GET_USER_PIPES(inst)),                \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, modem_cellular_pm_action);                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, modem_cellular_init, PM_DEVICE_DT_INST_GET(inst),              \
			      &MODEM_CELLULAR_INST_NAME(data, inst),                               \
			      &MODEM_CELLULAR_INST_NAME(config, inst), POST_KERNEL, 99,            \
			      &modem_cellular_api);

#define MODEM_CELLULAR_DEVICE_TELIT_ME910G1(inst)                                                  \
	MODEM_PPP_DEFINE(MODEM_CELLULAR_INST_NAME(ppp, inst), NULL, 98, 1500, 64);                 \
                                                                                                   \
	static struct modem_cellular_data MODEM_CELLULAR_INST_NAME(data, inst) = {                 \
		.chat_delimiter = "\r",                                                            \
		.chat_filter = "\n",                                                               \
		.ppp = &MODEM_CELLULAR_INST_NAME(ppp, inst),                                       \
	};                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(                                                      \
		inst,                                                                              \
		user_pipe_0,                                                                       \
		CONFIG_MODEM_SIMCOM7X00_USER_PIPE_BUFFER_SIZES                                     \
	);                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPES(                                                          \
		inst,                                                                              \
		MODEM_CELLULAR_INIT_USER_PIPE(inst, user_pipe_0, 3),                               \
	);                                                                                         \
                                                                                                   \
	static const struct modem_cellular_config MODEM_CELLULAR_INST_NAME(config, inst) = {       \
		.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                                          \
		.power_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_power_gpios, {}),                 \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_reset_gpios, {}),                 \
		.power_pulse_duration_ms = 5050,                                                   \
		.reset_pulse_duration_ms = 250,                                                    \
		.startup_time_ms = 15000,                                                          \
		.shutdown_time_ms = 5000,                                                          \
		.init_chat_script = &telit_me910g1_init_chat_script,                               \
		.dial_chat_script = &telit_me910g1_dial_chat_script,                               \
		.periodic_chat_script = &telit_me910g1_periodic_chat_script,                       \
		.user_pipes = MODEM_CELLULAR_GET_USER_PIPES(inst),                                 \
		.user_pipes_size = ARRAY_SIZE(MODEM_CELLULAR_GET_USER_PIPES(inst)),                \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, modem_cellular_pm_action);                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, modem_cellular_init, PM_DEVICE_DT_INST_GET(inst),              \
			      &MODEM_CELLULAR_INST_NAME(data, inst),                               \
			      &MODEM_CELLULAR_INST_NAME(config, inst), POST_KERNEL, 99,            \
			      &modem_cellular_api);

#define MODEM_CELLULAR_DEVICE_NORDIC_NRF91_SLM(inst)						   \
	MODEM_PPP_DEFINE(MODEM_CELLULAR_INST_NAME(ppp, inst), NULL, 98, 1500, 1500);               \
                                                                                                   \
	static struct modem_cellular_data MODEM_CELLULAR_INST_NAME(data, inst) = {                 \
		.chat_delimiter = "\r\n",                                                          \
		.ppp = &MODEM_CELLULAR_INST_NAME(ppp, inst),                                       \
	};                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(                                                      \
		inst,                                                                              \
		gnss_pipe,                                                                         \
		CONFIG_MODEM_SIMCOM7X00_USER_PIPE_BUFFER_SIZES                                     \
	);                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPES(                                                          \
		inst,                                                                              \
		MODEM_CELLULAR_INIT_USER_PIPE(inst, gnss_pipe, 3),                                 \
	);                                                                                         \
                                                                                                   \
	static const struct modem_cellular_config MODEM_CELLULAR_INST_NAME(config, inst) = {       \
		.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                                          \
		.power_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_power_gpios, {}),                 \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_reset_gpios, {}),                 \
		.power_pulse_duration_ms = 100,                                                    \
		.reset_pulse_duration_ms = 100,                                                    \
		.startup_time_ms = 2000,                                                           \
		.shutdown_time_ms = 10000,                                                         \
		.init_chat_script = &nordic_nrf91_slm_init_chat_script,                            \
		.dial_chat_script = &nordic_nrf91_slm_dial_chat_script,                            \
		.periodic_chat_script = &nordic_nrf91_slm_periodic_chat_script,                    \
		.user_pipes = MODEM_CELLULAR_GET_USER_PIPES(inst),                                 \
		.user_pipes_size = ARRAY_SIZE(MODEM_CELLULAR_GET_USER_PIPES(inst)),                \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, modem_cellular_pm_action);                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, modem_cellular_init, PM_DEVICE_DT_INST_GET(inst),              \
			      &MODEM_CELLULAR_INST_NAME(data, inst),                               \
			      &MODEM_CELLULAR_INST_NAME(config, inst), POST_KERNEL, 99,            \
			      &modem_cellular_api);

#define MODEM_CELLULAR_DEVICE_SQN_GM02S(inst)                                                      \
	MODEM_PPP_DEFINE(MODEM_CELLULAR_INST_NAME(ppp, inst), NULL, 98, 1500, 64);                 \
                                                                                                   \
	static struct modem_cellular_data MODEM_CELLULAR_INST_NAME(data, inst) = {                 \
		.chat_delimiter = "\r",                                                            \
		.chat_filter = "\n",                                                               \
		.ppp = &MODEM_CELLULAR_INST_NAME(ppp, inst),                                       \
	};                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(                                                      \
		inst,                                                                              \
		user_pipe_0,                                                                       \
		CONFIG_MODEM_SIMCOM7X00_USER_PIPE_BUFFER_SIZES                                     \
	);                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPE_DATA(                                                      \
		inst,                                                                              \
		user_pipe_1,                                                                       \
		CONFIG_MODEM_SIMCOM7X00_USER_PIPE_BUFFER_SIZES                                     \
	);                                                                                         \
                                                                                                   \
	MODEM_CELLULAR_DEFINE_USER_PIPES(                                                          \
		inst,                                                                              \
		MODEM_CELLULAR_INIT_USER_PIPE(inst, user_pipe_0, 3),                               \
		MODEM_CELLULAR_INIT_USER_PIPE(inst, user_pipe_1, 4),                               \
	);                                                                                         \
                                                                                                   \
	static const struct modem_cellular_config MODEM_CELLULAR_INST_NAME(config, inst) = {       \
		.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                                          \
		.power_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_power_gpios, {}),                 \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mdm_reset_gpios, {}),                 \
		.autostarts = true,                                                                \
		.power_pulse_duration_ms = 1500,                                                   \
		.reset_pulse_duration_ms = 100,                                                    \
		.startup_time_ms = 2000,                                                           \
		.shutdown_time_ms = 5000,                                                          \
		.init_chat_script = &sqn_gm02s_init_chat_script,                                   \
		.dial_chat_script = &sqn_gm02s_dial_chat_script,                                   \
		.periodic_chat_script = &sqn_gm02s_periodic_chat_script,                           \
		.user_pipes = MODEM_CELLULAR_GET_USER_PIPES(inst),                                 \
		.user_pipes_size = ARRAY_SIZE(MODEM_CELLULAR_GET_USER_PIPES(inst)),                \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, modem_cellular_pm_action);                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, modem_cellular_init, PM_DEVICE_DT_INST_GET(inst),              \
			      &MODEM_CELLULAR_INST_NAME(data, inst),                               \
			      &MODEM_CELLULAR_INST_NAME(config, inst), POST_KERNEL, 99,            \
			      &modem_cellular_api);

#define DT_DRV_COMPAT quectel_bg95
DT_INST_FOREACH_STATUS_OKAY(MODEM_CELLULAR_DEVICE_QUECTEL_BG95)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT quectel_eg25_g
DT_INST_FOREACH_STATUS_OKAY(MODEM_CELLULAR_DEVICE_QUECTEL_EG25_G)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT simcom_sim7x00
DT_INST_FOREACH_STATUS_OKAY(MODEM_CELLULAR_DEVICE_SIMCOM_SIM7X00)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT u_blox_sara_r4
DT_INST_FOREACH_STATUS_OKAY(MODEM_CELLULAR_DEVICE_U_BLOX_SARA_R4)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT u_blox_sara_r5
DT_INST_FOREACH_STATUS_OKAY(MODEM_CELLULAR_DEVICE_U_BLOX_SARA_R5)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT swir_hl7800
DT_INST_FOREACH_STATUS_OKAY(MODEM_CELLULAR_DEVICE_SWIR_HL7800)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT telit_me910g1
DT_INST_FOREACH_STATUS_OKAY(MODEM_CELLULAR_DEVICE_TELIT_ME910G1)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT nordic_nrf91_slm
DT_INST_FOREACH_STATUS_OKAY(MODEM_CELLULAR_DEVICE_NORDIC_NRF91_SLM)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT sqn_gm02s
DT_INST_FOREACH_STATUS_OKAY(MODEM_CELLULAR_DEVICE_SQN_GM02S)
#undef DT_DRV_COMPAT
