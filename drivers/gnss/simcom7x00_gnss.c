/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/modem/simcom7x00.h>
#include <zephyr/drivers/gnss/gnss_publish.h>
#include <zephyr/kernel.h>
#include <zephyr/modem/chat.h>
#include <zephyr/modem/pipe.h>
#include <zephyr/modem/pipelink.h>
#include <zephyr/pm/device.h>

#include <stdio.h>

#include "gnss_nmea0183.h"
#include "gnss_nmea0183_match.h"
#include "gnss_parse.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(simcom7x00_gnss, CONFIG_GNSS_SIMCOM7X00_LOG_LEVEL);

#define DT_DRV_COMPAT simcom_sim7x00_gnss

#define SIMCOM7X00_GNSS_CHAT_RECV_BUF_SZ 256
#define SIMCOM7X00_GNSS_CHAT_ARGV_SZ 32
#define SIMCOM7X00_GNSS_STANDALONE_MODE 1
#define SIMCOM7X00_GNSS_NMEA_RATE_1HZ 0
#define SIMCOM7X00_GNSS_FIX_INTERVAL_MS_DISABLED 0U
#define SIMCOM7X00_GNSS_FIX_INTERVAL_MS_MIN 1000U
#define SIMCOM7X00_GNSS_FIX_INTERVAL_MS_DEFAULT 1000U
#define SIMCOM7X00_GNSS_FIX_INTERVAL_MS_MAX 255000U
#define SIMCOM7X00_GNSS_NMEA_STREAM_MASK 31

struct simcom7x00_gnss_config {
	const struct device *parent_modem;
	struct modem_pipelink *gnss_pipe;
	const struct modem_chat_script *init_chat_script;
	const struct modem_chat_script *stop_chat_script;
};

struct simcom7x00_gnss_data {
	struct gnss_nmea0183_match_data match_data;
#if CONFIG_GNSS_SATELLITES
	struct gnss_satellite satellites[CONFIG_GNSS_SIMCOM7X00_SATELLITES_COUNT];
#endif
	struct modem_chat chat;
	uint8_t chat_receive_buf[SIMCOM7X00_GNSS_CHAT_RECV_BUF_SZ];
	uint8_t *chat_argv[SIMCOM7X00_GNSS_CHAT_ARGV_SZ];
	struct modem_pipe *pipe;
	struct k_mutex lock;
	uint32_t requested_fix_interval_ms;
	struct gnss_data latest_data;
	bool latest_data_valid;
	bool pipe_connected;
	bool active;
};

typedef int (*simcom7x00_gnss_ensure_ready_fn_t)(const struct device *modem_dev,
						 bool ppp_requested, void *user_data);

MODEM_CHAT_MATCH_DEFINE(ok_match, "OK", "", NULL);
MODEM_CHAT_MATCH_DEFINE(error_match, "ERROR", "", NULL);
MODEM_CHAT_MATCHES_DEFINE(optional_ok_error_matches, ok_match, error_match);

static void simcom7x00_gnss_update_snapshot_locked(struct simcom7x00_gnss_data *data)
{
	if ((data->match_data.gga_utc == 0U) || (data->match_data.rmc_utc == 0U)) {
		return;
	}

	if (data->match_data.gga_utc != data->match_data.rmc_utc) {
		return;
	}

	memcpy(&data->latest_data, &data->match_data.data, sizeof(data->latest_data));
	data->latest_data_valid = true;
}

static void simcom7x00_gnss_invalidate_snapshot_locked(struct simcom7x00_gnss_data *data)
{
	data->latest_data_valid = false;
}

static void simcom7x00_gnss_match_gga_callback(struct modem_chat *chat, char **argv, uint16_t argc,
					       void *user_data)
{
	struct simcom7x00_gnss_data *data = user_data;

	gnss_nmea0183_match_gga_callback(chat, argv, argc, user_data);

	k_mutex_lock(&data->lock, K_FOREVER);
	simcom7x00_gnss_update_snapshot_locked(data);
	k_mutex_unlock(&data->lock);
}

static void simcom7x00_gnss_match_rmc_callback(struct modem_chat *chat, char **argv, uint16_t argc,
					       void *user_data)
{
	struct simcom7x00_gnss_data *data = user_data;

	gnss_nmea0183_match_rmc_callback(chat, argv, argc, user_data);

	k_mutex_lock(&data->lock, K_FOREVER);
	simcom7x00_gnss_update_snapshot_locked(data);
	k_mutex_unlock(&data->lock);
}

MODEM_CHAT_MATCHES_DEFINE(unsol_matches,
	MODEM_CHAT_MATCH_WILDCARD("$??GGA,", ",*", simcom7x00_gnss_match_gga_callback),
	MODEM_CHAT_MATCH_WILDCARD("$??RMC,", ",*", simcom7x00_gnss_match_rmc_callback),
#if CONFIG_GNSS_SATELLITES
	MODEM_CHAT_MATCH_WILDCARD("$??GSV,", ",*", gnss_nmea0183_match_gsv_callback),
#endif
);

/*
 * SIM7500/SIM7600 exposes two different NMEA configuration paths in the AT
 * manual:
 * - AT+CGPSNMEA configures the default gpsOne NMEA sentence set, but the
 *   setting requires a reboot and fails while the GPS engine is running.
 * - AT+CGPSINFOCFG configures periodic runtime NMEA reporting and is therefore
 *   the better fit for the driver's resume path.
 *
 * The runtime sequence below follows the manual:
 * 1. Stop any previous reporting.
 * 2. Set the documented 1 Hz NMEA output rate before opening GPS.
 * 3. Start GPS in standalone mode using AT+CGPS=1,1.
 * 4. Enable periodic NMEA output with a mask that matches the currently parsed
 *    sentences (GGA, RMC, GSV, GSA, VTG => 31).
 */
MODEM_CHAT_SCRIPT_CMDS_DEFINE(simcom_sim7x00_gnss_stop_chat_script_cmds,
			      MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CGPSINFOCFG=0",
							      optional_ok_error_matches),
			      MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CGPS=0",
							      optional_ok_error_matches));

MODEM_CHAT_SCRIPT_NO_ABORT_DEFINE(simcom_sim7x00_gnss_stop_chat_script,
				  simcom_sim7x00_gnss_stop_chat_script_cmds, NULL, 3);

static uint8_t simcom7x00_gnss_chat_delimiter[] = {'\r', '\n'};

int simcom7x00_modem_ensure_ready(const struct device *dev, bool ppp_requested);

static int simcom7x00_gnss_ensure_ready_adapter(const struct device *modem_dev, bool ppp_requested,
						void *user_data)
{
	ARG_UNUSED(user_data);

	return simcom7x00_modem_ensure_ready(modem_dev, ppp_requested);
}

static bool simcom7x00_gnss_fix_interval_valid(uint32_t fix_interval_ms)
{
	if (fix_interval_ms == SIMCOM7X00_GNSS_FIX_INTERVAL_MS_DISABLED) {
		return true;
	}

	if ((fix_interval_ms < SIMCOM7X00_GNSS_FIX_INTERVAL_MS_MIN) ||
	    (fix_interval_ms > SIMCOM7X00_GNSS_FIX_INTERVAL_MS_MAX)) {
		return false;
	}

	return (fix_interval_ms % 1000U) == 0U;
}

static uint32_t simcom7x00_gnss_fix_interval_to_seconds(uint32_t fix_interval_ms)
{
	return fix_interval_ms / 1000U;
}

static int simcom7x00_gnss_open_pipe_locked(const struct device *dev)
{
	const struct simcom7x00_gnss_config *cfg = dev->config;
	struct simcom7x00_gnss_data *data = dev->data;
	int ret;

	if (data->pipe != NULL) {
		return 0;
	}

	if (!data->pipe_connected) {
		return -ENODEV;
	}

	data->pipe = modem_pipelink_get_pipe(cfg->gnss_pipe);
	if (data->pipe == NULL) {
		return -ENODEV;
	}

	ret = modem_pipe_open(data->pipe);
	if (ret < 0) {
		data->pipe = NULL;
		return ret;
	}

	ret = modem_chat_attach(&data->chat, data->pipe);
	if (ret < 0) {
		modem_pipe_close(data->pipe);
		data->pipe = NULL;
		return ret;
	}

	return 0;
}

static int simcom7x00_gnss_prepare_fix_rate_change(
	const struct device *dev, uint32_t *fix_interval_ms,
	simcom7x00_gnss_ensure_ready_fn_t ensure_ready, void *user_data)
{
	const struct simcom7x00_gnss_config *cfg = dev->config;
	struct simcom7x00_gnss_data *data = dev->data;
	bool need_ready;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->requested_fix_interval_ms = *fix_interval_ms;
	need_ready = (*fix_interval_ms != SIMCOM7X00_GNSS_FIX_INTERVAL_MS_DISABLED) &&
		    !data->pipe_connected;
	k_mutex_unlock(&data->lock);

	if (need_ready) {
		LOG_DBG("requesting parent modem ready for GNSS fix interval %u ms", *fix_interval_ms);
		ret = ensure_ready(cfg->parent_modem, false, user_data);
		if (ret < 0) {
			LOG_WRN("failed to wake parent modem for GNSS: %d", ret);
			return ret;
		}
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	*fix_interval_ms = data->requested_fix_interval_ms;

	if ((*fix_interval_ms != SIMCOM7X00_GNSS_FIX_INTERVAL_MS_DISABLED) && !data->pipe_connected) {
		data->active = false;
		LOG_WRN("GNSS pipe not connected after modem wake");
		ret = -ENODEV;
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

static void simcom7x00_gnss_close_pipe_locked(struct simcom7x00_gnss_data *data)
{
	if (data->pipe == NULL) {
		return;
	}

	modem_pipe_close(data->pipe);
	data->pipe = NULL;
}

static int simcom7x00_gnss_apply_fix_rate_locked(const struct device *dev)
{
	struct simcom7x00_gnss_data *data = dev->data;
	struct modem_chat_script_chat script_chats[5];
	struct modem_chat_script script;
	char gps_nmea_rate_cmd[32];
	char gps_start_cmd[32];
	char gps_info_cfg_cmd[48];
	uint32_t interval_ms = data->requested_fix_interval_ms;
	uint32_t interval_s;
	size_t script_chats_size = 0U;
	int ret;

	if (interval_ms == SIMCOM7X00_GNSS_FIX_INTERVAL_MS_DISABLED) {
		return modem_chat_run_script(&data->chat, &simcom_sim7x00_gnss_stop_chat_script);
	}

	interval_s = simcom7x00_gnss_fix_interval_to_seconds(interval_ms);

	ret = snprintf(gps_nmea_rate_cmd, sizeof(gps_nmea_rate_cmd), "AT+CGPSNMEARATE=%u",
		       SIMCOM7X00_GNSS_NMEA_RATE_1HZ);
	if ((ret < 0) || (ret >= sizeof(gps_nmea_rate_cmd))) {
		return -EINVAL;
	}

	ret = snprintf(gps_start_cmd, sizeof(gps_start_cmd), "AT+CGPS=1,%u",
		       SIMCOM7X00_GNSS_STANDALONE_MODE);
	if ((ret < 0) || (ret >= sizeof(gps_start_cmd))) {
		return -EINVAL;
	}

	ret = snprintf(gps_info_cfg_cmd, sizeof(gps_info_cfg_cmd), "AT+CGPSINFOCFG=%u,%u",
		       interval_s, SIMCOM7X00_GNSS_NMEA_STREAM_MASK);
	if ((ret < 0) || (ret >= sizeof(gps_info_cfg_cmd))) {
		return -EINVAL;
	}

	script_chats[script_chats_size++] =
		(struct modem_chat_script_chat)MODEM_CHAT_SCRIPT_CMD_RESP("AT", ok_match);
	script_chats[script_chats_size++] =
		(struct modem_chat_script_chat)MODEM_CHAT_SCRIPT_CMD_RESP("ATE0", ok_match);
	script_chats[script_chats_size++] =
		(struct modem_chat_script_chat)MODEM_CHAT_SCRIPT_CMD_RESP(gps_nmea_rate_cmd, ok_match);
	script_chats[script_chats_size++] =
		(struct modem_chat_script_chat)MODEM_CHAT_SCRIPT_CMD_RESP(gps_start_cmd, ok_match);
	script_chats[script_chats_size++] =
		(struct modem_chat_script_chat)MODEM_CHAT_SCRIPT_CMD_RESP(gps_info_cfg_cmd, ok_match);

	modem_chat_script_init(&script);
	modem_chat_script_set_name(&script, "simcom7x00_gnss_apply_fix_rate");
	ret = modem_chat_script_set_script_chats(&script, script_chats, script_chats_size);
	if (ret < 0) {
		return ret;
	}

	modem_chat_script_set_timeout(&script, 8);

	return modem_chat_run_script(&data->chat, &script);
}

static int simcom7x00_gnss_resume(const struct device *dev)
{
	struct simcom7x00_gnss_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (data->active) {
		goto out;
	}

	if (data->requested_fix_interval_ms == SIMCOM7X00_GNSS_FIX_INTERVAL_MS_DISABLED) {
		goto out;
	}

	ret = simcom7x00_gnss_open_pipe_locked(dev);
	if (ret < 0) {
		LOG_WRN("failed to open GNSS pipe on resume: %d", ret);
		goto out;
	}

	ret = simcom7x00_gnss_apply_fix_rate_locked(dev);
	if (ret < 0) {
		simcom7x00_gnss_close_pipe_locked(data);
		simcom7x00_gnss_invalidate_snapshot_locked(data);
		LOG_WRN("failed to apply GNSS fix rate on resume: %d", ret);
		goto out;
	}

	data->active = true;
	LOG_INF("GNSS resumed at %u ms fix interval", data->requested_fix_interval_ms);

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int simcom7x00_gnss_suspend(const struct device *dev)
{
	const struct simcom7x00_gnss_config *cfg = dev->config;
	struct simcom7x00_gnss_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (!data->active || data->pipe == NULL) {
		goto out;
	}

	ret = modem_chat_run_script(&data->chat, cfg->stop_chat_script);
	simcom7x00_gnss_close_pipe_locked(data);
	simcom7x00_gnss_invalidate_snapshot_locked(data);
	data->active = false;
	LOG_INF("GNSS suspended");

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static void simcom7x00_gnss_pipelink_callback(struct modem_pipelink *link,
					      enum modem_pipelink_event event,
					      void *user_data)
{
	const struct device *dev = user_data;
	struct simcom7x00_gnss_data *data = dev->data;

	ARG_UNUSED(link);

	k_mutex_lock(&data->lock, K_FOREVER);

	if (event == MODEM_PIPELINK_EVENT_CONNECTED) {
		data->pipe_connected = true;
		LOG_INF("GNSS pipe connected");
	} else {
		data->pipe_connected = false;
		data->pipe = NULL;
		simcom7x00_gnss_invalidate_snapshot_locked(data);
		if (data->active) {
			data->active = false;
		}
		LOG_WRN("GNSS pipe disconnected");
	}

	k_mutex_unlock(&data->lock);

	if (!IS_ENABLED(CONFIG_PM_DEVICE) && event == MODEM_PIPELINK_EVENT_CONNECTED) {
		(void)simcom7x00_gnss_resume(dev);
	}
}

static int simcom7x00_gnss_init_nmea0183_match(const struct device *dev)
{
	struct simcom7x00_gnss_data *data = dev->data;

	const struct gnss_nmea0183_match_config match_config = {
		.gnss = dev,
#if CONFIG_GNSS_SATELLITES
		.satellites = data->satellites,
		.satellites_size = ARRAY_SIZE(data->satellites),
#endif
	};

	return gnss_nmea0183_match_init(&data->match_data, &match_config);
}

static int simcom7x00_gnss_init_chat(const struct device *dev)
{
	struct simcom7x00_gnss_data *data = dev->data;

	const struct modem_chat_config chat_config = {
		.user_data = data,
		.receive_buf = data->chat_receive_buf,
		.receive_buf_size = sizeof(data->chat_receive_buf),
		.delimiter = simcom7x00_gnss_chat_delimiter,
		.delimiter_size = ARRAY_SIZE(simcom7x00_gnss_chat_delimiter),
		.filter = NULL,
		.filter_size = 0,
		.argv = data->chat_argv,
		.argv_size = ARRAY_SIZE(data->chat_argv),
		.unsol_matches = unsol_matches,
		.unsol_matches_size = ARRAY_SIZE(unsol_matches),
	};

	return modem_chat_init(&data->chat, &chat_config);
}

static int simcom7x00_gnss_get_supported_systems(const struct device *dev,
						 gnss_systems_t *systems)
{
	ARG_UNUSED(dev);

	if (systems == NULL) {
		return -EINVAL;
	}

	*systems = GNSS_SYSTEM_GPS;
	return 0;
}

static int simcom7x00_gnss_set_fix_rate(const struct device *dev, uint32_t fix_interval_ms)
{
	struct simcom7x00_gnss_data *data = dev->data;
	uint32_t effective_fix_interval_ms = fix_interval_ms;
	bool was_active;
	int ret = 0;

	if (!simcom7x00_gnss_fix_interval_valid(fix_interval_ms)) {
		return -EINVAL;
	}

	ret = simcom7x00_gnss_prepare_fix_rate_change(dev, &effective_fix_interval_ms,
						      simcom7x00_gnss_ensure_ready_adapter, NULL);
	if (ret < 0) {
		return ret;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	if (effective_fix_interval_ms == SIMCOM7X00_GNSS_FIX_INTERVAL_MS_DISABLED) {
		if (data->active && (data->pipe != NULL)) {
			ret = modem_chat_run_script(&data->chat, &simcom_sim7x00_gnss_stop_chat_script);
			simcom7x00_gnss_close_pipe_locked(data);
		}
		simcom7x00_gnss_invalidate_snapshot_locked(data);
		data->active = false;
		LOG_INF("GNSS disabled");
		goto out;
	}

	if (!data->pipe_connected) {
		data->active = false;
		LOG_WRN("cannot enable GNSS because pipe is not connected");
		ret = -ENODEV;
		goto out;
	}

	was_active = data->active;
	if (!data->active) {
		ret = simcom7x00_gnss_open_pipe_locked(dev);
		if (ret < 0) {
			LOG_WRN("failed to open GNSS pipe: %d", ret);
			goto out;
		}
	}

	ret = simcom7x00_gnss_apply_fix_rate_locked(dev);
	if (ret < 0) {
		if (!was_active) {
			simcom7x00_gnss_close_pipe_locked(data);
		}
		LOG_WRN("failed to apply GNSS fix rate %u ms: %d", effective_fix_interval_ms, ret);
		goto out;
	}

	data->active = true;
	LOG_INF("GNSS active with %u ms fix interval", effective_fix_interval_ms);

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int simcom7x00_gnss_get_fix_rate(const struct device *dev, uint32_t *fix_interval_ms)
{
	struct simcom7x00_gnss_data *data = dev->data;

	if (fix_interval_ms == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	*fix_interval_ms = data->requested_fix_interval_ms;
	k_mutex_unlock(&data->lock);

	return 0;
}

int simcom7x00_gnss_get_snapshot(const struct device *dev,
				 struct simcom7x00_gnss_snapshot *snapshot)
{
	struct simcom7x00_gnss_data *data;

	if ((dev == NULL) || (snapshot == NULL)) {
		return -EINVAL;
	}

	data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	memcpy(&snapshot->data, &data->latest_data, sizeof(snapshot->data));
	snapshot->fix_interval_ms = data->requested_fix_interval_ms;
	snapshot->data_valid = data->latest_data_valid;
	snapshot->active = data->active;
	snapshot->pipe_connected = data->pipe_connected;
	k_mutex_unlock(&data->lock);

	return 0;
}

#if defined(CONFIG_ZTEST)
struct simcom7x00_gnss_test_ready_context {
	const struct device *dev;
	bool lock_available;
};

static int simcom7x00_gnss_test_ensure_ready(const struct device *modem_dev, bool ppp_requested,
					     void *user_data)
{
	struct simcom7x00_gnss_test_ready_context *ctx = user_data;
	struct simcom7x00_gnss_data *data = ctx->dev->data;

	ARG_UNUSED(modem_dev);
	ARG_UNUSED(ppp_requested);

	ctx->lock_available = (k_mutex_lock(&data->lock, K_NO_WAIT) == 0);
	if (ctx->lock_available) {
		data->pipe_connected = true;
		k_mutex_unlock(&data->lock);
	}

	return 0;
}

int simcom7x00_test_gnss_snapshot_invalidation(void)
{
	struct simcom7x00_gnss_data data = { 0 };
	struct device dev = { .data = &data };

	k_mutex_init(&data.lock);

	k_mutex_lock(&data.lock, K_FOREVER);
	data.latest_data_valid = true;
	simcom7x00_gnss_invalidate_snapshot_locked(&data);
	k_mutex_unlock(&data.lock);
	if (data.latest_data_valid) {
		return -EINVAL;
	}

	data.latest_data_valid = true;
	data.pipe_connected = true;
	data.active = true;
	simcom7x00_gnss_pipelink_callback(NULL, MODEM_PIPELINK_EVENT_DISCONNECTED, &dev);

	return data.latest_data_valid ? -EIO : 0;
}

int simcom7x00_test_gnss_prepare_fix_rate_unlocks_mutex(void)
{
	struct simcom7x00_gnss_config cfg = { 0 };
	struct simcom7x00_gnss_data data = { 0 };
	struct device dev = { .config = &cfg, .data = &data };
	uint32_t fix_interval_ms = SIMCOM7X00_GNSS_FIX_INTERVAL_MS_DEFAULT;
	struct simcom7x00_gnss_test_ready_context ctx = {
		.dev = &dev,
	};
	int ret;

	k_mutex_init(&data.lock);
	data.pipe_connected = false;

	ret = simcom7x00_gnss_prepare_fix_rate_change(&dev, &fix_interval_ms,
						      simcom7x00_gnss_test_ensure_ready,
						      &ctx);
	if (ret < 0) {
		return ret;
	}

	if (!ctx.lock_available) {
		return -EBUSY;
	}

	if (!data.pipe_connected) {
		return -ENODEV;
	}

	return 0;
}
#endif

static const struct gnss_driver_api simcom7x00_gnss_api = {
	.set_fix_rate = simcom7x00_gnss_set_fix_rate,
	.get_fix_rate = simcom7x00_gnss_get_fix_rate,
	.get_supported_systems = simcom7x00_gnss_get_supported_systems,
};

static int simcom7x00_gnss_init(const struct device *dev)
{
	const struct simcom7x00_gnss_config *cfg = dev->config;
	struct simcom7x00_gnss_data *data = dev->data;
	int ret;

	k_mutex_init(&data->lock);
	data->requested_fix_interval_ms = SIMCOM7X00_GNSS_FIX_INTERVAL_MS_DEFAULT;

	ret = simcom7x00_gnss_init_nmea0183_match(dev);
	if (ret < 0) {
		return ret;
	}

	ret = simcom7x00_gnss_init_chat(dev);
	if (ret < 0) {
		return ret;
	}

	modem_pipelink_attach(cfg->gnss_pipe, simcom7x00_gnss_pipelink_callback, (void *)dev);
	data->pipe_connected = modem_pipelink_is_connected(cfg->gnss_pipe);

#if CONFIG_PM_DEVICE
	pm_device_init_suspended(dev);
#else
	if (data->pipe_connected) {
		ret = simcom7x00_gnss_resume(dev);
		if (ret < 0) {
			return ret;
		}
	}
#endif

	return 0;
}

#if CONFIG_PM_DEVICE
static int simcom7x00_gnss_pm_action(const struct device *dev, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		return simcom7x00_gnss_resume(dev);
	case PM_DEVICE_ACTION_SUSPEND:
		return simcom7x00_gnss_suspend(dev);
	default:
		return -ENOTSUP;
	}
}
#endif

#define SIMCOM7X00_GNSS(inst)                                                                  \
	MODEM_PIPELINK_DT_DECLARE(DT_PARENT(DT_DRV_INST(inst)), gnss_pipe);                   \
	static const struct simcom7x00_gnss_config simcom7x00_gnss_cfg_##inst = {             \
		.parent_modem = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(inst))),                  \
		.gnss_pipe = MODEM_PIPELINK_DT_GET(DT_PARENT(DT_DRV_INST(inst)), gnss_pipe),  \
		.stop_chat_script = &simcom_sim7x00_gnss_stop_chat_script,                    \
	};                                                                                    \
                                                                                              \
	static struct simcom7x00_gnss_data simcom7x00_gnss_data_##inst;                       \
                                                                                              \
	PM_DEVICE_DT_INST_DEFINE(inst, simcom7x00_gnss_pm_action);                            \
                                                                                              \
	DEVICE_DT_INST_DEFINE(inst, simcom7x00_gnss_init, PM_DEVICE_DT_INST_GET(inst),       \
			      &simcom7x00_gnss_data_##inst, &simcom7x00_gnss_cfg_##inst,      \
			      APPLICATION, 0, &simcom7x00_gnss_api);

DT_INST_FOREACH_STATUS_OKAY(SIMCOM7X00_GNSS)

#undef DT_DRV_COMPAT
