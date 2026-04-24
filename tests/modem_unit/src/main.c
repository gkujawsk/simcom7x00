/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/cellular.h>
#include <zephyr/drivers/modem/simcom7x00.h>
#include <zephyr/ztest.h>

enum simcom7x00_rat simcom7x00_test_parse_cpsi_rat(const char *system_mode);
enum cellular_registration_status simcom7x00_test_registration_snapshot(
	enum simcom7x00_rat current_rat, enum cellular_registration_status gsm,
	enum cellular_registration_status gprs, enum cellular_registration_status lte);
int simcom7x00_test_operator_cache(void);
int simcom7x00_test_control_arbitration(void);
int simcom7x00_test_diagnostics_sequence(struct simcom7x00_control_diagnostics *failed,
					 struct simcom7x00_control_diagnostics *succeeded);
int simcom7x00_test_periodic_busy_retry(void);
int simcom7x00_test_gnss_snapshot_invalidation(void);
int simcom7x00_test_gnss_prepare_fix_rate_unlocks_mutex(void);

ZTEST(simcom7x00_modem_unit, test_cpsi_rat_parser)
{
	zassert_equal(simcom7x00_test_parse_cpsi_rat("EDGE"), SIMCOM7X00_RAT_EDGE, NULL);
	zassert_equal(simcom7x00_test_parse_cpsi_rat("HSPA+"), SIMCOM7X00_RAT_UMTS, NULL);
	zassert_equal(simcom7x00_test_parse_cpsi_rat("LTE"), SIMCOM7X00_RAT_LTE, NULL);
	zassert_equal(simcom7x00_test_parse_cpsi_rat("GSM"), SIMCOM7X00_RAT_GSM, NULL);
	zassert_equal(simcom7x00_test_parse_cpsi_rat("CAT-M"), SIMCOM7X00_RAT_LTE, NULL);
}

ZTEST(simcom7x00_modem_unit, test_registration_snapshot)
{
	zassert_equal(simcom7x00_test_registration_snapshot(
			      SIMCOM7X00_RAT_LTE, CELLULAR_REGISTRATION_SEARCHING,
			      CELLULAR_REGISTRATION_UNKNOWN, CELLULAR_REGISTRATION_REGISTERED_HOME),
		      CELLULAR_REGISTRATION_REGISTERED_HOME, NULL);
	zassert_equal(simcom7x00_test_registration_snapshot(
			      SIMCOM7X00_RAT_EDGE, CELLULAR_REGISTRATION_NOT_REGISTERED,
			      CELLULAR_REGISTRATION_REGISTERED_ROAMING, CELLULAR_REGISTRATION_UNKNOWN),
		      CELLULAR_REGISTRATION_REGISTERED_ROAMING, NULL);
}

ZTEST(simcom7x00_modem_unit, test_operator_cache)
{
	zassert_ok(simcom7x00_test_operator_cache(), NULL);
}

ZTEST(simcom7x00_modem_unit, test_control_arbitration)
{
	zassert_ok(simcom7x00_test_control_arbitration(), NULL);
}

ZTEST(simcom7x00_modem_unit, test_diagnostics_freshness)
{
	struct simcom7x00_control_diagnostics failed = { 0 };
	struct simcom7x00_control_diagnostics succeeded = { 0 };

	zassert_ok(simcom7x00_test_diagnostics_sequence(&failed, &succeeded), NULL);
	zassert_equal(failed.last_result, SIMCOM7X00_CONTROL_RESULT_FAILED, NULL);
	zassert_true(failed.has_cme_error, NULL);
	zassert_equal(failed.last_cme_error, 515, NULL);
	zassert_equal(failed.transaction_id, failed.completed_transaction_id, NULL);
	zassert_false(failed.transaction_in_progress, NULL);

	zassert_equal(succeeded.last_result, SIMCOM7X00_CONTROL_RESULT_SUCCESS, NULL);
	zassert_false(succeeded.has_cme_error, NULL);
	zassert_equal(succeeded.last_cme_error, 0, NULL);
	zassert_true(succeeded.transaction_id > failed.transaction_id, NULL);
	zassert_equal(succeeded.transaction_id, succeeded.completed_transaction_id, NULL);
	zassert_false(succeeded.transaction_in_progress, NULL);
}

ZTEST(simcom7x00_modem_unit, test_periodic_busy_retry)
{
	zassert_ok(simcom7x00_test_periodic_busy_retry(), NULL);
}

ZTEST(simcom7x00_modem_unit, test_gnss_snapshot_invalidation)
{
	zassert_ok(simcom7x00_test_gnss_snapshot_invalidation(), NULL);
}

ZTEST(simcom7x00_modem_unit, test_gnss_fix_rate_unlocks_for_ready)
{
	zassert_ok(simcom7x00_test_gnss_prepare_fix_rate_unlocks_mutex(), NULL);
}

ZTEST_SUITE(simcom7x00_modem_unit, NULL, NULL, NULL, NULL, NULL);
