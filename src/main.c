/* Copyright (c) 2024 Modz, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(golioth_can_asset_tracker, LOG_LEVEL_DBG);

#include "app_rpc.h"
#include "app_settings.h"
#include "app_state.h"
#include "app_sensors.h"
#include "app_wdt.h"
#include <golioth/client.h>
#include <golioth/fw_update.h>
#include <samples/common/net_connect.h>
#include <samples/common/sample_credentials.h>
#include <zephyr/kernel.h>
#include <hal/nrf_power.h>


#ifdef CONFIG_SOC_NRF9160
#include <modem/lte_lc.h>
#endif
#ifdef CONFIG_LIB_OSTENTUS
#include <libostentus.h>
#endif
#ifdef CONFIG_ALUDEL_BATTERY_MONITOR
#include "battery_monitor/battery.h"
#endif

#include <zephyr/drivers/gpio.h>

#ifdef CONFIG_MODEM_INFO
#include <modem/modem_info.h>
#endif

/* Current firmware version; update in prj.conf or via build argument */
static const char *_current_version = CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION;

static struct golioth_client *client;
K_SEM_DEFINE(connected, 0, 1);

static k_tid_t _system_thread = 0;

#if DT_NODE_EXISTS(DT_ALIAS(golioth_led))
static const struct gpio_dt_spec golioth_led = GPIO_DT_SPEC_GET(DT_ALIAS(golioth_led), gpios);
#endif /* DT_NODE_EXISTS(DT_ALIAS(golioth_led)) */
static const struct gpio_dt_spec user_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static struct gpio_callback button_cb_data;

/* forward declarations */
void golioth_connection_led_set(uint8_t state);

void wake_system_thread(void)
{
	k_wakeup(_system_thread);
}

static void on_client_event(struct golioth_client *client, enum golioth_client_event event,
			    void *arg)
{
	bool is_connected = (event == GOLIOTH_CLIENT_EVENT_CONNECTED);

	if (is_connected) {
		k_sem_give(&connected);
		golioth_connection_led_set(1);
	}
	LOG_INF("Golioth client %s", is_connected ? "connected" : "disconnected");
}

static void start_golioth_client(void)
{
	/* Get the client configuration from auto-loaded settings */
	const struct golioth_client_config *client_config = golioth_sample_credentials_get();

	/* Create and start a Golioth Client */
	client = golioth_client_create(client_config);

	/* Register Golioth on_connect callback */
	golioth_client_register_event_callback(client, on_client_event, NULL);

	/* Initialize DFU components */
	golioth_fw_update_init(client, _current_version);

	/*** Call Golioth APIs for other services in dedicated app files ***/

	/* Observe State service data */
	app_state_observe(client);

	/* Set Golioth Client for streaming sensor data */
	app_sensors_set_client(client);

	/* Register Settings service */
	app_settings_register(client);

	/* Register RPC service */
	app_rpc_register(client);
}

#ifdef CONFIG_SOC_NRF9160

static void lte_handler(const struct lte_lc_evt *const evt)
{
	LOG_INF("LTE event: %d", evt->type);
	switch (evt->type) {
	case LTE_LC_EVT_MODEM_EVENT:
		LOG_INF("Modem event: %d", evt->modem_evt);
		break;

	case LTE_LC_EVT_LTE_MODE_UPDATE:
		LOG_INF("LTE mode update: %d", evt->lte_mode);
		break;

	case LTE_LC_EVT_NW_REG_STATUS:
		LOG_INF("Network registration status: %d", evt->nw_reg_status);
		if ((evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME) ||
		    (evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING)) {

			/* Change the state of the Internet LED on Ostentus */
			IF_ENABLED(CONFIG_LIB_OSTENTUS, (led_internet_set(1);));

			if (!client) {
				/* Create and start a Golioth Client */
				start_golioth_client();
			}
		}
		break;

	default:
		break;
	}
}

#endif /* CONFIG_SOC_NRF9160 */

#ifdef CONFIG_MODEM_INFO
static void log_modem_firmware_version(void)
{
	char sbuf[128];

	/* Initialize modem info */
	int err = modem_info_init();

	if (err) {
		LOG_ERR("Failed to initialize modem info: %d", err);
	}

	/* Log modem firmware version */
	modem_info_string_get(MODEM_INFO_FW_VERSION, sbuf, sizeof(sbuf));
	LOG_INF("Modem firmware version: %s", sbuf);
}
#endif

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	LOG_DBG("Button pressed at %d", k_cycle_get_32());
	/* This function is an Interrupt Service Routine. Do not call functions that
	 * use other threads, or perform long-running operations here
	 */
	k_wakeup(_system_thread);
}


/* Set (unset) LED indicators for active Golioth connection */
void golioth_connection_led_set(uint8_t state)
{
	uint8_t pin_state = state ? 1 : 0;
#if DT_NODE_EXISTS(DT_ALIAS(golioth_led))
	/* Turn on Golioth logo LED once connected */
	gpio_pin_set_dt(&golioth_led, pin_state);
#endif /* DT_NODE_EXISTS(DT_ALIAS(golioth_led)) */
	/* Change the state of the Golioth LED on Ostentus */
	IF_ENABLED(CONFIG_LIB_OSTENTUS, (led_golioth_set(pin_state);));
}

int main(void)
{
	int err;

	/*uint32_t reset_reason = nrf_power_resetreas_get(NRF_POWER);
	LOG_WRN("Reboot reason: %ld", reset_reason);
	if (reset_reason & NRF_POWER_RESETREAS_DOG_MASK ) {
		LOG_WRN("Wake up by DOG field detect\n");
	} else if (reset_reason & NRF_POWER_RESETREAS_RESETPIN_MASK) {
		LOG_WRN("Reset by pin-reset\n");
	} else if (reset_reason & NRF_POWER_RESETREAS_SREQ_MASK) {
		LOG_WRN("Reset by soft-reset\n");
	} else if (reset_reason & NRF_POWER_RESETREAS_LOCKUP_MASK ) {
		LOG_WRN("Reset by lockup\n");
	} else if (reset_reason) {
		LOG_WRN("Reset by a different source (0x%08X)\n", reset_reason);
	} else {
		LOG_WRN("Power-on-reset\n");
	}
	nrf_power_resetreas_clear(NRF_POWER, reset_reason);*/

	/* Initialize watchdog */
	app_wdt_init();

	/* Initialize sensors */
	app_sensors_init();

	LOG_DBG("Started CAN Asset Tracker app");

	LOG_INF("Firmware version: %s", CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION);
	IF_ENABLED(CONFIG_MODEM_INFO, (log_modem_firmware_version();));

	IF_ENABLED(CONFIG_LIB_OSTENTUS, (
		/* Clear Ostentus memory */
		clear_memory();
		/* Update Ostentus LEDS using bitmask (Power On and Battery) */
		led_bitmask(LED_POW | LED_BAT);
		/* Show Golioth Logo on Ostentus ePaper screen */
		show_splash();
	));

	/* Get system thread id so loop delay change event can wake main */
	_system_thread = k_current_get();

#if DT_NODE_EXISTS(DT_ALIAS(golioth_led))
	/* Initialize Golioth logo LED */
	err = gpio_pin_configure_dt(&golioth_led, GPIO_OUTPUT_INACTIVE);
	if (err) {
		LOG_ERR("Unable to configure LED for Golioth Logo");
	}
#endif /* DT_NODE_EXISTS(DT_ALIAS(golioth_led)) */

#ifdef CONFIG_SOC_NRF9160
	/* Start LTE asynchronously if the nRF9160 is used.
	 * Golioth Client will start automatically when LTE connects
	 */

	LOG_INF("Connecting to LTE, this may take some time...");
	lte_lc_init_and_connect_async(lte_handler);

#else
	/* If nRF9160 is not used, start the Golioth Client and block until connected */

	/* Run WiFi/DHCP if necessary */
	if (IS_ENABLED(CONFIG_GOLIOTH_SAMPLE_COMMON)) {
		net_connect();
	}

	/* Start Golioth client */
	start_golioth_client();

#endif /* CONFIG_SOC_NRF9160 */

	/* Block until connected to Golioth */
	k_sem_take(&connected, K_FOREVER);
	/* Set up user button */
	err = gpio_pin_configure_dt(&user_btn, GPIO_INPUT);
	if (err) {
		LOG_ERR("Error %d: failed to configure %s pin %d", err, user_btn.port->name,
			user_btn.pin);
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(&user_btn, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		LOG_ERR("Error %d: failed to configure interrupt on %s pin %d", err,
			user_btn.port->name, user_btn.pin);
		return err;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(user_btn.pin));
	gpio_add_callback(user_btn.port, &button_cb_data);

	IF_ENABLED(CONFIG_LIB_OSTENTUS, (
		/* Set up a slideshow on Ostentus
		 *  - add up to 256 slides
		 *  - use the enum in app_sensors.h to add new keys
		 *  - values are updated using these keys (see app_sensors.c)
		 */
		slide_add(LATITUDE, LABEL_LATITUDE, strlen(LABEL_LATITUDE));
		slide_add(LONGITUDE, LABEL_LONGITUDE, strlen(LABEL_LONGITUDE));
		slide_add(VEHICLE_SPEED, LABEL_VEHICLE_SPEED, strlen(LABEL_VEHICLE_SPEED));
		IF_ENABLED(CONFIG_ALUDEL_BATTERY_MONITOR, (
			slide_add(BATTERY_V, LABEL_BATTERY, strlen(LABEL_BATTERY));
			slide_add(BATTERY_LVL, LABEL_BATTERY, strlen(LABEL_BATTERY));
		));
		slide_add(FIRMWARE, LABEL_FIRMWARE, strlen(LABEL_FIRMWARE));

		/* Set the title of the Ostentus summary slide (optional) */
		summary_title(SUMMARY_TITLE, strlen(SUMMARY_TITLE));

		/* Update the Firmware slide with the firmware version */
		slide_set(FIRMWARE, CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION,
			  strlen(CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION));

		/* Start Ostentus slideshow with 30 second delay between slides */
		slideshow(30000);
	));

	while (true) {
		app_sensors_read_and_stream();
		app_wdt_feed();

		k_sleep(K_SECONDS(get_loop_delay_s()));
	}
}
