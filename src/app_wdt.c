#include "app_wdt.h"

#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <stdbool.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
LOG_MODULE_REGISTER(app_wdt, LOG_LEVEL_INF);


#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_window_watchdog)
#define WDT_MAX_WINDOW  100U
#elif DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_wdt)
/* Nordic supports a callback, but it has 61.2 us to complete before
 * the reset occurs, which is too short to do anything
 * useful. Explicitly disallow use of the callback.
 */
#define WDT_ALLOW_CALLBACK 0
#elif DT_HAS_COMPAT_STATUS_OKAY(raspberrypi_pico_watchdog)
#define WDT_ALLOW_CALLBACK 0
#elif DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_wwdgt)
#define WDT_MAX_WINDOW 24U
#define WDT_MIN_WINDOW 18U
#elif DT_HAS_COMPAT_STATUS_OKAY(intel_tco_wdt)
#define WDT_ALLOW_CALLBACK 0
#define WDT_MAX_WINDOW 3000U
#elif DT_HAS_COMPAT_STATUS_OKAY(nxp_fs26_wdog)
#define WDT_MAX_WINDOW  1024U
#define WDT_MIN_WINDOW	320U
#define WDT_OPT 0
#endif

#ifndef WDT_ALLOW_CALLBACK
#define WDT_ALLOW_CALLBACK 1
#endif

#ifndef WDT_MAX_WINDOW
#define WDT_MAX_WINDOW  1000U
#endif

#ifndef WDT_MIN_WINDOW
#define WDT_MIN_WINDOW  0U
#endif

#ifndef WDT_OPT
#define WDT_OPT WDT_OPT_PAUSE_HALTED_BY_DBG
#endif


#ifndef APP_WDT_PERIOD
#define APP_WDT_PERIOD  (16384U)
#endif


#ifndef APP_WDT_FEED_INTERVAL
#define APP_WDT_FEED_INTERVAL  MAX(WDT_MIN_WINDOW, MIN(APP_WDT_PERIOD,WDT_MAX_WINDOW) / 4)
#endif


#define WDT_DEVICE_NODE DT_ALIAS(watchdog0)
static const struct device *const wdt_dev = DEVICE_DT_GET(WDT_DEVICE_NODE);
static int wdt_channel_id;
static int32_t wdt_timeout = 0;


#define PROCESS_WDT_FEEDER_THREAD_STACK_SIZE 1024
#define PROCESS_WDT_FEEDER_THREAD_PRIORITY   2
static k_tid_t process_wdt_feeder_tid;
struct k_thread process_wdt_feeder_thread_data;
K_THREAD_STACK_DEFINE(process_wdt_feeder_thread_stack, PROCESS_WDT_FEEDER_THREAD_STACK_SIZE);


#if WDT_ALLOW_CALLBACK
static void wdt_callback(const struct device *wdt_dev, int channel_id)
{
	static bool handled_event;

	if (handled_event) {
		return;
	}

	wdt_feed(wdt_dev, channel_id);

	LOG_ERR("Handled WDT. Ready to reset");
	handled_event = true;
}
#endif /* WDT_ALLOW_CALLBACK */


static void reset_wdt_timeout(void)
{
    wdt_timeout = APP_WDT_PERIOD;
}


static void app_wdt_feed_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

    while (1) {
        if (wdt_timeout > 0) {
            LOG_DBG("Feeding watchdog...");
            wdt_feed(wdt_dev, wdt_channel_id);
            wdt_timeout -= APP_WDT_FEED_INTERVAL;
        }

        k_sleep(K_MSEC(APP_WDT_FEED_INTERVAL));
    }
}


void app_wdt_feed(void) {
    reset_wdt_timeout();
}


int app_wdt_init(void)
{
	int err;

	LOG_DBG("Watchdog init");

	if (!device_is_ready(wdt_dev)) {
		LOG_ERR("%s: device not ready", wdt_dev->name);
		return 0;
	}

	struct wdt_timeout_cfg wdt_config = {
		/* Reset SoC when watchdog timer expires. */
		.flags = WDT_FLAG_RESET_SOC,

		/* Expire watchdog after max window */
		.window.min = WDT_MIN_WINDOW,
		.window.max = WDT_MAX_WINDOW,
	};

#if WDT_ALLOW_CALLBACK
	/* Set up watchdog callback. */
	wdt_config.callback = wdt_callback;

	LOG_DBG("Attempting to setup pre-reset callback");
#else /* WDT_ALLOW_CALLBACK */
	LOG_INF("Callback in RESET_SOC disabled for this platform");
#endif /* WDT_ALLOW_CALLBACK */

	wdt_channel_id = wdt_install_timeout(wdt_dev, &wdt_config);
	if (wdt_channel_id == -ENOTSUP) {
		LOG_WRN("Callback support rejected, continuing anyway");
		wdt_config.callback = NULL;
		wdt_channel_id = wdt_install_timeout(wdt_dev, &wdt_config);
	}

	if (wdt_channel_id < 0) {
		LOG_ERR("Watchdog install error %d", wdt_channel_id);
		return wdt_channel_id;
	}

	err = wdt_setup(wdt_dev, WDT_OPT);
	if (err < 0) {
		LOG_ERR("Watchdog setup error %d", err);
		return err;
	}

    reset_wdt_timeout();

    process_wdt_feeder_tid = k_thread_create(
		&process_wdt_feeder_thread_data, process_wdt_feeder_thread_stack,
		K_THREAD_STACK_SIZEOF(process_wdt_feeder_thread_stack), app_wdt_feed_thread,
		NULL, NULL, NULL, PROCESS_WDT_FEEDER_THREAD_PRIORITY, 0, K_NO_WAIT);

	if (!process_wdt_feeder_tid) {
		LOG_ERR("Error spawning WDT feed processing thread");
	}

    return 0;
}
