/*
 * Copyright (c) 2024 Modz, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
LOG_MODULE_REGISTER(app_sensors, LOG_LEVEL_DBG);

#include <golioth/client.h>
#include <golioth/stream.h>
#include <modem/lte_lc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/reboot.h>

#include "app_sensors.h"
#include "app_settings.h"
#include "lib/minmea/minmea.h"

#ifdef CONFIG_LIB_OSTENTUS
#include <libostentus.h>
#endif
#ifdef CONFIG_ALUDEL_BATTERY_MONITOR
#include "battery_monitor/battery.h"
#endif

#define NMEA_SIZE		       128
#define OBD2_PID_REQUEST_ID	       0x7DF
#define ODB2_PID_REQUEST_DATA_LENGTH   2
#define OBD2_PID_RESPONSE_ID	       0x7E8
#define OBD2_PID_RESPONSE_DLC	       8
#define OBD2_SERVICE_SHOW_CURRENT_DATA 0x01
#define ODB2_PID_VEHICLE_SPEED	       0x0D
#define ODB2_PID_VEHICLE_SPEED_DLC     4

#define NAVITAS_TPDO4_PID              (0x103U)
#define NAVITAS_TPDO4_DLC              (8U)
#define NAVITAS_TPDO4_SPEED_HI_IDX     (5U)
#define NAVITAS_TPDO4_SPEED_LO_IDX     (4U)
#define NAVITAS_TPDO4_SPEED_KOEF       (0.1F)
#define NAVITAS_TPDO4_VEHICLE_INFO_IDX (0U)

#define NAVITAS_TPDO4_GEAR_SIGNAL_SHFT (0U)
#define NAVITAS_TPDO4_GEAR_SIGNAL_MASK (3U << NAVITAS_TPDO4_GEAR_SIGNAL_SHFT)
#define NAVITAS_TPDO4_THROTTLE_SW_SHFT (2U)
#define NAVITAS_TPDO4_THROTTLE_SW_MASK (1U << NAVITAS_TPDO4_THROTTLE_SW_SHFT)
#define NAVITAS_TPDO4_BRAKE_SW_SHFT    (3U)
#define NAVITAS_TPDO4_BRAKE_SW_MASK    (1U << NAVITAS_TPDO4_BRAKE_SW_SHFT)
#define NAVITAS_TPDO4_BRAKE_STATE_SHFT (4U)
#define NAVITAS_TPDO4_BRAKE_STATE_MASK (1U << NAVITAS_TPDO4_BRAKE_STATE_SHFT)
#define NAVITAS_TPDO4_SPEED_MODE_SHFT  (5U)
#define NAVITAS_TPDO4_SPEED_MODE_MASK  (1U << NAVITAS_TPDO4_SPEED_MODE_SHFT)

#define NAVITAS_TPDO4_GEAR_INVALID     (0U)
#define NAVITAS_TPDO4_GEAR_FORWARD     (1U)
#define NAVITAS_TPDO4_GEAR_NEUTRAL     (2U)
#define NAVITAS_TPDO4_GEAR_REVERSE     (3U)
#define NAVITAS_TPDO4_THROTTLE_OPEN    (0U)
#define NAVITAS_TPDO4_THROTTLE_CLOSED  (1U)
#define NAVITAS_TPDO4_BRAKE_OPEN       (0U)
#define NAVITAS_TPDO4_BRAKE_CLOSED     (1U)
#define NAVITAS_TPDO4_BRAKE_DRIVING    (0U)
#define NAVITAS_TPDO4_BRAKE_BRAKING    (1U)
#define NAVITAS_TPDO4_SPEED_TURTLE     (0U)
#define NAVITAS_TPDO4_SPEED_RABBIT     (1U)

#define NAVITAS_TPDO5_PID              (0x203U)
#define NAVITAS_TPDO5_DLC              (8U)
#define NAVITAS_TPDO5_UNITS_IDX        (7U)
#define NAVITAS_TPDO5_BAT_SOC_IDX      (6U)
#define NAVITAS_TPDO5_BAT_VOLT_HI_IDX  (5U)
#define NAVITAS_TPDO5_BAT_VOLT_LO_IDX  (4U)
#define NAVITAS_TPDO5_BAT_VOLT_KOEFF   (0.1F)
#define NAVITAS_TPDO5_BAT_CURR_HI_IDX  (3U)
#define NAVITAS_TPDO5_BAT_CURR_LO_IDX  (2U)
#define NAVITAS_TPDO5_BAT_CURR_KOEFF   (0.1F)
#define NAVITAS_TPDO5_FAULT_CODE_IDX   (1U)
#define NAVITAS_TPDO5_ALARM_CODE_IDX   (0U)

#define NAVITAS_TPDO6_PID              (0x303U)
#define NAVITAS_TPDO6_DLC              (8U)
#define NAVITAS_TPDO6_ODOMETER_KOEFF   (0.1F)
#define NAVITAS_TPDO6_ODOMETER_HI_IDX  (7U)
#define NAVITAS_TPDO6_ODOMETER_MID_IDX (6U)
#define NAVITAS_TPDO6_ODOMETER_LO_IDX  (5U)
#define NAVITAS_TPDO6_SW_VERSION_IDX   (4U)
#define NAVITAS_TPDO6_SW_ID_IDX        (3U)
#define NAVITAS_TPDO6_HW_VERSION_IDX   (2U)
#define NAVITAS_TPDO6_ECU_TEMP_IDX 	   (1U)
#define NAVITAS_TPDO6_MOTOR_TEMP_IDX   (0U)

#define GOLIOTH_STREAM_TIMEOUT_S       2

#ifndef APP_SENSORS_ERRORS_TO_REBOOT
#define APP_SENSORS_ERRORS_TO_REBOOT   (5)
#endif

static struct golioth_client *client;

#define UART_DEVICE_NODE DT_ALIAS(click_uart)
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

#define UART_SEL DT_ALIAS(gnss7_sel)
static const struct gpio_dt_spec gnss7_sel = GPIO_DT_SPEC_GET(UART_SEL, gpios);

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

/* Navitas data received by CAN bus*/
struct navitas_vehicle_data {
	int speed;
	int gear_signal;
	int throttle_sw;
	int brake_sw;
	int brake_state;
	int speed_mode;
};

struct navitas_battery_data {
	int SOC;
	int voltage;
	int current;
	int fault_code;
	int alarm_code;
};

struct navitas_temperature_data {
	int motor_temp;
	int controller_temp;
	int odometer;
};

struct navitas_all_data {
	struct navitas_vehicle_data vehicle_data;
	struct navitas_battery_data battery_data;
	struct navitas_temperature_data temperature_data;
};
/* Full information about car */
struct can_asset_tracker_data {
	struct minmea_sentence_rmc rmc_frame;
	struct navitas_vehicle_data vehicle_data;
	struct navitas_battery_data battery_data;
	struct navitas_temperature_data temperature_data;
};

typedef enum {
	GPS_STATE_NO_FIX = 0,
	GPS_STATE_FIX = 1,
	GPS_STATE_FAKE = -1,
} gps_state_t;

K_MSGQ_DEFINE(cat_msgq, sizeof(struct can_asset_tracker_data), 64, 4);
K_MSGQ_DEFINE(rmc_msgq, sizeof(struct minmea_sentence_rmc), 2, 4);
CAN_MSGQ_DEFINE(can_msgq, 10);

#define PROCESS_CAN_FRAMES_THREAD_STACK_SIZE 2048
#define PROCESS_CAN_FRAMES_THREAD_PRIORITY   2
static k_tid_t process_can_frames_tid;
struct k_thread process_can_frames_thread_data;
K_THREAD_STACK_DEFINE(process_can_frames_thread_stack, PROCESS_CAN_FRAMES_THREAD_STACK_SIZE);

#define PROCESS_RMC_FRAMES_THREAD_STACK_SIZE 2048
#define PROCESS_RMC_FRAMES_THREAD_PRIORITY   2
static k_tid_t process_rmc_frames_tid;
struct k_thread process_rmc_frames_thread_data;
K_THREAD_STACK_DEFINE(process_rmc_frames_thread_stack, PROCESS_RMC_FRAMES_THREAD_STACK_SIZE);

/* Global state shared between threads */
#define SHARED_DATA_MUTEX_TIMEOUT 1000
K_MUTEX_DEFINE(navitas_shared_data_mutex);
static struct navitas_all_data navitas_shared_data;

/* Formatting strings for sending sensor JSON to Golioth */
/* clang-format off */
#define JSON_FMT_TIMESTAM_FIELD_FMT "\"time\":\"20%02d-%02d-%02dT%02d:%02d:%02d.%03dZ\","
#define JSON_FMT \
"{" \
	"%s"\
	"\"gps\":" \
	"{" \
		"\"lat\":%s," \
		"\"lon\":%s," \
		"\"speed\":%d," \
		"\"state\":%d" \
	"}," \
	"\"vehicle\":" \
	"{" \
		"\"speed\":%d," \
		"\"gear signal\":%d," \
		"\"throttle switch\":%d," \
		"\"brake switch\":%d," \
		"\"braking state\":%d," \
		"\"rabbit / turtle\":%d," \
		"\"odometer\":%d" \
	"}," \
	"\"battery\":" \
	"{" \
		"\"SOC\":%d," \
		"\"voltage\":%d," \
		"\"current\":%d," \
		"\"fault code\":%d," \
		"\"alarm code\":%d" \
	"}," \
	"\"temp\":" \
	"{" \
		"\"motor\":%d," \
		"\"ecu\":%d" \
	"}" \
"}"
/* clang-format on */

/* GNSS processing functions*/
/**
 * Convert a floating point coordinate value to a minmea_float coordinate.
 *
 * NMEA latitude is represented as [+-]DDMM.MMMM... [-90.0, 90.0]
 * NMEA longitude is represented as [+-]DDDMM.MMMM... [-180.0, 180.0]
 *
 * For example, a float -123.456789 will be be converted to:
 *   degrees = -123
 *   minutes = -0.456789 * 60 = -27.40734
 *   NMEA = (degrees * 100) + minutes = -12327.40734
 *
 * minmea_float stores the .value as a int_least32_t with a .scale factor:
 *   .value = (int_least32_t)(NMEA * .scale)
 *
 * So, NMEA -12327.40734 will be converted to:
 *   .value = -1232740734
 *   .scale = 100000
 *
 * 100000 scaling factor provides 5 digits of precision (±2cm LSB at the
 * equator) for the minutes value.
 *
 * Note: 5 digits of precision is the max we can use. If we were to try to use
 * 6 digits of precision (a scaling factor of 1000000), NMEA -12327.40734 would
 * be converted to .value = -12327407340, which would overflow int32_t:
 *   INT32_MIN =  -2147483648
 *   .value    = -12327407340 <- overflow!
 */
static inline int coord_to_minmea(struct minmea_float *f, float coord)
{
	int32_t degrees = (int32_t)coord;
	float minutes = (coord - degrees) * 60;

	/* Convert degrees to NMEA [+-]DDDMM.MMMMM format */
	degrees *= 100;

	/**
	 * Use 100000 as the scaling factor so that we can store up to 5 decimal
	 * places of minutes in minmea_float.value
	 */
	degrees *= 100000;
	minutes *= 100000;

	/* Make sure we don't overflow int32_t */
	if (minutes < INT32_MIN || minutes > (float)(INT32_MAX - 1)) {
		return -ERANGE;
	}

	/* minmea_float uses int_least32_t internally */
	f->value = (int_least32_t)(degrees + minutes);
	f->scale = 100000;

	return 0;
}


static inline int float_to_minmea(struct minmea_float *f, float value)
{
	/* minmea_float uses int_least32_t internally */
	f->value = (int_least32_t)(value * 1024);
	f->scale = 1024;

	return 0;
}


/* Navitas Display Protocol processing functions */
void navitas_processing_tpdo_4(struct can_frame* tpdo4_frame, struct navitas_vehicle_data* vehicle_data)
{
	if (NAVITAS_TPDO4_DLC == can_dlc_to_bytes(tpdo4_frame->dlc)) {
		vehicle_data->speed = (int)(NAVITAS_TPDO4_SPEED_KOEF *
				      (float)((((int16_t)tpdo4_frame->data[NAVITAS_TPDO4_SPEED_HI_IDX]) << 8U) |
				              (((int16_t)tpdo4_frame->data[NAVITAS_TPDO4_SPEED_LO_IDX]) << 0U)));
		vehicle_data->gear_signal = (tpdo4_frame->data[NAVITAS_TPDO4_VEHICLE_INFO_IDX] & NAVITAS_TPDO4_GEAR_SIGNAL_MASK)
					     << NAVITAS_TPDO4_GEAR_SIGNAL_SHFT;
		vehicle_data->throttle_sw = (tpdo4_frame->data[NAVITAS_TPDO4_VEHICLE_INFO_IDX] & NAVITAS_TPDO4_THROTTLE_SW_MASK)
					     << NAVITAS_TPDO4_THROTTLE_SW_SHFT;
		vehicle_data->brake_sw    = (tpdo4_frame->data[NAVITAS_TPDO4_VEHICLE_INFO_IDX] & NAVITAS_TPDO4_BRAKE_SW_MASK)
					     << NAVITAS_TPDO4_BRAKE_SW_SHFT;
		vehicle_data->brake_state = (tpdo4_frame->data[NAVITAS_TPDO4_VEHICLE_INFO_IDX] & NAVITAS_TPDO4_BRAKE_STATE_MASK)
					     << NAVITAS_TPDO4_BRAKE_STATE_SHFT;
		vehicle_data->speed_mode  = (tpdo4_frame->data[NAVITAS_TPDO4_VEHICLE_INFO_IDX] & NAVITAS_TPDO4_SPEED_MODE_MASK)
					     << NAVITAS_TPDO4_SPEED_MODE_SHFT;
	}
	else {
		LOG_ERR("Wrong DLC(=[%d]) for TPDO4", can_dlc_to_bytes(tpdo4_frame->dlc));
	}
}

void navitas_processing_tpdo_5(struct can_frame* tpdo5_frame, struct navitas_battery_data* battery_data)
{
	if (NAVITAS_TPDO5_DLC == can_dlc_to_bytes(tpdo5_frame->dlc)) {
		battery_data->SOC = tpdo5_frame->data[NAVITAS_TPDO5_BAT_SOC_IDX];
		battery_data->voltage = (int)(NAVITAS_TPDO5_BAT_VOLT_KOEFF *
					(float)((((int16_t)tpdo5_frame->data[NAVITAS_TPDO5_BAT_VOLT_HI_IDX]) << 8U) |
					        (((int16_t)tpdo5_frame->data[NAVITAS_TPDO5_BAT_VOLT_LO_IDX]) << 0U)));
		battery_data->current = (int)(NAVITAS_TPDO5_BAT_CURR_KOEFF *
					(float)((((int16_t)tpdo5_frame->data[NAVITAS_TPDO5_BAT_CURR_HI_IDX]) << 8U) |
					        (((int16_t)tpdo5_frame->data[NAVITAS_TPDO5_BAT_CURR_LO_IDX]) << 0U)));
		battery_data->fault_code = tpdo5_frame->data[NAVITAS_TPDO5_FAULT_CODE_IDX];
		battery_data->alarm_code = tpdo5_frame->data[NAVITAS_TPDO5_ALARM_CODE_IDX];
	}
	else {
		LOG_ERR("Wrong DLC(=[%d]) for TPDO5", can_dlc_to_bytes(tpdo5_frame->dlc));
	}
}

void navitas_processing_tpdo_6(struct can_frame* tpdo_frame, struct navitas_temperature_data* temperature_data)
{
	if (NAVITAS_TPDO6_DLC == can_dlc_to_bytes(tpdo_frame->dlc)) {
		temperature_data->motor_temp = (int8_t)tpdo_frame->data[NAVITAS_TPDO6_MOTOR_TEMP_IDX];
		temperature_data->controller_temp = (int8_t)tpdo_frame->data[NAVITAS_TPDO6_ECU_TEMP_IDX];
		temperature_data->odometer = ((((uint32_t)tpdo_frame->data[NAVITAS_TPDO6_ODOMETER_HI_IDX]) << 16U) |
									(((uint32_t)tpdo_frame->data[NAVITAS_TPDO6_ODOMETER_MID_IDX]) << 8U) |
									(((uint32_t)tpdo_frame->data[NAVITAS_TPDO6_ODOMETER_LO_IDX]) << 0U));
	}
	else {
		LOG_ERR("Wrong DLC(=[%d]) for TPDO6", can_dlc_to_bytes(tpdo_frame->dlc));
	}
}

void can_add_filter(const struct device *dev, struct k_msgq *msgq, const struct can_filter *filter)
{
	int can_filter_id = -ECANCELED;

	/* Automatically put frames matching can_filter into can_msgq */
	can_filter_id = can_add_rx_filter_msgq(dev, msgq, filter);
	if (can_filter_id == -ENOSPC) {
		LOG_ERR("No free CAN filters [%d]", can_filter_id);
		return;
	} else if (can_filter_id == -ENOTSUP) {
		LOG_ERR("CAN filter type not supported [%d]", can_filter_id);
		return;
	} else if (can_filter_id < 0) {
		LOG_ERR("Error adding a message queue for the given filter [%d]", can_filter_id);
		return;
	}
	LOG_DBG("CAN bus receive filter id: %d", can_filter_id);
}



void process_can_frames_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	int err;
	struct can_frame can_frame;
	struct navitas_all_data navitas_local_data;
	bool navitas_tpdo4_received = false;
	bool navitas_tpdo5_received = false;
	bool navitas_tpdo6_received = false;
	const struct can_filter navitas_can_filter_1 = {.flags = CAN_FILTER_DATA, .id = NAVITAS_TPDO4_PID, .mask = CAN_EXT_ID_MASK};
	const struct can_filter navitas_can_filter_2 = {.flags = CAN_FILTER_DATA, .id = NAVITAS_TPDO5_PID, .mask = CAN_EXT_ID_MASK};
	const struct can_filter navitas_can_filter_3 = {.flags = CAN_FILTER_DATA, .id = NAVITAS_TPDO6_PID, .mask = CAN_EXT_ID_MASK};

	/* Add filter for Navitas can messages*/
	can_add_filter(can_dev, &can_msgq, &navitas_can_filter_1);
	can_add_filter(can_dev, &can_msgq, &navitas_can_filter_2);
	can_add_filter(can_dev, &can_msgq, &navitas_can_filter_3);

	while (1) {
		navitas_tpdo4_received = false;
		navitas_tpdo5_received = false;
		navitas_tpdo6_received = false;

		navitas_local_data.vehicle_data.speed = -1;
		navitas_local_data.vehicle_data.gear_signal = -1;
		navitas_local_data.vehicle_data.throttle_sw = -1;
		navitas_local_data.vehicle_data.brake_sw = -1;
		navitas_local_data.vehicle_data.brake_state = -1;
		navitas_local_data.vehicle_data.speed_mode = -1;

		navitas_local_data.battery_data.SOC = -1;
		navitas_local_data.battery_data.voltage = -1;
		navitas_local_data.battery_data.current = -1;
		navitas_local_data.battery_data.fault_code = -1;
		navitas_local_data.battery_data.alarm_code = -1;

		navitas_local_data.temperature_data.motor_temp = -128;
		navitas_local_data.temperature_data.controller_temp = -128;
		navitas_local_data.temperature_data.odometer = -1;

		/* Wait up to 500ms for a message */
		while (k_msgq_get(&can_msgq, &can_frame, K_MSEC(500)) == 0) {
			switch (can_frame.id) {
				case NAVITAS_TPDO4_PID:
					navitas_tpdo4_received = true;
					navitas_processing_tpdo_4(&can_frame, &navitas_local_data.vehicle_data);
					LOG_DBG("ID: 0x%.4X successfully parced", can_frame.id);
					break;

				case NAVITAS_TPDO5_PID:
					navitas_tpdo5_received = true;
					navitas_processing_tpdo_5(&can_frame, &navitas_local_data.battery_data);
					LOG_DBG("ID: 0x%.4X successfully parced", can_frame.id);
					break;

				case NAVITAS_TPDO6_PID:
					navitas_tpdo6_received = true;
					navitas_processing_tpdo_6(&can_frame, &navitas_local_data.temperature_data);
					LOG_DBG("ID: 0x%.4X successfully parced", can_frame.id);
					break;

				default:
					LOG_ERR("Unknown CAN frame ID: 0x%.4X", can_frame.id);
					break;
			}

			if (navitas_tpdo4_received &&
			    navitas_tpdo5_received &&
			    navitas_tpdo6_received) {
				break;
			}
		}
		/* Update shared global state */
		err = k_mutex_lock(&navitas_shared_data_mutex, K_MSEC(SHARED_DATA_MUTEX_TIMEOUT));
		if (err) {
			LOG_ERR("Error locking shared data mutex (lock count: %u): %d", err,
				navitas_shared_data_mutex.lock_count);
			k_sleep(K_SECONDS(get_vehicle_speed_delay_s()));
			continue;
		}
		navitas_shared_data.vehicle_data = navitas_local_data.vehicle_data;
		navitas_shared_data.battery_data = navitas_local_data.battery_data;
		navitas_shared_data.temperature_data = navitas_local_data.temperature_data;
		k_mutex_unlock(&navitas_shared_data_mutex);

		if (navitas_tpdo4_received)
		{
			/* Log vehicle speed */
			LOG_DBG("Vehicle Speed Sensor: %d km/h", navitas_local_data.vehicle_data.speed);
			LOG_DBG("Vehicle Gear Signal: %d km/h", navitas_local_data.vehicle_data.gear_signal);
			LOG_DBG("Vehicle Throttle sw: %d km/h", navitas_local_data.vehicle_data.throttle_sw);
			LOG_DBG("Vehicle brake sw: %d km/h", navitas_local_data.vehicle_data.brake_sw);
			LOG_DBG("Vehicle brake state: %d km/h", navitas_local_data.vehicle_data.brake_state);
			LOG_DBG("Vehicle speed mode: %d km/h", navitas_local_data.vehicle_data.speed_mode);
		}

		if (navitas_tpdo5_received)
		{
		/* Log vehicle speed */
			LOG_DBG("Battery SOC is: %d %%", navitas_local_data.battery_data.SOC);
			LOG_DBG("Battery voltage: %d V", navitas_local_data.battery_data.voltage);
			LOG_DBG("Battery current: %d A", navitas_local_data.battery_data.current);
			LOG_DBG("Battery fault code: %d", navitas_local_data.battery_data.fault_code);
			LOG_DBG("Battery alarm code: %d", navitas_local_data.battery_data.alarm_code);
		}

		if (navitas_tpdo6_received)
		{
		/* Log vehicle temperature and o */
			LOG_DBG("Motor Temp is: %d *C", navitas_local_data.temperature_data.motor_temp);
			LOG_DBG("Controller Temp is: %d *C", navitas_local_data.temperature_data.controller_temp);
			LOG_DBG("Odometer is: %.1f km", navitas_local_data.temperature_data.odometer * NAVITAS_TPDO6_ODOMETER_KOEFF);
		}

		if (false && !(navitas_tpdo4_received || navitas_tpdo5_received || navitas_tpdo6_received)) {
			int ret;
			enum can_state cstate = 0;
			struct can_bus_err_cnt cerr_cnt;
			ret = can_get_state(can_dev, &cstate, &cerr_cnt);
			LOG_DBG("No TPDO: %d %d %d, 0x%X, 0x%X, %u, %u", navitas_tpdo4_received, navitas_tpdo5_received, navitas_tpdo6_received,
				ret, cstate, cerr_cnt.tx_err_cnt, cerr_cnt.rx_err_cnt);
			// struct can_frame frame = {
			// 	.flags = 0,
			// 	.id = NAVITAS_TPDO6_PID,
			// 	.dlc = 8,
			// 	.data = {1,2,3,4,5,6,7,8}
			// };


			// ret = can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);
			// if (ret != 0) {
			// 	LOG_ERR("Sending failed [%d]", ret);
			// }
		}

		/* Update Ostentus slide values */
		IF_ENABLED(CONFIG_LIB_OSTENTUS, (
			char vehicle_speed_str[9];

			snprintk(vehicle_speed_str, sizeof(vehicle_speed_str), "%d km/h",
				 navitas_shared_data.vehicle_data.speed);
			slide_set(VEHICLE_SPEED, vehicle_speed_str, strlen(vehicle_speed_str));
		));

		k_sleep(K_SECONDS(get_vehicle_speed_delay_s()));
	}
}

void process_rmc_frames_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	int err;
	struct minmea_sentence_rmc rmc_frame;
	struct can_asset_tracker_data cat_frame;

	while (k_msgq_get(&rmc_msgq, &rmc_frame, K_FOREVER) == 0) {
		cat_frame.rmc_frame = rmc_frame;

		/* Use the latest vehicle speed reading received from the ECU */
		err = k_mutex_lock(&navitas_shared_data_mutex, K_MSEC(SHARED_DATA_MUTEX_TIMEOUT));
		if (err) {
			LOG_ERR("Error locking shared data mutex (lock count: %u): %d", err,
				navitas_shared_data_mutex.lock_count);
		}
		cat_frame.vehicle_data = navitas_shared_data.vehicle_data;
		cat_frame.battery_data = navitas_shared_data.battery_data;
		cat_frame.temperature_data = navitas_shared_data.temperature_data;
		k_mutex_unlock(&navitas_shared_data_mutex);

		err = k_msgq_put(&cat_msgq, &cat_frame, K_NO_WAIT);
		if (err) {
			LOG_ERR("Unable to add cat_frame to cat_msgq: %d", err);
		}

		LOG_DBG("GPS Position%s: %f, %f", cat_frame.rmc_frame.valid ? "" : " (fake)",
			minmea_tocoord(&rmc_frame.latitude), minmea_tocoord(&rmc_frame.longitude));

		LOG_DBG("GPS Speed%s: %f kmph", cat_frame.rmc_frame.valid ? "" : " (fake)",
			minmea_tofloat(&rmc_frame.speed));

		/* Update Ostentus slide values */
		IF_ENABLED(CONFIG_LIB_OSTENTUS, (
			char lat_str[12];
			char lon_str[12];

			snprintk(lat_str, sizeof(lat_str), "%f",
				 minmea_tocoord(&rmc_frame.latitude));
			snprintk(lon_str, sizeof(lon_str), "%f",
				 minmea_tocoord(&rmc_frame.longitude));
			slide_set(LATITUDE, lat_str, strlen(lat_str));
			slide_set(LONGITUDE, lon_str, strlen(lon_str));
		));
	}
}

/* This is called from the UART irq callback to try to get out fast */
static void process_reading(char *raw_nmea)
{
	/* _last_gps timestamp records when the previous GPS value was stored */
	static uint64_t _last_gps;
	enum minmea_sentence_id sid;
	sid = minmea_sentence_id(raw_nmea, false);
	//LOG_DBG("NMEA(%u): %s", sid, raw_nmea);
	if (sid == MINMEA_SENTENCE_RMC) {
		struct minmea_sentence_rmc rmc_frame;
		bool success = minmea_parse_rmc(&rmc_frame, raw_nmea);
		//LOG_DBG("RMC(%u): %s", success, raw_nmea);
		if (success) {
			uint64_t wait_for = _last_gps;
			if (k_uptime_delta(&wait_for) >= ((uint64_t)get_gps_delay_s() * 1000)) {
				if (!rmc_frame.valid) {
					if (get_fake_gps_enabled_s()) {
						/* use fake GPS coordinates from LightDB state */
						coord_to_minmea(&rmc_frame.latitude, get_fake_gps_latitude_s());
						coord_to_minmea(&rmc_frame.longitude, get_fake_gps_longitude_s());
						float_to_minmea(&rmc_frame.speed, 123U);
					} else {
						coord_to_minmea(&rmc_frame.latitude, 0);
						coord_to_minmea(&rmc_frame.longitude, 0);
						float_to_minmea(&rmc_frame.speed, 0);
					}
				}

				/* if queue is full, message is silently dropped */
				k_msgq_put(&rmc_msgq, &rmc_frame, K_NO_WAIT);

				/*
				* wait_for now contains the current timestamp. Store this
				* for the next reading.
				*/
				_last_gps = wait_for;
			} else {
				/* LOG_DBG("Ignoring reading due to gps_delay_s window"); */
			}
		}
	}
}

/* UART callback */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;
	static char rx_buf[NMEA_SIZE];
	static int rx_buf_pos;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	while (uart_irq_rx_ready(uart_dev)) {

		uart_fifo_read(uart_dev, &c, 1);

		if ((c == '\n') && rx_buf_pos > 0) {
			/* terminate string */
			if (rx_buf_pos == (NMEA_SIZE - 1)) {
				rx_buf[rx_buf_pos] = '\0';
			} else {
				rx_buf[rx_buf_pos] = '\n';
				rx_buf[rx_buf_pos + 1] = '\0';
			}

			process_reading(rx_buf);
			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}

void app_sensors_init(void)
{
	int err;
	struct can_timing timing;

	LOG_DBG("Initializing GNSS receiver");

	err = gpio_pin_configure_dt(&gnss7_sel, GPIO_OUTPUT_ACTIVE);
	if (err < 0) {
		LOG_ERR("Unable to configure GNSS SEL Pin: %d", err);
	}

	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART device %s not ready", uart_dev->name);
	}

	/* Configure UART interrupt and callback to receive data */
	uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	uart_irq_rx_enable(uart_dev);

	LOG_DBG("Initializing CAN controller");

	if (!device_is_ready(can_dev)) {
		LOG_ERR("CAN device %s not ready", can_dev->name);
	}

	err = can_calc_timing(can_dev, &timing, 250000, 800);
	if (err > 0) {
		LOG_INF("Sample-Point error: %d", err);
	}

	if (err < 0) {
		LOG_ERR("Failed to calc a valid timing: %d", err);
	}

	err = can_set_timing(can_dev, &timing);
	if (err != 0) {
		LOG_ERR("Failed to set timing: %d", err);
	}

	//can_set_mode(can_dev, CAN_MODE_LOOPBACK);

	/* Start the CAN controller */
	err = can_start(can_dev);
	if (err == -EALREADY) {
		LOG_ERR("CAN controller already started [%d]", err);
	} else if (err != 0) {
		LOG_ERR("Error starting CAN controller [%d]", err);
	}

	/* Spawn a thread to process CAN frames */
	process_can_frames_tid = k_thread_create(
		&process_can_frames_thread_data, process_can_frames_thread_stack,
		K_THREAD_STACK_SIZEOF(process_can_frames_thread_stack), process_can_frames_thread,
		NULL, NULL, NULL, PROCESS_CAN_FRAMES_THREAD_PRIORITY, 0, K_NO_WAIT);
	if (!process_can_frames_tid) {
		LOG_ERR("Error spawning CAN frame processing thread");
	}

	/* Spawn a thread to process RMC frames */
	process_rmc_frames_tid = k_thread_create(
		&process_rmc_frames_thread_data, process_rmc_frames_thread_stack,
		K_THREAD_STACK_SIZEOF(process_rmc_frames_thread_stack), process_rmc_frames_thread,
		NULL, NULL, NULL, PROCESS_RMC_FRAMES_THREAD_PRIORITY, 0, K_NO_WAIT);
	if (!process_rmc_frames_tid) {
		LOG_ERR("Error spawning RMC frame processing thread");
	}
}

/* This will be called by the main() loop */
/* Do all of your work here! */
void app_sensors_read_and_stream(void)
{
	static int network_errors_count = 0;
	static bool network_disabled = false;

	int err;
	struct can_asset_tracker_data cached_data;
	char json_buf[512];
	char ts_str[40];
	char lat_str[12];
	char lon_str[12];
	int speed;
	gps_state_t gps_state = GPS_STATE_NO_FIX;

	IF_ENABLED(CONFIG_ALUDEL_BATTERY_MONITOR, (
		read_and_report_battery(client);
		IF_ENABLED(CONFIG_LIB_OSTENTUS, (
			slide_set(BATTERY_V, get_batt_v_str(), strlen(get_batt_v_str()));
			slide_set(BATTERY_LVL, get_batt_lvl_str(), strlen(get_batt_lvl_str()));
		));
	));

	if (network_disabled) {
		LOG_WRN("Enable LTE network after Golioth errors");
		lte_lc_normal();
		network_disabled = false;
		return;
	}

	while (k_msgq_get(&cat_msgq, &cached_data, K_NO_WAIT) == 0) {
		snprintk(lat_str, sizeof(lat_str), "%f",
			 minmea_tocoord(&cached_data.rmc_frame.latitude));
		snprintk(lon_str, sizeof(lon_str), "%f",
			 minmea_tocoord(&cached_data.rmc_frame.longitude));

		speed = (int)minmea_tofloat(&cached_data.rmc_frame.speed);

		if (cached_data.rmc_frame.valid) {
			/*
			 * `time` will not appear in the `data` payload once received
			 * by Golioth LightDB Stream, but instead will override the
			 * `time` timestamp of the data.
			 */
			snprintk(ts_str, sizeof(ts_str), JSON_FMT_TIMESTAM_FIELD_FMT,
				cached_data.rmc_frame.date.year, cached_data.rmc_frame.date.month,
				cached_data.rmc_frame.date.day, cached_data.rmc_frame.time.hours,
				cached_data.rmc_frame.time.minutes, cached_data.rmc_frame.time.seconds,
				cached_data.rmc_frame.time.microseconds);

			gps_state = GPS_STATE_FIX;
		} else {
			/* Fake GPS data does not have a `time` field */
			LOG_DBG("Pack fake GPS data");
			ts_str[0] = '\0';
			gps_state = get_fake_gps_enabled_s() ? GPS_STATE_FAKE : GPS_STATE_NO_FIX; // Can be changed while the packet is enqueued, it can be stored with the packet in case of troubles
		}

		snprintk(json_buf, sizeof(json_buf), JSON_FMT,
			ts_str,
			lat_str,
			lon_str,
			speed,
			gps_state,
			cached_data.vehicle_data.speed,
			cached_data.vehicle_data.gear_signal,
			cached_data.vehicle_data.throttle_sw,
			cached_data.vehicle_data.brake_sw,
			cached_data.vehicle_data.brake_state,
			cached_data.vehicle_data.speed_mode,
			cached_data.temperature_data.odometer,
			cached_data.battery_data.SOC,
			cached_data.battery_data.voltage,
			cached_data.battery_data.current,
			cached_data.battery_data.fault_code,
			cached_data.battery_data.alarm_code,
			cached_data.temperature_data.motor_temp,
			cached_data.temperature_data.controller_temp);

		err = golioth_stream_set_sync(client, "tracker", GOLIOTH_CONTENT_TYPE_JSON,
					      json_buf, strlen(json_buf), GOLIOTH_STREAM_TIMEOUT_S);
		if (err) {
			LOG_ERR("Failed to send sensor data to Golioth: %d", err);

			if (err == GOLIOTH_ERR_QUEUE_FULL) {
				// Possible troubles with the network, try to restart it or the device
				if (network_errors_count++ < APP_SENSORS_ERRORS_TO_REBOOT) {
					LOG_WRN("Restart LTE network due to Golioth errors");
					lte_lc_power_off();
					network_disabled = true;
				} else {
					LOG_ERR("Restart the device due to Golioth errors");
					LOG_PANIC();
					sys_reboot(SYS_REBOOT_COLD);
				}
			}
		} else {
			network_errors_count = 0;
		}
	}
}

void app_sensors_set_client(struct golioth_client *sensors_client)
{
	client = sensors_client;
}
