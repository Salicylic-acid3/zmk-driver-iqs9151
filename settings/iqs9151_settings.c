/*
 * Copyright (c) 2026 Salicylic_acid3
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * The trackpad settings the owner can change from the app.
 *
 * The Kconfig switches are still here; they are the defaults. A keyboard that
 * has never been touched behaves exactly as its .conf says, and the app's copy
 * of the setting only starts to matter once someone moves it.
 *
 * Reading. zmk_custom_setting_read_by_key walks the registry and compares two
 * strings, which is cheap but not free, and the arbitration that uses the two
 * pinch switches runs on every frame while two fingers are down and undecided.
 * So the driver samples those once per gesture, at touch-down, rather than
 * calling in here from the hot path -- see iqs9151_two_finger_update. The
 * resolution pair is not read on any path: it is pushed into the IC when it
 * changes and then lives in its registers, so the gesture code sees it for
 * free in the coordinates themselves.
 *
 * Splits. Each half runs its own copy of this driver and reads its own copy of
 * the setting, because the gesture is classified on the half that owns the
 * sensor. That means a write has to reach both: the custom-settings relay does
 * it when the request carries source = ZMK_CUSTOM_SETTING_SOURCE_ALL, and a
 * write aimed at one side changes one pad's behaviour and not the other's.
 * The app writes with SOURCE_ALL for exactly this reason; a client that does
 * not will leave the two pads disagreeing, which is confusing but not broken.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zmk/event_manager.h>

#include <cormoran/zmk/custom_settings.h>

#include <keebon/iqs9151/control.h>
#include <keebon/iqs9151/settings.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * Readable while Studio is locked, writable only when unlocked -- the
 * convention every other settings module here follows. Neither value is a
 * secret, and being able to see how a pad is configured without unlocking is
 * the point when the question is "why is this gesture not working".
 */
ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_one_hand_pinch, IQS9151_SETTINGS_SUBSYSTEM_ID, IQS9151_SETTING_ONE_HAND_PINCH_KEY,
    ZMK_CUSTOM_SETTING_VALUE_TYPE_BOOL,
    ZMK_CUSTOM_SETTING_VALUE_BOOL(IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PINCH_ENABLE)),
    ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,
    ZMK_CUSTOM_SETTING_PERMISSION_SECURE, ZMK_CUSTOM_SETTING_NO_CONSTRAINT);

ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_pinch_invert, IQS9151_SETTINGS_SUBSYSTEM_ID, IQS9151_SETTING_PINCH_INVERT_KEY,
    ZMK_CUSTOM_SETTING_VALUE_TYPE_BOOL,
    ZMK_CUSTOM_SETTING_VALUE_BOOL(IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PINCH_INVERT)),
    ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,
    ZMK_CUSTOM_SETTING_PERMISSION_SECURE, ZMK_CUSTOM_SETTING_NO_CONSTRAINT);

ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_cursor_report_interval, IQS9151_SETTINGS_SUBSYSTEM_ID,
    IQS9151_SETTING_CURSOR_REPORT_INTERVAL_KEY, ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,
    ZMK_CUSTOM_SETTING_VALUE_INT32(CONFIG_INPUT_IQS9151_CURSOR_REPORT_INTERVAL_MS),
    ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,
    ZMK_CUSTOM_SETTING_PERMISSION_SECURE, ZMK_CUSTOM_SETTING_RANGE_INT32(0, 100));

ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_swipe3_threshold_x, IQS9151_SETTINGS_SUBSYSTEM_ID,
    IQS9151_SETTING_SWIPE3_THRESHOLD_X_KEY, ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,
    ZMK_CUSTOM_SETTING_VALUE_INT32(CONFIG_INPUT_IQS9151_3F_SWIPE_THRESHOLD),
    ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,
    ZMK_CUSTOM_SETTING_PERMISSION_SECURE, ZMK_CUSTOM_SETTING_RANGE_INT32(1, 1000));

ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_swipe3_threshold_y, IQS9151_SETTINGS_SUBSYSTEM_ID,
    IQS9151_SETTING_SWIPE3_THRESHOLD_Y_KEY, ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,
    ZMK_CUSTOM_SETTING_VALUE_INT32(CONFIG_INPUT_IQS9151_3F_SWIPE_THRESHOLD_Y),
    ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,
    ZMK_CUSTOM_SETTING_PERMISSION_SECURE, ZMK_CUSTOM_SETTING_RANGE_INT32(1, 1000));

/*
 * The pad's coordinate scale, as two numbers the owner can move.
 *
 * The range is the device configuration's 12-bit coordinate scale, and the
 * floor is not 1: below a couple of hundred counts an axis has less precision
 * than the pointer needs and the pad starts to feel like a d-pad. The Kconfig
 * values remain the defaults, so a keyboard nobody has touched behaves exactly
 * as its .conf says.
 *
 * Unlike the two switches above, these are not sampled per gesture -- they are
 * pushed into the IC when they change and then live in its registers, so the
 * arbitration sees them for free in the coordinates themselves.
 */
ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_resolution_x, IQS9151_SETTINGS_SUBSYSTEM_ID, IQS9151_SETTING_RESOLUTION_X_KEY,
    ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,
    ZMK_CUSTOM_SETTING_VALUE_INT32(CONFIG_INPUT_IQS9151_RESOLUTION_X),
    ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,
    ZMK_CUSTOM_SETTING_PERMISSION_SECURE, ZMK_CUSTOM_SETTING_RANGE_INT32(200, 4095));

ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_resolution_y, IQS9151_SETTINGS_SUBSYSTEM_ID, IQS9151_SETTING_RESOLUTION_Y_KEY,
    ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,
    ZMK_CUSTOM_SETTING_VALUE_INT32(CONFIG_INPUT_IQS9151_RESOLUTION_Y),
    ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,
    ZMK_CUSTOM_SETTING_PERMISSION_SECURE, ZMK_CUSTOM_SETTING_RANGE_INT32(200, 4095));

/*
 * Pointer speed, per axis, in tenths of the device's own reporting.
 *
 * The range tops out at 20x because past that a single report jumps the cursor
 * across a window; the floor is 1 rather than 0 because a gain of zero is not
 * a slow pad, it is a dead one.
 */
ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_cursor_gain_x, IQS9151_SETTINGS_SUBSYSTEM_ID, IQS9151_SETTING_CURSOR_GAIN_X_KEY,
    ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,
    ZMK_CUSTOM_SETTING_VALUE_INT32(CONFIG_INPUT_IQS9151_CURSOR_GAIN_X_X10),
    ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,
    ZMK_CUSTOM_SETTING_PERMISSION_SECURE, ZMK_CUSTOM_SETTING_RANGE_INT32(1, 200));

ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_cursor_gain_y, IQS9151_SETTINGS_SUBSYSTEM_ID, IQS9151_SETTING_CURSOR_GAIN_Y_KEY,
    ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,
    ZMK_CUSTOM_SETTING_VALUE_INT32(CONFIG_INPUT_IQS9151_CURSOR_GAIN_Y_X10),
    ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,
    ZMK_CUSTOM_SETTING_PERMISSION_SECURE, ZMK_CUSTOM_SETTING_RANGE_INT32(1, 200));

ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_cursor_smoothing, IQS9151_SETTINGS_SUBSYSTEM_ID,
    IQS9151_SETTING_CURSOR_SMOOTHING_KEY, ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,
    ZMK_CUSTOM_SETTING_VALUE_INT32(CONFIG_INPUT_IQS9151_CURSOR_SMOOTHING),
    ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,
    ZMK_CUSTOM_SETTING_PERMISSION_SECURE, ZMK_CUSTOM_SETTING_RANGE_INT32(1, 8));

ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_tap_dead_zone, IQS9151_SETTINGS_SUBSYSTEM_ID, IQS9151_SETTING_TAP_DEAD_ZONE_KEY,
    ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,
    ZMK_CUSTOM_SETTING_VALUE_INT32(CONFIG_INPUT_IQS9151_TAP_DEAD_ZONE),
    ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,
    ZMK_CUSTOM_SETTING_PERMISSION_SECURE, ZMK_CUSTOM_SETTING_RANGE_INT32(0, 200));

ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_cursor_distance_smoothing, IQS9151_SETTINGS_SUBSYSTEM_ID,
    IQS9151_SETTING_CURSOR_DISTANCE_SMOOTHING_KEY, ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,
    ZMK_CUSTOM_SETTING_VALUE_INT32(CONFIG_INPUT_IQS9151_CURSOR_DISTANCE_SMOOTHING),
    ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,
    ZMK_CUSTOM_SETTING_PERMISSION_SECURE, ZMK_CUSTOM_SETTING_RANGE_INT32(0, 512));

ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_ripple_period_x, IQS9151_SETTINGS_SUBSYSTEM_ID,
    IQS9151_SETTING_RIPPLE_PERIOD_X_X10_KEY, ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,
    ZMK_CUSTOM_SETTING_VALUE_INT32(CONFIG_INPUT_IQS9151_RIPPLE_PERIOD_X_X10),
    ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,
    ZMK_CUSTOM_SETTING_PERMISSION_SECURE, ZMK_CUSTOM_SETTING_RANGE_INT32(0, 1600));

ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_ripple_period_y, IQS9151_SETTINGS_SUBSYSTEM_ID,
    IQS9151_SETTING_RIPPLE_PERIOD_Y_X10_KEY, ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,
    ZMK_CUSTOM_SETTING_VALUE_INT32(CONFIG_INPUT_IQS9151_RIPPLE_PERIOD_Y_X10),
    ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,
    ZMK_CUSTOM_SETTING_PERMISSION_SECURE, ZMK_CUSTOM_SETTING_RANGE_INT32(0, 1600));

ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_ripple_auto, IQS9151_SETTINGS_SUBSYSTEM_ID, IQS9151_SETTING_RIPPLE_AUTO_KEY,
    ZMK_CUSTOM_SETTING_VALUE_TYPE_BOOL,
    ZMK_CUSTOM_SETTING_VALUE_BOOL(IS_ENABLED(CONFIG_INPUT_IQS9151_RIPPLE_AUTO)),
    ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,
    ZMK_CUSTOM_SETTING_PERMISSION_SECURE, ZMK_CUSTOM_SETTING_NO_CONSTRAINT);

ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_ripple_map, IQS9151_SETTINGS_SUBSYSTEM_ID, IQS9151_SETTING_RIPPLE_MAP_KEY,
    ZMK_CUSTOM_SETTING_VALUE_TYPE_BOOL,
    ZMK_CUSTOM_SETTING_VALUE_BOOL(IS_ENABLED(CONFIG_INPUT_IQS9151_RIPPLE_MAP)),
    ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,
    ZMK_CUSTOM_SETTING_PERMISSION_SECURE, ZMK_CUSTOM_SETTING_NO_CONSTRAINT);

/*
 * What the search concluded, written by the driver rather than the owner.
 * There is no read-only permission, so the app draws these as text rather
 * than a box; a write from outside is harmless, since the driver's own copy
 * is what it uses and the next conclusion overwrites this one.
 */
ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_ripple_found_x, IQS9151_SETTINGS_SUBSYSTEM_ID,
    IQS9151_SETTING_RIPPLE_FOUND_X_X10_KEY, ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,
    ZMK_CUSTOM_SETTING_VALUE_INT32(0), ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC,
    ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE, ZMK_CUSTOM_SETTING_PERMISSION_SECURE,
    ZMK_CUSTOM_SETTING_RANGE_INT32(0, 1600));

ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_ripple_found_y, IQS9151_SETTINGS_SUBSYSTEM_ID,
    IQS9151_SETTING_RIPPLE_FOUND_Y_X10_KEY, ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,
    ZMK_CUSTOM_SETTING_VALUE_INT32(0), ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC,
    ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE, ZMK_CUSTOM_SETTING_PERMISSION_SECURE,
    ZMK_CUSTOM_SETTING_RANGE_INT32(0, 1600));

/* In RAM only: it is a progress figure, rewritten at every lift, and a
 * flash write for each would be the stall the map's own save avoids. */
ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_ripple_map_learned_x, IQS9151_SETTINGS_SUBSYSTEM_ID,
    IQS9151_SETTING_RIPPLE_MAP_LEARNED_X_KEY, ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,
    ZMK_CUSTOM_SETTING_VALUE_INT32(0), ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC,
    ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE, ZMK_CUSTOM_SETTING_PERMISSION_SECURE,
    ZMK_CUSTOM_SETTING_RANGE_INT32(0, 256));

ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(
    iqs9151_ripple_map_learned_y, IQS9151_SETTINGS_SUBSYSTEM_ID,
    IQS9151_SETTING_RIPPLE_MAP_LEARNED_Y_KEY, ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,
    ZMK_CUSTOM_SETTING_VALUE_INT32(0), ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC,
    ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE, ZMK_CUSTOM_SETTING_PERMISSION_SECURE,
    ZMK_CUSTOM_SETTING_RANGE_INT32(0, 256));

void iqs9151_setting_map_learned(char axis, uint16_t bins) {
    const struct zmk_custom_setting *setting =
        (axis == 'y') ? &iqs9151_ripple_map_learned_y : &iqs9151_ripple_map_learned_x;
    (void)zmk_custom_setting_set_int32(setting, bins, ZMK_CUSTOM_SETTING_WRITE_MODE_MEMORY);
}

void iqs9151_setting_ripple_measured(char axis, uint16_t period_x10) {
    const struct zmk_custom_setting *setting =
        (axis == 'y') ? &iqs9151_ripple_found_y : &iqs9151_ripple_found_x;
    (void)zmk_custom_setting_set_int32(setting, period_x10, ZMK_CUSTOM_SETTING_WRITE_MODE_MEMORY);
}

void iqs9151_setting_ripple_found(char axis, uint16_t period_x10) {
    const struct zmk_custom_setting *setting =
        (axis == 'y') ? &iqs9151_ripple_found_y : &iqs9151_ripple_found_x;
    /* Persisted, so the app shows the same answer after a power cycle that
     * the driver is working from (it remembers the period in flash too). */
    const int ret = zmk_custom_setting_set_int32(setting, period_x10,
                                                 ZMK_CUSTOM_SETTING_WRITE_MODE_PERSIST);
    if (ret < 0) {
        LOG_WRN("Could not publish ripple %c period (%d)", axis, ret);
    }
}

/*
 * The device's own low-speed filtering. Six registers, six settings; the
 * ranges are the registers' widths. Pushed to the device as one block when any
 * of them changes -- see apply_filter.
 */
#define IQS9151_FILTER_SETTING(_name, _key, _default, _max)                                       \
    ZMK_CUSTOM_SETTING_DEFINE_WITH_CONSTRAINTS(                                                    \
        _name, IQS9151_SETTINGS_SUBSYSTEM_ID, _key, ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,           \
        ZMK_CUSTOM_SETTING_VALUE_INT32(_default), ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC,   \
        ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE, ZMK_CUSTOM_SETTING_PERMISSION_SECURE,             \
        ZMK_CUSTOM_SETTING_RANGE_INT32(0, _max))

IQS9151_FILTER_SETTING(iqs9151_filter_bottom_speed, IQS9151_SETTING_FILTER_BOTTOM_SPEED_KEY,
                       CONFIG_INPUT_IQS9151_DYNAMIC_FILTER_BOTTOM_SPEED, 2047);
IQS9151_FILTER_SETTING(iqs9151_filter_top_speed, IQS9151_SETTING_FILTER_TOP_SPEED_KEY,
                       CONFIG_INPUT_IQS9151_DYNAMIC_FILTER_TOP_SPEED, 2047);
IQS9151_FILTER_SETTING(iqs9151_filter_bottom_beta, IQS9151_SETTING_FILTER_BOTTOM_BETA_KEY,
                       CONFIG_INPUT_IQS9151_DYNAMIC_FILTER_BOTTOM_BETA, 255);
IQS9151_FILTER_SETTING(iqs9151_filter_static_beta, IQS9151_SETTING_FILTER_STATIC_BETA_KEY,
                       CONFIG_INPUT_IQS9151_STATIC_FILTER_BETA, 255);
IQS9151_FILTER_SETTING(iqs9151_stationary_threshold, IQS9151_SETTING_STATIONARY_THRESHOLD_KEY,
                       CONFIG_INPUT_IQS9151_STATIONARY_TOUCH_MOV_THRESHOLD, 255);
IQS9151_FILTER_SETTING(iqs9151_jitter_delta, IQS9151_SETTING_JITTER_DELTA_KEY,
                       CONFIG_INPUT_IQS9151_JITTER_FILTER_DELTA, 255);
/* The touch thresholds ride in the same block: the pad's sensitivity, and
 * how many electrodes the position is the centroid of. */
IQS9151_FILTER_SETTING(iqs9151_touch_set_threshold, IQS9151_SETTING_TOUCH_SET_THRESHOLD_KEY,
                       CONFIG_INPUT_IQS9151_TOUCH_SET_THRESHOLD, 255);
IQS9151_FILTER_SETTING(iqs9151_touch_clear_threshold, IQS9151_SETTING_TOUCH_CLEAR_THRESHOLD_KEY,
                       CONFIG_INPUT_IQS9151_TOUCH_CLEAR_THRESHOLD, 255);

/*
 * Fall back to the compiled-in value on any error, rather than propagating it.
 * The caller is a gesture decision with nowhere to report a failure to, and a
 * pad that behaves like its .conf says is a much better answer than a pad that
 * stops recognising anything because the settings subsystem had a bad moment.
 */
static bool read_bool(const char *key, bool fallback) {
    struct zmk_custom_setting_value value;
    int ret = zmk_custom_setting_read_by_key(IQS9151_SETTINGS_SUBSYSTEM_ID, key, &value);

    if (ret < 0) {
        LOG_DBG("Trackpad setting %s unreadable (%d), using the built-in default", key, ret);
        return fallback;
    }
    if (value.type != ZMK_CUSTOM_SETTING_VALUE_TYPE_BOOL) {
        LOG_WRN("Trackpad setting %s is not a boolean", key);
        return fallback;
    }

    return value.bool_value;
}

bool iqs9151_setting_one_hand_pinch(void) {
    return read_bool(IQS9151_SETTING_ONE_HAND_PINCH_KEY,
                     IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PINCH_ENABLE));
}

bool iqs9151_setting_pinch_invert(void) {
    return read_bool(IQS9151_SETTING_PINCH_INVERT_KEY,
                     IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PINCH_INVERT));
}

static int32_t read_int32(const char *key, int32_t fallback) {
    struct zmk_custom_setting_value value;
    int ret = zmk_custom_setting_read_by_key(IQS9151_SETTINGS_SUBSYSTEM_ID, key, &value);

    if (ret < 0) {
        LOG_DBG("Trackpad setting %s unreadable (%d), using the built-in default", key, ret);
        return fallback;
    }
    if (value.type != ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32) {
        LOG_WRN("Trackpad setting %s is not an integer", key);
        return fallback;
    }

    return value.int32_value;
}

/*
 * Hand the current pair to the driver, which writes it at the IC's next
 * communication window.
 *
 * Both axes go together every time, even when only one changed. The pair is
 * one statement about the pad's shape, and writing the register that did not
 * change costs a single I2C word inside a window that is already open.
 */
static void apply_cursor_gain(void) {
    const int32_t x = read_int32(IQS9151_SETTING_CURSOR_GAIN_X_KEY,
                                CONFIG_INPUT_IQS9151_CURSOR_GAIN_X_X10);
    const int32_t y = read_int32(IQS9151_SETTING_CURSOR_GAIN_Y_KEY,
                                CONFIG_INPUT_IQS9151_CURSOR_GAIN_Y_X10);

    int ret = iqs9151_set_cursor_gain((uint16_t)x, (uint16_t)y);
    if (ret < 0) {
        LOG_WRN("Refused cursor gain %d / %d (%d)", x, y, ret);
    }

    const int32_t smoothing = read_int32(IQS9151_SETTING_CURSOR_SMOOTHING_KEY,
                                        CONFIG_INPUT_IQS9151_CURSOR_SMOOTHING);
    (void)iqs9151_set_cursor_smoothing((uint16_t)smoothing);

    const int32_t dead_zone =
        read_int32(IQS9151_SETTING_TAP_DEAD_ZONE_KEY, CONFIG_INPUT_IQS9151_TAP_DEAD_ZONE);
    (void)iqs9151_set_tap_dead_zone((uint16_t)dead_zone);

    const int32_t distance = read_int32(IQS9151_SETTING_CURSOR_DISTANCE_SMOOTHING_KEY,
                                        CONFIG_INPUT_IQS9151_CURSOR_DISTANCE_SMOOTHING);
    (void)iqs9151_set_cursor_distance_smoothing((uint16_t)distance);

    const int32_t ripple_x = read_int32(IQS9151_SETTING_RIPPLE_PERIOD_X_X10_KEY,
                                        CONFIG_INPUT_IQS9151_RIPPLE_PERIOD_X_X10);
    const int32_t ripple_y = read_int32(IQS9151_SETTING_RIPPLE_PERIOD_Y_X10_KEY,
                                        CONFIG_INPUT_IQS9151_RIPPLE_PERIOD_Y_X10);
    ret = iqs9151_set_ripple_period((uint16_t)ripple_x, (uint16_t)ripple_y);
    if (ret < 0) {
        LOG_WRN("Refused ripple period %d / %d (%d)", ripple_x, ripple_y, ret);
    }
    (void)iqs9151_set_ripple_auto(read_bool(IQS9151_SETTING_RIPPLE_AUTO_KEY,
                                            IS_ENABLED(CONFIG_INPUT_IQS9151_RIPPLE_AUTO)));
    (void)iqs9151_set_ripple_map(read_bool(IQS9151_SETTING_RIPPLE_MAP_KEY,
                                           IS_ENABLED(CONFIG_INPUT_IQS9151_RIPPLE_MAP)));

    const int32_t interval = read_int32(IQS9151_SETTING_CURSOR_REPORT_INTERVAL_KEY,
                                        CONFIG_INPUT_IQS9151_CURSOR_REPORT_INTERVAL_MS);
    (void)iqs9151_set_cursor_report_interval((uint16_t)MAX(0, interval));

    const int32_t swipe_x = read_int32(IQS9151_SETTING_SWIPE3_THRESHOLD_X_KEY,
                                       CONFIG_INPUT_IQS9151_3F_SWIPE_THRESHOLD);
    const int32_t swipe_y = read_int32(IQS9151_SETTING_SWIPE3_THRESHOLD_Y_KEY,
                                       CONFIG_INPUT_IQS9151_3F_SWIPE_THRESHOLD_Y);
    ret = iqs9151_set_swipe3_threshold((uint16_t)swipe_x, (uint16_t)swipe_y);
    if (ret < 0) {
        LOG_WRN("Refused swipe threshold %d / %d (%d)", swipe_x, swipe_y, ret);
    }
}

static void apply_filter(void) {
    const struct iqs9151_filter_tune tune = {
        .bottom_speed = (uint16_t)read_int32(IQS9151_SETTING_FILTER_BOTTOM_SPEED_KEY,
                                             CONFIG_INPUT_IQS9151_DYNAMIC_FILTER_BOTTOM_SPEED),
        .top_speed = (uint16_t)read_int32(IQS9151_SETTING_FILTER_TOP_SPEED_KEY,
                                          CONFIG_INPUT_IQS9151_DYNAMIC_FILTER_TOP_SPEED),
        .bottom_beta = (uint8_t)read_int32(IQS9151_SETTING_FILTER_BOTTOM_BETA_KEY,
                                           CONFIG_INPUT_IQS9151_DYNAMIC_FILTER_BOTTOM_BETA),
        .static_beta = (uint8_t)read_int32(IQS9151_SETTING_FILTER_STATIC_BETA_KEY,
                                           CONFIG_INPUT_IQS9151_STATIC_FILTER_BETA),
        .stationary_threshold =
            (uint8_t)read_int32(IQS9151_SETTING_STATIONARY_THRESHOLD_KEY,
                                CONFIG_INPUT_IQS9151_STATIONARY_TOUCH_MOV_THRESHOLD),
        .jitter_delta = (uint8_t)read_int32(IQS9151_SETTING_JITTER_DELTA_KEY,
                                            CONFIG_INPUT_IQS9151_JITTER_FILTER_DELTA),
        .touch_set = (uint8_t)CLAMP(read_int32(IQS9151_SETTING_TOUCH_SET_THRESHOLD_KEY,
                                               CONFIG_INPUT_IQS9151_TOUCH_SET_THRESHOLD),
                                    1, 255),
        .touch_clear = (uint8_t)CLAMP(read_int32(IQS9151_SETTING_TOUCH_CLEAR_THRESHOLD_KEY,
                                                 CONFIG_INPUT_IQS9151_TOUCH_CLEAR_THRESHOLD),
                                      1, 255),
    };

    (void)iqs9151_request_filter(&tune);
}

static void apply_resolution(void) {
    const int32_t x = read_int32(IQS9151_SETTING_RESOLUTION_X_KEY,
                                CONFIG_INPUT_IQS9151_RESOLUTION_X);
    const int32_t y = read_int32(IQS9151_SETTING_RESOLUTION_Y_KEY,
                                CONFIG_INPUT_IQS9151_RESOLUTION_Y);

    int ret = iqs9151_request_resolution((uint16_t)x, (uint16_t)y);
    if (ret < 0) {
        LOG_WRN("Refused trackpad resolution %d x %d (%d)", x, y, ret);
    }
}

/*
 * Two events, one listener.
 *
 * zmk_custom_settings_initialized is the only safe moment to read a persisted
 * value at boot: a plain SYS_INIT can run before settings_load has filled the
 * registry, and would push the compiled-in default over the owner's choice.
 * zmk_custom_setting_changed then covers every later write, including the ones
 * relayed from the other half.
 *
 * Only the keys that the driver has to be told about are acted on: the
 * resolutions and the filter block go to the IC's registers, the gains and
 * smoothing to the driver's own scaling. Every write in this subsystem raises
 * the same event, and the two pinch switches are sampled at touch-down rather
 * than pushed anywhere, so re-writing registers when one of them is flipped
 * would be work for nothing.
 */
static int iqs9151_settings_event_listener(const zmk_event_t *eh) {
    if (as_zmk_custom_settings_initialized(eh) != NULL) {
        apply_resolution();
        apply_cursor_gain();
        apply_filter();
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_custom_setting_changed *changed = as_zmk_custom_setting_changed(eh);
    if (changed == NULL || changed->setting == NULL ||
        changed->setting->custom_subsystem_id == NULL || changed->setting->key == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (strcmp(changed->setting->custom_subsystem_id, IQS9151_SETTINGS_SUBSYSTEM_ID) != 0) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (strcmp(changed->setting->key, IQS9151_SETTING_RESOLUTION_X_KEY) == 0 ||
        strcmp(changed->setting->key, IQS9151_SETTING_RESOLUTION_Y_KEY) == 0) {
        apply_resolution();
    } else if (strcmp(changed->setting->key, IQS9151_SETTING_CURSOR_GAIN_X_KEY) == 0 ||
               strcmp(changed->setting->key, IQS9151_SETTING_CURSOR_GAIN_Y_KEY) == 0 ||
               strcmp(changed->setting->key, IQS9151_SETTING_CURSOR_SMOOTHING_KEY) == 0 ||
               strcmp(changed->setting->key, IQS9151_SETTING_TAP_DEAD_ZONE_KEY) == 0 ||
               strcmp(changed->setting->key, IQS9151_SETTING_CURSOR_DISTANCE_SMOOTHING_KEY) == 0 ||
               strcmp(changed->setting->key, IQS9151_SETTING_RIPPLE_PERIOD_X_X10_KEY) == 0 ||
               strcmp(changed->setting->key, IQS9151_SETTING_RIPPLE_PERIOD_Y_X10_KEY) == 0 ||
               strcmp(changed->setting->key, IQS9151_SETTING_RIPPLE_AUTO_KEY) == 0 ||
               strcmp(changed->setting->key, IQS9151_SETTING_RIPPLE_MAP_KEY) == 0 ||
               strcmp(changed->setting->key, IQS9151_SETTING_CURSOR_REPORT_INTERVAL_KEY) == 0 ||
               strcmp(changed->setting->key, IQS9151_SETTING_SWIPE3_THRESHOLD_X_KEY) == 0 ||
               strcmp(changed->setting->key, IQS9151_SETTING_SWIPE3_THRESHOLD_Y_KEY) == 0) {
        apply_cursor_gain();
    } else if (strncmp(changed->setting->key, "filter_", 7) == 0 ||
               strcmp(changed->setting->key, IQS9151_SETTING_STATIONARY_THRESHOLD_KEY) == 0 ||
               strcmp(changed->setting->key, IQS9151_SETTING_JITTER_DELTA_KEY) == 0 ||
               strcmp(changed->setting->key, IQS9151_SETTING_TOUCH_SET_THRESHOLD_KEY) == 0 ||
               strcmp(changed->setting->key, IQS9151_SETTING_TOUCH_CLEAR_THRESHOLD_KEY) == 0) {
        apply_filter();
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(iqs9151_resolution_apply, iqs9151_settings_event_listener);
ZMK_SUBSCRIPTION(iqs9151_resolution_apply, zmk_custom_settings_initialized);
ZMK_SUBSCRIPTION(iqs9151_resolution_apply, zmk_custom_setting_changed);
