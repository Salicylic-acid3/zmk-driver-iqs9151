/*
 * Copyright (c) 2026 Salicylic_acid3
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * The two trackpad settings the owner can change from the app.
 *
 * The Kconfig switches are still here; they are the defaults. A keyboard that
 * has never been touched behaves exactly as its .conf says, and the app's copy
 * of the setting only starts to matter once someone moves it.
 *
 * Reading. zmk_custom_setting_read_by_key walks the registry and compares two
 * strings, which is cheap but not free, and the arbitration that uses these
 * runs on every frame while two fingers are down and undecided. So the driver
 * samples them once per gesture, at touch-down, rather than calling in here
 * from the hot path -- see iqs9151_two_finger_update.
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

#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <cormoran/zmk/custom_settings.h>

#include "iqs9151_settings.h"

LOG_MODULE_DECLARE(iqs9151, CONFIG_INPUT_IQS9151_LOG_LEVEL);

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
