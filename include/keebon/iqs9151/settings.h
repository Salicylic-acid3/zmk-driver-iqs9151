/*
 * Copyright (c) 2026 Salicylic_acid3
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * The handful of trackpad decisions that are the user's rather than the
 * board's, read at runtime instead of compiled in.
 *
 * Which gestures a pad should offer is not a property of the hardware. Whether
 * a two-finger pinch on one pad is useful depends on how big the pad is and on
 * what else the keyboard already does; which way it should zoom depends on the
 * host's convention. Both were build switches, which meant a rebuild, a flash
 * and a keymap reset to answer a question the owner could answer by trying it
 * for ten seconds.
 *
 * So they are custom settings, and the Kconfig switches become their defaults.
 * A build without CONFIG_INPUT_IQS9151_RUNTIME_SETTINGS behaves exactly as
 * before: the inline fallbacks below return the compiled-in values and no
 * settings machinery is linked in at all, which is what keeps this driver
 * usable outside a ZMK build.
 *
 * On a split, each half registers its own copy and each half reads its own.
 * The app has to write both -- see the note in settings/iqs9151_settings.c.
 *
 * This header deliberately includes nothing but Zephyr, and the code behind it
 * lives in settings/ rather than beside the driver, compiled into `app`. The
 * driver amends Zephyr's own input library, which has no ZMK includes; and a
 * library of our own is not enough either, because <zmk/studio/custom.h> pulls
 * in a header nanopb generates into the build tree, which only `app` can see.
 * Both were learned the expensive way, one CI run each.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

/** Must match the app's TRACKPAD_SUBSYSTEM_ID. */
#define IQS9151_SETTINGS_SUBSYSTEM_ID "keebon__trackpad"

/** Two fingers on one pad pinch to zoom. */
#define IQS9151_SETTING_ONE_HAND_PINCH_KEY "one_hand_pinch"

/** That pinch turns the wheel the other way. */
#define IQS9151_SETTING_PINCH_INVERT_KEY "pinch_invert"

/*
 * Counts spread across each axis of the pad.
 *
 * Runtime rather than build-time because what matters is the ratio between the
 * two, the ratio has to match the ratio of the pad's sides, and the sides are
 * not reliably knowable from the drawing -- the copper outline is not the
 * electrode array, and the difference between them is easily a factor of two.
 * Getting it wrong does not present as "the resolution is wrong". It presents
 * as the pointer being reluctant in one direction, and as every gesture leaning
 * the other way, because every axis decision in this driver compares counts.
 *
 * An evening of turning a knob settles what a week of rebuild-flash-reform-an-
 * opinion would still be arguing about.
 */
#define IQS9151_SETTING_RESOLUTION_X_KEY "resolution_x"
#define IQS9151_SETTING_RESOLUTION_Y_KEY "resolution_y"

/*
 * How far the pointer travels per count, per axis, in tenths.
 *
 * Separate from the resolution pair above because on this device they are
 * genuinely separate: the resolutions set the absolute coordinate range the
 * gestures compare, and have no effect whatever on the relative movement the
 * cursor is built from. That cost three flashes to establish, so it is worth
 * writing down twice.
 */
#define IQS9151_SETTING_CURSOR_GAIN_X_KEY "cursor_gain_x"
#define IQS9151_SETTING_CURSOR_GAIN_Y_KEY "cursor_gain_y"

/* Reports to spread each movement across; 1 is immediate. See
 * INPUT_IQS9151_CURSOR_SMOOTHING -- it is the companion to the gains, because
 * amplifying a quantised axis is what makes it step. */
#define IQS9151_SETTING_CURSOR_SMOOTHING_KEY "cursor_smoothing"

/* Counts of finger travel to average the pointer over; 0 is off. A window in
 * distance rather than reports, so it flattens a ripple fixed in millimetres at
 * any speed. See iqs9151_set_cursor_distance_smoothing in control.h. */
#define IQS9151_SETTING_CURSOR_DISTANCE_SMOOTHING_KEY "cursor_distance_smoothing"

/* Period of the positional ripple per axis, in tenths of a count; 0 is off.
 * The self-calibrating, lag-free alternative to the distance window above.
 * Tenths because the equaliser needs the period to within a percent or two,
 * and the pad's real period is not a whole number. See iqs9151_set_ripple_period
 * in control.h. (The earlier whole-count keys "ripple_period_x/y" are retired
 * rather than reinterpreted, so a stored 76 cannot silently become 7.6.) */
#define IQS9151_SETTING_RIPPLE_PERIOD_X_X10_KEY "ripple_period_x_x10"
#define IQS9151_SETTING_RIPPLE_PERIOD_Y_X10_KEY "ripple_period_y_x10"

/* Let the driver find the periods itself; the values above are then only the
 * starting point. See iqs9151_set_ripple_auto in control.h. */
#define IQS9151_SETTING_RIPPLE_AUTO_KEY "ripple_auto"

/* Correct with a map learned over the pad's positions rather than a wave at a
 * period; the period keys and the search are idle while this is on. See
 * iqs9151_set_ripple_map in control.h. */
#define IQS9151_SETTING_RIPPLE_MAP_KEY "ripple_map"

/* What the search found, per axis, in tenths of a count; 0 is nothing yet.
 * Written by the driver, for the app to show -- the one way to tell, without
 * a debug build, whether a pad has locked on to its wave or is still looking.
 * Writing it from the app changes nothing: the driver keeps its own copy and
 * overwrites this one the next time the search concludes. */
#define IQS9151_SETTING_RIPPLE_FOUND_X_X10_KEY "ripple_found_x_x10"
#define IQS9151_SETTING_RIPPLE_FOUND_Y_X10_KEY "ripple_found_y_x10"

/* Report pointer movement at most this often, in ms; 0 is every frame. For a
 * half whose pointer crosses a BLE link. See iqs9151_set_cursor_report_interval
 * in control.h. */
#define IQS9151_SETTING_CURSOR_REPORT_INTERVAL_KEY "cursor_report_interval_ms"

/* Three-finger swipe thresholds per sensor axis, in counts. */
#define IQS9151_SETTING_SWIPE3_THRESHOLD_X_KEY "swipe3_threshold_x"
#define IQS9151_SETTING_SWIPE3_THRESHOLD_Y_KEY "swipe3_threshold_y"

/*
 * The device's own low-speed filter block, one key per register. See
 * struct iqs9151_filter_tune in control.h for what each does. Exposed because
 * "stops on each electrode, then jumps" on a coarse axis lives here, upstream
 * of anything the driver can do about it afterwards.
 */
#define IQS9151_SETTING_FILTER_BOTTOM_SPEED_KEY "filter_bottom_speed"
#define IQS9151_SETTING_FILTER_TOP_SPEED_KEY "filter_top_speed"
#define IQS9151_SETTING_FILTER_BOTTOM_BETA_KEY "filter_bottom_beta"
#define IQS9151_SETTING_FILTER_STATIC_BETA_KEY "filter_static_beta"
#define IQS9151_SETTING_STATIONARY_THRESHOLD_KEY "stationary_threshold"
#define IQS9151_SETTING_JITTER_DELTA_KEY "jitter_delta"

#if IS_ENABLED(CONFIG_INPUT_IQS9151_RUNTIME_SETTINGS)

bool iqs9151_setting_one_hand_pinch(void);
bool iqs9151_setting_pinch_invert(void);

/* The driver reporting what its period search concluded ('x' or 'y'; 0 for
 * nothing, or forgotten). Published as the ripple_found_* settings above. */
void iqs9151_setting_ripple_found(char axis, uint16_t period_x10);

#else

static inline bool iqs9151_setting_one_hand_pinch(void) {
    return IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PINCH_ENABLE);
}

static inline bool iqs9151_setting_pinch_invert(void) {
    return IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PINCH_INVERT);
}

static inline void iqs9151_setting_ripple_found(char axis, uint16_t period_x10) {
    ARG_UNUSED(axis);
    ARG_UNUSED(period_x10);
}

#endif /* CONFIG_INPUT_IQS9151_RUNTIME_SETTINGS */
