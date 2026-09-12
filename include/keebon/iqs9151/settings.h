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

#if IS_ENABLED(CONFIG_INPUT_IQS9151_RUNTIME_SETTINGS)

bool iqs9151_setting_one_hand_pinch(void);
bool iqs9151_setting_pinch_invert(void);

#else

static inline bool iqs9151_setting_one_hand_pinch(void) {
    return IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PINCH_ENABLE);
}

static inline bool iqs9151_setting_pinch_invert(void) {
    return IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PINCH_INVERT);
}

#endif /* CONFIG_INPUT_IQS9151_RUNTIME_SETTINGS */
