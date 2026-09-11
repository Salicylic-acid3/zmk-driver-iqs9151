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
 * The app has to write both -- see the note in iqs9151_settings.c.
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
