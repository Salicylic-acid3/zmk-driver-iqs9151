/*
 * Copyright (c) 2026 Salicylic_acid3
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Changing the pad's coordinate scale while it is running.
 *
 * The IC spreads a fixed number of counts across each axis of the pad, so
 * counts per millimetre is resolution divided by that side's length, and the
 * two resolutions have to be set in proportion to the two sides or the long
 * one comes out slower. That much is arithmetic. What is not arithmetic is the
 * length of the sides: the drawing gives the copper, not the electrode array,
 * and the difference between them is exactly the kind of thing that only shows
 * up as "the pointer feels reluctant going up".
 *
 * Finding the right pair by rebuilding, reflashing and re-forming an opinion
 * takes ten minutes a step. Finding it by turning a knob and moving a finger
 * takes five seconds, so the resolutions are runtime settings and this is the
 * driver's side of that: hand it a pair and it writes them to every IQS9151 on
 * this half.
 *
 * The write is deferred rather than immediate. The IC only listens during the
 * communication window it opens with its RDY line, and the settings subsystem
 * calls from whatever thread happened to service the RPC. So this records the
 * request and the driver performs it inside the window it is already using to
 * read the next frame. A pad that is not being touched still gets there: the
 * IC reports on its own cadence, not only under a finger.
 *
 * Deliberately free of ZMK headers. The driver compiles into Zephyr's input
 * library, which has none of them.
 */

#pragma once

#include <stdint.h>

/**
 * Ask every IQS9151 on this half to adopt a new coordinate scale.
 *
 * Applied at the next communication window, not on return. Values are the
 * counts spread across the sensor's X and Y axes; both must be non-zero, and
 * the device configuration treats them as 12-bit (1..4095).
 *
 * @retval 0 on success, -EINVAL if either value is zero.
 */
int iqs9151_request_resolution(uint16_t x_resolution, uint16_t y_resolution);

/**
 * Set how far the pointer travels per count, per axis, in tenths.
 *
 * 10 leaves an axis alone; 16 makes it 1.6x. Applied to the relative movement
 * the device reports, with the tenths carried between reports so a fractional
 * gain is real travel rather than rounding.
 *
 * This is the lever for pointer speed. The resolution pair above is the lever
 * for the gesture arbitration, which compares absolute coordinates. They are
 * genuinely separate on this device, which took some finding.
 *
 * @retval 0 on success, -EINVAL if either gain is zero.
 */
int iqs9151_set_cursor_gain(uint16_t x_gain_x10, uint16_t y_gain_x10);

/**
 * Spread each report's pointer movement across this many reports.
 *
 * 1 is immediate. Above that, the accumulator is drained a fraction at a time,
 * which is what keeps an amplified axis from stepping: a gain much above 1
 * turns the device's 1, 0, 1, 0 into 2, 0, 3, 0, and this turns that back into
 * 1, 1, 1, 1. Costs exactly that many reports of lag and no more.
 *
 * @retval 0 always; values below 1 are treated as 1.
 */
int iqs9151_set_cursor_smoothing(uint16_t reports);

/**
 * The device's own low-speed filtering, as one block.
 *
 * Every field maps to a register in the 0x11EA..0x11F4 run of the trackpad
 * settings, written together from inside the communication window like the
 * resolutions. They decide what the device does with a finger that is barely
 * moving, which on a coarse axis is where "stops on an electrode, then jumps
 * to the next" comes from -- and that is not something gain or smoothing
 * downstream can put right, because the sample was never taken.
 *
 *   bottom_speed / top_speed   the speed band the dynamic filter ramps over
 *   bottom_beta                filter strength below bottom_speed (higher = more)
 *   static_beta                filter strength when the finger is still
 *   stationary_threshold       movement below this is reported as none at all
 *   jitter_delta               dead band on the raw coordinate
 */
struct iqs9151_filter_tune {
    uint16_t bottom_speed;
    uint16_t top_speed;
    uint8_t bottom_beta;
    uint8_t static_beta;
    uint8_t stationary_threshold;
    uint8_t jitter_delta;
};

/**
 * Ask every IQS9151 on this half to adopt a new filter block. Applied at the
 * next communication window, not on return.
 *
 * @retval 0 always.
 */
int iqs9151_request_filter(const struct iqs9151_filter_tune *tune);
