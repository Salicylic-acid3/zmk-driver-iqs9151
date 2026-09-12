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
