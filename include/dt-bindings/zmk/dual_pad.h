/*
 * param2 of a &dual_pad reference. Bits 0..1 flip the configured directions
 * for that reference; bits 8..11 pick which entry of the node's
 * scroll-processors list this chain runs its scroll through, so the two-hand
 * scroll follows the same runtime invert and speed as two fingers on one pad.
 */
#pragma once

#define DUAL_PAD_INVERT_SCROLL (1 << 0)
#define DUAL_PAD_INVERT_ZOOM (1 << 1)

#define DUAL_PAD_SCROLL_PROC(n) (((n) & 0xF) << 8)
