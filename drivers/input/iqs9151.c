/*
 * File:   IQS9151.c
 * Author: ShiniNet
 */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>
#endif
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <keebon/iqs9151/control.h>
#include <keebon/iqs9151/settings.h>

#include "iqs9151_init.h"
#include "iqs9151_regs.h"
#include "iqs9151_test.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

LOG_MODULE_REGISTER(iqs9151, CONFIG_INPUT_IQS9151_LOG_LEVEL);

#define DT_DRV_COMPAT azoteq_iqs9151

#define IQS9151_I2C_CHUNK_SIZE 30
#define IQS9151_RXTX_MAP_SIZE 46
#define IQS9151_RXTX_OPEN_INDEX (IQS9151_RXTX_MAP_SIZE - 1)
#define IQS9151_RXTX_MAP_DATA_SIZE (IQS9151_RXTX_MAP_SIZE - 1)
#define IQS9151_ADDR_TRACKPAD_RX_CHANNEL 0x11E3
#define IQS9151_ADDR_TRACKPAD_TX_CHANNEL 0x11E4
#define IQS9151_RSTD_DELAY_MS 100
#define IQS9151_ATI_TIMEOUT_MS 1000
#define IQS9151_ATI_POLL_INTERVAL_MS 10
#define IQS9151_ATI_AUTO_TUNE_START_DIV 9
#define IQS9151_ATI_AUTO_TUNE_MIN_DIV 6
#define IQS9151_ATI_AUTO_TUNE_MIN_COUNT 600
#define IQS9151_ATI_RESULT_MASK 0x03FF
#define INERTIA_FP_SHIFT 8
#define EMA_FP_SHIFT INERTIA_FP_SHIFT
#define EMA_ALPHA_DEN (1 << EMA_FP_SHIFT)
#define IQS9151_FRAME_READ_SIZE 28
#define IQS9151_INERTIA_MOTION_HISTORY_SIZE 12

#define SCROLL_INERTIA_INTERVAL_MS 10
#define SCROLL_INERTIA_MAX_DURATION_MS 3000
#define SCROLL_INERTIA_DECAY_NUM CONFIG_INPUT_IQS9151_SCROLL_INERTIA_DECAY
#define SCROLL_INERTIA_DECAY_DEN 1000
#define SCROLL_INERTIA_START_THRESHOLD 1
#define SCROLL_INERTIA_MIN_VELOCITY 1
#define SCROLL_EMA_ALPHA 10
#define SCROLL_INERTIA_RECENT_WINDOW_MS CONFIG_INPUT_IQS9151_SCROLL_INERTIA_RECENT_WINDOW_MS
#define SCROLL_INERTIA_STALE_GAP_MS CONFIG_INPUT_IQS9151_SCROLL_INERTIA_STALE_GAP_MS
#define SCROLL_INERTIA_MIN_SAMPLES CONFIG_INPUT_IQS9151_SCROLL_INERTIA_MIN_SAMPLES
#define SCROLL_INERTIA_MIN_AVG_SPEED CONFIG_INPUT_IQS9151_SCROLL_INERTIA_MIN_AVG_SPEED

#define CURSOR_INERTIA_INTERVAL_MS 10
#define CURSOR_INERTIA_MAX_DURATION_MS 3000
#define CURSOR_INERTIA_DECAY_NUM CONFIG_INPUT_IQS9151_CURSOR_INERTIA_DECAY
#define CURSOR_INERTIA_DECAY_DEN 1000
#define CURSOR_INERTIA_START_THRESHOLD 2
#define CURSOR_INERTIA_MIN_VELOCITY 2
#define CURSOR_EMA_ALPHA 30
#define CURSOR_INERTIA_RECENT_WINDOW_MS CONFIG_INPUT_IQS9151_CURSOR_INERTIA_RECENT_WINDOW_MS
#define CURSOR_INERTIA_STALE_GAP_MS CONFIG_INPUT_IQS9151_CURSOR_INERTIA_STALE_GAP_MS
#define CURSOR_INERTIA_MIN_SAMPLES CONFIG_INPUT_IQS9151_CURSOR_INERTIA_MIN_SAMPLES
#define CURSOR_INERTIA_MIN_AVG_SPEED CONFIG_INPUT_IQS9151_CURSOR_INERTIA_MIN_AVG_SPEED
#define ONE_FINGER_TAP_MAX_MS CONFIG_INPUT_IQS9151_1F_TAP_MAX_MS
#define TWO_FINGER_TAP_MAX_MS CONFIG_INPUT_IQS9151_2F_TAP_MAX_MS
#define IQS9151_TAP_REENTRY_WINDOW_MS 30
#define ONE_FINGER_TAPDRAG_GAP_MAX_MS CONFIG_INPUT_IQS9151_1F_TAPDRAG_GAP_MAX_MS
#define ONE_FINGER_CLICK_HOLD_MAX_MS ONE_FINGER_TAPDRAG_GAP_MAX_MS
#define TWO_FINGER_TAPDRAG_GAP_MAX_MS CONFIG_INPUT_IQS9151_2F_TAPDRAG_GAP_MAX_MS
#define TWO_FINGER_CLICK_HOLD_MAX_MS TWO_FINGER_TAPDRAG_GAP_MAX_MS
#define THREE_FINGER_TAPDRAG_GAP_MAX_MS CONFIG_INPUT_IQS9151_3F_TAPDRAG_GAP_MAX_MS
#define THREE_FINGER_CLICK_HOLD_MAX_MS THREE_FINGER_TAPDRAG_GAP_MAX_MS
#define TWO_FINGER_RELEASE_PENDING_MAX_MS 150
#define THREE_FINGER_RELEASE_PENDING_MAX_MS 150
#define TWO_FINGER_ONE_LEAD_MAX_MS 120
#define THREE_FINGER_ONE_LEAD_MAX_MS 120
#define THREE_FINGER_TWO_LEAD_MAX_MS 120
#define IQS9151_FINGER_HISTORY_SIZE 5
/*
 * Per axis, because a pad is rarely square and three fingers are not a point.
 *
 * Three fingers line up across the short side, which leaves almost no room to
 * travel that way before one of them runs out of pad -- while the long side has
 * room to spare. One threshold for both means the cramped direction is the hard
 * one, however comfortable the other feels. _Y defaults to _X, so a board that
 * has nothing to say about this keeps the single number it always had.
 */
#define THREE_FINGER_SWIPE_THRESHOLD_X CONFIG_INPUT_IQS9151_3F_SWIPE_THRESHOLD
#define THREE_FINGER_SWIPE_THRESHOLD_Y CONFIG_INPUT_IQS9151_3F_SWIPE_THRESHOLD_Y
/*
 * How long a three-finger gesture survives the device reporting two.
 *
 * Three fingers on a pad 53 mm across are not three clean touches to the
 * device: as they move, two of them merge for a frame or two, or one lifts a
 * hair, and the count reads 2 -- and the swipe accumulator used to reset on
 * every such frame, so on a short pad the threshold was rarely reached at
 * all. A gesture that was three fingers a moment ago stays a three-finger
 * gesture for this long at fewer.
 */
#define THREE_FINGER_FLICKER_GRACE_MS 250
/* A first-finger position that moves further than this between two frames
 * (5 ms) is the device renumbering its fingers, not a finger moving: at
 * ~23 counts/mm that would be over a metre a second. */
#define THREE_FINGER_RENUMBER_JUMP 120
#define THREE_FINGER_TAP_MAX_MS CONFIG_INPUT_IQS9151_3F_TAP_MAX_MS
#define THREE_FINGER_TAP_MOVE CONFIG_INPUT_IQS9151_3F_TAP_MOVE
#define ONE_FINGER_TAP_MOVE CONFIG_INPUT_IQS9151_1F_TAP_MOVE
#define TWO_FINGER_TAP_MOVE CONFIG_INPUT_IQS9151_2F_TAP_MOVE
#define TWO_FINGER_SCROLL_START_MOVE CONFIG_INPUT_IQS9151_2F_SCROLL_START_MOVE
#define TWO_FINGER_PINCH_START_DISTANCE CONFIG_INPUT_IQS9151_2F_PINCH_START_DISTANCE
/*
 * Pinch reports a vertical wheel, because that is what a host turns into zoom
 * when a modifier is held. A board whose pad is mounted rotated corrects the
 * orientation with zip_scroll_transform INPUT_TRANSFORM_XY_SWAP, which swaps
 * REL_WHEEL and REL_HWHEEL for every event on the listener - including this
 * one, so the zoom ends up on the horizontal wheel and does nothing. Emitting
 * pinch on the horizontal wheel makes that swap land it back on the vertical
 * one at the host.
 */
#if IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PINCH_WHEEL_HORIZONTAL)
#define IQS9151_PINCH_WHEEL_CODE INPUT_REL_HWHEEL
#else
#define IQS9151_PINCH_WHEEL_CODE INPUT_REL_WHEEL
#endif

#define TWO_FINGER_PINCH_WHEEL_DIV 12
#define TWO_FINGER_PINCH_WHEEL_GAIN_X10 CONFIG_INPUT_IQS9151_2F_PINCH_WHEEL_GAIN_X10
#define TWO_FINGER_PINCH_WHEEL_GAIN_DEN 10

/*
 * Swipes are classified here, on the sensor's own coordinates, before any of
 * the listener's input processors run. That is deliberate -- a swipe is one
 * decision about a whole gesture, not a stream of deltas to be transformed --
 * but it means the listener's XY_SWAP/X_INVERT/Y_INVERT chain, which is what
 * normally squares a rotated pad with the screen, never touches it. A pad
 * mounted the other way round therefore moves the cursor correctly and swipes
 * backwards.
 *
 * These two flip the axes for swipe classification only, two-finger and
 * three-finger alike. Both set is the 180-degree case: the mirror-image half of
 * a split, whose sensor is the same part fitted upside down.
 */
#if IS_ENABLED(CONFIG_INPUT_IQS9151_SWIPE_INVERT_X)
#define IQS9151_SWIPE_SIGN_X (-1)
#else
#define IQS9151_SWIPE_SIGN_X (1)
#endif
#if IS_ENABLED(CONFIG_INPUT_IQS9151_SWIPE_INVERT_Y)
#define IQS9151_SWIPE_SIGN_Y (-1)
#else
#define IQS9151_SWIPE_SIGN_Y (1)
#endif

/*
 * How decisively the fingers must be spreading, rather than travelling
 * together, before two fingers mean pinch instead of scroll.
 *
 * The two measurements are taken from the same pair of fingers: the centroid's
 * travel and the change in the distance between them. A careful scroll moves
 * the centroid and leaves the distance alone, so either test alone would do.
 * A hurried one does not: fingers splay a little as they go, the distance
 * creeps up, and requiring only that it creep up *more* than the centroid
 * moved lets an untidy scroll land on pinch. Tightening the threshold instead
 * would make a deliberate pinch need a bigger spread; this leaves the spread
 * alone and asks that it be the dominant thing happening.
 *
 * Tenths, so 10 is "merely larger" and is the behaviour this replaces.
 */
#define TWO_FINGER_PINCH_DOMINANCE_X10 CONFIG_INPUT_IQS9151_2F_PINCH_DOMINANCE_X10

/*
 * Two-finger horizontal swipe: a discrete gesture, not a stream.
 *
 * "Horizontal" is the sensor's Y axis, because every listener on a board that
 * needs this starts with XY_SWAP; see the note above about why the driver has
 * to reason in its own coordinates.
 *
 * Enabling this costs two-finger horizontal *scroll*, and the trade is not
 * hidden: a decisively sideways two-finger movement stops being able to start a
 * scroll, because the gesture has to be allowed to finish before it can be
 * recognised. Vertical scroll is untouched -- which is the point, since the
 * pads this is for are small enough that a sideways flick is a deliberate act
 * and a sideways scroll almost never is.
 */
#define TWO_FINGER_SWIPE_THRESHOLD CONFIG_INPUT_IQS9151_2F_SWIPE_THRESHOLD
#define TWO_FINGER_SWIPE_DOMINANCE_X10 CONFIG_INPUT_IQS9151_2F_SWIPE_DOMINANCE_X10

/*
 * Keeping a scroll on one axis once it has picked one.
 *
 * A trackpad in a keyboard is not always approached square-on. Reaching for it
 * from the home row puts the hand at an angle, and then "straight up" is a
 * diagonal as far as the pad is concerned -- so a scroll the user means as
 * vertical arrives as vertical-plus-a-bit-sideways, and the page drifts.
 *
 * So the axis is chosen once, when the scroll starts, and held for the rest of
 * the gesture: below this ratio the movement counts as diagonal and both axes
 * pass through as before. In tenths, and generous on purpose -- 12 is "the
 * dominant axis is 1.2x the other", which is everything within about 40 degrees
 * of straight. 0 turns the whole thing off.
 *
 * Note this compares the two axes' *counts*, which only means an angle if both
 * axes report the same counts per millimetre. On a pad whose sides differ that
 * is a resolution question, not a gesture one; see the note on
 * INPUT_IQS9151_RESOLUTION_X.
 */
#define TWO_FINGER_SCROLL_AXIS_LOCK_X10 CONFIG_INPUT_IQS9151_2F_SCROLL_AXIS_LOCK_X10

/*
 * The two codes a two-finger horizontal swipe reports. Deliberately not named
 * for a direction: which sign of the sensor's Y axis points which way on screen
 * depends on how the pad is mounted, and the keymap decides what either one
 * does anyway. BTN_0..BTN_7 are taken by the other gestures and BTN_8/BTN_9 by
 * the touch state, so these are the next two that collide with nothing.
 */
#define IQS9151_2F_SWIPE_CODE_A INPUT_BTN_SIDE
#define IQS9151_2F_SWIPE_CODE_B INPUT_BTN_EXTRA

struct iqs9151_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec irq_gpio;
};
struct iqs9151_frame {
    int16_t rel_x;
    int16_t rel_y;
    uint16_t info_flags;
    uint16_t trackpad_flags;
    uint8_t finger_count;
    uint16_t finger1_x;
    uint16_t finger1_y;
    uint16_t finger2_x;
    uint16_t finger2_y;
};
enum iqs9151_two_finger_mode {
    IQS9151_2F_MODE_NONE = 0,
    IQS9151_2F_MODE_SCROLL,
    IQS9151_2F_MODE_PINCH,
    /*
     * Entered once, when the swipe is recognised, and held until the fingers
     * lift. The key has already been sent by then; the mode exists so the rest
     * of the gesture cannot also become a scroll on the way out.
     */
    IQS9151_2F_MODE_SWIPE,
};
struct iqs9151_one_finger_state {
    bool active;
    bool hold_sent;
    bool tap_candidate;
    bool hold_candidate;
    bool tapdrag_second_touch;
    int64_t down_ms;
    int32_t dx;
    int32_t dy;
    uint16_t last_x;
    uint16_t last_y;
};
struct iqs9151_two_finger_state {
    bool active;
    bool hold_sent;
    bool tap_candidate;
    bool hold_candidate;
    bool tapdrag_second_touch;
    bool release_pending;
    int64_t down_ms;
    int64_t release_pending_ms;
    int32_t centroid_dx;
    int32_t centroid_dy;
    int32_t distance_delta;
    int32_t centroid_last_x;
    int32_t centroid_last_y;
    int32_t distance_last;
    int32_t pinch_wheel_remainder;
    /*
     * Sampled once, at touch-down, from the runtime settings. Reading a
     * setting means walking the registry and comparing strings, and the
     * arbitration that uses these runs on every frame while two fingers are
     * down and undecided -- but a gesture that changed its mind about the
     * rules halfway through would be worse than stale by one gesture anyway.
     */
    bool pinch_enabled;
    bool pinch_invert;
    /*
     * Which axis this scroll settled on, decided once when it started. 0 is
     * "diagonal, let both through"; otherwise the other axis is dropped for the
     * rest of the gesture, so a scroll meant as vertical stays vertical even
     * when the hand is at an angle to the pad.
     */
    bool scroll_lock_x;
    bool scroll_lock_y;
    enum iqs9151_two_finger_mode mode;
};
struct iqs9151_two_finger_result {
    bool scroll_active;
    bool scroll_started;
    bool scroll_ended;
    bool pinch_active;
    bool pinch_started;
    bool pinch_ended;
    int16_t scroll_x;
    int16_t scroll_y;
    int16_t pinch_wheel;
    /* 0 when no swipe was recognised this frame, otherwise the code to tap. */
    uint16_t swipe_code;
};
struct iqs9151_inertia_params {
    uint16_t interval_ms;
    uint16_t max_duration_ms;
    uint16_t decay_num;
    uint16_t decay_den;
    uint8_t fp_shift;
    int16_t start_threshold;
    int16_t min_velocity;
    uint16_t ema_alpha;
};
struct iqs9151_inertia_gate_params {
    uint16_t recent_window_ms;
    uint16_t stale_gap_ms;
    uint8_t min_samples;
    int16_t min_avg_speed;
};
struct iqs9151_inertia_state {
    bool active;
    int32_t vx_fp;
    int32_t vy_fp;
    int32_t accum_x_fp;
    int32_t accum_y_fp;
    int64_t last_ms;
    uint32_t elapsed_ms;
};

struct iqs9151_finger_history_entry {
    int64_t ms;
    uint8_t finger_count;
};
struct iqs9151_motion_sample {
    int64_t ms;
    int16_t x;
    int16_t y;
};
struct iqs9151_motion_history {
    struct iqs9151_motion_sample samples[IQS9151_INERTIA_MOTION_HISTORY_SIZE];
    uint8_t head;
    uint8_t count;
};

/*
 * A ring of recent per-report deltas for the distance-window smoother.
 *
 * The window is measured in finger-travel counts, not reports, so its length in
 * samples is however many recent reports it takes to cover that distance --
 * few when the finger is quick, many when it crawls. The cap bounds the crawl:
 * at roughly half a count per report the window stops growing and the
 * smoothing eases off, which is where the ripple is slowest and least felt
 * anyway. `sum` and `dist` are kept incrementally so each report is O(1).
 */
#define IQS9151_DIST_SMOOTH_CAP 96
struct iqs9151_dist_smoother {
    int16_t buf[IQS9151_DIST_SMOOTH_CAP];
    uint8_t head;   /* next slot to write */
    uint8_t count;  /* samples currently in the window */
    int32_t sum;    /* signed sum of the samples in the window */
    int32_t dist;   /* sum of the samples' magnitudes: the window's length in counts */
    int32_t rem_fp; /* output carry, in 1/256 of a count */
};

/*
 * Self-calibrating equaliser for the device's positional ripple.
 *
 * Measured on the pad: the reported position, against where the finger really
 * is, carries a wave that repeats every half electrode pitch on the coarse
 * axis -- 76 counts, 3.3 mm -- and swings the reported speed better than two to
 * one, deeper the slower the finger crawls. It is a fixed function of where
 * the finger is (the phase within one electrode), which is the one thing the
 * driver also knows: the absolute coordinate arrives with every frame.
 *
 * So rather than average it away after the fact (a distance window one period
 * long does that perfectly and costs half a period of lag, which is what the
 * hand objected to), model it and divide it out, frame by frame, with no lag:
 * reported speed = true speed x g(phase), g = 1 + sum over k of (ak cos k.phase
 * + bk sin k.phase) for four harmonics, and the pointer gets rel / g. The
 * coefficients are learned on the fly by LMS from the ratio of each report to a
 * one-period distance average -- that average is used only as the reference to
 * learn from, never for output -- so every unit calibrates itself in about one
 * stroke, and keeps the answer while the finger is up. A correction table
 * indexed by phase is rebuilt every few updates and normalised so the mean of
 * 1/g over a period is exactly one, which is what keeps a stroke's total travel
 * the same with and without this.
 *
 * Only the period has to be told to it, in tenths of the device's counts. The
 * geometry says X_RESOLUTION / (2 x 13 electrodes) = 76.0 on this pad; the
 * pad itself, measured, says 72 to 74, and the equaliser is unforgiving about
 * the difference -- five percent off halves the cancellation, ten percent off
 * undoes it, because the phase it learns against drifts a full turn every
 * twenty periods. Hence tenths, and hence a runtime setting: the right value
 * is found by scanning it half a count at a time with the counting tool open.
 *
 * Phase is kept as a fraction of the period in 128 bins, so the tables are the
 * same size whatever the period, and the k-th harmonic is simply k times the
 * bin index.
 */
#define IQS9151_RIPPLE_MAX_PERIOD_X10 1600 /* 160.0 counts */
#define IQS9151_RIPPLE_BINS 128
#define IQS9151_RIPPLE_HARMONICS 4
#define IQS9151_RIPPLE_MU_SHIFT 7      /* LMS step 1/128 */
/* Below this learned fundamental the equaliser applies nothing. Coefficients
 * learned from noise alone sit around 0.05-0.1 each, and eight of them at a
 * period the pad does not have add up to a wave of their own: on a pad with
 * no ripple, the correction was the ripple. */
#define IQS9151_RIPPLE_MIN_FUNDAMENTAL 4915 /* 0.15 in Q15 */
#define IQS9151_RIPPLE_SCAN_MU_SHIFT 9 /* the period search remembers longer: 1/512 */
#define IQS9151_RIPPLE_NORM_EVERY 16   /* rebuild the correction table this often */
#define IQS9151_RIPPLE_COEF_LIMIT 29491 /* 0.9 in Q15 for the fundamental */
/* Coefficients carry eight guard bits below Q15. Without them the LMS step,
 * floored, is short by half a unit every update, which is the same drift on
 * every coefficient and, being the same, is a spurious wave: in simulation it
 * pulled the period search nearly a percent high. */
#define IQS9151_RIPPLE_COEF_GUARD 8
#define IQS9151_RIPPLE_COEF_ONE (32768 << IQS9151_RIPPLE_COEF_GUARD)
#define IQS9151_RIPPLE_G_FLOOR 3277     /* 0.1 in Q15: g never crosses zero */
#define IQS9151_RIPPLE_TRACE_EVERY 128  /* log the coefficients this often (devtool) */
#define IQS9151_RIPPLE_REF_CAP 96
#define IQS9151_RIPPLE_IDLE_CAP 8

/*
 * The reference the equaliser learns against: the last period of travel on one
 * axis, as mean movement per frame. Kept as moving reports with the still
 * frames that followed each one folded in as a count, so a slow crawl -- 1, 0,
 * 0, 1, 0 -- does not fill the ring with zeros before it spans a period, and
 * so that the crawl still teaches: at that speed the wave shows not in the
 * size of the reports, which are all 1, but in how many still frames come
 * between them, so each report is learned from as movement per frame since
 * the report before. A report that follows more than IQS9151_RIPPLE_IDLE_CAP
 * still frames is not learned from at all: a finger that rested mid-stroke
 * says nothing about the wave, and letting it would teach a dip wherever it
 * happened to rest.
 */
struct iqs9151_ripple_ref {
    uint8_t mag[IQS9151_RIPPLE_REF_CAP];  /* |rel| of each moving report */
    uint8_t idle[IQS9151_RIPPLE_REF_CAP]; /* still frames after it, saturating */
    uint8_t head, count;
    uint32_t dist;   /* sum of mag: the window's length in counts */
    uint32_t frames; /* sum of 1 + min(idle, cap): what the window averages over */
};

/*
 * Finding the period without being told it.
 *
 * The first two pads measured did not agree: one cancelled at 75.5, the
 * other's wave would not lock to anything between 76 and 78, and the
 * devtool log showed why -- its learned phase wandered, which is what a
 * wrong period looks like from the inside. Every unit has its own value, to a
 * tenth, and nobody should have to scan for it. So a bank of candidate
 * periods learns in parallel, fundamental only, from the same samples the
 * equaliser learns from; the one whose wave grows largest is the period,
 * because at any other period the phase slides and the wave averages itself
 * away. Coarse first, half a count apart across the plausible range, then a
 * fine bank a tenth apart around the winner. The result is adopted as the
 * live period and, since nothing else costs so little, the fine bank keeps
 * running so the lock can follow a drift.
 */
#define IQS9151_RIPPLE_SCAN_MAX 29
/*
 * The coarse sweep is centred on what the geometry predicts for the axis --
 * resolution / (2 x electrodes along it) -- and spans +/-14 steps of a
 * hundredth of that, so +/-14%. A fixed 70..84 was right for the long axis
 * at its original scale and for nothing else: the short axis (10 electrodes,
 * 1200 counts) lives at 60, and a pad whose scale has been changed moves
 * with it. A bank that does not contain the period cannot find it, and
 * shows it only as a peak pinned to one end.
 */
#define IQS9151_RIPPLE_SCAN_COARSE_HALF 14
#define IQS9151_RIPPLE_SCAN_COARSE_PERCENT 1
#define IQS9151_RIPPLE_MIN_PERIOD_X10 200 /* 20.0 counts: below this nothing is a wave */
#define IQS9151_RIPPLE_SCAN_FINE_STEP_X10 1
#define IQS9151_RIPPLE_SCAN_FINE_HALF 10 /* +/- 1.0 around the coarse winner */
#define IQS9151_RIPPLE_SCAN_EVERY 256          /* learning steps per verdict */
#define IQS9151_RIPPLE_SCAN_MIN_AMP 3277       /* 0.10 in Q15: below this, no wave */
#define IQS9151_RIPPLE_SCAN_AGREE 2            /* consecutive verdicts before acting */
#define IQS9151_RIPPLE_SCAN_HOLD_X10 4         /* a found period moves only by this much */
#define IQS9151_RIPPLE_SCAN_REAGREE 3          /* verdicts before moving one already found */
#define IQS9151_RIPPLE_SCAN_LOST 6             /* fine verdicts without a wave before sweeping again */

enum iqs9151_ripple_scan_stage {
    IQS9151_RIPPLE_SCAN_COARSE,
    IQS9151_RIPPLE_SCAN_FINE,
};

struct iqs9151_ripple_scan {
    int32_t a[IQS9151_RIPPLE_SCAN_MAX];  /* Q15+guard fundamental per candidate */
    int32_t b[IQS9151_RIPPLE_SCAN_MAX];
    uint16_t base_x10;                   /* candidate i is base + i * step */
    uint16_t step_x10;
    uint8_t count;
    uint8_t stage;
    uint16_t updates;
    uint16_t last_winner_x10;            /* the previous verdict, for hysteresis */
    uint8_t agree;
    uint8_t lost;                        /* fine verdicts in a row with no wave */
    uint16_t found_x10;                  /* 0 until a period has been adopted */
    uint16_t resolution;                 /* the axis resolution this search is for */
};

struct iqs9151_ripple_eq {
    int32_t a[IQS9151_RIPPLE_HARMONICS];     /* Q15+guard, cosine terms */
    int32_t b[IQS9151_RIPPLE_HARMONICS];     /* Q15+guard, sine terms */
    uint16_t corr[IQS9151_RIPPLE_BINS];      /* Q8 correction per phase bin */
    struct iqs9151_ripple_ref ref;           /* one-period reference average */
    struct iqs9151_ripple_scan scan;         /* the period search */
    int32_t rem_fp;                          /* output carry, 1/256 count */
    uint16_t tick;
    uint16_t updates;                        /* learning steps since the last trace */
    uint16_t period_x10;                     /* the period in use; 0 = off */
    bool scanning;                           /* the search is on for this axis */
};

/*
 * The ripple as a map over absolute position, no period required.
 *
 * The equaliser above needs the period of the wave, and the period search
 * needs the wave to be a clean function of (position mod period) -- which,
 * on the third pad measured, it evidently is not well enough for the search
 * to lock, while the wave itself is plainly there, fixed to the pad, at
 * +/-50% of the speed. So instead of a shape indexed by phase, a table
 * indexed by where on the axis the finger is: 256 bins across the
 * resolution (eight counts, a third of a millimetre, on the long axis),
 * each holding the average of "this frame's movement over the local mean
 * movement" seen there. Whatever shape the nonlinearity has -- electrode
 * pitch, edges, an uneven electrode, a place where the overlay is thicker
 * -- it is in the table after a few strokes along the axis, and divided out
 * of every report from then on.
 *
 * The cost is convergence: 256 numbers learn slower than eight, and each
 * bin only learns when the finger passes through it. Three or four
 * full-length strokes cover an axis; the table is kept across power cycles
 * so that is once per pad, not once per morning.
 */
#define IQS9151_MAP_BINS 256
#define IQS9151_MAP_ONE 4096             /* Q12: 1.0 */
#define IQS9151_MAP_MEAN_CAP 32          /* per-bin: a true mean this far, then a 1/32 average */
#define IQS9151_MAP_MIN_SAMPLES 6        /* a bin corrects only after this many */
#define IQS9151_MAP_REBUILD_EVERY 64     /* learning steps between table rebuilds */
#define IQS9151_MAP_SAVE_MIN_MS 600000   /* flash writes at most every ten minutes ... */
#define IQS9151_MAP_SAVE_MIN_STEPS 256   /* ... and only after this much new learning */
#define IQS9151_MAP_TRACE_EVERY 512      /* devtool: learning steps per dump */
#define IQS9151_MAP_CORR_MIN 64          /* Q8: never more than /4 ... */
#define IQS9151_MAP_CORR_MAX 1024        /* ... or x4 */

struct iqs9151_ripple_map {
    int16_t g[IQS9151_MAP_BINS];     /* learned speed ratio, Q12; 0 = never visited */
    uint8_t n[IQS9151_MAP_BINS];     /* samples seen, saturating */
    uint16_t corr[IQS9151_MAP_BINS]; /* Q8 local slope of the correction: mean(g) / g; 256 = none */
    /* L: the corrected absolute position at each bin edge, in 1/256 bin, the
     * running sum of corr. The report is L(abs now) - L(abs before): the
     * true distance the finger moved, whatever the reported one was. */
    uint32_t lut[IQS9151_MAP_BINS + 1];
    int32_t prev_l;                  /* L at the previous frame's position */
    struct iqs9151_ripple_ref ref;   /* local reference: one electrode pitch of travel */
    uint16_t resolution;             /* the axis resolution the table is for */
    uint16_t prev_abs;               /* where the finger was last frame */
    bool have_prev;
    int32_t rem_fp;                  /* output carry, 1/256 count */
    uint16_t tick;                   /* learning steps since the last rebuild */
    uint16_t updates;                /* learning steps since the last trace */
    bool dirty;                      /* learned something since the last save */
    uint32_t steps_since_save;
    int64_t saved_ms;
    uint16_t learned;                /* bins with enough samples, as last published */
    uint16_t period_geo_x10;  /* the wave's period from the geometry, the window until measured */
    uint16_t period_x10;      /* the wave's period as measured from the table; 0 = not yet */
    uint16_t period_cand_x10; /* the previous measurement, which the next must agree with */
    uint16_t period_told_x10; /* ... as last published to the app */
    int64_t published_ms;     /* when the learned count was last published */
};

struct iqs9151_data {
    const struct device *dev;
    struct gpio_callback gpio_cb;
    struct k_work work;
    struct k_work_delayable one_finger_click_work;
    struct k_work_delayable two_finger_click_work;
    struct k_work_delayable three_finger_click_work;
    struct k_work_delayable inertia_scroll_work;
    struct k_work_delayable inertia_cursor_work;
    struct iqs9151_inertia_state inertia_scroll;
    struct iqs9151_inertia_state inertia_cursor;
    int32_t scroll_ema_x_fp;
    int32_t scroll_ema_y_fp;
    int32_t cursor_ema_x_fp;
    int32_t cursor_ema_y_fp;
    struct iqs9151_motion_history scroll_motion_history;
    struct iqs9151_motion_history cursor_motion_history;
    struct iqs9151_one_finger_state one_finger;
    struct iqs9151_two_finger_state two_finger;
    bool one_finger_click_pending;
    int64_t one_finger_click_pending_ms;
    bool two_finger_click_pending;
    int64_t two_finger_click_pending_ms;
    bool three_finger_click_pending;
    int64_t three_finger_click_pending_ms;
    bool two_finger_one_lead_valid;
    bool two_finger_tail_suppresses_cursor;
    bool three_finger_one_lead_valid;
    bool three_finger_two_lead_valid;
    struct iqs9151_frame prev_frame;
    bool touch_state_sent;
    bool touch_state_2f_sent;
    bool three_active;
    bool three_hold_sent;
    bool three_swipe_sent;
    bool three_tap_candidate;
    bool three_hold_candidate;
    bool three_tapdrag_second_touch;
    bool three_release_pending;
    bool three_have_last;
    int64_t three_down_ms;
    int64_t three_release_pending_ms;
    int32_t three_dx;
    int32_t three_dy;
    uint16_t three_last_x;
    uint16_t three_last_y;
    /* When the count last dropped below three mid-gesture; 0 = it has not. */
    int64_t three_flicker_ms;
    uint16_t hold_button;
    /*
     * Cursor movement owed but not yet reported, and when the last report
     * went out. See iqs9151_report_cursor: a half whose pointer travels over
     * a BLE link has a rate the link can carry, and 200 frames a second is
     * not it.
     */
    int32_t cursor_pending_x;
    int32_t cursor_pending_y;
    int64_t cursor_report_ms;
    struct iqs9151_finger_history_entry finger_history[IQS9151_FINGER_HISTORY_SIZE];
    uint8_t finger_history_head;
    uint8_t finger_history_count;
    /* Which generation of the runtime resolution this instance has written. */
    atomic_t resolution_generation;
    /* Same, for the low-speed filter block. */
    atomic_t filter_generation;
    /*
     * Set from the moment the device reports it has reset itself until its
     * configuration has been written back. Frames are ignored throughout: the
     * coordinates an unconfigured IQS9151 produces are not this pad's.
     */
    atomic_t recovering;
    struct k_work_delayable recover_work;
    uint32_t recover_count;
    int64_t recover_last_ms;
    /*
     * Tenths carried between reports when the cursor gain is not a whole
     * number. Without them a 1.6x gain on a stream of 1-count reports rounds
     * to 1 every time and the axis never actually speeds up.
     */
    int32_t cursor_gain_remainder_x;
    int32_t cursor_gain_remainder_y;
    /*
     * Distance-window smoothing state, one per axis. See
     * iqs9151_distance_smooth_axis: a moving average of the reported movement
     * taken over a fixed window of finger travel, which flattens a ripple that
     * repeats every so many millimetres whatever the speed it is crossed at.
     */
    struct iqs9151_dist_smoother dist_smoother_x;
    struct iqs9151_dist_smoother dist_smoother_y;
    /* Positional ripple equalisers, one per axis. See struct iqs9151_ripple_eq. */
    struct iqs9151_ripple_eq ripple_x;
    struct iqs9151_ripple_eq ripple_y;
    struct iqs9151_ripple_map map_x;
    struct iqs9151_ripple_map map_y;
    struct k_work map_save_work;     /* flash, off the system work queue */
    /* The landing dead zone: where the finger came down, when, and whether
     * its movement is still being withheld. */
    uint16_t land_x, land_y;
    int64_t land_ms;
    bool landing;
#if IS_ENABLED(CONFIG_INPUT_IQS9151_MOTION_TRACE)
    int64_t trace_last_read_ms;
    uint16_t trace_quiet;
#endif
};

#ifdef CONFIG_INPUT_IQS9151_TEST
static struct {
    iqs9151_test_event_hook_t hook;
    void *user_data;
} iqs9151_test_hook;
#endif

static int iqs9151_report_key_event(const struct device *dev, uint16_t code,
                                    int32_t value, bool sync, k_timeout_t timeout) {
#ifdef CONFIG_INPUT_IQS9151_TEST
    if (iqs9151_test_hook.hook != NULL) {
        const struct iqs9151_test_event event = {
            .type = IQS9151_TEST_EVENT_KEY,
            .dev = dev,
            .code = code,
            .value = !!value,
            .sync = sync,
            .timeout = timeout,
        };
        iqs9151_test_hook.hook(&event, iqs9151_test_hook.user_data);
        return 0;
    }
#endif
    return input_report_key(dev, code, value, sync, timeout);
}

static int iqs9151_report_rel_event(const struct device *dev, uint16_t code,
                                    int32_t value, bool sync, k_timeout_t timeout) {
#ifdef CONFIG_INPUT_IQS9151_TEST
    if (iqs9151_test_hook.hook != NULL) {
        const struct iqs9151_test_event event = {
            .type = IQS9151_TEST_EVENT_REL,
            .dev = dev,
            .code = code,
            .value = value,
            .sync = sync,
            .timeout = timeout,
        };
        iqs9151_test_hook.hook(&event, iqs9151_test_hook.user_data);
        return 0;
    }
#endif
    return input_report_rel(dev, code, value, sync, timeout);
}

static const uint8_t iqs9151_alp_compensation[] = {
    ALP_COMPENSATION_RX0_0,  ALP_COMPENSATION_RX0_1,  ALP_COMPENSATION_RX1_0,
    ALP_COMPENSATION_RX1_1,  ALP_COMPENSATION_RX2_0,  ALP_COMPENSATION_RX2_1,
    ALP_COMPENSATION_RX3_0,  ALP_COMPENSATION_RX3_1,  ALP_COMPENSATION_RX4_0,
    ALP_COMPENSATION_RX4_1,  ALP_COMPENSATION_RX5_0,  ALP_COMPENSATION_RX5_1,
    ALP_COMPENSATION_RX6_0,  ALP_COMPENSATION_RX6_1,  ALP_COMPENSATION_RX7_0,
    ALP_COMPENSATION_RX7_1,  ALP_COMPENSATION_RX8_0,  ALP_COMPENSATION_RX8_1,
    ALP_COMPENSATION_RX9_0,  ALP_COMPENSATION_RX9_1,  ALP_COMPENSATION_RX10_0,
    ALP_COMPENSATION_RX10_1, ALP_COMPENSATION_RX11_0, ALP_COMPENSATION_RX11_1,
    ALP_COMPENSATION_RX12_0, ALP_COMPENSATION_RX12_1,
};
static const uint8_t iqs9151_main_config[] = {
    MINOR_VERSION,
    MAJOR_VERSION,
    TP_ATI_MULTDIV_L,
    TP_ATI_MULTDIV_H,
    ALP_ATI_COARSE_RX0_L,
    ALP_ATI_COARSE_RX0_H,
    ALP_ATI_COARSE_RX1_L,
    ALP_ATI_COARSE_RX1_H,
    ALP_ATI_COARSE_RX2_L,
    ALP_ATI_COARSE_RX2_H,
    ALP_ATI_COARSE_RX3_L,
    ALP_ATI_COARSE_RX3_H,
    ALP_ATI_COARSE_RX4_L,
    ALP_ATI_COARSE_RX4_H,
    ALP_ATI_COARSE_RX5_L,
    ALP_ATI_COARSE_RX5_H,
    ALP_ATI_COARSE_RX6_L,
    ALP_ATI_COARSE_RX6_H,
    ALP_ATI_COARSE_RX7_L,
    ALP_ATI_COARSE_RX7_H,
    ALP_ATI_COARSE_RX8_L,
    ALP_ATI_COARSE_RX8_H,
    ALP_ATI_COARSE_RX9_L,
    ALP_ATI_COARSE_RX9_H,
    ALP_ATI_COARSE_RX10_L,
    ALP_ATI_COARSE_RX10_H,
    ALP_ATI_COARSE_RX11_L,
    ALP_ATI_COARSE_RX11_H,
    ALP_ATI_COARSE_RX12_L,
    ALP_ATI_COARSE_RX12_H,
    TP_ATI_TARGET_0,
    TP_ATI_TARGET_1,
    ALP_ATI_TARGET_0,
    ALP_ATI_TARGET_1,
    ALP_BASE_TARGET_0,
    ALP_BASE_TARGET_1,
    TP_NEG_DELTA_REATI_0,
    TP_NEG_DELTA_REATI_1,
    TP_POS_DELTA_REATI_0,
    TP_POS_DELTA_REATI_1,
    TP_REF_DRIFT_LIMIT,
    ALP_LTA_DRIFT_LIMIT,
    ACTIVE_MODE_SAMPLING_PERIOD_0,
    ACTIVE_MODE_SAMPLING_PERIOD_1,
    IDLE_TOUCH_MODE_SAMPLING_PERIOD_0,
    IDLE_TOUCH_MODE_SAMPLING_PERIOD_1,
    IDLE_MODE_SAMPLING_PERIOD_0,
    IDLE_MODE_SAMPLING_PERIOD_1,
    LP1_MODE_SAMPLING_PERIOD_0,
    LP1_MODE_SAMPLING_PERIOD_1,
    LP2_MODE_SAMPLING_PERIOD_0,
    LP2_MODE_SAMPLING_PERIOD_1,
    STATIONARY_TOUCH_TIMEOUT_0,
    STATIONARY_TOUCH_TIMEOUT_1,
    IDLE_TOUCH_MODE_TIMEOUT_0,
    IDLE_TOUCH_MODE_TIMEOUT_1,
    IDLE_MODE_TIMEOUT_0,
    IDLE_MODE_TIMEOUT_1,
    LP1_MODE_TIMEOUT_0,
    LP1_MODE_TIMEOUT_1,
    ACTIVE_MODE_TIMEOUT_0,
    ACTIVE_MODE_TIMEOUT_1,
    REATI_RETRY_TIME,
    REF_UPDATE_TIME,
    I2C_TIMEOUT_0,
    I2C_TIMEOUT_1,
    SNAP_TIMEOUT,
    OPEN_TIMING,
    SYSTEM_CONTROL_0,
    SYSTEM_CONTROL_1,
    CONFIG_SETTINGS_0,
    CONFIG_SETTINGS_1,
    OTHER_SETTINGS_0,
    OTHER_SETTINGS_1,
    ALP_SETUP_0,
    ALP_SETUP_1,
    ALP_SETUP_2,
    ALP_SETUP_3,
    ALP_TX_ENABLE_0,
    ALP_TX_ENABLE_1,
    ALP_TX_ENABLE_2,
    ALP_TX_ENABLE_3,
    ALP_TX_ENABLE_4,
    ALP_TX_ENABLE_5,
    TRACKPAD_TOUCH_SET_THRESHOLD,
    TRACKPAD_TOUCH_CLEAR_THRESHOLD,
    ALP_THRESHOLD,
    ALP_AUTOPROX_THRESHOLD,
    ALP_SET_DEBOUNCE,
    ALP_CLEAR_DEBOUNCE,
    SNAP_SET_THRESHOLD,
    SNAP_CLEAR_THRESHOLD,
    ALP_COUNT_BETA_LP1,
    ALP_LTA_BETA_LP1,
    ALP_COUNT_BETA_LP2,
    ALP_LTA_BETA_LP2,
    TP_FRAC,
    TP_PERIOD1,
    TP_PERIOD2,
    ALP_FRAC,
    ALP_PERIOD1,
    ALP_PERIOD2,
    TRACKPAD_HARDWARE_SETTINGS_0,
    TRACKPAD_HARDWARE_SETTINGS_1,
    ALP_HARDWARE_SETTINGS_0,
    ALP_HARDWARE_SETTINGS_1,
    TRACKPAD_SETTINGS_0_0,
    TRACKPAD_SETTINGS_0_1,
    TRACKPAD_SETTINGS_1_0,
    TRACKPAD_SETTINGS_1_1,
    X_RESOLUTION_0,
    X_RESOLUTION_1,
    Y_RESOLUTION_0,
    Y_RESOLUTION_1,
    XY_DYNAMIC_FILTER_BOTTOM_SPEED_0,
    XY_DYNAMIC_FILTER_BOTTOM_SPEED_1,
    XY_DYNAMIC_FILTER_TOP_SPEED_0,
    XY_DYNAMIC_FILTER_TOP_SPEED_1,
    XY_DYNAMIC_FILTER_BOTTOM_BETA,
    XY_DYNAMIC_FILTER_STATIC_FILTER_BETA,
    STATIONARY_TOUCH_MOV_THRESHOLD,
    FINGER_SPLIT_FACTOR,
    X_TRIM_VALUE,
    Y_TRIM_VALUE,
    JITTER_FILTER_DELTA,
    FINGER_CONFIDENCE_THRESHOLD,
};

static const uint8_t iqs9151_rxtx_map[] = {
    RX_TX_MAP_0,  RX_TX_MAP_1,  RX_TX_MAP_2,  RX_TX_MAP_3,  RX_TX_MAP_4,
    RX_TX_MAP_5,  RX_TX_MAP_6,  RX_TX_MAP_7,  RX_TX_MAP_8,  RX_TX_MAP_9,
    RX_TX_MAP_10, RX_TX_MAP_11, RX_TX_MAP_12, RX_TX_MAP_13, RX_TX_MAP_14,
    RX_TX_MAP_15, RX_TX_MAP_16, RX_TX_MAP_17, RX_TX_MAP_18, RX_TX_MAP_19,
    RX_TX_MAP_20, RX_TX_MAP_21, RX_TX_MAP_22, RX_TX_MAP_23, RX_TX_MAP_24,
    RX_TX_MAP_25, RX_TX_MAP_26, RX_TX_MAP_27, RX_TX_MAP_28, RX_TX_MAP_29,
    RX_TX_MAP_30, RX_TX_MAP_31, RX_TX_MAP_32, RX_TX_MAP_33, RX_TX_MAP_34,
    RX_TX_MAP_35, RX_TX_MAP_36, RX_TX_MAP_37, RX_TX_MAP_38, RX_TX_MAP_39,
    RX_TX_MAP_40, RX_TX_MAP_41, RX_TX_MAP_42, RX_TX_MAP_43, RX_TX_MAP_44,
    RX_TX_OPEN,
};

static const uint8_t iqs9151_channel_disable[] = {
    TPCHANNELDISABLE_0,  TPCHANNELDISABLE_1,  TPCHANNELDISABLE_2,
    TPCHANNELDISABLE_3,  TPCHANNELDISABLE_4,  TPCHANNELDISABLE_5,
    TPCHANNELDISABLE_6,  TPCHANNELDISABLE_7,  TPCHANNELDISABLE_8,
    TPCHANNELDISABLE_9,  TPCHANNELDISABLE_10, TPCHANNELDISABLE_11,
    TPCHANNELDISABLE_12, TPCHANNELDISABLE_13, TPCHANNELDISABLE_14,
    TPCHANNELDISABLE_15, TPCHANNELDISABLE_16, TPCHANNELDISABLE_17,
    TPCHANNELDISABLE_18, TPCHANNELDISABLE_19, TPCHANNELDISABLE_20,
    TPCHANNELDISABLE_21, TPCHANNELDISABLE_22, TPCHANNELDISABLE_23,
    TPCHANNELDISABLE_24, TPCHANNELDISABLE_25, TPCHANNELDISABLE_26,
    TPCHANNELDISABLE_27, TPCHANNELDISABLE_28, TPCHANNELDISABLE_29,
    TPCHANNELDISABLE_30, TPCHANNELDISABLE_31, TPCHANNELDISABLE_32,
    TPCHANNELDISABLE_33, TPCHANNELDISABLE_34, TPCHANNELDISABLE_35,
    TPCHANNELDISABLE_36, TPCHANNELDISABLE_37, TPCHANNELDISABLE_38,
    TPCHANNELDISABLE_39, TPCHANNELDISABLE_40, TPCHANNELDISABLE_41,
    TPCHANNELDISABLE_42, TPCHANNELDISABLE_43, TPCHANNELDISABLE_44,
    TPCHANNELDISABLE_45, TPCHANNELDISABLE_46, TPCHANNELDISABLE_47,
    TPCHANNELDISABLE_48, TPCHANNELDISABLE_49, TPCHANNELDISABLE_50,
    TPCHANNELDISABLE_51, TPCHANNELDISABLE_52, TPCHANNELDISABLE_53,
    TPCHANNELDISABLE_54, TPCHANNELDISABLE_55, TPCHANNELDISABLE_56,
    TPCHANNELDISABLE_57, TPCHANNELDISABLE_58, TPCHANNELDISABLE_59,
    TPCHANNELDISABLE_60, TPCHANNELDISABLE_61, TPCHANNELDISABLE_62,
    TPCHANNELDISABLE_63, TPCHANNELDISABLE_64, TPCHANNELDISABLE_65,
    TPCHANNELDISABLE_66, TPCHANNELDISABLE_67, TPCHANNELDISABLE_68,
    TPCHANNELDISABLE_69, TPCHANNELDISABLE_70, TPCHANNELDISABLE_71,
    TPCHANNELDISABLE_72, TPCHANNELDISABLE_73, TPCHANNELDISABLE_74,
    TPCHANNELDISABLE_75, TPCHANNELDISABLE_76, TPCHANNELDISABLE_77,
    TPCHANNELDISABLE_78, TPCHANNELDISABLE_79, TPCHANNELDISABLE_80,
    TPCHANNELDISABLE_81, TPCHANNELDISABLE_82, TPCHANNELDISABLE_83,
    TPCHANNELDISABLE_84, TPCHANNELDISABLE_85, TPCHANNELDISABLE_86,
    TPCHANNELDISABLE_87,
};
static const uint8_t iqs9151_snap_enable[] = {
    SNAPCHANNELENABLE_0,  SNAPCHANNELENABLE_1,  SNAPCHANNELENABLE_2,
    SNAPCHANNELENABLE_3,  SNAPCHANNELENABLE_4,  SNAPCHANNELENABLE_5,
    SNAPCHANNELENABLE_6,  SNAPCHANNELENABLE_7,  SNAPCHANNELENABLE_8,
    SNAPCHANNELENABLE_9,  SNAPCHANNELENABLE_10, SNAPCHANNELENABLE_11,
    SNAPCHANNELENABLE_12, SNAPCHANNELENABLE_13, SNAPCHANNELENABLE_14,
    SNAPCHANNELENABLE_15, SNAPCHANNELENABLE_16, SNAPCHANNELENABLE_17,
    SNAPCHANNELENABLE_18, SNAPCHANNELENABLE_19, SNAPCHANNELENABLE_20,
    SNAPCHANNELENABLE_21, SNAPCHANNELENABLE_22, SNAPCHANNELENABLE_23,
    SNAPCHANNELENABLE_24, SNAPCHANNELENABLE_25, SNAPCHANNELENABLE_26,
    SNAPCHANNELENABLE_27, SNAPCHANNELENABLE_28, SNAPCHANNELENABLE_29,
    SNAPCHANNELENABLE_30, SNAPCHANNELENABLE_31, SNAPCHANNELENABLE_32,
    SNAPCHANNELENABLE_33, SNAPCHANNELENABLE_34, SNAPCHANNELENABLE_35,
    SNAPCHANNELENABLE_36, SNAPCHANNELENABLE_37, SNAPCHANNELENABLE_38,
    SNAPCHANNELENABLE_39, SNAPCHANNELENABLE_40, SNAPCHANNELENABLE_41,
    SNAPCHANNELENABLE_42, SNAPCHANNELENABLE_43, SNAPCHANNELENABLE_44,
    SNAPCHANNELENABLE_45, SNAPCHANNELENABLE_46, SNAPCHANNELENABLE_47,
    SNAPCHANNELENABLE_48, SNAPCHANNELENABLE_49, SNAPCHANNELENABLE_50,
    SNAPCHANNELENABLE_51, SNAPCHANNELENABLE_52, SNAPCHANNELENABLE_53,
    SNAPCHANNELENABLE_54, SNAPCHANNELENABLE_55, SNAPCHANNELENABLE_56,
    SNAPCHANNELENABLE_57, SNAPCHANNELENABLE_58, SNAPCHANNELENABLE_59,
    SNAPCHANNELENABLE_60, SNAPCHANNELENABLE_61, SNAPCHANNELENABLE_62,
    SNAPCHANNELENABLE_63, SNAPCHANNELENABLE_64, SNAPCHANNELENABLE_65,
    SNAPCHANNELENABLE_66, SNAPCHANNELENABLE_67, SNAPCHANNELENABLE_68,
    SNAPCHANNELENABLE_69, SNAPCHANNELENABLE_70, SNAPCHANNELENABLE_71,
    SNAPCHANNELENABLE_72, SNAPCHANNELENABLE_73, SNAPCHANNELENABLE_74,
    SNAPCHANNELENABLE_75, SNAPCHANNELENABLE_76, SNAPCHANNELENABLE_77,
    SNAPCHANNELENABLE_78, SNAPCHANNELENABLE_79, SNAPCHANNELENABLE_80,
    SNAPCHANNELENABLE_81, SNAPCHANNELENABLE_82, SNAPCHANNELENABLE_83,
    SNAPCHANNELENABLE_84, SNAPCHANNELENABLE_85, SNAPCHANNELENABLE_86,
    SNAPCHANNELENABLE_87,
};
static const struct iqs9151_inertia_params iqs9151_scroll_params = {
    .interval_ms = SCROLL_INERTIA_INTERVAL_MS,
    .max_duration_ms = SCROLL_INERTIA_MAX_DURATION_MS,
    .decay_num = SCROLL_INERTIA_DECAY_NUM,
    .decay_den = SCROLL_INERTIA_DECAY_DEN,
    .fp_shift = INERTIA_FP_SHIFT,
    .start_threshold = SCROLL_INERTIA_START_THRESHOLD,
    .min_velocity = SCROLL_INERTIA_MIN_VELOCITY,
    .ema_alpha = SCROLL_EMA_ALPHA,
};
static const struct iqs9151_inertia_gate_params iqs9151_scroll_gate_params = {
    .recent_window_ms = SCROLL_INERTIA_RECENT_WINDOW_MS,
    .stale_gap_ms = SCROLL_INERTIA_STALE_GAP_MS,
    .min_samples = SCROLL_INERTIA_MIN_SAMPLES,
    .min_avg_speed = SCROLL_INERTIA_MIN_AVG_SPEED,
};
static const struct iqs9151_inertia_params iqs9151_cursor_params = {
    .interval_ms = CURSOR_INERTIA_INTERVAL_MS,
    .max_duration_ms = CURSOR_INERTIA_MAX_DURATION_MS,
    .decay_num = CURSOR_INERTIA_DECAY_NUM,
    .decay_den = CURSOR_INERTIA_DECAY_DEN,
    .fp_shift = INERTIA_FP_SHIFT,
    .start_threshold = CURSOR_INERTIA_START_THRESHOLD,
    .min_velocity = CURSOR_INERTIA_MIN_VELOCITY,
    .ema_alpha = CURSOR_EMA_ALPHA,
};
static const struct iqs9151_inertia_gate_params iqs9151_cursor_gate_params = {
    .recent_window_ms = CURSOR_INERTIA_RECENT_WINDOW_MS,
    .stale_gap_ms = CURSOR_INERTIA_STALE_GAP_MS,
    .min_samples = CURSOR_INERTIA_MIN_SAMPLES,
    .min_avg_speed = CURSOR_INERTIA_MIN_AVG_SPEED,
};

static int iqs9151_i2c_write(const struct iqs9151_config *cfg, uint16_t reg, const uint8_t *buf, size_t len) {
    uint8_t tx[2 + IQS9151_I2C_CHUNK_SIZE];

    if (len > (sizeof(tx) - 2)) {
        return -EINVAL;
    }

    sys_put_le16(reg, tx);
    memcpy(&tx[2], buf, len);
    return i2c_write_dt(&cfg->i2c, tx, len + 2);
}

static int iqs9151_i2c_read(const struct iqs9151_config *cfg, uint16_t reg, uint8_t *buf, size_t len) {
    uint8_t addr_buf[2];

    sys_put_le16(reg, addr_buf);
    return i2c_write_read_dt(&cfg->i2c, addr_buf, sizeof(addr_buf), buf, len);
}

static int iqs9151_write_u16(const struct iqs9151_config *cfg, uint16_t reg, uint16_t value) {
    uint8_t buf[2];

    sys_put_le16(value, buf);
    return iqs9151_i2c_write(cfg, reg, buf, sizeof(buf));
}

static uint8_t iqs9151_tp_ati_multdiv_h(uint8_t div) {
    return (uint8_t)((1U << 7) | (div << 1) | 1U);
}

static int iqs9151_write_tp_ati_div(const struct iqs9151_config *cfg, uint8_t div) {
    const uint8_t ati_multdiv[2] = {
        TP_ATI_MULTDIV_L,
        iqs9151_tp_ati_multdiv_h(div),
    };

    return iqs9151_i2c_write(cfg, IQS9151_ADDR_ATI_MULTIPLIERS, ati_multdiv,
                             sizeof(ati_multdiv));
}

static int iqs9151_read_u16(const struct iqs9151_config *cfg, uint16_t reg, uint16_t *value) {
    uint8_t buf[2];
    int ret = iqs9151_i2c_read(cfg, reg, buf, sizeof(buf));

    if (ret != 0) {
        return ret;
    }

    *value = sys_get_le16(buf);
    return 0;
}

static int iqs9151_update_bits_u16(const struct iqs9151_config *cfg, uint16_t reg,
                                   uint16_t mask, uint16_t value) {
    uint16_t current;
    int ret = iqs9151_read_u16(cfg, reg, &current);

    if (ret != 0) {
        return ret;
    }

    current = (uint16_t)((current & ~mask) | (value & mask));
    return iqs9151_write_u16(cfg, reg, current);
}

static void iqs9151_wait_for_ready(const struct device *dev, uint16_t timeout_ms) {
    const struct iqs9151_config *cfg = dev->config;
    uint16_t elapsed = 0;

    while (!gpio_pin_get_dt(&cfg->irq_gpio) && elapsed < timeout_ms) {
        k_sleep(K_MSEC(1));
        elapsed++;
    }

    if (elapsed >= timeout_ms) {
        LOG_WRN("RDY timeout after %dms", timeout_ms);
    }
    LOG_DBG("IRQGPIO=%d,TIME=%dms", gpio_pin_get_dt(&cfg->irq_gpio), elapsed);
}

/*
 * Ask a device that is not in a communication window to open one.
 *
 * The configuration this driver writes puts the IQS9151 in event mode with the
 * force-comms method set to "request a window" (Config Settings bit 4): with
 * no finger on the pad RDY stays high, and an I2C transaction outside a window
 * is refused rather than clock-stretched. That is the right setting while the
 * pad is running -- but it is also the state the IC is left in when the MCU
 * resets and the pad does not lose power: a watchdog reset, a wake from
 * sys_poweroff on a key, a brownout the nRF noticed and the IC did not. The
 * IC keeps its configuration, RDY never comes, the product-number read at the
 * top of init fails, and the pad is dead until a power cycle -- keys working
 * all the while, because they are the MCU's business, not the pad's.
 *
 * The request is the sequence in the datasheet's Figure 12.5 (v1.1, 12.9.2):
 * a write of 0xFF, a write of the System Control address, a one-byte read.
 * The IC then opens a window at its next cycle -- up to one LP2 sampling
 * period, 200 ms as configured. A freshly powered IC streams and ACKs
 * anyway, so sending this to one costs nothing.
 */
static int iqs9151_force_comms(const struct device *dev) {
    const struct iqs9151_config *cfg = dev->config;
    const uint8_t request = 0xFF;
    uint8_t addr[2];
    uint8_t dummy;

    sys_put_le16(IQS9151_ADDR_SYSTEM_CONTROL, addr);

    int ret = i2c_write_dt(&cfg->i2c, &request, sizeof(request));
    if (ret == 0) {
        ret = i2c_write_dt(&cfg->i2c, addr, sizeof(addr));
    }
    if (ret == 0) {
        /* The read is the tail of the sequence; the byte it returns is not
         * data, and a NAK on it is what the figure shows. */
        (void)i2c_read_dt(&cfg->i2c, &dummy, sizeof(dummy));
    }
    return ret;
}

static int iqs9151_write_chunks(const struct device *dev, const struct iqs9151_config *cfg
                                    , uint16_t start_reg, const uint8_t *buf, size_t len) {
    size_t offset = 0U;

    while (offset < len) {
        const size_t chunk_len = MIN(IQS9151_I2C_CHUNK_SIZE, len - offset);
        
        iqs9151_wait_for_ready(dev, 200);

        const int ret = iqs9151_i2c_write(cfg, start_reg + offset, buf + offset, chunk_len);
        if (ret != 0) {
            return ret;
        }
        offset += chunk_len;
    }
    return 0;
}

static int iqs9151_check_product_number(const struct device *dev) {
    const struct iqs9151_config *cfg = dev->config;
    uint8_t product[2] = {0};
    int ret;
    
    ret = iqs9151_i2c_read(cfg, IQS9151_ADDR_PRODUCT_NUMBER, product, sizeof(product));
    if (ret != 0) {
        return ret;
    }

    uint16_t product_num = sys_get_le16(product);
    if (ret == 0 && product_num != IQS9151_PRODUCT_NUMBER) {
        LOG_ERR("unexpected product number 0x%04x", product_num);
        return -ENODEV;
    }

    LOG_DBG("product number 0x%04x", product_num);
    return ret;
}

static void iqs9151_parse_frame(const uint8_t *raw, struct iqs9151_frame *frame) {
    frame->rel_x = (int16_t)sys_get_le16(&raw[0]);
    frame->rel_y = (int16_t)sys_get_le16(&raw[2]);
    frame->info_flags = sys_get_le16(&raw[12]);
    frame->trackpad_flags = sys_get_le16(&raw[14]);
    frame->finger_count =
        (uint8_t)(frame->trackpad_flags & IQS9151_TP_FINGER_COUNT_MASK);
    frame->finger1_x = sys_get_le16(&raw[16]);
    frame->finger1_y = sys_get_le16(&raw[18]);
    frame->finger2_x = sys_get_le16(&raw[24]);
    frame->finger2_y = sys_get_le16(&raw[26]);
}

static bool iqs9151_finger1_valid(const struct iqs9151_frame *frame) {
    const bool finger1_confident =
        (frame->trackpad_flags & IQS9151_TP_FINGER1_CONFIDENCE) != 0U;
    const bool finger1_coord_valid =
        (frame->finger1_x != UINT16_MAX) && (frame->finger1_y != UINT16_MAX);

    return finger1_confident && finger1_coord_valid;
}

static bool iqs9151_finger2_valid(const struct iqs9151_frame *frame) {
    const bool finger2_confident =
        (frame->trackpad_flags & IQS9151_TP_FINGER2_CONFIDENCE) != 0U;
    const bool finger2_coord_valid =
        (frame->finger2_x != UINT16_MAX) && (frame->finger2_y != UINT16_MAX);

    return finger2_confident && finger2_coord_valid;
}

static void iqs9151_update_prev_frame(struct iqs9151_data *data,
                                      const struct iqs9151_frame *frame,
                                      const struct iqs9151_frame *prev_frame) {
    data->prev_frame = *frame;

    if (frame->finger_count == 0U) {
        data->prev_frame.finger1_x = 0;
        data->prev_frame.finger1_y = 0;
        data->prev_frame.finger2_x = 0;
        data->prev_frame.finger2_y = 0;
        return;
    }

    if (!iqs9151_finger1_valid(frame)) {
        data->prev_frame.finger1_x = prev_frame->finger1_x;
        data->prev_frame.finger1_y = prev_frame->finger1_y;
    }

    if (frame->finger_count < 2U) {
        data->prev_frame.finger2_x = 0;
        data->prev_frame.finger2_y = 0;
        return;
    }

    if (!iqs9151_finger2_valid(frame)) {
        data->prev_frame.finger2_x = prev_frame->finger2_x;
        data->prev_frame.finger2_y = prev_frame->finger2_y;
    }
}

static int32_t iqs9151_abs32(int32_t value) {
    return (value < 0) ? -value : value;
}

static void iqs9151_ema_reset(int32_t *ema_x_fp, int32_t *ema_y_fp) {
    *ema_x_fp = 0;
    *ema_y_fp = 0;
}

static void iqs9151_ema_update(int32_t *ema_x_fp, int32_t *ema_y_fp,
                               int16_t sample_x, int16_t sample_y, uint16_t alpha) {
    const int32_t sample_x_fp = ((int32_t)sample_x) << EMA_FP_SHIFT;
    const int32_t sample_y_fp = ((int32_t)sample_y) << EMA_FP_SHIFT;
    const int32_t inv_alpha = (int32_t)(EMA_ALPHA_DEN - alpha);

    *ema_x_fp =
        ((*ema_x_fp * (int32_t)alpha) + (sample_x_fp * inv_alpha)) >> EMA_FP_SHIFT;
    *ema_y_fp =
        ((*ema_y_fp * (int32_t)alpha) + (sample_y_fp * inv_alpha)) >> EMA_FP_SHIFT;
}

static void iqs9151_motion_history_reset(struct iqs9151_motion_history *history) {
    memset(history, 0, sizeof(*history));
}

static void iqs9151_motion_history_push(struct iqs9151_motion_history *history,
                                        int16_t sample_x, int16_t sample_y,
                                        int64_t now_ms) {
    struct iqs9151_motion_sample *entry;

    if (sample_x == 0 && sample_y == 0) {
        return;
    }

    entry = &history->samples[history->head];
    entry->ms = now_ms;
    entry->x = sample_x;
    entry->y = sample_y;

    history->head =
        (uint8_t)((history->head + 1U) % IQS9151_INERTIA_MOTION_HISTORY_SIZE);
    if (history->count < IQS9151_INERTIA_MOTION_HISTORY_SIZE) {
        history->count++;
    }
}

static bool iqs9151_inertia_seed_from_history(
    const struct iqs9151_motion_history *history,
    const struct iqs9151_inertia_params *params,
    const struct iqs9151_inertia_gate_params *gate,
    int64_t now_ms,
    int32_t *seed_vx_fp,
    int32_t *seed_vy_fp) {
    struct iqs9151_motion_sample recent[IQS9151_INERTIA_MOTION_HISTORY_SIZE];
    uint8_t recent_count = 0U;
    int32_t total_x = 0;
    int32_t total_y = 0;
    int64_t latest_ms;
    int64_t earliest_ms;
    int64_t span_ms;
    int32_t avg_speed;
    int32_t dominant_total;
    uint8_t consistent_count = 0U;

    for (uint8_t i = 0U; i < history->count; i++) {
        const uint8_t idx =
            (uint8_t)((history->head + IQS9151_INERTIA_MOTION_HISTORY_SIZE - 1U - i) %
                      IQS9151_INERTIA_MOTION_HISTORY_SIZE);
        const struct iqs9151_motion_sample *entry = &history->samples[idx];
        const int64_t elapsed_ms = now_ms - entry->ms;

        if (elapsed_ms > gate->recent_window_ms) {
            break;
        }

        recent[recent_count++] = *entry;
    }

    if (recent_count < gate->min_samples) {
        return false;
    }

    latest_ms = recent[0].ms;
    if ((now_ms - latest_ms) > gate->stale_gap_ms) {
        return false;
    }

    earliest_ms = recent[recent_count - 1U].ms;
    for (uint8_t i = 0U; i < recent_count; i++) {
        total_x += recent[i].x;
        total_y += recent[i].y;
    }

    if (total_x == 0 && total_y == 0) {
        return false;
    }

    if (iqs9151_abs32(total_x) >= iqs9151_abs32(total_y)) {
        dominant_total = total_x;
        for (uint8_t i = 0U; i < recent_count; i++) {
            if ((recent[i].x > 0 && dominant_total > 0) ||
                (recent[i].x < 0 && dominant_total < 0)) {
                consistent_count++;
            }
        }
    } else {
        dominant_total = total_y;
        for (uint8_t i = 0U; i < recent_count; i++) {
            if ((recent[i].y > 0 && dominant_total > 0) ||
                (recent[i].y < 0 && dominant_total < 0)) {
                consistent_count++;
            }
        }
    }

    if (consistent_count < gate->min_samples) {
        return false;
    }

    span_ms = latest_ms - earliest_ms;
    if (span_ms < params->interval_ms) {
        span_ms = params->interval_ms;
    }

    avg_speed = (int32_t)(((int64_t)(iqs9151_abs32(total_x) + iqs9151_abs32(total_y)) *
                           params->interval_ms) /
                          span_ms);
    if (avg_speed < gate->min_avg_speed) {
        return false;
    }

    *seed_vx_fp = (int32_t)((((int64_t)total_x * params->interval_ms) << params->fp_shift) /
                            span_ms);
    *seed_vy_fp = (int32_t)((((int64_t)total_y * params->interval_ms) << params->fp_shift) /
                            span_ms);
    return true;
}

static void iqs9151_inertia_state_reset(struct iqs9151_inertia_state *state) {
    state->active = false;
    state->vx_fp = 0;
    state->vy_fp = 0;
    state->accum_x_fp = 0;
    state->accum_y_fp = 0;
    state->last_ms = 0;
    state->elapsed_ms = 0U;
}

static void iqs9151_inertia_cancel(struct iqs9151_inertia_state *state,
                                   struct k_work_delayable *work) {
    iqs9151_inertia_state_reset(state);
    k_work_cancel_delayable(work);
}

static void iqs9151_release_hold(struct iqs9151_data *data, const struct device *dev) {
    if (data->hold_button == 0U) {
        return;
    }

    iqs9151_report_key_event(dev, data->hold_button, false, true, K_NO_WAIT);
    data->hold_button = 0U;
}

static void iqs9151_clear_one_finger_click_pending(struct iqs9151_data *data) {
    data->one_finger_click_pending = false;
    data->one_finger_click_pending_ms = 0;
}

static void iqs9151_clear_two_finger_click_pending(struct iqs9151_data *data) {
    data->two_finger_click_pending = false;
    data->two_finger_click_pending_ms = 0;
}

static void iqs9151_clear_three_finger_click_pending(struct iqs9151_data *data) {
    data->three_finger_click_pending = false;
    data->three_finger_click_pending_ms = 0;
}

static void iqs9151_reset_finger_history(struct iqs9151_data *data) {
    memset(data->finger_history, 0, sizeof(data->finger_history));
    data->finger_history_head = 0U;
    data->finger_history_count = 0U;
}

static void iqs9151_push_finger_history(struct iqs9151_data *data,
                                        uint8_t finger_count,
                                        int64_t now_ms) {
    struct iqs9151_finger_history_entry *entry =
        &data->finger_history[data->finger_history_head];

    entry->ms = now_ms;
    entry->finger_count = finger_count;

    data->finger_history_head =
        (uint8_t)((data->finger_history_head + 1U) % IQS9151_FINGER_HISTORY_SIZE);
    if (data->finger_history_count < IQS9151_FINGER_HISTORY_SIZE) {
        data->finger_history_count++;
    }
}

static bool iqs9151_has_recent_finger_count(const struct iqs9151_data *data,
                                            uint8_t finger_count,
                                            int64_t now_ms,
                                            int32_t window_ms) {
    for (uint8_t i = 0U; i < data->finger_history_count; i++) {
        const uint8_t idx =
            (uint8_t)((data->finger_history_head + IQS9151_FINGER_HISTORY_SIZE - 1U - i) %
                      IQS9151_FINGER_HISTORY_SIZE);
        const struct iqs9151_finger_history_entry *entry = &data->finger_history[idx];
        const int64_t elapsed_ms = now_ms - entry->ms;

        if (elapsed_ms > window_ms) {
            break;
        }
        if (entry->finger_count == finger_count) {
            return true;
        }
    }

    return false;
}

static bool iqs9151_try_tap_hold_emit(struct iqs9151_data *data,
                                      const struct device *dev) {
    if (data->hold_button == 0U) {
        return true;
    }

    /*
     * Hold is latched until another Tap/Hold condition is recognized.
     * In that case, release the current hold and suppress the new event.
     */
    iqs9151_release_hold(data, dev);
    return false;
}

static bool iqs9151_emit_click(struct iqs9151_data *data,
                               const struct device *dev,
                               uint16_t button) {
    if (!iqs9151_try_tap_hold_emit(data, dev)) {
        return false;
    }

    iqs9151_report_key_event(dev, button, true, true, K_FOREVER);
    iqs9151_report_key_event(dev, button, false, true, K_FOREVER);
    return true;
}

static bool iqs9151_emit_hold_press(struct iqs9151_data *data,
                                    const struct device *dev,
                                    uint16_t button) {
    if (!iqs9151_try_tap_hold_emit(data, dev)) {
        return false;
    }

    iqs9151_report_key_event(dev, button, true, true, K_FOREVER);
    data->hold_button = button;
    return true;
}

static bool iqs9151_get_finger1_xy(const struct iqs9151_frame *frame,
                                   const struct iqs9151_frame *prev_frame,
                                   uint16_t *x, uint16_t *y) {
    if (iqs9151_finger1_valid(frame)) {
        *x = frame->finger1_x;
        *y = frame->finger1_y;
        return true;
    }

    if (iqs9151_finger1_valid(prev_frame)) {
        *x = prev_frame->finger1_x;
        *y = prev_frame->finger1_y;
        return true;
    }

    return false;
}

static bool iqs9151_get_finger2_xy(const struct iqs9151_frame *frame,
                                   const struct iqs9151_frame *prev_frame,
                                   uint16_t *x, uint16_t *y) {
    if (iqs9151_finger2_valid(frame)) {
        *x = frame->finger2_x;
        *y = frame->finger2_y;
        return true;
    }

    if (iqs9151_finger2_valid(prev_frame)) {
        *x = prev_frame->finger2_x;
        *y = prev_frame->finger2_y;
        return true;
    }

    return false;
}

static int32_t iqs9151_two_finger_distance(uint16_t x1, uint16_t y1,
                                           uint16_t x2, uint16_t y2) {
    const int32_t dx = (int32_t)x1 - (int32_t)x2;
    const int32_t dy = (int32_t)y1 - (int32_t)y2;

    return iqs9151_abs32(dx) + iqs9151_abs32(dy);
}

static void iqs9151_one_finger_reset(struct iqs9151_one_finger_state *state) {
    state->active = false;
    state->hold_sent = false;
    state->tap_candidate = false;
    state->hold_candidate = false;
    state->tapdrag_second_touch = false;
    state->down_ms = 0;
    state->dx = 0;
    state->dy = 0;
    state->last_x = 0;
    state->last_y = 0;
}

static void iqs9151_two_finger_reset(struct iqs9151_two_finger_state *state) {
    state->active = false;
    state->hold_sent = false;
    state->tap_candidate = false;
    state->hold_candidate = false;
    state->tapdrag_second_touch = false;
    state->release_pending = false;
    state->down_ms = 0;
    state->release_pending_ms = 0;
    state->centroid_dx = 0;
    state->centroid_dy = 0;
    state->distance_delta = 0;
    state->centroid_last_x = 0;
    state->centroid_last_y = 0;
    state->distance_last = 0;
    state->pinch_wheel_remainder = 0;
    state->scroll_lock_x = false;
    state->scroll_lock_y = false;
    /* pinch_enabled/pinch_invert are not cleared: they are re-sampled at the
     * next touch-down, and leaving the last known answer in place keeps a
     * stray read between gestures harmless. */
    state->mode = IQS9151_2F_MODE_NONE;
}

static void iqs9151_two_finger_result_reset(struct iqs9151_two_finger_result *result) {
    memset(result, 0, sizeof(*result));
}

static bool iqs9151_one_finger_update(struct iqs9151_data *data,
                                      const struct iqs9151_frame *frame,
                                      const struct iqs9151_frame *prev_frame,
                                      const struct device *dev) {
    struct iqs9151_one_finger_state *state = &data->one_finger;
    const bool one_now = frame->finger_count == 1U;
    const int64_t now_ms = k_uptime_get();
    uint16_t x = 0U;
    uint16_t y = 0U;
    const bool have_xy = one_now && iqs9151_get_finger1_xy(frame, prev_frame, &x, &y);
    bool released_from_hold = false;
    bool tap_detected = false;
    bool tap_emitted = false;

    if (!state->active && one_now) {
        bool tapdrag_second_touch = false;

        if (!have_xy) {
            return false;
        }
        if (data->one_finger_click_pending) {
            const int64_t armed_elapsed_ms =
                now_ms - data->one_finger_click_pending_ms;

            tapdrag_second_touch = (armed_elapsed_ms >= 0) &&
                                   (armed_elapsed_ms <= ONE_FINGER_CLICK_HOLD_MAX_MS);
            if (!tapdrag_second_touch && data->hold_button == INPUT_BTN_0) {
                iqs9151_release_hold(data, dev);
            }
            iqs9151_clear_one_finger_click_pending(data);
            (void)k_work_cancel_delayable(&data->one_finger_click_work);
        }
        state->active = true;
        state->hold_sent = tapdrag_second_touch;
        state->tap_candidate = !tapdrag_second_touch &&
            ((prev_frame->finger_count == 0U) ||
             iqs9151_has_recent_finger_count(data, 0U, now_ms, IQS9151_TAP_REENTRY_WINDOW_MS));
        state->hold_candidate = tapdrag_second_touch;
        state->tapdrag_second_touch = tapdrag_second_touch;
        state->down_ms = now_ms;
        state->dx = 0;
        state->dy = 0;
        state->last_x = x;
        state->last_y = y;
    }

    if (!state->active) {
        return false;
    }

    if (one_now) {
        const int64_t elapsed_ms = now_ms - state->down_ms;

        if (have_xy) {
            const int32_t step_x = (int32_t)x - (int32_t)state->last_x;
            const int32_t step_y = (int32_t)y - (int32_t)state->last_y;
            state->dx += step_x;
            state->dy += step_y;
            state->last_x = x;
            state->last_y = y;
        }

        if (state->tap_candidate &&
            (elapsed_ms > ONE_FINGER_TAP_MAX_MS ||
             iqs9151_abs32(state->dx) > ONE_FINGER_TAP_MOVE ||
             iqs9151_abs32(state->dy) > ONE_FINGER_TAP_MOVE)) {
            state->tap_candidate = false;
        }
        if (state->tapdrag_second_touch && state->hold_candidate &&
            (elapsed_ms > ONE_FINGER_TAP_MAX_MS ||
             iqs9151_abs32(state->dx) > ONE_FINGER_TAP_MOVE ||
             iqs9151_abs32(state->dy) > ONE_FINGER_TAP_MOVE)) {
            state->hold_candidate = false;
        }
        return false;
    }

    if (state->tapdrag_second_touch) {
        const int64_t elapsed_ms = now_ms - state->down_ms;
        const bool second_tap_detected =
            (frame->finger_count == 0U) &&
            state->hold_candidate &&
            elapsed_ms <= ONE_FINGER_TAP_MAX_MS &&
            iqs9151_abs32(state->dx) <= ONE_FINGER_TAP_MOVE &&
            iqs9151_abs32(state->dy) <= ONE_FINGER_TAP_MOVE;

        released_from_hold = state->hold_sent;
        if (state->hold_sent) {
            iqs9151_release_hold(data, dev);
        }
        if (second_tap_detected &&
            IS_ENABLED(CONFIG_INPUT_IQS9151_1F_TAP_ENABLE)) {
            (void)iqs9151_emit_click(data, dev, INPUT_BTN_0);
        }
        iqs9151_one_finger_reset(state);
        return released_from_hold;
    }

    if (frame->finger_count == 0U && state->tap_candidate) {
        const int64_t elapsed_ms = now_ms - state->down_ms;

        if (elapsed_ms <= ONE_FINGER_TAP_MAX_MS &&
            iqs9151_abs32(state->dx) <= ONE_FINGER_TAP_MOVE &&
            iqs9151_abs32(state->dy) <= ONE_FINGER_TAP_MOVE) {
            tap_detected = true;
            if (IS_ENABLED(CONFIG_INPUT_IQS9151_1F_PRESSHOLD_ENABLE)) {
                tap_emitted = iqs9151_emit_hold_press(data, dev, INPUT_BTN_0);
            } else if (IS_ENABLED(CONFIG_INPUT_IQS9151_1F_TAP_ENABLE)) {
                tap_emitted = iqs9151_emit_click(data, dev, INPUT_BTN_0);
            } else {
                tap_emitted = true;
            }
        }
    }

    if (tap_detected &&
        tap_emitted &&
        IS_ENABLED(CONFIG_INPUT_IQS9151_1F_PRESSHOLD_ENABLE)) {
        data->one_finger_click_pending = true;
        data->one_finger_click_pending_ms = now_ms;
        k_work_reschedule(&data->one_finger_click_work,
                          K_MSEC(ONE_FINGER_CLICK_HOLD_MAX_MS));
    } else if (frame->finger_count != 0U) {
        iqs9151_clear_one_finger_click_pending(data);
        (void)k_work_cancel_delayable(&data->one_finger_click_work);
    }

    iqs9151_one_finger_reset(state);
    return released_from_hold;
}

static void iqs9151_two_finger_update(struct iqs9151_data *data,
                                      const struct iqs9151_frame *frame,
                                      const struct iqs9151_frame *prev_frame,
                                      const struct device *dev,
                                      struct iqs9151_two_finger_result *result) {
    struct iqs9151_two_finger_state *state = &data->two_finger;
    const bool two_now = frame->finger_count == 2U;
    const bool one_lead_tap_candidate = data->two_finger_one_lead_valid;
    const int64_t now_ms = k_uptime_get();
    uint16_t f1x = 0U;
    uint16_t f1y = 0U;
    uint16_t f2x = 0U;
    uint16_t f2y = 0U;
    const bool have_xy = two_now &&
        iqs9151_get_finger1_xy(frame, prev_frame, &f1x, &f1y) &&
        iqs9151_get_finger2_xy(frame, prev_frame, &f2x, &f2y);
    bool tap_detected = false;
    bool tap_emitted = false;

    iqs9151_two_finger_result_reset(result);

    if (!state->active && two_now) {
        bool tapdrag_second_touch = false;

        if (!have_xy) {
            return;
        }
        if (data->two_finger_click_pending) {
            const int64_t armed_elapsed_ms =
                now_ms - data->two_finger_click_pending_ms;

            tapdrag_second_touch = (armed_elapsed_ms >= 0) &&
                                   (armed_elapsed_ms <= TWO_FINGER_CLICK_HOLD_MAX_MS);
            if (!tapdrag_second_touch && data->hold_button == INPUT_BTN_1) {
                iqs9151_release_hold(data, dev);
            }
            iqs9151_clear_two_finger_click_pending(data);
            (void)k_work_cancel_delayable(&data->two_finger_click_work);
        }
        state->active = true;
        state->hold_sent = tapdrag_second_touch;
        state->tap_candidate = !tapdrag_second_touch &&
            ((prev_frame->finger_count == 0U) ||
             iqs9151_has_recent_finger_count(data, 0U, now_ms, IQS9151_TAP_REENTRY_WINDOW_MS) ||
             one_lead_tap_candidate);
        state->hold_candidate = tapdrag_second_touch;
        state->tapdrag_second_touch = tapdrag_second_touch;
        state->release_pending = false;
        state->down_ms = now_ms;
        state->release_pending_ms = 0;
        state->centroid_dx = 0;
        state->centroid_dy = 0;
        state->distance_delta = 0;
        state->pinch_wheel_remainder = 0;
        state->pinch_enabled = iqs9151_setting_one_hand_pinch();
        state->pinch_invert = iqs9151_setting_pinch_invert();
        state->mode = IQS9151_2F_MODE_NONE;
        if (have_xy) {
            state->centroid_last_x = ((int32_t)f1x + (int32_t)f2x) / 2;
            state->centroid_last_y = ((int32_t)f1y + (int32_t)f2y) / 2;
            state->distance_last = iqs9151_two_finger_distance(f1x, f1y, f2x, f2y);
        }
    }

    if (!state->active) {
        data->two_finger_one_lead_valid = false;
        return;
    }

    data->two_finger_one_lead_valid = false;

    if (two_now) {
        const int64_t elapsed_ms = now_ms - state->down_ms;
        int32_t step_x = 0;
        int32_t step_y = 0;
        int32_t step_dist = 0;

        if (state->release_pending) {
            state->release_pending = false;
            state->release_pending_ms = 0;
        }

        if (have_xy) {
            const int32_t center_x = ((int32_t)f1x + (int32_t)f2x) / 2;
            const int32_t center_y = ((int32_t)f1y + (int32_t)f2y) / 2;
            const int32_t distance = iqs9151_two_finger_distance(f1x, f1y, f2x, f2y);

            step_x = center_x - state->centroid_last_x;
            step_y = center_y - state->centroid_last_y;
            step_dist = distance - state->distance_last;

            state->centroid_last_x = center_x;
            state->centroid_last_y = center_y;
            state->distance_last = distance;
            state->centroid_dx += step_x;
            state->centroid_dy += step_y;
            state->distance_delta += step_dist;
        }

        if (state->tap_candidate &&
            (elapsed_ms > TWO_FINGER_TAP_MAX_MS ||
             iqs9151_abs32(state->centroid_dx) > TWO_FINGER_TAP_MOVE ||
             iqs9151_abs32(state->centroid_dy) > TWO_FINGER_TAP_MOVE ||
             iqs9151_abs32(state->distance_delta) > TWO_FINGER_TAP_MOVE)) {
            state->tap_candidate = false;
        }
        if (state->tapdrag_second_touch && state->hold_candidate &&
            (elapsed_ms > TWO_FINGER_TAP_MAX_MS ||
             iqs9151_abs32(state->centroid_dx) > TWO_FINGER_TAP_MOVE ||
             iqs9151_abs32(state->centroid_dy) > TWO_FINGER_TAP_MOVE ||
             iqs9151_abs32(state->distance_delta) > TWO_FINGER_TAP_MOVE)) {
            state->hold_candidate = false;
        }
        if (state->tapdrag_second_touch) {
            return;
        }

        if (state->mode == IQS9151_2F_MODE_NONE) {
            /* dy is the sensor's Y, which is the screen's horizontal here. */
            const int32_t abs_dx = iqs9151_abs32(state->centroid_dx);
            const int32_t abs_dy = iqs9151_abs32(state->centroid_dy);
            const int32_t abs_center = MAX(abs_dx, abs_dy);
            const int32_t abs_dist = iqs9151_abs32(state->distance_delta);
            const bool scroll_enabled = IS_ENABLED(CONFIG_INPUT_IQS9151_SCROLL_X_ENABLE) ||
                                        IS_ENABLED(CONFIG_INPUT_IQS9151_SCROLL_Y_ENABLE);
            /*
             * Sideways enough that this cannot be a scroll going slightly
             * crooked. While it holds, scroll is not allowed to start: the
             * swipe has to be given room to finish, and a movement that turns
             * out to be vertical after all stops being sideways-dominant and
             * scrolls as usual.
             */
            const bool sideways =
                IS_ENABLED(CONFIG_INPUT_IQS9151_2F_SWIPE_ENABLE) &&
                ((int64_t)abs_dy * 10 >
                 (int64_t)abs_dx * TWO_FINGER_SWIPE_DOMINANCE_X10);

            if (sideways && abs_dy >= TWO_FINGER_SWIPE_THRESHOLD) {
                const int32_t swipe = IQS9151_SWIPE_SIGN_Y * state->centroid_dy;

                state->mode = IQS9151_2F_MODE_SWIPE;
                result->swipe_code =
                    (swipe < 0) ? IQS9151_2F_SWIPE_CODE_A : IQS9151_2F_SWIPE_CODE_B;
                state->tap_candidate = false;
            } else if (scroll_enabled && !sideways &&
                       abs_center >= TWO_FINGER_SCROLL_START_MOVE) {
                /*
                 * Pick the axis now, once, and keep it. A hand reaching across
                 * from the home row meets the pad at an angle, so a scroll the
                 * user means as vertical arrives as a diagonal -- and letting
                 * both axes through for the whole gesture is what makes the
                 * page drift sideways while they scroll down.
                 */
                const int64_t lock = TWO_FINGER_SCROLL_AXIS_LOCK_X10;

                state->scroll_lock_x =
                    lock > 0 && (int64_t)abs_dx * 10 >= (int64_t)abs_dy * lock;
                state->scroll_lock_y =
                    lock > 0 && (int64_t)abs_dy * 10 >= (int64_t)abs_dx * lock;
                state->mode = IQS9151_2F_MODE_SCROLL;
                result->scroll_started = true;
                state->tap_candidate = false;
            } else if (state->pinch_enabled &&
                       abs_dist >= TWO_FINGER_PINCH_START_DISTANCE &&
                       (int64_t)abs_dist * 10 >
                           (int64_t)abs_center * TWO_FINGER_PINCH_DOMINANCE_X10) {
                state->mode = IQS9151_2F_MODE_PINCH;
                result->pinch_started = true;
                state->tap_candidate = false;
            }
        }

        if (state->mode == IQS9151_2F_MODE_SCROLL) {
            result->scroll_active = true;
            /* Both false means the gesture was diagonal enough to be meant
             * that way, and both axes pass through as they always did. */
            if (IS_ENABLED(CONFIG_INPUT_IQS9151_SCROLL_X_ENABLE) &&
                !state->scroll_lock_y) {
                result->scroll_x = (int16_t)CLAMP(step_x, INT16_MIN, INT16_MAX);
            }
            if (IS_ENABLED(CONFIG_INPUT_IQS9151_SCROLL_Y_ENABLE) &&
                !state->scroll_lock_x) {
                result->scroll_y = (int16_t)CLAMP(step_y, INT16_MIN, INT16_MAX);
            }
        } else if (state->mode == IQS9151_2F_MODE_PINCH) {
            const int32_t wheel_div =
                TWO_FINGER_PINCH_WHEEL_DIV * TWO_FINGER_PINCH_WHEEL_GAIN_DEN;
            const int32_t wheel_acc =
                state->pinch_wheel_remainder +
                (step_dist * TWO_FINGER_PINCH_WHEEL_GAIN_X10);
            const int32_t wheel = wheel_acc / wheel_div;

            state->pinch_wheel_remainder =
                wheel_acc - (wheel * wheel_div);
            result->pinch_active = true;
            /* The direction is the host's convention, so it is the owner's to
             * set; the sign goes on here rather than at the emit site because
             * this is where the sampled setting lives. */
            result->pinch_wheel =
                (int16_t)CLAMP(state->pinch_invert ? -wheel : wheel, INT16_MIN, INT16_MAX);
        }
        return;
    }

    if (state->mode == IQS9151_2F_MODE_SCROLL) {
        result->scroll_ended = true;
    } else if (state->mode == IQS9151_2F_MODE_PINCH) {
        result->pinch_ended = true;
    }

    if (state->tapdrag_second_touch) {
        const int64_t elapsed_ms = now_ms - state->down_ms;
        const bool second_tap_detected =
            (frame->finger_count == 0U) &&
            state->hold_candidate &&
            elapsed_ms <= TWO_FINGER_TAP_MAX_MS &&
            iqs9151_abs32(state->centroid_dx) <= TWO_FINGER_TAP_MOVE &&
            iqs9151_abs32(state->centroid_dy) <= TWO_FINGER_TAP_MOVE &&
            iqs9151_abs32(state->distance_delta) <= TWO_FINGER_TAP_MOVE;

        if (frame->finger_count > 0U) {
            state->hold_candidate = false;
            return;
        }

        if (state->hold_sent) {
            iqs9151_release_hold(data, dev);
        }
        if (second_tap_detected &&
            IS_ENABLED(CONFIG_INPUT_IQS9151_2F_TAP_ENABLE)) {
            (void)iqs9151_emit_click(data, dev, INPUT_BTN_1);
        }

        iqs9151_two_finger_reset(state);
        return;
    }

    if (state->release_pending) {
        const int64_t pending_ms = now_ms - state->release_pending_ms;

        if (frame->finger_count == 1U &&
            pending_ms <= TWO_FINGER_RELEASE_PENDING_MAX_MS) {
            return;
        }

        if (frame->finger_count == 0U &&
            pending_ms <= TWO_FINGER_RELEASE_PENDING_MAX_MS &&
            !state->hold_sent &&
            state->mode == IQS9151_2F_MODE_NONE &&
            state->tap_candidate) {
            tap_detected = true;
        }

        if (tap_detected) {
            if (IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PRESSHOLD_ENABLE)) {
                tap_emitted = iqs9151_emit_hold_press(data, dev, INPUT_BTN_1);
            } else if (IS_ENABLED(CONFIG_INPUT_IQS9151_2F_TAP_ENABLE)) {
                tap_emitted = iqs9151_emit_click(data, dev, INPUT_BTN_1);
            } else {
                tap_emitted = true;
            }
        }
        if (tap_detected &&
            tap_emitted &&
            IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PRESSHOLD_ENABLE)) {
            data->two_finger_click_pending = true;
            data->two_finger_click_pending_ms = now_ms;
            k_work_reschedule(&data->two_finger_click_work,
                              K_MSEC(TWO_FINGER_CLICK_HOLD_MAX_MS));
        }

        iqs9151_two_finger_reset(state);
        return;
    }

    if (!state->hold_sent &&
        state->mode == IQS9151_2F_MODE_NONE && state->tap_candidate &&
        IS_ENABLED(CONFIG_INPUT_IQS9151_2F_TAP_ENABLE)) {
        const int64_t elapsed_ms = now_ms - state->down_ms;

        if (frame->finger_count == 1U &&
            elapsed_ms <= TWO_FINGER_TAP_MAX_MS &&
            iqs9151_abs32(state->centroid_dx) <= TWO_FINGER_TAP_MOVE &&
            iqs9151_abs32(state->centroid_dy) <= TWO_FINGER_TAP_MOVE &&
            iqs9151_abs32(state->distance_delta) <= TWO_FINGER_TAP_MOVE) {
            state->release_pending = true;
            state->release_pending_ms = now_ms;
            return;
        }

        if (frame->finger_count == 0U &&
            elapsed_ms <= TWO_FINGER_TAP_MAX_MS &&
            iqs9151_abs32(state->centroid_dx) <= TWO_FINGER_TAP_MOVE &&
            iqs9151_abs32(state->centroid_dy) <= TWO_FINGER_TAP_MOVE &&
            iqs9151_abs32(state->distance_delta) <= TWO_FINGER_TAP_MOVE) {
            tap_detected = true;
        }
    }

    if (tap_detected) {
        if (IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PRESSHOLD_ENABLE)) {
            tap_emitted = iqs9151_emit_hold_press(data, dev, INPUT_BTN_1);
        } else if (IS_ENABLED(CONFIG_INPUT_IQS9151_2F_TAP_ENABLE)) {
            tap_emitted = iqs9151_emit_click(data, dev, INPUT_BTN_1);
        } else {
            tap_emitted = true;
        }
    }
    if (tap_detected &&
        tap_emitted &&
        IS_ENABLED(CONFIG_INPUT_IQS9151_2F_PRESSHOLD_ENABLE)) {
        data->two_finger_click_pending = true;
        data->two_finger_click_pending_ms = now_ms;
        k_work_reschedule(&data->two_finger_click_work,
                          K_MSEC(TWO_FINGER_CLICK_HOLD_MAX_MS));
    }

    iqs9151_two_finger_reset(state);
}

static void iqs9151_three_finger_reset(struct iqs9151_data *data) {
    data->three_active = false;
    data->three_hold_sent = false;
    data->three_swipe_sent = false;
    data->three_tap_candidate = false;
    data->three_hold_candidate = false;
    data->three_tapdrag_second_touch = false;
    data->three_release_pending = false;
    data->three_have_last = false;
    data->three_down_ms = 0;
    data->three_release_pending_ms = 0;
    data->three_dx = 0;
    data->three_dy = 0;
    data->three_last_x = 0;
    data->three_last_y = 0;
    data->three_flicker_ms = 0;
}

static atomic_t iqs9151_swipe3_threshold_x = ATOMIC_INIT(THREE_FINGER_SWIPE_THRESHOLD_X);
static atomic_t iqs9151_swipe3_threshold_y = ATOMIC_INIT(THREE_FINGER_SWIPE_THRESHOLD_Y);

int iqs9151_set_swipe3_threshold(uint16_t x_counts, uint16_t y_counts) {
    if (x_counts == 0U || y_counts == 0U) {
        return -EINVAL;
    }
    atomic_set(&iqs9151_swipe3_threshold_x, (atomic_val_t)x_counts);
    atomic_set(&iqs9151_swipe3_threshold_y, (atomic_val_t)y_counts);
    return 0;
}

static bool iqs9151_three_finger_update(struct iqs9151_data *data,
                                        const struct iqs9151_frame *frame,
                                        const struct iqs9151_frame *prev_frame,
                                        const struct device *dev) {
    const bool finger1_valid = iqs9151_finger1_valid(frame);
    const bool one_lead_tap_candidate = data->three_finger_one_lead_valid;
    const bool two_lead_tap_candidate = data->three_finger_two_lead_valid;
    const int64_t now_ms = k_uptime_get();
    bool tap_detected = false;
    bool tap_emitted = false;

    if (!data->three_active && frame->finger_count == 3U) {
        bool tapdrag_second_touch = false;

        if (data->three_finger_click_pending) {
            const int64_t armed_elapsed_ms =
                now_ms - data->three_finger_click_pending_ms;

            tapdrag_second_touch = (armed_elapsed_ms >= 0) &&
                                   (armed_elapsed_ms <= THREE_FINGER_CLICK_HOLD_MAX_MS);
            if (!tapdrag_second_touch && data->hold_button == INPUT_BTN_2) {
                iqs9151_release_hold(data, dev);
            }
            iqs9151_clear_three_finger_click_pending(data);
            (void)k_work_cancel_delayable(&data->three_finger_click_work);
        }
        data->three_active = true;
        data->three_hold_sent = tapdrag_second_touch;
        data->three_swipe_sent = false;
        data->three_tap_candidate = !tapdrag_second_touch &&
            ((prev_frame->finger_count == 0U) ||
             iqs9151_has_recent_finger_count(data, 0U, now_ms,
                                             IQS9151_TAP_REENTRY_WINDOW_MS) ||
             one_lead_tap_candidate ||
             two_lead_tap_candidate);
        data->three_hold_candidate = tapdrag_second_touch;
        data->three_tapdrag_second_touch = tapdrag_second_touch;
        data->three_release_pending = false;
        data->three_have_last = false;
        data->three_down_ms = now_ms;
        data->three_release_pending_ms = 0;
        data->three_dx = 0;
        data->three_dy = 0;
        if (finger1_valid) {
            data->three_last_x = frame->finger1_x;
            data->three_last_y = frame->finger1_y;
            data->three_have_last = true;
        } else if (iqs9151_finger1_valid(prev_frame)) {
            data->three_last_x = prev_frame->finger1_x;
            data->three_last_y = prev_frame->finger1_y;
            data->three_have_last = true;
        }
    }

    if (!data->three_active) {
        data->three_finger_one_lead_valid = false;
        data->three_finger_two_lead_valid = false;
        return false;
    }

    data->three_finger_one_lead_valid = false;
    data->three_finger_two_lead_valid = false;

    /*
     * A frame at two (or one) fingers inside the grace window is treated as
     * the three-finger frame it almost certainly is. The tap paths below are
     * not affected: they only apply to a gesture that has not moved, and this
     * only holds a gesture that is being held or swiped.
     */
    bool flicker_frame = false;
    if (data->three_active && frame->finger_count > 0U && frame->finger_count < 3U &&
        !data->three_tapdrag_second_touch && !data->three_tap_candidate) {
        if (data->three_flicker_ms == 0) {
            data->three_flicker_ms = now_ms;
        }
        flicker_frame = (now_ms - data->three_flicker_ms) <= THREE_FINGER_FLICKER_GRACE_MS;
    }

    if (frame->finger_count == 3U || flicker_frame) {
        const int64_t elapsed = now_ms - data->three_down_ms;

        if (frame->finger_count == 3U) {
            data->three_flicker_ms = 0;
        }
        if (data->three_release_pending) {
            data->three_release_pending = false;
            data->three_release_pending_ms = 0;
        }

        /* Differences of the first finger's absolute position. (The device's
         * relative report is not an option: it is the cursor's, and reads
         * zero with more than one finger down.) When fingers merge and split
         * the device renumbers them, and one difference then jumps by the
         * spacing between two fingers -- far more than a finger moves in
         * 5 ms -- so a jump that size is a renumbering, not a swipe, and is
         * left out. */
        if (finger1_valid) {
            if (data->three_have_last) {
                const int32_t dx = (int32_t)frame->finger1_x - (int32_t)data->three_last_x;
                const int32_t dy = (int32_t)frame->finger1_y - (int32_t)data->three_last_y;
                if (iqs9151_abs32(dx) <= THREE_FINGER_RENUMBER_JUMP &&
                    iqs9151_abs32(dy) <= THREE_FINGER_RENUMBER_JUMP) {
                    data->three_dx += dx;
                    data->three_dy += dy;
                }
            }
            data->three_last_x = frame->finger1_x;
            data->three_last_y = frame->finger1_y;
            data->three_have_last = true;
        }

        if (data->three_tap_candidate &&
            (elapsed > THREE_FINGER_TAP_MAX_MS ||
             iqs9151_abs32(data->three_dx) > THREE_FINGER_TAP_MOVE ||
             iqs9151_abs32(data->three_dy) > THREE_FINGER_TAP_MOVE)) {
            data->three_tap_candidate = false;
        }
        if (data->three_tapdrag_second_touch && data->three_hold_candidate &&
            (elapsed > THREE_FINGER_TAP_MAX_MS ||
             iqs9151_abs32(data->three_dx) > THREE_FINGER_TAP_MOVE ||
             iqs9151_abs32(data->three_dy) > THREE_FINGER_TAP_MOVE)) {
            data->three_hold_candidate = false;
        }
        if (data->three_tapdrag_second_touch) {
            return true;
        }

        if (!data->three_swipe_sent && !data->three_hold_sent) {
            const int32_t swipe_dx = IQS9151_SWIPE_SIGN_X * data->three_dx;
            const int32_t swipe_dy = IQS9151_SWIPE_SIGN_Y * data->three_dy;
            const int32_t threshold_x = (int32_t)atomic_get(&iqs9151_swipe3_threshold_x);
            const int32_t threshold_y = (int32_t)atomic_get(&iqs9151_swipe3_threshold_y);

            if (iqs9151_abs32(swipe_dx) >= threshold_x &&
                iqs9151_abs32(swipe_dx) >= iqs9151_abs32(swipe_dy)) {
                const uint16_t key = (swipe_dx < 0) ? INPUT_BTN_4 : INPUT_BTN_3;
                iqs9151_report_key_event(dev, key, true, true, K_FOREVER);
                iqs9151_report_key_event(dev, key, false, true, K_FOREVER);
                data->three_swipe_sent = true;
                return true;
            } else if (iqs9151_abs32(swipe_dy) >= threshold_y &&
                       iqs9151_abs32(swipe_dy) > iqs9151_abs32(swipe_dx)) {
                const uint16_t key = (swipe_dy < 0) ? INPUT_BTN_5 : INPUT_BTN_6;
                iqs9151_report_key_event(dev, key, true, true, K_FOREVER);
                iqs9151_report_key_event(dev, key, false, true, K_FOREVER);
                data->three_swipe_sent = true;
                return true;
            }
        }
        return true;
    }

    if (data->three_tapdrag_second_touch) {
        const int64_t elapsed = now_ms - data->three_down_ms;
        const bool second_tap_detected =
            (frame->finger_count == 0U) &&
            data->three_hold_candidate &&
            elapsed <= THREE_FINGER_TAP_MAX_MS &&
            iqs9151_abs32(data->three_dx) <= THREE_FINGER_TAP_MOVE &&
            iqs9151_abs32(data->three_dy) <= THREE_FINGER_TAP_MOVE;

        if (frame->finger_count > 0U) {
            data->three_hold_candidate = false;
            return true;
        }

        if (data->three_hold_sent) {
            iqs9151_release_hold(data, dev);
        }
        if (second_tap_detected &&
            IS_ENABLED(CONFIG_INPUT_IQS9151_3F_TAP_ENABLE)) {
            (void)iqs9151_emit_click(data, dev, INPUT_BTN_2);
        }

        iqs9151_three_finger_reset(data);
        return true;
    }

    if (data->three_release_pending) {
        const int64_t pending_ms = now_ms - data->three_release_pending_ms;

        if (frame->finger_count > 0U && frame->finger_count < 3U &&
            pending_ms <= THREE_FINGER_RELEASE_PENDING_MAX_MS) {
            return true;
        }

        if (frame->finger_count == 0U &&
            pending_ms <= THREE_FINGER_RELEASE_PENDING_MAX_MS &&
            !data->three_hold_sent && !data->three_swipe_sent &&
            data->three_tap_candidate) {
            const int64_t elapsed = now_ms - data->three_down_ms;
            if (elapsed <= THREE_FINGER_TAP_MAX_MS &&
                iqs9151_abs32(data->three_dx) <= THREE_FINGER_TAP_MOVE &&
                iqs9151_abs32(data->three_dy) <= THREE_FINGER_TAP_MOVE) {
                tap_detected = true;
            }
        }

        if (tap_detected) {
            if (IS_ENABLED(CONFIG_INPUT_IQS9151_3F_PRESSHOLD_ENABLE)) {
                tap_emitted = iqs9151_emit_hold_press(data, dev, INPUT_BTN_2);
            } else if (IS_ENABLED(CONFIG_INPUT_IQS9151_3F_TAP_ENABLE)) {
                tap_emitted = iqs9151_emit_click(data, dev, INPUT_BTN_2);
            } else {
                tap_emitted = true;
            }
        }
        if (tap_detected &&
            tap_emitted &&
            IS_ENABLED(CONFIG_INPUT_IQS9151_3F_PRESSHOLD_ENABLE)) {
            data->three_finger_click_pending = true;
            data->three_finger_click_pending_ms = now_ms;
            k_work_reschedule(&data->three_finger_click_work,
                              K_MSEC(THREE_FINGER_CLICK_HOLD_MAX_MS));
        }

        iqs9151_three_finger_reset(data);
        return true;
    }

    if (IS_ENABLED(CONFIG_INPUT_IQS9151_3F_TAP_ENABLE) &&
        !data->three_hold_sent && !data->three_swipe_sent &&
        data->three_tap_candidate) {
        const int64_t elapsed = now_ms - data->three_down_ms;

        if (frame->finger_count > 0U && frame->finger_count < 3U &&
            elapsed <= THREE_FINGER_TAP_MAX_MS &&
            iqs9151_abs32(data->three_dx) <= THREE_FINGER_TAP_MOVE &&
            iqs9151_abs32(data->three_dy) <= THREE_FINGER_TAP_MOVE) {
            data->three_release_pending = true;
            data->three_release_pending_ms = now_ms;
            return true;
        }

        if (frame->finger_count == 0U &&
            elapsed <= THREE_FINGER_TAP_MAX_MS &&
            iqs9151_abs32(data->three_dx) <= THREE_FINGER_TAP_MOVE &&
            iqs9151_abs32(data->three_dy) <= THREE_FINGER_TAP_MOVE) {
            tap_detected = true;
        }
    }

    if (tap_detected) {
        if (IS_ENABLED(CONFIG_INPUT_IQS9151_3F_PRESSHOLD_ENABLE)) {
            tap_emitted = iqs9151_emit_hold_press(data, dev, INPUT_BTN_2);
        } else if (IS_ENABLED(CONFIG_INPUT_IQS9151_3F_TAP_ENABLE)) {
            tap_emitted = iqs9151_emit_click(data, dev, INPUT_BTN_2);
        } else {
            tap_emitted = true;
        }
    }
    if (tap_detected &&
        tap_emitted &&
        IS_ENABLED(CONFIG_INPUT_IQS9151_3F_PRESSHOLD_ENABLE)) {
        data->three_finger_click_pending = true;
        data->three_finger_click_pending_ms = now_ms;
        k_work_reschedule(&data->three_finger_click_work,
                          K_MSEC(THREE_FINGER_CLICK_HOLD_MAX_MS));
    }

    iqs9151_three_finger_reset(data);
    return true;
}

static void iqs9151_reset_gesture_states(struct iqs9151_data *data,
                                         const struct device *dev,
                                         bool release_hold) {
    if (data->two_finger.active && data->two_finger.mode == IQS9151_2F_MODE_PINCH) {
        iqs9151_report_key_event(dev, INPUT_BTN_7, false, true, K_FOREVER);
    }
    if (release_hold) {
        iqs9151_release_hold(data, dev);
    }

    iqs9151_one_finger_reset(&data->one_finger);
    iqs9151_two_finger_reset(&data->two_finger);
    iqs9151_clear_one_finger_click_pending(data);
    iqs9151_clear_two_finger_click_pending(data);
    iqs9151_clear_three_finger_click_pending(data);
    (void)k_work_cancel_delayable(&data->one_finger_click_work);
    (void)k_work_cancel_delayable(&data->two_finger_click_work);
    (void)k_work_cancel_delayable(&data->three_finger_click_work);
    data->two_finger_one_lead_valid = false;
    data->two_finger_tail_suppresses_cursor = false;
    data->three_finger_one_lead_valid = false;
    data->three_finger_two_lead_valid = false;
    iqs9151_three_finger_reset(data);
    iqs9151_reset_finger_history(data);
}

static void iqs9151_inertia_start(struct iqs9151_inertia_state *state,
                                  struct k_work_delayable *work,
                                  const struct iqs9151_inertia_params *params,
                                  int32_t ema_vx_fp, int32_t ema_vy_fp) {
    const int32_t threshold_fp =
        (int32_t)params->start_threshold << params->fp_shift;
    const int32_t start_vx_fp =
        (iqs9151_abs32(ema_vx_fp) >= threshold_fp) ? ema_vx_fp : 0;
    const int32_t start_vy_fp =
        (iqs9151_abs32(ema_vy_fp) >= threshold_fp) ? ema_vy_fp : 0;

    if (start_vx_fp == 0 && start_vy_fp == 0) {
        return;
    }

    state->vx_fp = start_vx_fp;
    state->vy_fp = start_vy_fp;
    state->accum_x_fp = 0;
    state->accum_y_fp = 0;
    state->elapsed_ms = 0U;
    state->last_ms = k_uptime_get();
    state->active = true;
    k_work_schedule(work, K_MSEC(params->interval_ms));
}

static bool iqs9151_inertia_step(struct iqs9151_inertia_state *state,
                                 const struct iqs9151_inertia_params *params,
                                 int32_t *out_x, int32_t *out_y) {
    int64_t now;
    int64_t dt_ms;
    uint32_t steps;

    if (!state->active) {
        *out_x = 0;
        *out_y = 0;
        return false;
    }

    now = k_uptime_get();
    dt_ms = now - state->last_ms;
    if (dt_ms <= 0) {
        dt_ms = params->interval_ms;
    }
    steps = (uint32_t)((dt_ms + params->interval_ms - 1) / params->interval_ms);
    if (steps == 0U) {
        steps = 1U;
    }

    for (uint32_t i = 0U; i < steps; i++) {
        state->accum_x_fp += state->vx_fp;
        state->accum_y_fp += state->vy_fp;
        state->vx_fp = (state->vx_fp * params->decay_num) / params->decay_den;
        state->vy_fp = (state->vy_fp * params->decay_num) / params->decay_den;
    }

    state->last_ms = now;
    state->elapsed_ms += steps * params->interval_ms;

    *out_x = state->accum_x_fp >> params->fp_shift;
    *out_y = state->accum_y_fp >> params->fp_shift;
    state->accum_x_fp -= (*out_x) << params->fp_shift;
    state->accum_y_fp -= (*out_y) << params->fp_shift;

    const int32_t min_v_fp = (int32_t)(params->min_velocity << params->fp_shift);
    if (state->elapsed_ms >= params->max_duration_ms ||
        (iqs9151_abs32(state->vx_fp) < min_v_fp &&
         iqs9151_abs32(state->vy_fp) < min_v_fp)) {
        state->active = false;
        return false;
    }

    return true;
}

static void iqs9151_inertia_scroll_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs9151_data *data =
        CONTAINER_OF(dwork, struct iqs9151_data, inertia_scroll_work);
    const struct device *dev = data->dev;
    int32_t out_x;
    int32_t out_y;

    const bool active =
        iqs9151_inertia_step(&data->inertia_scroll, &iqs9151_scroll_params, &out_x, &out_y);

    if (out_x > INT16_MAX) {
        out_x = INT16_MAX;
    } else if (out_x < INT16_MIN) {
        out_x = INT16_MIN;
    }
    if (out_y > INT16_MAX) {
        out_y = INT16_MAX;
    } else if (out_y < INT16_MIN) {
        out_y = INT16_MIN;
    }

    const bool have_x = out_x != 0;
    const bool have_y = out_y != 0;
    if (have_x) {
        iqs9151_report_rel_event(dev, INPUT_REL_HWHEEL, (int16_t)(-out_x), !have_y, K_NO_WAIT);
    }
    if (have_y) {
        iqs9151_report_rel_event(dev, INPUT_REL_WHEEL, (int16_t)out_y, true, K_NO_WAIT);
    }

    if (active) {
        k_work_schedule(&data->inertia_scroll_work,
                        K_MSEC(iqs9151_scroll_params.interval_ms));
    }
}

static bool iqs9151_should_suppress_cursor_for_two_finger_tail(
    struct iqs9151_data *data,
    const struct iqs9151_frame *frame,
    const struct iqs9151_frame *prev_frame,
    const struct iqs9151_two_finger_result *two_result) {
    bool suppress = data->two_finger_tail_suppresses_cursor;

    if (prev_frame->finger_count == 2U && frame->finger_count == 1U &&
        (two_result->scroll_ended || two_result->pinch_ended)) {
        suppress = true;
        data->two_finger_tail_suppresses_cursor = true;
    }

    if (frame->finger_count == 0U || frame->finger_count >= 2U) {
        data->two_finger_tail_suppresses_cursor = false;
    }

    return suppress;
}

static void iqs9151_inertia_cursor_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs9151_data *data =
        CONTAINER_OF(dwork, struct iqs9151_data, inertia_cursor_work);
    const struct device *dev = data->dev;
    int32_t out_x;
    int32_t out_y;

    const bool active =
        iqs9151_inertia_step(&data->inertia_cursor, &iqs9151_cursor_params, &out_x, &out_y);

    if (out_x > INT16_MAX) {
        out_x = INT16_MAX;
    } else if (out_x < INT16_MIN) {
        out_x = INT16_MIN;
    }
    if (out_y > INT16_MAX) {
        out_y = INT16_MAX;
    } else if (out_y < INT16_MIN) {
        out_y = INT16_MIN;
    }

    const bool have_x = out_x != 0;
    const bool have_y = out_y != 0;
    if (have_x) {
        iqs9151_report_rel_event(dev, INPUT_REL_X, (int16_t)out_x, !have_y, K_NO_WAIT);
    }
    if (have_y) {
        iqs9151_report_rel_event(dev, INPUT_REL_Y, (int16_t)out_y, true, K_NO_WAIT);
    }

    if (active) {
        k_work_schedule(&data->inertia_cursor_work,
                        K_MSEC(iqs9151_cursor_params.interval_ms));
    }
}

static void iqs9151_one_finger_click_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs9151_data *data =
        CONTAINER_OF(dwork, struct iqs9151_data, one_finger_click_work);

    if (!data->one_finger_click_pending) {
        return;
    }

    iqs9151_clear_one_finger_click_pending(data);
    if (data->hold_button == INPUT_BTN_0) {
        iqs9151_release_hold(data, data->dev);
    }
}

static void iqs9151_two_finger_click_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs9151_data *data =
        CONTAINER_OF(dwork, struct iqs9151_data, two_finger_click_work);

    if (!data->two_finger_click_pending) {
        return;
    }

    iqs9151_clear_two_finger_click_pending(data);
    if (data->hold_button == INPUT_BTN_1) {
        iqs9151_release_hold(data, data->dev);
    }
}

static void iqs9151_three_finger_click_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs9151_data *data =
        CONTAINER_OF(dwork, struct iqs9151_data, three_finger_click_work);

    if (!data->three_finger_click_pending) {
        return;
    }

    iqs9151_clear_three_finger_click_pending(data);
    if (data->hold_button == INPUT_BTN_2) {
        iqs9151_release_hold(data, data->dev);
    }
}

static int iqs9151_read_frame(const struct iqs9151_config *cfg,
                              struct iqs9151_frame *frame) {
    uint8_t raw_frame[IQS9151_FRAME_READ_SIZE];
    int ret;

    /* Read RelativeX(0x1014) .. Finger2Y(0x102E) in one transaction. */
    ret = iqs9151_i2c_read(cfg, IQS9151_ADDR_RELATIVE_X, raw_frame, sizeof(raw_frame));
    if (ret != 0) {
        return ret;
    }

    iqs9151_parse_frame(raw_frame, frame);
    return 0;
}

#if IS_ENABLED(CONFIG_INPUT_IQS9151_TOUCH_STATE_ENABLE)

/*
 * Bounded, unlike the K_FOREVER the gesture events use. These fire on every
 * touch and release, so a full input queue here would block the work queue
 * this driver runs on - and on a split peripheral that stalls the half until
 * the watchdog resets it. The cached state is only updated once the report is
 * actually queued, so a dropped one is retried on the next frame instead of
 * leaving the two halves disagreeing about whether the pad is being touched.
 */
#define IQS9151_TOUCH_STATE_TIMEOUT K_MSEC(10)

/*
 * Report a plain "a finger is on the pad" key event, independent of any gesture
 * decision. Split keyboards relay input events (not raw coordinates) from the
 * peripheral half, so this is what lets the central half know that the other
 * pad is being touched at all - e.g. to hold a layer while the other hand is
 * resting on its trackpad.
 */
static void iqs9151_set_touch_state(struct iqs9151_data *data, uint8_t finger_count) {
    const bool touched = (finger_count > 0U);
    const bool multi = (finger_count >= 2U);

    if (data->touch_state_sent != touched) {
        int ret = iqs9151_report_key_event(data->dev,
                                           (uint16_t)CONFIG_INPUT_IQS9151_TOUCH_STATE_CODE,
                                           touched ? 1 : 0, true, IQS9151_TOUCH_STATE_TIMEOUT);
        if (ret == 0) {
            data->touch_state_sent = touched;
        } else {
            LOG_WRN("Touch state report dropped (%d), retrying on the next frame", ret);
        }
    }

    if (data->touch_state_2f_sent != multi) {
        int ret = iqs9151_report_key_event(data->dev,
                                           (uint16_t)CONFIG_INPUT_IQS9151_TOUCH_STATE_2F_CODE,
                                           multi ? 1 : 0, true, IQS9151_TOUCH_STATE_TIMEOUT);
        if (ret == 0) {
            data->touch_state_2f_sent = multi;
        } else {
            LOG_WRN("Two-finger touch state report dropped (%d), retrying on the next frame", ret);
        }
    }
}
#else
static inline void iqs9151_set_touch_state(struct iqs9151_data *data, uint8_t finger_count) {
    ARG_UNUSED(data);
    ARG_UNUSED(finger_count);
}
#endif

/* Defined next to init, which is where everything it needs to redo lives. */
static void iqs9151_request_recovery(struct iqs9151_data *data);

/*
 * The device telling us it has reset itself.
 *
 * SHOW_RESET means the IQS9151 restarted -- a brownout, or its own
 * communication timeout firing because nobody serviced the window it opened --
 * and a restarted device is back on its power-on defaults. Not only the tuning:
 * the Rx/Tx mapping, the disabled channels and the ATI compensation that make
 * these electrodes into *this* pad are gone too, so what it reports afterwards
 * is not a worse version of this trackpad, it is a different one. Spurious
 * clicks and floods of movement are what that looks like from the outside.
 *
 * This used to reset the driver's own bookkeeping and return, which left two
 * things undone. The flag is only cleared by acknowledging it, so every frame
 * after the first came back through here -- a warning per frame, the gesture
 * state torn down per frame, and no input ever processed again. And nothing
 * ever wrote the configuration back, so the pad stayed on its defaults until
 * the keyboard was next power-cycled. That is why changing a resolution in the
 * .conf could stop making any difference: the value was written at init and
 * then thrown away by the first reset, for the rest of that boot.
 *
 * So: claim the recovery once, tear down the stale state once, and hand the
 * rebuild to a thread of our own (see iqs9151_request_recovery).
 */
static bool iqs9151_handle_show_reset(struct iqs9151_data *data,
                                      const struct iqs9151_frame *frame) {
    const struct device *dev = data->dev;

    if ((frame->info_flags & IQS9151_INFO_SHOW_RESET) == 0U) {
        return false;
    }

    if (!atomic_cas(&data->recovering, 0, 1)) {
        /* Already rebuilding; frames until it finishes are not this pad's. */
        return true;
    }

    LOG_WRN("SHOW_RESET info=0x%04x: restoring configuration", frame->info_flags);
    iqs9151_set_touch_state(data, 0U);
    iqs9151_reset_gesture_states(data, dev, true);
    iqs9151_inertia_cancel(&data->inertia_scroll, &data->inertia_scroll_work);
    iqs9151_inertia_cancel(&data->inertia_cursor, &data->inertia_cursor_work);
    iqs9151_ema_reset(&data->scroll_ema_x_fp, &data->scroll_ema_y_fp);
    iqs9151_ema_reset(&data->cursor_ema_x_fp, &data->cursor_ema_y_fp);
    iqs9151_motion_history_reset(&data->scroll_motion_history);
    iqs9151_motion_history_reset(&data->cursor_motion_history);
    memset(&data->prev_frame, 0, sizeof(data->prev_frame));
    iqs9151_request_recovery(data);
    return true;
}

static bool iqs9151_update_gesture_sessions(struct iqs9151_data *data,
                                            const struct iqs9151_frame *frame,
                                            const struct iqs9151_frame *prev_frame,
                                            struct iqs9151_two_finger_result *two_result) {
    const struct device *dev = data->dev;
    bool released_from_hold = false;

    if (frame->finger_count > 1U && data->one_finger_click_pending) {
        if (data->hold_button == INPUT_BTN_0) {
            iqs9151_release_hold(data, dev);
            released_from_hold = true;
        }
        iqs9151_clear_one_finger_click_pending(data);
        (void)k_work_cancel_delayable(&data->one_finger_click_work);
    }
    if (frame->finger_count > 2U && data->two_finger_click_pending) {
        if (data->hold_button == INPUT_BTN_1) {
            iqs9151_release_hold(data, dev);
            released_from_hold = true;
        }
        iqs9151_clear_two_finger_click_pending(data);
        (void)k_work_cancel_delayable(&data->two_finger_click_work);
    }
    if (frame->finger_count != 0U &&
        frame->finger_count != 3U &&
        data->three_finger_click_pending) {
        const int64_t armed_elapsed_ms =
            k_uptime_get() - data->three_finger_click_pending_ms;

        if (armed_elapsed_ms > THREE_FINGER_CLICK_HOLD_MAX_MS) {
            if (data->hold_button == INPUT_BTN_2) {
                iqs9151_release_hold(data, dev);
                released_from_hold = true;
            }
            iqs9151_clear_three_finger_click_pending(data);
            (void)k_work_cancel_delayable(&data->three_finger_click_work);
        } else {
            /* Keep 3F deferred-click armed during staged 0->1->2->3 re-entry. */
            return released_from_hold;
        }
    }
    if (frame->finger_count == 3U && data->one_finger.active) {
        const int64_t elapsed_ms = k_uptime_get() - data->one_finger.down_ms;

        data->three_finger_one_lead_valid =
            !data->one_finger.hold_sent &&
            data->one_finger.tap_candidate &&
            elapsed_ms <= THREE_FINGER_ONE_LEAD_MAX_MS &&
            iqs9151_abs32(data->one_finger.dx) <= ONE_FINGER_TAP_MOVE &&
            iqs9151_abs32(data->one_finger.dy) <= ONE_FINGER_TAP_MOVE;
        if (data->one_finger.hold_sent) {
            iqs9151_release_hold(data, dev);
            released_from_hold = true;
        }
        iqs9151_one_finger_reset(&data->one_finger);
    } else {
        data->three_finger_one_lead_valid = false;
    }

    if (frame->finger_count == 3U && data->two_finger.active) {
        const int64_t elapsed_ms = k_uptime_get() - data->two_finger.down_ms;

        data->three_finger_two_lead_valid =
            !data->two_finger.hold_sent &&
            data->two_finger.mode == IQS9151_2F_MODE_NONE &&
            data->two_finger.tap_candidate &&
            elapsed_ms <= THREE_FINGER_TWO_LEAD_MAX_MS &&
            iqs9151_abs32(data->two_finger.centroid_dx) <= TWO_FINGER_TAP_MOVE &&
            iqs9151_abs32(data->two_finger.centroid_dy) <= TWO_FINGER_TAP_MOVE &&
            iqs9151_abs32(data->two_finger.distance_delta) <= TWO_FINGER_TAP_MOVE;

        if (data->two_finger.mode == IQS9151_2F_MODE_SCROLL) {
            two_result->scroll_ended = true;
        } else if (data->two_finger.mode == IQS9151_2F_MODE_PINCH) {
            two_result->pinch_ended = true;
        }
        if (data->two_finger.hold_sent) {
            iqs9151_release_hold(data, dev);
            released_from_hold = true;
        }

        iqs9151_two_finger_reset(&data->two_finger);
        data->two_finger_one_lead_valid = false;
    } else {
        data->three_finger_two_lead_valid = false;
    }

    if (frame->finger_count == 2U && data->one_finger.active) {
        const int64_t elapsed_ms = k_uptime_get() - data->one_finger.down_ms;

        data->two_finger_one_lead_valid =
            !data->one_finger.hold_sent &&
            data->one_finger.tap_candidate &&
            elapsed_ms <= TWO_FINGER_ONE_LEAD_MAX_MS &&
            iqs9151_abs32(data->one_finger.dx) <= ONE_FINGER_TAP_MOVE &&
            iqs9151_abs32(data->one_finger.dy) <= ONE_FINGER_TAP_MOVE;
        if (data->one_finger.hold_sent) {
            iqs9151_release_hold(data, dev);
            released_from_hold = true;
        }
        iqs9151_one_finger_reset(&data->one_finger);
    } else {
        data->two_finger_one_lead_valid = false;
    }

    if (frame->finger_count != 1U && data->one_finger.active) {
        released_from_hold = iqs9151_one_finger_update(data, frame, prev_frame, dev);
    }
    if (frame->finger_count != 2U && data->two_finger.active) {
        iqs9151_two_finger_update(data, frame, prev_frame, dev, two_result);
    }
    if (frame->finger_count != 3U && data->three_active) {
        (void)iqs9151_three_finger_update(data, frame, prev_frame, dev);
    }

    /* A three-finger gesture that is waiting out a release, or riding out the
     * device's count dropping for a frame, owns the fingers it still has. */
    const bool three_holds_on =
        data->three_active && (data->three_release_pending || data->three_flicker_ms != 0);

    switch (frame->finger_count) {
    case 1U:
        if (!(data->two_finger.active && data->two_finger.release_pending)) {
            if (!three_holds_on) {
                released_from_hold = iqs9151_one_finger_update(data, frame, prev_frame, dev);
            }
        }
        break;
    case 2U:
        if (!three_holds_on) {
            iqs9151_two_finger_update(data, frame, prev_frame, dev, two_result);
        }
        break;
    case 3U:
        (void)iqs9151_three_finger_update(data, frame, prev_frame, dev);
        break;
    default:
        break;
    }

    return released_from_hold;
}

static void iqs9151_update_inertia_ema(struct iqs9151_data *data,
                                       const struct iqs9151_frame *frame,
                                       const struct iqs9151_frame *prev_frame,
                                       const struct iqs9151_two_finger_result *two_result,
                                       int64_t now_ms,
                                       bool released_from_hold,
                                       bool cursor_moving,
                                       bool suppress_cursor_tail) {
    const bool finger1_started =
        (prev_frame->finger_count == 0U) && (frame->finger_count == 1U);
    const bool cursor_released =
        (prev_frame->finger_count == 1U) && (frame->finger_count == 0U);
    int32_t seed_vx_fp;
    int32_t seed_vy_fp;

    /* Cancel Inertial */
    if (two_result->scroll_started || frame->finger_count == 2U || finger1_started) {
        iqs9151_ema_reset(&data->scroll_ema_x_fp, &data->scroll_ema_y_fp);
        iqs9151_inertia_cancel(&data->inertia_scroll, &data->inertia_scroll_work);
    }
    if (two_result->scroll_active) {
        iqs9151_ema_update(&data->scroll_ema_x_fp, &data->scroll_ema_y_fp,
                           two_result->scroll_x, two_result->scroll_y,
                           iqs9151_scroll_params.ema_alpha);
        iqs9151_motion_history_push(&data->scroll_motion_history, two_result->scroll_x,
                                    two_result->scroll_y, now_ms);
    }

    if (suppress_cursor_tail) {
        iqs9151_inertia_cancel(&data->inertia_cursor, &data->inertia_cursor_work);
        iqs9151_ema_reset(&data->cursor_ema_x_fp, &data->cursor_ema_y_fp);
        iqs9151_motion_history_reset(&data->cursor_motion_history);
    } else {
        if (finger1_started) {
            iqs9151_ema_reset(&data->cursor_ema_x_fp, &data->cursor_ema_y_fp);
            iqs9151_motion_history_reset(&data->cursor_motion_history);
        }
        if (frame->finger_count == 1U && cursor_moving) {
            iqs9151_inertia_cancel(&data->inertia_cursor, &data->inertia_cursor_work);
            iqs9151_ema_update(&data->cursor_ema_x_fp, &data->cursor_ema_y_fp,
                               frame->rel_x, frame->rel_y, iqs9151_cursor_params.ema_alpha);
            iqs9151_motion_history_push(&data->cursor_motion_history, frame->rel_x,
                                        frame->rel_y, now_ms);
        }
    }

    /* Inertial Cursolling */
    if (cursor_released && !released_from_hold && !suppress_cursor_tail) {
        if (IS_ENABLED(CONFIG_INPUT_IQS9151_CURSOR_INERTIA_ENABLE) &&
            iqs9151_inertia_seed_from_history(&data->cursor_motion_history,
                                              &iqs9151_cursor_params,
                                              &iqs9151_cursor_gate_params, now_ms,
                                              &seed_vx_fp, &seed_vy_fp)) {
            iqs9151_inertia_start(&data->inertia_cursor, &data->inertia_cursor_work,
                                  &iqs9151_cursor_params, seed_vx_fp, seed_vy_fp);
        }
        iqs9151_ema_reset(&data->cursor_ema_x_fp, &data->cursor_ema_y_fp);
        iqs9151_motion_history_reset(&data->cursor_motion_history);
    }

    /* Inertial Scrolling */
    if (two_result->scroll_ended) {
        if (IS_ENABLED(CONFIG_INPUT_IQS9151_SCROLL_INERTIA_ENABLE) &&
            iqs9151_inertia_seed_from_history(&data->scroll_motion_history,
                                              &iqs9151_scroll_params,
                                              &iqs9151_scroll_gate_params, now_ms,
                                              &seed_vx_fp, &seed_vy_fp)) {
            iqs9151_inertia_start(&data->inertia_scroll, &data->inertia_scroll_work,
                                  &iqs9151_scroll_params, seed_vx_fp, seed_vy_fp);
        }
        iqs9151_ema_reset(&data->scroll_ema_x_fp, &data->scroll_ema_y_fp);
        iqs9151_motion_history_reset(&data->scroll_motion_history);
    }
    if (two_result->pinch_active) {
        iqs9151_inertia_cancel(&data->inertia_scroll, &data->inertia_scroll_work);
        iqs9151_ema_reset(&data->scroll_ema_x_fp, &data->scroll_ema_y_fp);
        iqs9151_motion_history_reset(&data->scroll_motion_history);
    }
}

static atomic_t iqs9151_cursor_report_interval_ms =
    ATOMIC_INIT(CONFIG_INPUT_IQS9151_CURSOR_REPORT_INTERVAL_MS);

int iqs9151_set_cursor_report_interval(uint16_t ms) {
    atomic_set(&iqs9151_cursor_report_interval_ms, (atomic_val_t)MIN(ms, 100U));
    return 0;
}

/*
 * Report cursor movement, at most once per interval.
 *
 * The device delivers a frame every 5 ms and each carried two input events,
 * which on the peripheral half is two BLE notifications per frame -- four
 * hundred a second into a link that carries perhaps a hundred. Nothing was
 * lost, only queued, and a queue is lag: the pointer on that half trailed the
 * finger by a growing fraction of a second, and the half's key presses,
 * waiting in the same queue, arrived late enough to mistype. Before the
 * cursor was gated on real movement rather than the device's movement bit,
 * only one frame in sixteen got through and the link never noticed.
 *
 * So movement is owed rather than sent: added up and reported when the
 * interval has passed, in one or two events with the sync on the last, and
 * immediately when the frame is anything but a cursor frame. 0 reports every
 * frame, which is right for the half on USB.
 */
static void iqs9151_report_cursor(struct iqs9151_data *data, int32_t dx, int32_t dy,
                                  int64_t now_ms, bool flush) {
    const struct device *dev = data->dev;
    const int64_t interval = (int64_t)atomic_get(&iqs9151_cursor_report_interval_ms);

    data->cursor_pending_x += dx;
    data->cursor_pending_y += dy;
    if (data->cursor_pending_x == 0 && data->cursor_pending_y == 0) {
        return;
    }
    if (!flush && interval > 0 && (now_ms - data->cursor_report_ms) < interval) {
        return;
    }

    const int32_t x = data->cursor_pending_x;
    const int32_t y = data->cursor_pending_y;
    data->cursor_pending_x = 0;
    data->cursor_pending_y = 0;
    data->cursor_report_ms = now_ms;

    if (x != 0) {
        iqs9151_report_rel_event(dev, INPUT_REL_X, x, y == 0, K_NO_WAIT);
    }
    if (y != 0) {
        iqs9151_report_rel_event(dev, INPUT_REL_Y, y, true, K_NO_WAIT);
    }
}

static void iqs9151_report_frame_events(struct iqs9151_data *data,
                                        const struct iqs9151_frame *frame,
                                        const struct iqs9151_two_finger_result *two_result,
                                        bool cursor_moving,
                                        bool suppress_cursor_tail, int64_t now_ms) {
    const struct device *dev = data->dev;
    /*
     * Tapped rather than held, like the three-finger swipes: the gesture is
     * over by the time it is recognised, so there is nothing left to hold.
     * K_FOREVER because dropping half of a press/release pair latches the key.
     */
    /* Cursor movement owed from before this frame goes out first, whatever
     * the frame is: otherwise a second finger landing leaves it waiting
     * through the whole scroll, to arrive as a stray jump at the end. */
    if (frame->finger_count != 1U || !cursor_moving || suppress_cursor_tail ||
        data->three_active) {
        iqs9151_report_cursor(data, 0, 0, now_ms, true);
    }

    if (two_result->swipe_code != 0U) {
        iqs9151_report_key_event(dev, two_result->swipe_code, true, true, K_FOREVER);
        iqs9151_report_key_event(dev, two_result->swipe_code, false, true, K_FOREVER);
    }

    if (two_result->pinch_started) {
        iqs9151_report_key_event(dev, INPUT_BTN_7, true, true, K_FOREVER);
    }
    if (two_result->pinch_ended) {
        iqs9151_report_key_event(dev, INPUT_BTN_7, false, true, K_FOREVER);
    }

    if (two_result->pinch_active) {
        if (two_result->pinch_wheel != 0) {
            iqs9151_report_rel_event(dev, IQS9151_PINCH_WHEEL_CODE, two_result->pinch_wheel, true,
                                     K_NO_WAIT);
        }
    } else if (two_result->scroll_active) {
        const bool have_x = two_result->scroll_x != 0;
        const bool have_y = two_result->scroll_y != 0;
        if (have_x) {
            iqs9151_report_rel_event(dev, INPUT_REL_HWHEEL, (int16_t)(-two_result->scroll_x),
                                     !have_y, K_NO_WAIT);
        }
        if (have_y) {
            iqs9151_report_rel_event(dev, INPUT_REL_WHEEL, two_result->scroll_y, true, K_NO_WAIT);
        }
    } else if (frame->finger_count == 1U && cursor_moving && !suppress_cursor_tail &&
               !data->three_active) {
        iqs9151_report_cursor(data, frame->rel_x, frame->rel_y, now_ms, false);
    }
}

static bool iqs9151_in_landing_zone(struct iqs9151_data *data, const struct iqs9151_frame *frame,
                                    const struct iqs9151_frame *prev_frame, int64_t now_ms);

static void iqs9151_process_frame(struct iqs9151_data *data,
                                  const struct iqs9151_frame *frame,
                                  int64_t now_ms) {
    const struct device *dev = data->dev;
    const struct iqs9151_frame prev_frame = data->prev_frame;
    struct iqs9151_two_finger_result two_result;
    /*
     * "Is the finger moving?" -- answered from the movement itself, not from
     * the device's MOVEMENT_DETECTED bit.
     *
     * At a slow, steady drag the device keeps reporting a real -1 count per
     * frame but toggles MOVEMENT_DETECTED off on most of them: a motion trace
     * showed roughly fifteen frames carrying r=-1 with the bit clear for every
     * one frame with it set. Gating the cursor on the bit alone threw those
     * fifteen counts away and emitted the sixteenth in a lump, which is the
     * periodic deceleration -- the pointer stalls for ~80 ms, jumps, stalls
     * again -- that no amount of IC filtering or output smoothing could touch,
     * because the movement was already gone before either ran.
     *
     * The relative registers are zero when the finger is genuinely still, so a
     * nonzero delta with a single finger down is the honest signal. The bit is
     * kept as an OR so a frame the device flags as moving but that happens to
     * carry a zero delta (a direction reversal within the frame) still counts.
     * Every consumer of this flag is already guarded by finger_count == 1.
     */
    const bool cursor_moving = (frame->finger_count == 1U) &&
                               (((frame->trackpad_flags & IQS9151_TP_MOVEMENT_DETECTED) != 0U) ||
                                frame->rel_x != 0 || frame->rel_y != 0) &&
                               !iqs9151_in_landing_zone(data, frame, &prev_frame, now_ms);
    bool released_from_hold;
    bool suppress_cursor_tail;

    iqs9151_two_finger_result_reset(&two_result);

    if (iqs9151_handle_show_reset(data, frame)) {
        return;
    }

    iqs9151_set_touch_state(data, frame->finger_count);

    released_from_hold =
        iqs9151_update_gesture_sessions(data, frame, &prev_frame, &two_result);
    suppress_cursor_tail =
        iqs9151_should_suppress_cursor_for_two_finger_tail(data, frame, &prev_frame,
                                                           &two_result);

    if (frame->finger_count == 3U || data->three_active) {
        iqs9151_inertia_cancel(&data->inertia_scroll, &data->inertia_scroll_work);
        iqs9151_inertia_cancel(&data->inertia_cursor, &data->inertia_cursor_work);
        iqs9151_ema_reset(&data->scroll_ema_x_fp, &data->scroll_ema_y_fp);
        iqs9151_ema_reset(&data->cursor_ema_x_fp, &data->cursor_ema_y_fp);
        iqs9151_motion_history_reset(&data->scroll_motion_history);
        iqs9151_motion_history_reset(&data->cursor_motion_history);
    }

    if (data->one_finger.active && data->one_finger.hold_sent) {
        iqs9151_inertia_cancel(&data->inertia_cursor, &data->inertia_cursor_work);
        iqs9151_motion_history_reset(&data->cursor_motion_history);
    }

    iqs9151_report_frame_events(data, frame, &two_result, cursor_moving,
                                suppress_cursor_tail, now_ms);

    LOG_DBG("rel x=%d y=%d info=0x%04x tp=0x%04x finger=%d f1x=%u f1y=%u f2x=%u f2y=%u",
            frame->rel_x, frame->rel_y, frame->info_flags, frame->trackpad_flags,
            frame->finger_count, frame->finger1_x, frame->finger1_y,
            frame->finger2_x, frame->finger2_y);
    LOG_DBG("gesture_state: hold_button=0x%04x 2f_mode=%d",
            data->hold_button,
            data->two_finger.mode);

    iqs9151_update_inertia_ema(data, frame, &prev_frame, &two_result, now_ms,
                               released_from_hold, cursor_moving,
                               suppress_cursor_tail);
    iqs9151_update_prev_frame(data, frame, &prev_frame);
    iqs9151_push_finger_history(data, frame->finger_count, now_ms);
}

static int iqs9151_set_interrupt(const struct device *dev, const bool en);

/* Defined below, next to the rest of the register writes. */
static void iqs9151_apply_requested_resolution(const struct device *dev);
static void iqs9151_apply_requested_filter(const struct device *dev);

/*
 * Per-axis cursor gain, applied to what the device reports.
 *
 * The two resolution registers do not do this. They set the range of the
 * *absolute* finger coordinates -- which is what the gesture arbitration
 * compares, so they remain the right lever for "which axis is this scroll on"
 * -- but the Relative X/Y the cursor is built from does not come out of them.
 * Moving X resolution from 1300 to 1974 was measurable in none of three
 * flashes, on either half. The value was reaching the device the whole time;
 * the device simply was not using it for this.
 *
 * So the pointer's axes are squared up here, where it cannot fail to work. A
 * pad taller than it is wide needs its long axis multiplied to move the
 * pointer as far for the same finger travel -- and on a keyboard it wants a
 * little more than that again, because the full 87 mm cannot be swept in one
 * stroke the way the 53 mm can, so parity in millimetres still reads as
 * reluctance in the hand.
 *
 * The remainder is the whole trick. Reports arrive one or two counts at a
 * time, and multiplying those by 16 and dividing by 10 in integers gives 1
 * every single time -- an axis made "faster" by nothing at all. Carrying the
 * tenths between reports is what turns a fractional gain into real travel.
 */
static atomic_t iqs9151_cursor_gain_x_x10 =
    ATOMIC_INIT(CONFIG_INPUT_IQS9151_CURSOR_GAIN_X_X10);
static atomic_t iqs9151_cursor_gain_y_x10 =
    ATOMIC_INIT(CONFIG_INPUT_IQS9151_CURSOR_GAIN_Y_X10);

static atomic_t iqs9151_cursor_smoothing =
    ATOMIC_INIT(CONFIG_INPUT_IQS9151_CURSOR_SMOOTHING);

static atomic_t iqs9151_tap_dead_zone = ATOMIC_INIT(CONFIG_INPUT_IQS9151_TAP_DEAD_ZONE);

int iqs9151_set_tap_dead_zone(uint16_t counts) {
    atomic_set(&iqs9151_tap_dead_zone, (atomic_val_t)counts);
    return 0;
}

/*
 * A finger landing does not stay put. As the fingertip flattens over the
 * first few frames the centroid the device reports slides toward the
 * finger's base -- a millimetre or so, toward the palm, which on this pad
 * is "down" on the screen -- and lifting slides it back. A tap meant as a
 * click therefore dragged the pointer a little every time. Withhold
 * movement while the finger is still near where it landed and the landing
 * is recent: a tap never leaves that zone, a stroke leaves it in a frame or
 * two and loses only its first fraction of a millimetre, and a finger that
 * rests there past the time limit is not tapping and gets its movement
 * back. The withheld frames are dropped, not replayed: replaying them
 * would put the drift back.
 */
static bool iqs9151_in_landing_zone(struct iqs9151_data *data, const struct iqs9151_frame *frame,
                                    const struct iqs9151_frame *prev_frame, int64_t now_ms) {
    const uint16_t zone = (uint16_t)atomic_get(&iqs9151_tap_dead_zone);
    if (prev_frame->finger_count == 0U && frame->finger_count == 1U) {
        data->landing = zone != 0U;
        data->land_x = frame->finger1_x;
        data->land_y = frame->finger1_y;
        data->land_ms = now_ms;
    }
    if (!data->landing || frame->finger_count != 1U) {
        data->landing = data->landing && frame->finger_count == 1U;
        return false;
    }
    const int32_t dx = (int32_t)frame->finger1_x - (int32_t)data->land_x;
    const int32_t dy = (int32_t)frame->finger1_y - (int32_t)data->land_y;
    const int32_t away = MAX(dx < 0 ? -dx : dx, dy < 0 ? -dy : dy);
    if (away >= (int32_t)zone ||
        (now_ms - data->land_ms) >= CONFIG_INPUT_IQS9151_TAP_DEAD_ZONE_MS) {
        data->landing = false;
        return false;
    }
    return true;
}

int iqs9151_set_cursor_smoothing(uint16_t reports) {
    atomic_set(&iqs9151_cursor_smoothing, (atomic_val_t)MAX(1U, reports));
    return 0;
}

uint16_t iqs9151_cursor_gain_x10(char axis) {
    return (uint16_t)atomic_get((axis == 'y') ? &iqs9151_cursor_gain_y_x10
                                              : &iqs9151_cursor_gain_x_x10);
}

int iqs9151_set_cursor_gain(uint16_t x_gain_x10, uint16_t y_gain_x10) {
    if (x_gain_x10 == 0U || y_gain_x10 == 0U) {
        return -EINVAL;
    }

    atomic_set(&iqs9151_cursor_gain_x_x10, (atomic_val_t)x_gain_x10);
    atomic_set(&iqs9151_cursor_gain_y_x10, (atomic_val_t)y_gain_x10);
    return 0;
}

/*
 * Gain and smoothing in one pass, over an accumulator held in tenths of an
 * output count.
 *
 * Gain alone is not enough once it is much above 1. The device reports whole
 * counts, and slowly it reports them as 1, 0, 1, 0 -- which a 2.4x gain turns
 * into 2, 0, 3, 0. The distance is right and the motion is visibly stepped,
 * because the gaps are still gaps and the steps are now nearly three pixels.
 * That is not the filter; it is the quantisation, amplified.
 *
 * So the accumulator is drained a fraction at a time instead of emptied every
 * report, which turns 2, 0, 3, 0 into 1, 1, 1, 1 and fills the gaps from what
 * the previous report did not spend. `spread` is how many reports to smear
 * across: 1 empties it immediately (the old behaviour, exactly), 2 halves it
 * each time, and so on. The cost is that much lag and no more -- at speed the
 * accumulator settles at `spread` reports' worth and output tracks input
 * exactly, so this smooths the crawl without rubber-banding the sweep.
 *
 * The floor matters as much as the fraction: once a fraction rounds to zero
 * the accumulator would sit there forever, so anything worth a whole count
 * emits one. That is what actually fills the gaps.
 */
static int16_t iqs9151_scale_axis(int16_t value, int32_t gain_x10, int32_t spread,
                                  int32_t *pending) {
    *pending += (int32_t)value * gain_x10;

    int32_t out = (*pending / spread) / 10;
    if (out == 0 && (*pending >= 10 || *pending <= -10)) {
        out = (*pending > 0) ? 1 : -1;
    }

    /* Division truncates toward zero and the carry keeps its sign, so
     * reversing direction spends the accumulator instead of fighting it. */
    *pending -= out * 10;

    return (int16_t)CLAMP(out, INT16_MIN, INT16_MAX);
}

/*
 * Distance-window smoothing: a moving average of the reported movement taken
 * over a fixed span of finger travel rather than a fixed number of reports.
 *
 * This is for a defect the gain and its report smoothing cannot touch: the
 * device's position, plotted against where the finger really is, is not a
 * straight line but a gentle wave, and on this pad's coarse long axis the wave
 * repeats about every two millimetres and swings the reported speed by better
 * than two to one. Because it is fixed in *distance*, dragging faster only
 * crosses it faster -- the ripple stays two millimetres apart however quickly
 * you move -- which is exactly why a smoother counted in reports slides off it:
 * the right number of reports to average over changes with speed.
 *
 * Averaged over a window one ripple long, the wave sums to nothing and the
 * genuine travel survives. The window is counted in the device's own counts
 * (~23 to the millimetre here), so a value near a ripple period -- 45 or so --
 * is the setting; 0 turns it off and this becomes a passthrough. The group
 * delay is half the window, and being a distance it is paid in millimetres of
 * following distance, not milliseconds -- fixed in the hand, and shorter in
 * time the faster you go.
 *
 * It runs before the gain, on the raw reported counts, because that is where
 * the wave lives; the gain then amplifies something already smooth. Only the
 * cursor reads these deltas, so the gestures -- which work from the absolute
 * finger coordinates -- are untouched.
 */
static atomic_t iqs9151_cursor_distance_window =
    ATOMIC_INIT(CONFIG_INPUT_IQS9151_CURSOR_DISTANCE_SMOOTHING);

int iqs9151_set_cursor_distance_smoothing(uint16_t counts) {
    atomic_set(&iqs9151_cursor_distance_window, (atomic_val_t)counts);
    return 0;
}

static void iqs9151_dist_smoother_reset(struct iqs9151_dist_smoother *s) {
    s->head = 0;
    s->count = 0;
    s->sum = 0;
    s->dist = 0;
    s->rem_fp = 0;
}

static int16_t iqs9151_distance_smooth_axis(struct iqs9151_dist_smoother *s, int16_t value,
                                            int32_t window) {
    if (window <= 0) {
        /* Off: nothing accumulated, so a later enable starts clean. */
        if (s->count != 0 || s->rem_fp != 0) {
            iqs9151_dist_smoother_reset(s);
        }
        return value;
    }

    const int32_t mag = (value < 0) ? -(int32_t)value : (int32_t)value;

    /* Push the newest report, evicting the oldest first if the ring is full. */
    if (s->count == IQS9151_DIST_SMOOTH_CAP) {
        const uint8_t tail = (uint8_t)((s->head + IQS9151_DIST_SMOOTH_CAP - s->count) %
                                       IQS9151_DIST_SMOOTH_CAP);
        const int16_t old = s->buf[tail];
        s->sum -= old;
        s->dist -= (old < 0) ? -(int32_t)old : (int32_t)old;
        s->count--;
    }
    s->buf[s->head] = value;
    s->head = (uint8_t)((s->head + 1U) % IQS9151_DIST_SMOOTH_CAP);
    s->count++;
    s->sum += value;
    s->dist += mag;

    /* Drop the oldest reports until the window is no longer than one ripple,
     * always keeping at least the newest so a lone large report still moves. */
    while (s->count > 1 && s->dist > window) {
        const uint8_t tail = (uint8_t)((s->head + IQS9151_DIST_SMOOTH_CAP - s->count) %
                                       IQS9151_DIST_SMOOTH_CAP);
        const int16_t old = s->buf[tail];
        const int32_t old_mag = (old < 0) ? -(int32_t)old : (int32_t)old;
        /* Stop before the window would collapse below one ripple. */
        if (s->dist - old_mag < window) {
            break;
        }
        s->sum -= old;
        s->dist -= old_mag;
        s->count--;
    }

    /* Emit this report's share of the window average, carrying the fraction so
     * a long crawl still adds up to the distance the finger actually covered. */
    s->rem_fp += ((int32_t)s->sum * 256) / (int32_t)s->count;
    int32_t out = s->rem_fp / 256; /* truncates toward zero; the carry keeps its sign */
    s->rem_fp -= out * 256;

    return (int16_t)CLAMP(out, INT16_MIN, INT16_MAX);
}

/* sin(2*pi*i/128) in Q15. cos is the same table a quarter turn on. */
static const int16_t iqs9151_sin_q15[128] = {
         0,   1608,   3212,   4808,   6393,   7962,   9512,  11039,
     12539,  14010,  15446,  16846,  18204,  19519,  20787,  22005,
     23170,  24279,  25329,  26319,  27245,  28105,  28898,  29621,
     30273,  30852,  31356,  31785,  32137,  32412,  32609,  32728,
     32767,  32728,  32609,  32412,  32137,  31785,  31356,  30852,
     30273,  29621,  28898,  28105,  27245,  26319,  25329,  24279,
     23170,  22005,  20787,  19519,  18204,  16846,  15446,  14010,
     12539,  11039,   9512,   7962,   6393,   4808,   3212,   1608,
         0,  -1608,  -3212,  -4808,  -6393,  -7962,  -9512, -11039,
    -12539, -14010, -15446, -16846, -18204, -19519, -20787, -22005,
    -23170, -24279, -25329, -26319, -27245, -28105, -28898, -29621,
    -30273, -30852, -31356, -31785, -32137, -32412, -32609, -32728,
    -32767, -32728, -32609, -32412, -32137, -31785, -31356, -30852,
    -30273, -29621, -28898, -28105, -27245, -26319, -25329, -24279,
    -23170, -22005, -20787, -19519, -18204, -16846, -15446, -14010,
    -12539, -11039,  -9512,  -7962,  -6393,  -4808,  -3212,  -1608,
};

/* Defined with the resolution request below; the period search watches them. */
static atomic_t iqs9151_requested_resolution_x;
static atomic_t iqs9151_requested_resolution_y;

static atomic_t iqs9151_ripple_period_x = ATOMIC_INIT(CONFIG_INPUT_IQS9151_RIPPLE_PERIOD_X_X10);
static atomic_t iqs9151_ripple_period_y = ATOMIC_INIT(CONFIG_INPUT_IQS9151_RIPPLE_PERIOD_Y_X10);
static atomic_t iqs9151_ripple_auto = ATOMIC_INIT(IS_ENABLED(CONFIG_INPUT_IQS9151_RIPPLE_AUTO));

int iqs9151_set_ripple_period(uint16_t x_tenths, uint16_t y_tenths) {
    if (x_tenths > IQS9151_RIPPLE_MAX_PERIOD_X10 || y_tenths > IQS9151_RIPPLE_MAX_PERIOD_X10) {
        return -EINVAL;
    }
    atomic_set(&iqs9151_ripple_period_x, (atomic_val_t)x_tenths);
    atomic_set(&iqs9151_ripple_period_y, (atomic_val_t)y_tenths);
    return 0;
}

int iqs9151_set_ripple_auto(bool enabled) {
    atomic_set(&iqs9151_ripple_auto, enabled ? 1 : 0);
    return 0;
}

static atomic_t iqs9151_ripple_map_on = ATOMIC_INIT(IS_ENABLED(CONFIG_INPUT_IQS9151_RIPPLE_MAP));

int iqs9151_set_ripple_map(bool enabled) {
    atomic_set(&iqs9151_ripple_map_on, enabled ? 1 : 0);
    return 0;
}

static void iqs9151_ripple_scan_start(struct iqs9151_ripple_scan *scan, uint8_t stage,
                                      uint16_t base_x10, uint16_t step_x10, uint8_t count) {
    memset(scan->a, 0, sizeof(scan->a));
    memset(scan->b, 0, sizeof(scan->b));
    scan->stage = stage;
    scan->base_x10 = base_x10;
    scan->step_x10 = step_x10;
    scan->count = MIN(count, IQS9151_RIPPLE_SCAN_MAX);
    scan->updates = 0;
    scan->last_winner_x10 = 0;
    scan->agree = 0;
    scan->lost = 0;
}

/*
 * What the search found last time, kept across power cycles.
 *
 * Finding the period takes ten or twenty seconds of strokes, which is fine
 * once and tiresome every morning. So an adopted period is written to the
 * settings backend under iqs9151/ripple/<axis>, and at the next boot the
 * search starts from it -- in the fine stage, so a drift of a tenth or two
 * is still followed, but nothing has to be found again. Kept outside the
 * device data because the settings handler runs before it knows which
 * device it is for; one pad per half is the case this driver serves.
 */
static uint16_t iqs9151_ripple_remembered[2];

#if IS_ENABLED(CONFIG_SETTINGS)
static int iqs9151_ripple_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                       void *cb_arg) {
    const char *next;
    int axis = -1;
    if (settings_name_steq(name, "x", &next) && next == NULL) {
        axis = 0;
    } else if (settings_name_steq(name, "y", &next) && next == NULL) {
        axis = 1;
    }
    if (axis < 0 || len != sizeof(uint16_t)) {
        return -ENOENT;
    }
    uint16_t value;
    if (read_cb(cb_arg, &value, sizeof(value)) != sizeof(value)) {
        return -EIO;
    }
    if (value > IQS9151_RIPPLE_MAX_PERIOD_X10) {
        return -EINVAL;
    }
    iqs9151_ripple_remembered[axis] = value;
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(iqs9151_ripple, "iqs9151/ripple", NULL,
                               iqs9151_ripple_settings_set, NULL, NULL);
#endif

static void iqs9151_ripple_remember(char axis, uint16_t period_x10) {
    const int i = (axis == 'y') ? 1 : 0;
    if (iqs9151_ripple_remembered[i] == period_x10) {
        return;
    }
    iqs9151_ripple_remembered[i] = period_x10;
#if IS_ENABLED(CONFIG_SETTINGS)
    const int ret = settings_save_one((axis == 'y') ? "iqs9151/ripple/y" : "iqs9151/ripple/x",
                                      &period_x10, sizeof(period_x10));
    if (ret != 0) {
        LOG_WRN("ripple %c: could not save period (%d)", axis, ret);
    }
#endif
    /* And where the app can see it. */
    iqs9151_setting_ripple_found(axis, period_x10);
}

static void iqs9151_ripple_scan_fine_around(struct iqs9151_ripple_scan *scan, uint16_t centre);

/*
 * Where the geometry puts the period of an axis, in tenths: the axis
 * resolution divided by twice the electrodes along it. Electrodes are along
 * the Rxs for X unless the IC is told to switch the axes; the counts are the
 * ones this driver configures.
 */
static uint16_t iqs9151_ripple_expected_x10(char axis, uint16_t resolution) {
    const bool switched = (TRACKPAD_SETTINGS_0_0 & IQS9151_TRACKPAD_SETTING_SWITCH_XY) != 0U;
    const uint32_t along_x = switched ? TRACKPAD_SETTINGS_1_0 : TRACKPAD_SETTINGS_0_1;
    const uint32_t along_y = switched ? TRACKPAD_SETTINGS_0_1 : TRACKPAD_SETTINGS_1_0;
    const uint32_t electrodes = (axis == 'y') ? along_y : along_x;
    if (resolution == 0U) {
        resolution =
            (axis == 'y') ? CONFIG_INPUT_IQS9151_RESOLUTION_Y : CONFIG_INPUT_IQS9151_RESOLUTION_X;
    }
    const uint32_t x10 = (10U * resolution + electrodes) / (2U * MAX(1U, electrodes));
    return (uint16_t)CLAMP(x10, IQS9151_RIPPLE_MIN_PERIOD_X10, IQS9151_RIPPLE_MAX_PERIOD_X10);
}

/* The coarse sweep for an axis: +/-14% around the geometric period, one
 * percent apart, clipped to what a period can be. */
static void iqs9151_ripple_scan_coarse(struct iqs9151_ripple_scan *scan, char axis) {
    const uint16_t centre = iqs9151_ripple_expected_x10(axis, scan->resolution);
    const uint16_t step = MAX(1U, (centre * IQS9151_RIPPLE_SCAN_COARSE_PERCENT + 50U) / 100U);
    uint16_t base = (centre > IQS9151_RIPPLE_SCAN_COARSE_HALF * step)
                        ? centre - IQS9151_RIPPLE_SCAN_COARSE_HALF * step
                        : 0U;
    base = MAX(base, IQS9151_RIPPLE_MIN_PERIOD_X10);
    uint8_t count = IQS9151_RIPPLE_SCAN_MAX;
    while (count > 1U && base + (count - 1U) * step > IQS9151_RIPPLE_MAX_PERIOD_X10) {
        count--;
    }
    iqs9151_ripple_scan_start(scan, IQS9151_RIPPLE_SCAN_COARSE, base, step, count);
}

/* Start over: from what was remembered if there is something, else the
 * coarse sweep. */
static void iqs9151_ripple_scan_reset(struct iqs9151_ripple_scan *scan, char axis) {
    const uint16_t remembered = iqs9151_ripple_remembered[(axis == 'y') ? 1 : 0];
    if (remembered >= IQS9151_RIPPLE_MIN_PERIOD_X10 + IQS9151_RIPPLE_SCAN_FINE_HALF &&
        remembered + IQS9151_RIPPLE_SCAN_FINE_HALF <= IQS9151_RIPPLE_MAX_PERIOD_X10) {
        iqs9151_ripple_scan_fine_around(scan, remembered);
        scan->found_x10 = remembered;
        return;
    }
    iqs9151_ripple_scan_coarse(scan, axis);
    scan->found_x10 = 0;
}

/* The equaliser's learned wave and per-stroke state, not the period search:
 * a new period means a new wave. */
static void iqs9151_ripple_reset(struct iqs9151_ripple_eq *eq, uint16_t period_x10) {
    for (size_t k = 0; k < IQS9151_RIPPLE_HARMONICS; k++) {
        eq->a[k] = 0;
        eq->b[k] = 0;
    }
    for (size_t i = 0; i < IQS9151_RIPPLE_BINS; i++) {
        eq->corr[i] = 256U;
    }
    memset(&eq->ref, 0, sizeof(eq->ref));
    eq->rem_fp = 0;
    eq->tick = 0;
    eq->updates = 0;
    eq->period_x10 = period_x10;
}

/* Only the per-stroke state; the learned coefficients are the calibration. */
static void iqs9151_ripple_lift(struct iqs9151_ripple_eq *eq) {
    memset(&eq->ref, 0, sizeof(eq->ref));
    eq->rem_fp = 0;
}

static void iqs9151_ripple_ref_drop_oldest(struct iqs9151_ripple_ref *ref) {
    const uint8_t tail =
        (uint8_t)((ref->head + IQS9151_RIPPLE_REF_CAP - ref->count) % IQS9151_RIPPLE_REF_CAP);
    ref->dist -= ref->mag[tail];
    ref->frames -= 1U + MIN(ref->idle[tail], IQS9151_RIPPLE_IDLE_CAP);
    ref->count--;
}

/*
 * Fold one frame into the reference window. For a moving report, returns the
 * number of frames it stands for -- itself plus the still frames before it --
 * or 0 if it is not one to learn from (nothing before it, or a rest too long
 * to be part of a stroke). A still frame always returns 0.
 */
static uint32_t iqs9151_ripple_ref_push(struct iqs9151_ripple_ref *ref, int16_t value,
                                        int32_t period) {
    /* Still frames before the first report of a stroke are not part of it. */
    if (ref->count == 0U && value == 0) {
        return 0;
    }

    uint32_t spans = 0U; /* the first report of a stroke has no "since" */
    if (ref->count > 0U) {
        const uint8_t last =
            (uint8_t)((ref->head + IQS9151_RIPPLE_REF_CAP - 1U) % IQS9151_RIPPLE_REF_CAP);
        if (value == 0) {
            if (ref->idle[last] < IQS9151_RIPPLE_IDLE_CAP) {
                ref->frames++;
            }
            if (ref->idle[last] < UINT8_MAX) {
                ref->idle[last]++;
            }
            return 0;
        }
        spans = (ref->idle[last] > IQS9151_RIPPLE_IDLE_CAP) ? 0U : 1U + ref->idle[last];
    }

    const int32_t mag = MIN(255, (value < 0) ? -(int32_t)value : (int32_t)value);
    if (ref->count == IQS9151_RIPPLE_REF_CAP) {
        iqs9151_ripple_ref_drop_oldest(ref);
    }
    ref->mag[ref->head] = (uint8_t)mag;
    ref->idle[ref->head] = 0;
    ref->head = (uint8_t)((ref->head + 1U) % IQS9151_RIPPLE_REF_CAP);
    ref->count++;
    ref->dist += (uint32_t)mag;
    ref->frames++;

    /* Trim the tail to one period of travel, never below it. */
    while (ref->count > 1U) {
        const uint8_t tail = (uint8_t)((ref->head + IQS9151_RIPPLE_REF_CAP - ref->count) %
                                       IQS9151_RIPPLE_REF_CAP);
        if ((int32_t)(ref->dist - ref->mag[tail]) < period) {
            break;
        }
        iqs9151_ripple_ref_drop_oldest(ref);
    }
    return spans;
}

/* Where in the wave the finger is, as one of 128 bins. The position is in
 * whole counts and the period in tenths, so both go to tenths first. */
static uint32_t iqs9151_ripple_bin(uint16_t abs_pos, uint16_t period_x10) {
    const uint32_t phase_x10 = ((uint32_t)abs_pos * 10U) % period_x10;
    return (phase_x10 * IQS9151_RIPPLE_BINS) / period_x10;
}

/* cos and sin of the k-th harmonic at this bin, Q15. */
static void iqs9151_ripple_basis(uint32_t bin, uint32_t k, int32_t *c, int32_t *s) {
    const uint32_t idx = bin * k;
    *s = iqs9151_sin_q15[idx & (IQS9151_RIPPLE_BINS - 1U)];
    *c = iqs9151_sin_q15[(idx + IQS9151_RIPPLE_BINS / 4U) & (IQS9151_RIPPLE_BINS - 1U)];
}

static uint32_t iqs9151_isqrt(uint64_t v) {
    uint64_t r = 0, bit = 1ULL << 62;
    while (bit > v) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (v >= r + bit) {
            v -= r + bit;
            r = (r >> 1) + bit;
        } else {
            r >>= 1;
        }
        bit >>= 2;
    }
    return (uint32_t)r;
}

/* g(bin) = 1 + sum(ak cos k.phase + bk sin k.phase), Q15, floored above zero. */
static int32_t iqs9151_ripple_g(const struct iqs9151_ripple_eq *eq, uint32_t bin) {
    int64_t sum = 0;
    for (uint32_t k = 0; k < IQS9151_RIPPLE_HARMONICS; k++) {
        int32_t c, s;
        iqs9151_ripple_basis(bin, k + 1U, &c, &s);
        sum += (int64_t)eq->a[k] * c + (int64_t)eq->b[k] * s;
    }
    const int32_t g = 32768 + (int32_t)(sum >> (15 + IQS9151_RIPPLE_COEF_GUARD));
    return MAX(IQS9151_RIPPLE_G_FLOOR, g);
}

/*
 * 1/g per bin, scaled so its mean over the period is exactly 256: dividing a
 * ripple out must not also change how far a stroke goes.
 */
static void iqs9151_ripple_rebuild(struct iqs9151_ripple_eq *eq) {
    /* No convincing wave learned: correct nothing rather than something
     * imagined. The learning goes on underneath. */
    const uint32_t fundamental =
        iqs9151_isqrt((int64_t)eq->a[0] * eq->a[0] + (int64_t)eq->b[0] * eq->b[0]);
    if (fundamental < ((uint32_t)IQS9151_RIPPLE_MIN_FUNDAMENTAL << IQS9151_RIPPLE_COEF_GUARD)) {
        for (uint32_t p = 0; p < IQS9151_RIPPLE_BINS; p++) {
            eq->corr[p] = 256U;
        }
        return;
    }

    /* Two passes rather than a table on the stack: this runs on the system
     * work queue, whose stack is already the tightest thing on this board. */
    uint32_t total = 0;
    for (uint32_t p = 0; p < IQS9151_RIPPLE_BINS; p++) {
        total += (1U << 24) / (uint32_t)iqs9151_ripple_g(eq, p);
    }
    const uint32_t mean = MAX(1U, total / IQS9151_RIPPLE_BINS);
    for (uint32_t p = 0; p < IQS9151_RIPPLE_BINS; p++) {
        const uint32_t recip = (1U << 24) / (uint32_t)iqs9151_ripple_g(eq, p);
        eq->corr[p] = (uint16_t)MIN(65535U, (recip * 256U) / mean);
    }
}

/* A guarded coefficient in thousandths, for the logs. */
static int iqs9151_ripple_milli(int32_t coef) {
    return (int)(((int64_t)coef * 1000) / IQS9151_RIPPLE_COEF_ONE);
}

/* mu * e * basis, into a guarded coefficient, rounded rather than floored. */
static int32_t iqs9151_ripple_step(int32_t e, int32_t basis, int mu_shift) {
    const int shift = 15 + mu_shift - IQS9151_RIPPLE_COEF_GUARD;
    const int64_t prod = (int64_t)e * basis + (1LL << (shift - 1));
    return (int32_t)(prod >> shift);
}

/* One learning step for every candidate period, fundamental only. */
static void iqs9151_ripple_scan_learn(struct iqs9151_ripple_scan *scan, uint16_t abs_pos,
                                      int32_t sample) {
    for (uint32_t i = 0; i < scan->count; i++) {
        const uint16_t period_x10 = scan->base_x10 + i * scan->step_x10;
        const uint32_t bin = iqs9151_ripple_bin(abs_pos, period_x10);
        int32_t c, sn;
        iqs9151_ripple_basis(bin, 1U, &c, &sn);
        const int32_t g =
            32768 + (int32_t)(((int64_t)scan->a[i] * c + (int64_t)scan->b[i] * sn) >>
                              (15 + IQS9151_RIPPLE_COEF_GUARD));
        const int32_t e = sample - g;
        const int32_t limit = IQS9151_RIPPLE_COEF_LIMIT << IQS9151_RIPPLE_COEF_GUARD;
        scan->a[i] = CLAMP(scan->a[i] + iqs9151_ripple_step(e, c, IQS9151_RIPPLE_SCAN_MU_SHIFT),
                           -limit, limit);
        scan->b[i] = CLAMP(scan->b[i] + iqs9151_ripple_step(e, sn, IQS9151_RIPPLE_SCAN_MU_SHIFT),
                           -limit, limit);
    }
}

/*
 * Where the bank's amplitude peaks, to a tenth, by fitting a parabola through
 * the best candidate and the two `reach` steps either side of it. The top of
 * the peak is flat -- a candidate half a count off still learns most of the
 * wave -- so the argmax alone wanders; the fit uses the slopes.
 */
static uint16_t iqs9151_ripple_scan_peak(const struct iqs9151_ripple_scan *scan,
                                         const uint32_t *amp, uint32_t best, uint32_t reach) {
    const uint16_t at = scan->base_x10 + best * scan->step_x10;
    if (best < reach || best + reach >= scan->count) {
        return at;
    }
    const int64_t lo = amp[best - reach], mid = amp[best], hi = amp[best + reach];
    const int64_t denom = 2 * (lo - 2 * mid + hi);
    if (denom >= 0) {
        return at; /* not a peak */
    }
    /* offset in candidate steps, scaled by 16 for the rounding below */
    const int64_t off16 = ((lo - hi) * 16 * (int64_t)reach) / denom;
    const int32_t off_x10 = (int32_t)((off16 * scan->step_x10 + (off16 < 0 ? -8 : 8)) / 16);
    return (uint16_t)CLAMP((int32_t)at + off_x10, IQS9151_RIPPLE_MIN_PERIOD_X10,
                           IQS9151_RIPPLE_MAX_PERIOD_X10);
}

static void iqs9151_ripple_scan_fine_around(struct iqs9151_ripple_scan *scan, uint16_t centre) {
    iqs9151_ripple_scan_start(scan, IQS9151_RIPPLE_SCAN_FINE,
                              centre - IQS9151_RIPPLE_SCAN_FINE_HALF,
                              IQS9151_RIPPLE_SCAN_FINE_STEP_X10,
                              2U * IQS9151_RIPPLE_SCAN_FINE_HALF + 1U);
}

/*
 * Every so many steps, look at what the bank has learned. Returns a newly
 * adopted period in tenths, or 0 if nothing changed this time.
 */
static uint16_t iqs9151_ripple_scan_verdict(struct iqs9151_ripple_scan *scan, char axis) {
    if (++scan->updates < IQS9151_RIPPLE_SCAN_EVERY) {
        return 0;
    }
    scan->updates = 0;

    uint32_t amp[IQS9151_RIPPLE_SCAN_MAX];
    uint32_t best = 0;
    for (uint32_t i = 0; i < scan->count; i++) {
        amp[i] = iqs9151_isqrt((int64_t)scan->a[i] * scan->a[i] +
                               (int64_t)scan->b[i] * scan->b[i]);
        if (amp[i] > amp[best]) {
            best = i;
        }
    }

    /* The pack: everything more than two steps from the best. In the coarse
     * sweep a wrong candidate learns only noise, so this is the noise. */
    uint64_t pack_total = 0;
    uint32_t pack_n = 0;
    for (uint32_t i = 0; i < scan->count; i++) {
        if (i + 2U < best || i > best + 2U) {
            pack_total += amp[i];
            pack_n++;
        }
    }
    const uint32_t pack = (uint32_t)(pack_total / MAX(1U, pack_n));
    const bool coarse = scan->stage == IQS9151_RIPPLE_SCAN_COARSE;
    const uint16_t estimate =
        iqs9151_ripple_scan_peak(scan, amp, best, coarse ? 1U : IQS9151_RIPPLE_SCAN_FINE_HALF);

#if IS_ENABLED(CONFIG_INPUT_IQS9151_MOTION_TRACE)
    {
        const uint16_t top = scan->base_x10 + (scan->count - 1U) * scan->step_x10;
        /* Short: the devtool keeps 47 characters of a line. */
        LOG_WRN("eqs %c %c %u.%u-%u.%u pk %u.%u a%d p%d", axis, coarse ? 'c' : 'f',
                scan->base_x10 / 10U, scan->base_x10 % 10U, top / 10U, top % 10U,
                estimate / 10U, estimate % 10U, iqs9151_ripple_milli((int32_t)amp[best]),
                iqs9151_ripple_milli((int32_t)pack));
    }
#else
    ARG_UNUSED(axis);
#endif

    /* A wave, not noise: clearly above the floor and, in the coarse sweep,
     * clearly above the pack. */
    if (amp[best] < ((uint32_t)IQS9151_RIPPLE_SCAN_MIN_AMP << IQS9151_RIPPLE_COEF_GUARD) ||
        (coarse && amp[best] < (5U * pack) / 2U)) {
        scan->agree = 0;
        scan->last_winner_x10 = 0;
        /* A fine bank that sees no wave for a while is looking in the wrong
         * place -- a remembered period from before a resolution change, say.
         * Sweep again; what was found stays in use until something better
         * turns up. */
        if (!coarse && ++scan->lost >= IQS9151_RIPPLE_SCAN_LOST) {
            const uint16_t found = scan->found_x10;
            iqs9151_ripple_scan_coarse(scan, axis);
            scan->found_x10 = found;
        }
        return 0;
    }
    scan->lost = 0;

    if (!coarse && scan->found_x10 != 0U &&
        abs((int32_t)estimate - (int32_t)scan->found_x10) < IQS9151_RIPPLE_SCAN_HOLD_X10) {
        /* Still where it was: nothing to do, and no flapping over a tenth. */
        scan->agree = 0;
        return 0;
    }

    /* Two verdicts in a row within a step of each other before acting. */
    if (scan->last_winner_x10 != 0U &&
        abs((int32_t)estimate - (int32_t)scan->last_winner_x10) <= (int32_t)scan->step_x10) {
        scan->agree = MIN(scan->agree + 1U, IQS9151_RIPPLE_SCAN_REAGREE);
    } else {
        scan->agree = 1;
    }
    scan->last_winner_x10 = estimate;
    const uint8_t needed = (!coarse && scan->found_x10 != 0U) ? IQS9151_RIPPLE_SCAN_REAGREE
                                                              : IQS9151_RIPPLE_SCAN_AGREE;
    if (scan->agree < needed) {
        return 0;
    }

    if (coarse) {
        /* Narrow in: a tenth apart, half a count either side. Nothing is
         * adopted yet; half a count is still a visible error. */
        iqs9151_ripple_scan_fine_around(scan, estimate);
        return 0;
    }

    /* Fine: adopt, and re-centre the bank on it so a later drift can still be
     * followed in either direction. */
    const uint16_t found = scan->found_x10;
    iqs9151_ripple_scan_fine_around(scan, estimate);
    scan->found_x10 = estimate;
    return (estimate != found) ? estimate : 0;
}

/*
 * The period this axis should use: what the search found, when the search is
 * on -- nothing at all until it has found one, because a correction at a
 * period the pad does not have is a ripple of its own -- else the setting.
 * Learning and the search run whenever either is non-zero or the search is
 * on.
 */
/* ---- The position map ---- */

static struct k_work_q iqs9151_recovery_q; /* defined with the recovery below */

/*
 * What was learned last time, kept across power cycles: the table in Q6
 * (64 = 1.0) behind the resolution it was learned at, under
 * iqs9151/map/<axis>. Loaded by the settings handler before the device
 * knows about it, so it waits here until the map is first used.
 */
/* What is saved. The layout number is part of the record's size, so a table
 * learned by a driver that learned differently (before the speed gate below,
 * say) is not restored: the size check in the settings handler refuses it. */
#define IQS9151_MAP_RECORD_LAYOUT 2

struct iqs9151_map_record {
    uint8_t layout[IQS9151_MAP_RECORD_LAYOUT];
    uint16_t resolution;
    uint8_t g[IQS9151_MAP_BINS];
} __packed;

static struct iqs9151_map_record iqs9151_map_saved[2];
static bool iqs9151_map_saved_valid[2];

#if IS_ENABLED(CONFIG_SETTINGS)
static int iqs9151_map_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                    void *cb_arg) {
    const char *next;
    int axis = -1;
    if (settings_name_steq(name, "x", &next) && next == NULL) {
        axis = 0;
    } else if (settings_name_steq(name, "y", &next) && next == NULL) {
        axis = 1;
    }
    if (axis < 0 || len != sizeof(struct iqs9151_map_record)) {
        return -ENOENT;
    }
    if (read_cb(cb_arg, &iqs9151_map_saved[axis], sizeof(iqs9151_map_saved[axis])) !=
        sizeof(iqs9151_map_saved[axis])) {
        return -EIO;
    }
    iqs9151_map_saved_valid[axis] = iqs9151_map_saved[axis].resolution != 0U;
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(iqs9151_map, "iqs9151/map", NULL, iqs9151_map_settings_set, NULL,
                               NULL);
#endif

static void iqs9151_map_rebuild(struct iqs9151_ripple_map *map);
static void iqs9151_map_publish(struct iqs9151_ripple_map *map, char axis);
static uint32_t iqs9151_map_l(const struct iqs9151_ripple_map *map, uint32_t pos);

static void iqs9151_map_clear(struct iqs9151_ripple_map *map, uint16_t resolution) {
    memset(map->g, 0, sizeof(map->g));
    memset(map->n, 0, sizeof(map->n));
    for (size_t i = 0; i < IQS9151_MAP_BINS; i++) {
        map->corr[i] = 256;
    }
    for (size_t i = 0; i <= IQS9151_MAP_BINS; i++) {
        map->lut[i] = (uint32_t)i * 256U;
    }
    memset(&map->ref, 0, sizeof(map->ref));
    map->resolution = resolution;
    map->have_prev = false;
    map->rem_fp = 0;
    map->tick = 0;
    map->dirty = false;
    map->steps_since_save = 0;
    map->period_x10 = 0;
    map->period_cand_x10 = 0;
}

/* Start from what was saved, if it was learned at this resolution. */
static void iqs9151_map_restore(struct iqs9151_ripple_map *map, char axis, uint16_t resolution) {
    const int i = (axis == 'y') ? 1 : 0;
    iqs9151_map_clear(map, resolution);
    map->period_geo_x10 = iqs9151_ripple_expected_x10(axis, resolution);
    if (!iqs9151_map_saved_valid[i] || iqs9151_map_saved[i].resolution != resolution) {
        return;
    }
    uint32_t restored = 0;
    for (size_t b = 0; b < IQS9151_MAP_BINS; b++) {
        const uint8_t q6 = iqs9151_map_saved[i].g[b];
        if (q6 != 0U) {
            map->g[b] = (int16_t)((uint32_t)q6 * IQS9151_MAP_ONE / 64U);
            map->n[b] = IQS9151_MAP_MIN_SAMPLES;
            restored++;
        }
    }
    iqs9151_map_rebuild(map);
    LOG_INF("ripple map %c: %u of %u bins restored", axis, restored, IQS9151_MAP_BINS);
    iqs9151_map_publish(map, axis);
}

/*
 * Snapshot the table for saving. Returns true when there is something new
 * enough and old enough to be worth a flash write. The write itself does
 * not happen here: this runs in the frame work on the system work queue,
 * and a flash write on this SoC waits for gaps between radio events -- tens
 * of milliseconds during which everything else queued there, the key
 * matrix scan included, stands still. The first build wrote from here every
 * thirty seconds, and typed keys arrived late and doubled.
 */
static bool iqs9151_map_snapshot(struct iqs9151_ripple_map *map, char axis, int64_t now_ms) {
    if (!map->dirty || map->steps_since_save < IQS9151_MAP_SAVE_MIN_STEPS ||
        (now_ms - map->saved_ms) < IQS9151_MAP_SAVE_MIN_MS) {
        return false;
    }
    const int i = (axis == 'y') ? 1 : 0;
    struct iqs9151_map_record *rec = &iqs9151_map_saved[i];
    memset(rec->layout, IQS9151_MAP_RECORD_LAYOUT, sizeof(rec->layout));
    rec->resolution = map->resolution;
    for (size_t b = 0; b < IQS9151_MAP_BINS; b++) {
        rec->g[b] = (map->n[b] >= IQS9151_MAP_MIN_SAMPLES)
                        ? (uint8_t)CLAMP((map->g[b] * 64 + IQS9151_MAP_ONE / 2) / IQS9151_MAP_ONE,
                                         1, 255)
                        : 0U;
    }
    iqs9151_map_saved_valid[i] = true;
    map->dirty = false;
    map->steps_since_save = 0;
    map->saved_ms = now_ms;
    return true;
}

static void iqs9151_map_save_work_handler(struct k_work *work) {
    struct iqs9151_data *data = CONTAINER_OF(work, struct iqs9151_data, map_save_work);
    ARG_UNUSED(data);
#if IS_ENABLED(CONFIG_SETTINGS)
    for (int i = 0; i < 2; i++) {
        if (!iqs9151_map_saved_valid[i]) {
            continue;
        }
        const int ret = settings_save_one(i ? "iqs9151/map/y" : "iqs9151/map/x",
                                          &iqs9151_map_saved[i], sizeof(iqs9151_map_saved[i]));
        if (ret != 0) {
            LOG_WRN("ripple map %c: could not save (%d)", i ? 'y' : 'x', ret);
        }
    }
#endif
}

/* How many bins can correct, for the app: written only when it changes. */
static void iqs9151_map_publish(struct iqs9151_ripple_map *map, char axis) {
    uint16_t learned = 0;
    for (size_t b = 0; b < IQS9151_MAP_BINS; b++) {
        learned += (map->n[b] >= IQS9151_MAP_MIN_SAMPLES) ? 1U : 0U;
    }
    if (learned != map->learned) {
        map->learned = learned;
        iqs9151_setting_map_learned(axis, learned);
    }
}

/*
 * The correction table from the learned ratios: mean(g) / g per bin, so
 * that a stroke's total travel is what it was. A bin that has not seen
 * enough corrects nothing. Not smoothed across bins: a bin is a tenth of
 * the wave and the wave's second harmonic is a fifth, so even a three-bin
 * kernel takes a third of that harmonic away again (measured: the residual
 * doubles). The per-bin average does the smoothing, over samples.
 */
/*
 * The wave's period, read off the table itself.
 *
 * The reference the ratios are taken against is a trailing window one
 * period long, so that the wave averages out of it. The window was sized
 * from the geometry -- half an electrode pitch -- and on this pad that is
 * wrong by a third along the long axis: the geometry says 76 counts, the
 * table (and the waveform tool before it) says 100. A window that is not a
 * whole number of periods leaves part of the wave in the reference, and
 * what the table learns is the wave divided by a shifted copy of itself:
 * the right period, the wrong depth.
 *
 * So once enough of the table is filled, measure the period from it: the
 * lag at which the learned slope best matches itself, over the lags a
 * period could plausibly be (half to twice the geometric one), refined to
 * a fraction of a bin by the parabola through the three points around the
 * peak. Bins that have not learned are left out of every pair. The
 * distortion from a wrong window does not move the period, only the
 * shape, so this converges: the next window is right, the next table is
 * true.
 */
#define IQS9151_MAP_LAG_SLOTS 68 /* lags tried, plus one either side for the parabola */

static void iqs9151_map_measure_period(struct iqs9151_ripple_map *map) {
    const uint32_t res = MAX(1U, (uint32_t)map->resolution);
    /* The geometric period in bins, x10: period_x10 / res * 256 / 10. */
    const uint32_t geo_bins_x10 =
        ((uint32_t)map->period_geo_x10 * IQS9151_MAP_BINS + res / 2U) / res;
    const int lag_min = MAX(3, (int)(geo_bins_x10 / 20U)); /* half */
    const int lag_max =
        MIN(lag_min + IQS9151_MAP_LAG_SLOTS - 3, (int)(geo_bins_x10 / 5U)); /* twice */
    if (lag_max <= lag_min + 2) {
        return;
    }

    /* The match at lag zero, per bin: what a perfect match would score. */
    int64_t r0 = 0;
    uint32_t n0 = 0;
    for (size_t b = 0; b < IQS9151_MAP_BINS; b++) {
        if (map->n[b] >= IQS9151_MAP_MIN_SAMPLES) {
            const int32_t d = (int32_t)map->corr[b] - 256;
            r0 += (int64_t)d * d;
            n0++;
        }
    }
    if (n0 < 24U || r0 == 0) {
        return;
    }
    r0 /= n0;

    /* The match at every lag from one below the range to one above, per
     * pair of learned bins. INT32_MIN marks a lag with too few pairs. */
    int32_t r[IQS9151_MAP_LAG_SLOTS];
    const int first = lag_min - 1;
    for (int lag = first; lag <= lag_max + 1; lag++) {
        int64_t acc = 0;
        uint32_t pairs = 0;
        for (size_t b = 0; b + (size_t)lag < IQS9151_MAP_BINS; b++) {
            if (map->n[b] >= IQS9151_MAP_MIN_SAMPLES &&
                map->n[b + (size_t)lag] >= IQS9151_MAP_MIN_SAMPLES) {
                acc += (int64_t)((int32_t)map->corr[b] - 256) *
                       ((int32_t)map->corr[b + (size_t)lag] - 256);
                pairs++;
            }
        }
        r[lag - first] = (pairs >= 16U) ? (int32_t)(acc / (int64_t)pairs) : INT32_MIN;
    }

    int best = -1;
    int32_t best_r = 0;
    for (int lag = lag_min; lag <= lag_max; lag++) {
        const int32_t v = r[lag - first];
        if (v != INT32_MIN && v > best_r) {
            best_r = v;
            best = lag;
        }
    }
    /* A real wave: the match at its period is at least a third of the match
     * at lag zero (a clean sine would give all of it; noise gives nothing). */
    if (best < 0 || (int64_t)best_r * 3 < r0) {
        return;
    }

    /* Parabolic refinement through the three points around the peak, in
     * sixteenths of a bin, when both neighbours were measured. */
    int32_t lag_x16 = best * 16;
    const int32_t rl = r[best - first - 1];
    const int32_t rr = r[best - first + 1];
    if (rl != INT32_MIN && rr != INT32_MIN) {
        const int64_t denom = (int64_t)rl - 2 * (int64_t)best_r + (int64_t)rr;
        if (denom < 0) {
            lag_x16 += (int32_t)((((int64_t)rl - (int64_t)rr) * 16) / (2 * denom));
        }
    }
    /* Bins to tenths of counts: lag * res / 256. */
    const uint32_t period_x10 =
        ((uint32_t)lag_x16 * res * 10U + 8U * IQS9151_MAP_BINS) / (16U * IQS9151_MAP_BINS);
    const uint16_t measured =
        (uint16_t)CLAMP(period_x10, IQS9151_RIPPLE_MIN_PERIOD_X10, IQS9151_RIPPLE_MAX_PERIOD_X10);
    /* On a faint wave the best lag wanders from one rebuild to the next.
     * Take a measurement only when it agrees with the previous one to
     * within two counts; a wandering one changes nothing. */
    const int32_t agree = (int32_t)measured - (int32_t)map->period_cand_x10;
    map->period_cand_x10 = measured;
    if (agree >= 20 || agree <= -20) {
        return;
    }
    const int32_t moved = (int32_t)measured - (int32_t)map->period_x10;
    if (moved < 20 && moved > -20) {
        return;
    }
    map->period_x10 = measured;
    /* A new window means what was learned so far was against a reference
     * with some of the wave still in it. Keep it -- it has the right period
     * -- but let the samples taken against the right reference outweigh it
     * soon, rather than one thirty-second at a time. */
    for (size_t b = 0; b < IQS9151_MAP_BINS; b++) {
        map->n[b] = MIN(map->n[b], 8U);
    }
}

static void iqs9151_map_rebuild(struct iqs9151_ripple_map *map) {
    /*
     * The slope is 1/g, normalised so that its mean over the bins -- which
     * are equal widths of *reported* position -- is one: then a stroke's
     * corrected length equals its reported length and only the pace inside
     * it changes. Normalising by mean(g) instead is off by mean(g) x
     * mean(1/g), a tenth on a wave this deep.
     */
    /* Two passes rather than a table of 1/g: this runs in the frame work,
     * on the system work queue, and a kilobyte of locals there is a
     * kilobyte closer to the overflow that reboots the half. */
    uint64_t sum = 0;
    uint32_t cnt = 0;
    for (size_t b = 0; b < IQS9151_MAP_BINS; b++) {
        if (map->n[b] >= IQS9151_MAP_MIN_SAMPLES && map->g[b] > 0) {
            sum += ((uint32_t)IQS9151_MAP_ONE * IQS9151_MAP_ONE + (uint32_t)map->g[b] / 2U) /
                   (uint32_t)map->g[b];
            cnt++;
        }
    }
    if (cnt == 0U) {
        return;
    }
    const uint32_t mean_inv = MAX(1U, (uint32_t)(sum / cnt)); /* 1/g in Q12 */
    for (size_t b = 0; b < IQS9151_MAP_BINS; b++) {
        if (map->n[b] >= IQS9151_MAP_MIN_SAMPLES && map->g[b] > 0) {
            const uint32_t inv =
                ((uint32_t)IQS9151_MAP_ONE * IQS9151_MAP_ONE + (uint32_t)map->g[b] / 2U) /
                (uint32_t)map->g[b];
            map->corr[b] = (uint16_t)CLAMP((inv * 256U + mean_inv / 2U) / mean_inv,
                                           IQS9151_MAP_CORR_MIN, IQS9151_MAP_CORR_MAX);
        } else {
            map->corr[b] = 256;
        }
    }
    /* L at every bin edge: the slopes summed from the left edge of the pad.
     * The report is a difference of two of these, so where the sum starts
     * does not matter; that it is one table for both ends of the movement
     * does -- the same frame can then never be counted twice or not at all. */
    map->lut[0] = 0;
    for (size_t b = 0; b < IQS9151_MAP_BINS; b++) {
        map->lut[b + 1] = map->lut[b] + map->corr[b];
    }
    /* prev_l is against the old table; re-anchor it so the rebuild itself
     * does not move the pointer. */
    if (map->have_prev) {
        map->prev_l = (int32_t)iqs9151_map_l(map, map->prev_abs);
    }
    iqs9151_map_measure_period(map);
}

static uint32_t iqs9151_map_bin(const struct iqs9151_ripple_map *map, uint32_t pos) {
    const uint32_t res = MAX(1U, (uint32_t)map->resolution);
    return MIN(IQS9151_MAP_BINS - 1U, (pos * IQS9151_MAP_BINS) / res);
}

/*
 * The correction at a position, interpolated between the two nearest bin
 * centres. A bin is a tenth of the wave on the long axis, and the wave is
 * steep between its crest and trough: read as a staircase the table would
 * leave a fifth of the ripple behind; read as a line through the bin
 * centres it leaves a few percent.
 */
/*
 * The corrected absolute position, L(pos), in 1/256 bin: the position the
 * finger is really at, given where the pad says it is. Piecewise linear
 * between the bin edges of lut[], whose slope in each bin is that bin's
 * corr. Where nothing has been learned the slope is one and L is the
 * identity.
 */
static uint32_t iqs9151_map_l(const struct iqs9151_ripple_map *map, uint32_t pos) {
    const uint32_t res = MAX(1U, (uint32_t)map->resolution);
    const uint32_t q = MIN((IQS9151_MAP_BINS << 8) - 1U, (pos * (IQS9151_MAP_BINS << 8)) / res);
    const uint32_t b = q >> 8;
    const uint32_t frac = q & 0xFFU;
    return map->lut[b] + ((uint32_t)map->corr[b] * frac + 128U) / 256U;
}

/* Returns true when a snapshot was taken and wants writing. */
/*
 * Nothing here may touch flash or the radio: this runs in the frame work
 * on the system work queue, on the wireless half too, and a flash write
 * there waits for gaps between radio events while the pointer and the keys
 * wait behind it. The first version of the period measurement wrote the
 * period to flash from here whenever it moved, and on a faint wave it
 * moved often: after a few minutes of use the wireless half's pointer
 * went heavy. What the app is told goes out as a memory-only value, and
 * the learned count no more than every ten seconds, since each one crosses
 * the link between the halves.
 */
#define IQS9151_MAP_PUBLISH_MIN_MS 10000

static bool iqs9151_map_lift(struct iqs9151_ripple_map *map, char axis, int64_t now_ms) {
    memset(&map->ref, 0, sizeof(map->ref));
    map->have_prev = false;
    map->rem_fp = 0;
    if (map->dirty && (now_ms - map->published_ms) >= IQS9151_MAP_PUBLISH_MIN_MS) {
        map->published_ms = now_ms;
        iqs9151_map_publish(map, axis);
    }
    if (map->period_x10 != 0U && map->period_x10 != map->period_told_x10) {
        map->period_told_x10 = map->period_x10;
        iqs9151_setting_ripple_measured(axis, map->period_x10);
#if IS_ENABLED(CONFIG_INPUT_IQS9151_MOTION_TRACE)
        LOG_WRN("m%c period %u.%u", axis, map->period_x10 / 10U, map->period_x10 % 10U);
#endif
    }
    return iqs9151_map_snapshot(map, axis, now_ms);
}

static int16_t iqs9151_map_apply(struct iqs9151_ripple_map *map, char axis, uint16_t abs_pos,
                                 int16_t value, int64_t now_ms) {
    const uint16_t resolution = (uint16_t)atomic_get(
        (axis == 'y') ? &iqs9151_requested_resolution_y : &iqs9151_requested_resolution_x);
    if (resolution == 0U) {
        return value; /* before the first resolution request is served */
    }
    if (map->resolution != resolution) {
        /* A new scale is a new pad as far as the table is concerned. */
        iqs9151_map_restore(map, axis, resolution);
    }

    /* The movement in this report happened between last frame's position
     * and this one's; what it teaches belongs to the bin in the middle. */
    const uint32_t mid = map->have_prev ? ((uint32_t)map->prev_abs + abs_pos) / 2U : abs_pos;
    const bool had_prev = map->have_prev;
    const int32_t l_now = (int32_t)iqs9151_map_l(map, abs_pos);
    const int32_t l_before = had_prev ? map->prev_l : l_now;
    map->prev_abs = abs_pos;
    map->prev_l = l_now;
    map->have_prev = true;
    const uint32_t bin = iqs9151_map_bin(map, mid);

    /* The reference: the local mean over one period of the wave -- from
     * the geometry until the table has learned enough to measure it, then
     * from the table (see iqs9151_map_measure_period) -- so the ratio is to
     * what the finger really did with the wave averaged out. The window
     * holds a limited number of reports, so it must not be longer than a
     * slow stroke can fill. */
    const int32_t period =
        MAX(8, (int32_t)((map->period_x10 != 0U) ? map->period_x10 : map->period_geo_x10) / 10);
    const uint32_t spans = iqs9151_ripple_ref_push(&map->ref, value, period);

    /*
     * Learn only from movement fast enough to be measured: at least two
     * counts a frame on average over the window. Below that the pad's
     * whole-count output is mostly quantisation -- a finger drifting a
     * third of a count a frame reports 0, 0, 1, 0, 0, 1 -- and a ratio of
     * one such report to the window's mean is noise of half its size.
     * That noise was what the horizontal table learned from vertical
     * strokes (whose horizontal component is exactly that drift): a
     * random slope per bin, felt as a wave that was never there. The
     * correction still applies at every speed; the wave is positional.
     */
    const bool measurable = map->ref.dist >= 2U * map->ref.frames;

    if (had_prev && spans > 0U && measurable && (int32_t)map->ref.dist >= period) {
        const int32_t mag = MIN(255, (value < 0) ? -(int32_t)value : (int32_t)value);
        /* sample = (mag/spans) / (dist/frames), Q12, capped at 4.0. */
        const int64_t ratio = ((int64_t)mag * map->ref.frames * IQS9151_MAP_ONE) /
                              ((int64_t)map->ref.dist * spans);
        const int32_t sample = (int32_t)MIN(ratio, 4 * IQS9151_MAP_ONE);

        /* A true running mean for the first samples, an exponential one
         * after: a bin at ordinary stroke speed sees one or two samples a
         * pass, and a 1/32 average that started from a single quantised
         * frame would still be that frame twenty passes later. */
        if (map->n[bin] == 0U) {
            map->g[bin] = (int16_t)sample;
        } else {
            const int32_t weight = MIN((int32_t)map->n[bin] + 1, IQS9151_MAP_MEAN_CAP);
            map->g[bin] = (int16_t)(map->g[bin] + (sample - map->g[bin]) / weight);
        }
        if (map->n[bin] < UINT8_MAX) {
            map->n[bin]++;
        }
        map->dirty = true;
        map->steps_since_save++;

        if (++map->tick >= IQS9151_MAP_REBUILD_EVERY) {
            map->tick = 0;
            iqs9151_map_rebuild(map);
        }

#if IS_ENABLED(CONFIG_INPUT_IQS9151_MOTION_TRACE)
        /* Almost two periods from the middle of the pad, in percent, so
         * the devtool log shows the wave's shape and depth as learned; and
         * how many bins have enough to correct. Two lines of nine bins:
         * the devtool keeps 47 characters of a line and drops the rest.
         *   "mx 96 104: 100 100 ..." = axis x, 96 bins learned, bins 104..
         */
        if (++map->updates >= IQS9151_MAP_TRACE_EVERY) {
            map->updates = 0;
            uint32_t learned = 0;
            for (size_t b = 0; b < IQS9151_MAP_BINS; b++) {
                learned += (map->n[b] >= IQS9151_MAP_MIN_SAMPLES) ? 1U : 0U;
            }
            for (size_t from = 104; from < 122; from += 9) {
                char line[40];
                size_t at = 0;
                for (size_t b = from; b < from + 9 && at + 5 < sizeof(line); b++) {
                    at += (size_t)snprintf(&line[at], sizeof(line) - at, " %u",
                                           (unsigned)((map->corr[b] * 100U + 128U) / 256U));
                }
                LOG_WRN("m%c %u %u:%s", axis, learned, (unsigned)from, line);
            }
        }
#endif
    }

    /*
     * The report: how far the finger really moved, L(now) - L(before), in
     * counts. L is in 1/256 bin, so scale by the bin width; the fraction is
     * carried. The pad's own relative value is not used here at all -- it
     * is the difference of the two reported positions (checked against the
     * frame trace: rel equals the change in abs, frame for frame), and this
     * is the same difference taken after the correction.
     *
     * Before the first frame of a stroke there is no "before", and the pad's
     * value is passed through: the finger has landed, not moved.
     */
    if (!had_prev) {
        return value;
    }
    const int64_t moved_q8 =
        ((int64_t)(l_now - l_before) * (int64_t)map->resolution + IQS9151_MAP_BINS / 2) /
        IQS9151_MAP_BINS; /* counts, Q8 */
    map->rem_fp += (int32_t)moved_q8;
    int32_t out = map->rem_fp / 256; /* toward zero; the carry keeps its sign */
    map->rem_fp -= out * 256;
    return (int16_t)CLAMP(out, INT16_MIN, INT16_MAX);
}

static int16_t iqs9151_ripple_apply(struct iqs9151_ripple_eq *eq, char axis, uint16_t abs_pos,
                                    int16_t value, uint16_t setting_x10, bool scanning) {
    const uint16_t resolution = (uint16_t)atomic_get(
        (axis == 'y') ? &iqs9151_requested_resolution_y : &iqs9151_requested_resolution_x);
    if (scanning != eq->scanning) {
        eq->scanning = scanning;
        eq->scan.resolution = resolution;
        iqs9151_ripple_scan_reset(&eq->scan, axis);
    } else if (scanning && eq->scan.resolution != resolution) {
        /* The period is in the device's absolute units, so a new resolution
         * is a new period: forget the old one and sweep. Zero is not a
         * resolution but the moment before the first request is served. */
        const bool changed = eq->scan.resolution != 0U;
        eq->scan.resolution = resolution;
        if (changed) {
            /* Forgotten in flash too, or the old period would be back at
             * the next boot. */
            iqs9151_ripple_remember(axis, 0);
            iqs9151_ripple_scan_reset(&eq->scan, axis);
        }
    }

    const uint16_t period_x10 = scanning ? eq->scan.found_x10 : setting_x10;

    if (period_x10 == 0U && !scanning) {
        if (eq->period_x10 != 0U) {
            iqs9151_ripple_reset(eq, 0);
        }
        return value;
    }
    if (eq->period_x10 != period_x10) {
        iqs9151_ripple_reset(eq, period_x10);
    }

    /* The reference window is in whole counts; a tenth either way is nothing
     * to an average, and while the search has not found a period yet any
     * plausible one will do. */
    const uint16_t learn_x10 = (period_x10 != 0U) ? period_x10 : 760U;
    const int32_t period = MAX(1, (learn_x10 + 5) / 10);

    /* The reference: this axis's movement averaged over the last period of
     * travel, which is what the finger really did with the ripple summed out.
     * Travel, not displacement -- a stroke that turns round inside the window
     * still moved, and the ratio below is of magnitudes. */
    const uint32_t spans = iqs9151_ripple_ref_push(&eq->ref, value, period);

    /* Learn only once the reference spans a whole period. The ratio of this
     * report's movement per frame to that average is one noisy sample of
     * g(phase); LMS on the sinusoidal regressors averages the noise out in
     * about a stroke. */
    if (spans > 0U && (int32_t)eq->ref.dist >= period) {
        const int32_t mag = MIN(255, (value < 0) ? -(int32_t)value : (int32_t)value);
        /* sample = (mag/spans) / (dist/frames), in Q15, capped at 4.0. */
        const int64_t ratio =
            ((int64_t)mag * eq->ref.frames * 32768) / ((int64_t)eq->ref.dist * spans);
        const int32_t sample = (int32_t)MIN(ratio, 4 * 32768);

        if (scanning) {
            iqs9151_ripple_scan_learn(&eq->scan, abs_pos, sample);
            const uint16_t found = iqs9151_ripple_scan_verdict(&eq->scan, axis);
            if (found != 0U) {
                /* WRN so it shows in the devtool log without raising the level. */
                LOG_WRN("ripple %c: period %u.%u found", axis, found / 10U, found % 10U);
                iqs9151_ripple_remember(axis, found);
                /* Adopted from the next frame; this one is still learned at
                 * the old period below, which the reset then discards. */
            }
        }

        if (period_x10 != 0U) {
            const uint32_t bin = iqs9151_ripple_bin(abs_pos, period_x10);
            const int32_t e = sample - iqs9151_ripple_g(eq, bin);
            for (uint32_t k = 0; k < IQS9151_RIPPLE_HARMONICS; k++) {
                int32_t c, s;
                iqs9151_ripple_basis(bin, k + 1U, &c, &s);
                /* Each harmonic is allowed less than the one below it: the
                 * wave is mostly fundamental, and the limits together keep g
                 * above zero. */
                const int32_t limit = (IQS9151_RIPPLE_COEF_LIMIT / (int32_t)(k + 1U))
                                      << IQS9151_RIPPLE_COEF_GUARD;
                eq->a[k] = CLAMP(eq->a[k] + iqs9151_ripple_step(e, c, IQS9151_RIPPLE_MU_SHIFT),
                                 -limit, limit);
                eq->b[k] = CLAMP(eq->b[k] + iqs9151_ripple_step(e, s, IQS9151_RIPPLE_MU_SHIFT),
                                 -limit, limit);
            }

            if (++eq->tick >= IQS9151_RIPPLE_NORM_EVERY) {
                eq->tick = 0;
                iqs9151_ripple_rebuild(eq);
            }

#if IS_ENABLED(CONFIG_INPUT_IQS9151_MOTION_TRACE)
            /* The learned wave, in thousandths, so a devtool log shows whether
             * it has settled and how much of it is in each harmonic. */
            if (++eq->updates >= IQS9151_RIPPLE_TRACE_EVERY) {
                eq->updates = 0;
                LOG_WRN("eq %c p%u.%u a%d,%d,%d,%d b%d,%d,%d,%d", axis, period_x10 / 10U,
                        period_x10 % 10U, iqs9151_ripple_milli(eq->a[0]),
                        iqs9151_ripple_milli(eq->a[1]), iqs9151_ripple_milli(eq->a[2]),
                        iqs9151_ripple_milli(eq->a[3]), iqs9151_ripple_milli(eq->b[0]),
                        iqs9151_ripple_milli(eq->b[1]), iqs9151_ripple_milli(eq->b[2]),
                        iqs9151_ripple_milli(eq->b[3]));
            }
#endif
        }
    }

    if (period_x10 == 0U) {
        return value; /* searching, nothing found yet: pass through */
    }

    /* Divide the ripple out of this report, carrying the fraction. */
    const uint32_t bin = iqs9151_ripple_bin(abs_pos, period_x10);
    eq->rem_fp += (int32_t)value * (int32_t)eq->corr[bin];
    int32_t out = eq->rem_fp / 256; /* toward zero; the carry keeps its sign */
    eq->rem_fp -= out * 256;

    return (int16_t)CLAMP(out, INT16_MIN, INT16_MAX);
}

static void iqs9151_apply_cursor_gain(struct iqs9151_data *data, struct iqs9151_frame *frame,
                                      int64_t now_ms) {
    if (frame->finger_count != 1U) {
        /*
         * Finger gone, or a second one down: drop what is owed rather than
         * glide on for a few reports. A pointer that keeps moving after you
         * lift is worse than one that stops a pixel short. The distance
         * smoother is reset for the same reason -- its window must not span a
         * lift, or a two-finger gesture.
         */
        data->cursor_gain_remainder_x = 0;
        data->cursor_gain_remainder_y = 0;
        iqs9151_dist_smoother_reset(&data->dist_smoother_x);
        iqs9151_dist_smoother_reset(&data->dist_smoother_y);
        /* The equalisers keep what they learned; only the stroke state goes. */
        iqs9151_ripple_lift(&data->ripple_x);
        iqs9151_ripple_lift(&data->ripple_y);
        const bool save_x = iqs9151_map_lift(&data->map_x, 'x', now_ms);
        const bool save_y = iqs9151_map_lift(&data->map_y, 'y', now_ms);
        if (save_x || save_y) {
            k_work_submit_to_queue(&iqs9151_recovery_q, &data->map_save_work);
        }
        return;
    }

    /* First, because it is the only stage that needs the absolute position,
     * and because the ripple lives in the raw counts: everything after this
     * sees a report with the wave already divided out. The position map
     * when it is on; else the period equaliser, searching or told. */
    if (atomic_get(&iqs9151_ripple_map_on) != 0) {
        frame->rel_x = iqs9151_map_apply(&data->map_x, 'x', frame->finger1_x, frame->rel_x, now_ms);
        frame->rel_y = iqs9151_map_apply(&data->map_y, 'y', frame->finger1_y, frame->rel_y, now_ms);
    } else {
        const bool scanning = atomic_get(&iqs9151_ripple_auto) != 0;
        frame->rel_x =
            iqs9151_ripple_apply(&data->ripple_x, 'x', frame->finger1_x, frame->rel_x,
                                 (uint16_t)atomic_get(&iqs9151_ripple_period_x), scanning);
        frame->rel_y =
            iqs9151_ripple_apply(&data->ripple_y, 'y', frame->finger1_y, frame->rel_y,
                                 (uint16_t)atomic_get(&iqs9151_ripple_period_y), scanning);
    }

    const int32_t window = (int32_t)atomic_get(&iqs9151_cursor_distance_window);
    frame->rel_x = iqs9151_distance_smooth_axis(&data->dist_smoother_x, frame->rel_x, window);
    frame->rel_y = iqs9151_distance_smooth_axis(&data->dist_smoother_y, frame->rel_y, window);

    const int32_t spread = MAX(1, (int32_t)atomic_get(&iqs9151_cursor_smoothing));

    frame->rel_x = iqs9151_scale_axis(frame->rel_x,
                                      (int32_t)atomic_get(&iqs9151_cursor_gain_x_x10),
                                      spread, &data->cursor_gain_remainder_x);
    frame->rel_y = iqs9151_scale_axis(frame->rel_y,
                                      (int32_t)atomic_get(&iqs9151_cursor_gain_y_x10),
                                      spread, &data->cursor_gain_remainder_y);
}

#if IS_ENABLED(CONFIG_INPUT_IQS9151_MOTION_TRACE)
/*
 * One line per frame that carries movement, as the device delivered it --
 * before the gain, before anything decides what the frame means.
 *
 * Written to answer one question: when the pointer surges and stalls at a
 * steady finger speed, is the device holding the position and releasing it in
 * lumps, or is the driver simply not reading every frame? "+ms" is the time
 * since the previous read of any frame; "q" is how many frames were read in
 * between that carried nothing. Every frame read at +5 with q>0 means the
 * device sat still and then jumped; +30 with q=0 means frames were missed and
 * their movement arrived in one piece.
 */
static void iqs9151_trace_frame(struct iqs9151_data *data, const struct iqs9151_frame *frame,
                                int64_t now_ms) {
    const uint32_t gap = (uint32_t)(now_ms - data->trace_last_read_ms);
    data->trace_last_read_ms = now_ms;

    if (frame->finger_count == 0U) {
        data->trace_quiet = 0;
        return;
    }

    const bool moving = (frame->trackpad_flags & IQS9151_TP_MOVEMENT_DETECTED) != 0U;
    if (frame->rel_x == 0 && frame->rel_y == 0 && !moving) {
        data->trace_quiet++;
        return;
    }

    LOG_WRN("tp +%u q%u x%u y%u r%d,%d m%u i%04x", gap, data->trace_quiet, frame->finger1_x,
            frame->finger1_y, frame->rel_x, frame->rel_y, moving ? 1U : 0U, frame->info_flags);
    data->trace_quiet = 0;
}
#endif

static void iqs9151_work_cb(struct k_work *work) {
    struct iqs9151_data *data = CONTAINER_OF(work, struct iqs9151_data, work);
    const struct device *dev = data->dev;
    const struct iqs9151_config *cfg = dev->config;
    struct iqs9151_frame frame;
    int ret;
    const int64_t now_ms = k_uptime_get();

    /*
     * Leave the bus alone while the recovery thread is rebuilding the device.
     * The interrupt is re-armed so RDY keeps arriving; the frames it brings are
     * ignored until the configuration is back, because until then they describe
     * a differently-shaped pad.
     */
    /*
     * Not re-armed here. RDY is a level interrupt and stays asserted until the
     * window the device opened is serviced; re-arming without reading is an
     * interrupt that fires again the moment it is enabled, a work item per
     * firing, and a system work queue that does nothing else until the
     * rebuild happens to close the window -- which it cannot, starved of the
     * CPU. The rebuild re-arms it when it is done (see the recovery handler).
     */
    if (atomic_get(&data->recovering) != 0) {
        return;
    }

    ret = iqs9151_read_frame(cfg, &frame);
    if (ret != 0) {
        LOG_ERR("frame read failed (%d)", ret);
        (void)iqs9151_set_interrupt(dev, true);
        return;
    }

    /* Inside the communication window, and before the frame is acted on, so a
     * scale change takes effect from the very next report rather than halfway
     * through the gesture that asked for it. */
    iqs9151_apply_requested_resolution(dev);
    iqs9151_apply_requested_filter(dev);

#if IS_ENABLED(CONFIG_INPUT_IQS9151_MOTION_TRACE)
    iqs9151_trace_frame(data, &frame, now_ms);
#endif

    /* Before anything reads the deltas. Only the cursor path uses rel_x/rel_y
     * -- the gestures work from the absolute finger coordinates -- so this
     * changes pointer speed and nothing else. */
    iqs9151_apply_cursor_gain(data, &frame, now_ms);

    iqs9151_process_frame(data, &frame, now_ms);

    /* A frame that asked for a rebuild leaves the interrupt to the rebuild. */
    if (atomic_get(&data->recovering) == 0) {
        (void)iqs9151_set_interrupt(dev, true);
    }
}

static void iqs9151_gpio_cb(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct iqs9151_data *data = CONTAINER_OF(cb, struct iqs9151_data, gpio_cb);

    (void)iqs9151_set_interrupt(data->dev, false);

    ARG_UNUSED(port);
    ARG_UNUSED(pins);

    k_work_submit(&data->work);
}

static int iqs9151_set_interrupt(const struct device *dev, const bool en) {
    const struct iqs9151_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(
        &config->irq_gpio, en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
    return ret;
}

static int iqs9151_run_ati(const struct iqs9151_config *config) {
    uint8_t ctrl[2] = {
        SYSTEM_CONTROL_0,
        SYSTEM_CONTROL_1 | IQS9151_SYS_CTRL_ALP_RE_ATI | IQS9151_SYS_CTRL_TP_RE_ATI,
    };
    return iqs9151_i2c_write(config, IQS9151_ADDR_SYSTEM_CONTROL, ctrl, sizeof(ctrl));
}

static int iqs9151_wait_for_ati(const struct device *dev, uint16_t timeout_ms) {
    const struct iqs9151_config *cfg = dev->config;
    int64_t start_ms = k_uptime_get();

    while ((k_uptime_get() - start_ms) < timeout_ms) {
        uint8_t ctrl[2];
        int ret;

        iqs9151_wait_for_ready(dev, 100);
        ret = iqs9151_i2c_read(cfg, IQS9151_ADDR_SYSTEM_CONTROL, ctrl, sizeof(ctrl));
        if (ret != 0) {
            return ret;
        }

        if ((sys_get_le16(ctrl) &
             (IQS9151_SYS_CTRL_ALP_RE_ATI | IQS9151_SYS_CTRL_TP_RE_ATI)) == 0U) {
            return 0;
        }

        k_sleep(K_MSEC(IQS9151_ATI_POLL_INTERVAL_MS));
    }

    LOG_ERR("ATI timeout after %dms", timeout_ms);
    return -EIO;
}

static int iqs9151_wait_for_post_ati_ready(const struct device *dev, uint16_t timeout_ms) {
    const struct iqs9151_config *cfg = dev->config;
    int64_t start_ms = k_uptime_get();
    bool force_poll = false;
    int64_t rdy_busy_start_ms = 0;

    while ((k_uptime_get() - start_ms) < timeout_ms) {
        uint8_t info[2];
        int ret;

        if (!force_poll && !gpio_pin_get_dt(&cfg->irq_gpio)) {
            if (rdy_busy_start_ms == 0) {
                rdy_busy_start_ms = k_uptime_get();
            }

            if ((k_uptime_get() - rdy_busy_start_ms) >= IQS9151_ATI_TIMEOUT_MS) {
                force_poll = true;
                LOG_DBG("ATI RDY busy too long, force polling I2C");
            } else {
                k_sleep(K_MSEC(IQS9151_ATI_POLL_INTERVAL_MS));
                continue;
            }
        } else {
            rdy_busy_start_ms = 0;
        }

        ret = iqs9151_i2c_read(cfg, IQS9151_ADDR_INFO_FLAGS, info, sizeof(info));
        if (ret == 0) {
            return 0;
        }

        k_sleep(K_MSEC(IQS9151_ATI_POLL_INTERVAL_MS));
    }

    LOG_ERR("ATI post-ready timeout after %dms", timeout_ms);
    return -EIO;
}

static int iqs9151_read_ati_min_count(const struct device *dev, uint16_t *min_count) {
    const struct iqs9151_config *cfg = dev->config;
    const size_t node_count = TRACKPAD_SETTINGS_0_1 * TRACKPAD_SETTINGS_1_0;
    uint16_t min_value = UINT16_MAX;

    if (min_count == NULL || node_count == 0U) {
        return -EINVAL;
    }

    for (size_t index = 0U; index < node_count; index++) {
        uint8_t raw[2];
        int ret;

        ret = iqs9151_i2c_read(cfg, IQS9151_ADDR_ATI_RESULT_BASE + (uint16_t)(index * 2U),
                               raw, sizeof(raw));
        if (ret != 0) {
            return ret;
        }

        const uint16_t value = (uint16_t)(sys_get_le16(raw) & IQS9151_ATI_RESULT_MASK);
        if (value < min_value && value != 0U) {
            min_value = value;
        }
    }

    *min_count = min_value;
    return 0;
}

static int iqs9151_auto_tune_ati(const struct device *dev) {
    const struct iqs9151_config *cfg = dev->config;
    uint8_t div = IQS9151_ATI_AUTO_TUNE_START_DIV;

    while (true) {
        uint16_t min_count = 0U;
        int ret;

        ret = iqs9151_write_tp_ati_div(cfg, div);
        if (ret != 0) {
            LOG_ERR("Failed to set ATI divider %u (%d)", div, ret);
            return ret;
        }

        ret = iqs9151_run_ati(cfg);
        if (ret != 0) {
            LOG_ERR("ATI request failed at div=%u (%d)", div, ret);
            return ret;
        }

        ret = iqs9151_wait_for_ati(dev, IQS9151_ATI_TIMEOUT_MS);
        if (ret != 0) {
            LOG_ERR("ATI wait failed at div=%u (%d)", div, ret);
            return ret;
        }

        ret = iqs9151_wait_for_post_ati_ready(dev, IQS9151_ATI_TIMEOUT_MS);
        if (ret != 0) {
            LOG_ERR("ATI post-ready failed at div=%u (%d)", div, ret);
            return ret;
        }

        ret = iqs9151_read_ati_min_count(dev, &min_count);
        if (ret != 0) {
            LOG_ERR("ATI result read failed at div=%u (%d)", div, ret);
            return ret;
        }

        LOG_DBG("ATI tune div=%u min_count=%u threshold=%u", div, min_count,
                IQS9151_ATI_AUTO_TUNE_MIN_COUNT);

        if (min_count >= IQS9151_ATI_AUTO_TUNE_MIN_COUNT || div <= IQS9151_ATI_AUTO_TUNE_MIN_DIV) {
            return 0;
        }

        div--;
    }
}

static int iqs9151_ack_reset(const struct device *dev) {
    const struct iqs9151_config *cfg = dev->config;
    uint8_t ctrl[2];
    int ret;

    ret = iqs9151_i2c_read(cfg, IQS9151_ADDR_SYSTEM_CONTROL, ctrl, sizeof(ctrl));
    if (ret != 0) {
        LOG_ERR("Read SYSTEM CONTROL(ACK_RESET) failed (%d)", ret);
        return ret;
    }

    uint16_t config = sys_get_le16(ctrl);
    config |= IQS9151_SYS_CTRL_ACK_RESET;
    sys_put_le16(config, ctrl);

    iqs9151_wait_for_ready(dev, 500);

    ret = iqs9151_i2c_write(cfg, IQS9151_ADDR_SYSTEM_CONTROL, ctrl, sizeof(ctrl));
    if (ret != 0) {
        LOG_ERR("Wrte SYSTEM CONTROL(ACK_RESET) failed (%d)", ret);
        return ret;
    }

    k_msleep(IQS9151_RSTD_DELAY_MS);
    return ret;
}

static int iqs9151_wait_for_show_reset(const struct device *dev, uint16_t timeout_ms) {
    const struct iqs9151_config *cfg = dev->config;
    int64_t start_ms = k_uptime_get();

    while ((k_uptime_get() - start_ms) < timeout_ms) {
        uint8_t info[2];
        int ret;

        iqs9151_wait_for_ready(dev, 100);
        ret = iqs9151_i2c_read(cfg, IQS9151_ADDR_INFO_FLAGS, info, sizeof(info));
        if (ret != 0) {
            return ret;
        }

        if ((sys_get_le16(info) & IQS9151_INFO_SHOW_RESET) != 0U) {
            return 0;
        }

        k_sleep(K_MSEC(IQS9151_ATI_POLL_INTERVAL_MS));
    }

    LOG_ERR("Show Reset timeout after %dms", timeout_ms);
    return -EIO;
}

static int iqs9151_sw_reset(const struct device *dev) {
    const struct iqs9151_config *cfg = dev->config;
    uint8_t ctrl[2];
    int ret;

    ret = iqs9151_i2c_read(cfg, IQS9151_ADDR_SYSTEM_CONTROL, ctrl, sizeof(ctrl));
    if (ret != 0) {
        LOG_ERR("Read SYSTEM CONTROL(SW_RESET) failed (%d)", ret);
        return ret;
    }

    uint16_t config = sys_get_le16(ctrl);
    config |= IQS9151_SYS_CTRL_SW_RESET;
    sys_put_le16(config, ctrl);

    iqs9151_wait_for_ready(dev, 500);

    ret = iqs9151_i2c_write(cfg, IQS9151_ADDR_SYSTEM_CONTROL, ctrl, sizeof(ctrl));
    if (ret != 0) {
        LOG_ERR("Wrte SYSTEM CONTROL(SW_RESET) failed (%d)", ret);
        return ret;
    }

    ret = iqs9151_wait_for_show_reset(dev, 3000);
    if (ret != 0) {
        return ret;
    }

    return ret;
}

static int iqs9151_set_event_mode(const struct device *dev) {
    const struct iqs9151_config *cfg = dev->config;
    uint8_t config_settings[2];

    int ret = iqs9151_i2c_read(cfg, IQS9151_ADDR_CONFIG_SETTINGS, config_settings, sizeof(config_settings));
    if (ret != 0) {
        return ret;
    }

    uint16_t settings = sys_get_le16(config_settings);
    settings |= IQS9151_CFG_EVENT_MODE;
    sys_put_le16(settings, config_settings);

    iqs9151_wait_for_ready(dev, 500);

    return iqs9151_i2c_write(cfg, IQS9151_ADDR_CONFIG_SETTINGS, config_settings, sizeof(config_settings));
}

static int iqs9151_configure(const struct device *dev) {
    const struct iqs9151_config *cfg = dev->config;
    int ret;

    iqs9151_wait_for_ready(dev, 500);

    ret = iqs9151_write_chunks(dev, cfg, IQS9151_ADDR_ALP_COMPENSATION,
                                    iqs9151_alp_compensation,
                                    ARRAY_SIZE(iqs9151_alp_compensation));
    if (ret) {
        return ret;
    }
    ret = iqs9151_write_chunks(dev, cfg, IQS9151_ADDR_SETTINGS_MINOR,
                                    iqs9151_main_config,
                                    ARRAY_SIZE(iqs9151_main_config));
    if (ret) {
        return ret;
    }
    ret = iqs9151_write_chunks(dev, cfg, IQS9151_ADDR_RX_TX_MAPPING,
                                    iqs9151_rxtx_map,
                                    ARRAY_SIZE(iqs9151_rxtx_map));
    if (ret) {
        return ret;
    }
    ret = iqs9151_write_chunks(dev, cfg, IQS9151_ADDR_CHANNEL_DISABLE,
                                    iqs9151_channel_disable,
                                    ARRAY_SIZE(iqs9151_channel_disable));
    if (ret) {
        return ret;
    }
    ret = iqs9151_write_chunks(dev, cfg, IQS9151_ADDR_SNAP_ENABLE,
                                    iqs9151_snap_enable,
                                    ARRAY_SIZE(iqs9151_snap_enable));
    if (ret) {
        return ret;
    }
    return ret;
}

/*
 * The device's low-speed filter block, requested from anywhere and written
 * from inside the communication window -- the same arrangement as the
 * resolution, for the same reason.
 *
 * Six values behind one generation counter rather than six counters, because
 * they only make sense together and because 0x11EA..0x11F0 is one contiguous
 * run: bottom speed, top speed, bottom beta, static beta, stationary threshold.
 * Seven bytes in one write, then the jitter delta at 0x11F4 on its own. The
 * spinlock is for the struct copy; the generation is what the frame work
 * compares.
 */
static struct iqs9151_filter_tune iqs9151_requested_filter;
static struct k_spinlock iqs9151_filter_lock;
static atomic_t iqs9151_filter_request_generation = ATOMIC_INIT(0);

int iqs9151_request_filter(const struct iqs9151_filter_tune *tune) {
    k_spinlock_key_t key = k_spin_lock(&iqs9151_filter_lock);
    iqs9151_requested_filter = *tune;
    k_spin_unlock(&iqs9151_filter_lock, key);
    atomic_inc(&iqs9151_filter_request_generation);
    return 0;
}

static void iqs9151_apply_requested_filter(const struct device *dev) {
    const struct iqs9151_config *cfg = dev->config;
    struct iqs9151_data *data = dev->data;

    const atomic_val_t generation = atomic_get(&iqs9151_filter_request_generation);
    if (generation == atomic_get(&data->filter_generation)) {
        return;
    }

    struct iqs9151_filter_tune tune;
    k_spinlock_key_t key = k_spin_lock(&iqs9151_filter_lock);
    tune = iqs9151_requested_filter;
    k_spin_unlock(&iqs9151_filter_lock, key);

    uint8_t block[7];
    sys_put_le16(tune.bottom_speed, &block[0]);
    sys_put_le16(tune.top_speed, &block[2]);
    block[4] = tune.bottom_beta;
    block[5] = tune.static_beta;
    block[6] = tune.stationary_threshold;

    int ret = iqs9151_i2c_write(cfg, IQS9151_ADDR_XY_DYNAMIC_FILTER_BOTTOM_SPEED, block,
                                sizeof(block));
    if (ret != 0) {
        LOG_WRN("Failed to apply filter block (%d), retrying next frame", ret);
        return;
    }

    ret = iqs9151_i2c_write(cfg, IQS9151_ADDR_JITTER_FILTER_DELTA, &tune.jitter_delta, 1);
    if (ret != 0) {
        LOG_WRN("Failed to apply jitter delta (%d), retrying next frame", ret);
        return;
    }

    /* The touch thresholds, adjacent bytes; 0 means "leave the register". */
    if (tune.touch_set != 0U && tune.touch_clear != 0U) {
        const uint8_t thresholds[2] = {tune.touch_set, tune.touch_clear};
        ret = iqs9151_i2c_write(cfg, IQS9151_ADDR_TOUCH_SET_THRESHOLD, thresholds,
                                sizeof(thresholds));
        if (ret != 0) {
            LOG_WRN("Failed to apply touch thresholds (%d), retrying next frame", ret);
            return;
        }
    }

    atomic_set(&data->filter_generation, generation);
    LOG_INF("Trackpad filter: speed %u..%u, beta %u/%u, stationary %u, jitter %u, touch %u/%u",
            tune.bottom_speed, tune.top_speed, tune.bottom_beta, tune.static_beta,
            tune.stationary_threshold, tune.jitter_delta, tune.touch_set, tune.touch_clear);
}

static const struct iqs9151_filter_tune iqs9151_kconfig_filter = {
    .bottom_speed = CONFIG_INPUT_IQS9151_DYNAMIC_FILTER_BOTTOM_SPEED,
    .top_speed = CONFIG_INPUT_IQS9151_DYNAMIC_FILTER_TOP_SPEED,
    .bottom_beta = CONFIG_INPUT_IQS9151_DYNAMIC_FILTER_BOTTOM_BETA,
    .static_beta = CONFIG_INPUT_IQS9151_STATIC_FILTER_BETA,
    .stationary_threshold = CONFIG_INPUT_IQS9151_STATIONARY_TOUCH_MOV_THRESHOLD,
    .jitter_delta = CONFIG_INPUT_IQS9151_JITTER_FILTER_DELTA,
    .touch_set = CONFIG_INPUT_IQS9151_TOUCH_SET_THRESHOLD,
    .touch_clear = CONFIG_INPUT_IQS9151_TOUCH_CLEAR_THRESHOLD,
};

/*
 * These writes only tune the device; none of them is required for the trackpad
 * to report. A failure used to abort iqs9151_init(), which left the pad
 * completely silent - no cursor, no scroll, no gestures - for what may be a
 * single unacknowledged I2C transfer. Warn and carry on with the value already
 * in the device configuration instead.
 */
static int iqs9151_apply_kconfig_overrides(const struct device *dev) {
    const struct iqs9151_config *cfg = dev->config;
    uint16_t rotate_bits = 0U;
    int ret;

    /* 90/270 are counterclockwise. */
    if (IS_ENABLED(CONFIG_INPUT_IQS9151_ROTATE_90)) {
        rotate_bits = IQS9151_TRACKPAD_SETTING_SWITCH_XY |
                      IQS9151_TRACKPAD_SETTING_FLIP_Y;
    } else if (IS_ENABLED(CONFIG_INPUT_IQS9151_ROTATE_180)) {
        rotate_bits = IQS9151_TRACKPAD_SETTING_FLIP_X |
                      IQS9151_TRACKPAD_SETTING_FLIP_Y;
    } else if (IS_ENABLED(CONFIG_INPUT_IQS9151_ROTATE_270)) {
        rotate_bits = IQS9151_TRACKPAD_SETTING_SWITCH_XY |
                      IQS9151_TRACKPAD_SETTING_FLIP_X;
    }

    ret = iqs9151_update_bits_u16(cfg, IQS9151_ADDR_TRACKPAD_SETTINGS,
                                  IQS9151_TRACKPAD_SETTING_FLIP_X |
                                      IQS9151_TRACKPAD_SETTING_FLIP_Y |
                                      IQS9151_TRACKPAD_SETTING_SWITCH_XY,
                                  rotate_bits);
    if (ret != 0) {
        LOG_WRN("Failed to apply rotate settings (%d), keeping the device default", ret);
    }

    ret = iqs9151_write_u16(cfg, IQS9151_ADDR_TRACKPAD_ATI_TARGET,
                            (uint16_t)CONFIG_INPUT_IQS9151_ATI_TARGETCOUNT);
    if (ret != 0) {
        LOG_WRN("Failed to apply ATI target (%d), keeping the device default", ret);
    }

    /* Set and clear are adjacent bytes; one write for the pair. */
    {
        const uint8_t thresholds[2] = {
            (uint8_t)CONFIG_INPUT_IQS9151_TOUCH_SET_THRESHOLD,
            (uint8_t)CONFIG_INPUT_IQS9151_TOUCH_CLEAR_THRESHOLD,
        };
        ret = iqs9151_i2c_write(cfg, IQS9151_ADDR_TOUCH_SET_THRESHOLD, thresholds,
                                sizeof(thresholds));
        if (ret != 0) {
            LOG_WRN("Failed to apply touch thresholds (%d), keeping the init table's", ret);
        }
    }

    ret = iqs9151_write_u16(cfg, IQS9151_ADDR_X_RESOLUTION,
                            (uint16_t)CONFIG_INPUT_IQS9151_RESOLUTION_X);
    if (ret != 0) {
        LOG_WRN("Failed to apply X resolution (%d), keeping the device default", ret);
    }

    ret = iqs9151_write_u16(cfg, IQS9151_ADDR_Y_RESOLUTION,
                            (uint16_t)CONFIG_INPUT_IQS9151_RESOLUTION_Y);
    if (ret != 0) {
        LOG_WRN("Failed to apply Y resolution (%d), keeping the device default", ret);
    }

    ret = iqs9151_write_u16(cfg, IQS9151_ADDR_XY_DYNAMIC_FILTER_BOTTOM_SPEED,
                            (uint16_t)CONFIG_INPUT_IQS9151_DYNAMIC_FILTER_BOTTOM_SPEED);
    if (ret != 0) {
        LOG_WRN("Failed to apply dynamic filter bottom speed (%d), keeping the device default", ret);
    }

    ret = iqs9151_write_u16(cfg, IQS9151_ADDR_XY_DYNAMIC_FILTER_TOP_SPEED,
                            (uint16_t)CONFIG_INPUT_IQS9151_DYNAMIC_FILTER_TOP_SPEED);
    if (ret != 0) {
        LOG_WRN("Failed to apply dynamic filter top speed (%d), keeping the device default", ret);
    }

    ret = iqs9151_i2c_write(
        cfg, IQS9151_ADDR_XY_DYNAMIC_FILTER_BOTTOM_BETA,
        (const uint8_t[]){(uint8_t)CONFIG_INPUT_IQS9151_DYNAMIC_FILTER_BOTTOM_BETA}, 1);
    if (ret != 0) {
        LOG_WRN("Failed to apply dynamic filter bottom beta (%d), keeping the device default", ret);
    }

    /*
     * And ask for the resolution a second time, through the deferred path.
     *
     * The write above happens here in init, after a wait_for_ready that is
     * allowed to time out -- and when it does, the device is not listening, the
     * write is NAKed, and all that comes of it is a LOG_WRN nobody sees on a
     * keyboard with no console. The evidence says that is what has been
     * happening: a pad whose resolution measurably did not change after the
     * value in the .conf did.
     *
     * The deferred path writes from inside the frame work, which runs on RDY,
     * so the window is not in question. Requesting it here costs one I2C word
     * on the first frame and makes the init-time write the redundant one rather
     * than the load-bearing one.
     */
    ret = iqs9151_request_resolution((uint16_t)CONFIG_INPUT_IQS9151_RESOLUTION_X,
                                     (uint16_t)CONFIG_INPUT_IQS9151_RESOLUTION_Y);
    if (ret < 0) {
        LOG_WRN("Refused the built-in resolution %d x %d (%d)",
                CONFIG_INPUT_IQS9151_RESOLUTION_X, CONFIG_INPUT_IQS9151_RESOLUTION_Y, ret);
    }

    /* The filter block the same way, and this is the only place the static
     * beta, stationary threshold and jitter delta are set at all -- the direct
     * writes above never covered them, so they stayed at the blob's values. */
    (void)iqs9151_request_filter(&iqs9151_kconfig_filter);

    return 0;
}

/*
 * The runtime coordinate scale: requested from anywhere, written from inside
 * the IC's own communication window.
 *
 * One pair for the whole half rather than one per device, because the pair is
 * a statement about a pad shape and every IQS9151 on a half is the same pad.
 * The generation counter is what lets a second instance notice a request the
 * first one has already served.
 */
static atomic_t iqs9151_requested_resolution_x = ATOMIC_INIT(0);
static atomic_t iqs9151_requested_resolution_y = ATOMIC_INIT(0);
static atomic_t iqs9151_resolution_request_generation = ATOMIC_INIT(0);

int iqs9151_request_resolution(uint16_t x_resolution, uint16_t y_resolution) {
    if (x_resolution == 0U || y_resolution == 0U) {
        return -EINVAL;
    }

    atomic_set(&iqs9151_requested_resolution_x, (atomic_val_t)x_resolution);
    atomic_set(&iqs9151_requested_resolution_y, (atomic_val_t)y_resolution);
    atomic_inc(&iqs9151_resolution_request_generation);

    return 0;
}

/*
 * Called from the frame work, which runs on RDY - so the device is listening.
 * A failed write leaves the generation unclaimed, and the next frame tries
 * again; that is the right answer for a single dropped I2C transfer and costs
 * nothing when there is no request outstanding.
 */
static void iqs9151_apply_requested_resolution(const struct device *dev) {
    const struct iqs9151_config *cfg = dev->config;
    struct iqs9151_data *data = dev->data;

    const atomic_val_t generation = atomic_get(&iqs9151_resolution_request_generation);
    if (generation == atomic_get(&data->resolution_generation)) {
        return;
    }

    const uint16_t x_resolution = (uint16_t)atomic_get(&iqs9151_requested_resolution_x);
    const uint16_t y_resolution = (uint16_t)atomic_get(&iqs9151_requested_resolution_y);

    int ret = iqs9151_write_u16(cfg, IQS9151_ADDR_X_RESOLUTION, x_resolution);
    if (ret != 0) {
        LOG_WRN("Failed to apply X resolution %u (%d), retrying next frame", x_resolution, ret);
        return;
    }

    ret = iqs9151_write_u16(cfg, IQS9151_ADDR_Y_RESOLUTION, y_resolution);
    if (ret != 0) {
        LOG_WRN("Failed to apply Y resolution %u (%d), retrying next frame", y_resolution, ret);
        return;
    }

    atomic_set(&data->resolution_generation, generation);
    LOG_INF("Trackpad resolution set to %u x %u", x_resolution, y_resolution);
}

/*
 * Putting the pad back together after it has reset itself.
 *
 * On a thread of the driver's own, deliberately. The rebuild is the same
 * sequence init runs -- acknowledge the reset, write the settings blob, apply
 * the overrides, re-tune ATI -- and that is seconds of I2C with sleeps in it.
 * The frame work runs on the system work queue, and ZMK's watchdog watches that
 * queue by feeding from a work item on it: anything that occupies it for long
 * enough stops the feeding and the board reboots with nothing in the log to say
 * why. A trackpad having a bad moment must not be able to do that.
 *
 * Rate-limited because a device that resets once often resets again, and
 * rebuilding in a tight loop is how a recoverable fault becomes an unusable
 * keyboard. Past the limit the pad stays down until the next reset report,
 * which is at least honest about what is happening.
 */
#define IQS9151_RECOVERY_MIN_INTERVAL_MS 2000

static K_THREAD_STACK_DEFINE(iqs9151_recovery_stack,
                             CONFIG_INPUT_IQS9151_RECOVERY_STACK_SIZE);
static struct k_work_q iqs9151_recovery_q;
static bool iqs9151_recovery_q_started;

static void iqs9151_recover_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs9151_data *data = CONTAINER_OF(dwork, struct iqs9151_data, recover_work);
    const struct device *dev = data->dev;
    int ret;

    /*
     * The owner's scale, not the built-in one, gets the last word.
     * apply_kconfig_overrides below writes the .conf pair and re-requests it,
     * which would quietly undo whatever was set from the app -- so remember
     * what is wanted now and ask for it again once the rebuild is done.
     */
    const uint16_t wanted_x = (uint16_t)atomic_get(&iqs9151_requested_resolution_x);
    const uint16_t wanted_y = (uint16_t)atomic_get(&iqs9151_requested_resolution_y);
    struct iqs9151_filter_tune wanted_filter;
    {
        k_spinlock_key_t key = k_spin_lock(&iqs9151_filter_lock);
        wanted_filter = iqs9151_requested_filter;
        k_spin_unlock(&iqs9151_filter_lock, key);
    }

    data->recover_count++;
    data->recover_last_ms = k_uptime_get();

    iqs9151_wait_for_ready(dev, 500);

    /* Until this is acknowledged the device keeps reporting the flag, and every
     * frame comes back through the handler that got us here. */
    ret = iqs9151_ack_reset(dev);
    if (ret != 0) {
        LOG_ERR("Could not acknowledge the trackpad's reset (%d); pad stays down", ret);
        goto done;
    }

    iqs9151_wait_for_ready(dev, 500);

    ret = iqs9151_configure(dev);
    if (ret != 0) {
        LOG_ERR("Could not rewrite the trackpad configuration (%d); pad stays down", ret);
        goto done;
    }

    iqs9151_wait_for_ready(dev, 100);
    (void)iqs9151_apply_kconfig_overrides(dev);

    if (wanted_x != 0U && wanted_y != 0U) {
        (void)iqs9151_request_resolution(wanted_x, wanted_y);
    }
    if (wanted_filter.top_speed != 0U || wanted_filter.bottom_beta != 0U) {
        (void)iqs9151_request_filter(&wanted_filter);
    }

    iqs9151_wait_for_ready(dev, 100);
    ret = iqs9151_auto_tune_ati(dev);
    if (ret != 0) {
        LOG_WRN("ATI re-tune failed after reset (%d); carrying on with what is set", ret);
    }

    /* A warning, like the reset it answers: a devtool build that keeps only
     * warnings still shows how many times the pad has come back. */
    LOG_WRN("Trackpad configuration restored (reset #%u)", data->recover_count);

done:
    atomic_set(&data->recovering, 0);
    (void)iqs9151_set_interrupt(dev, true);
}

/*
 * Rate-limited by *scheduling* the rebuild for when the interval is up, not
 * by refusing it. The first version refused, cleared the recovering flag and
 * stamped the time -- so a pad that reset twice in two seconds was handed
 * back with its reset flag unacknowledged, every frame after that came
 * through the reset handler, every one of them refreshed the stamp, and as
 * long as frames kept coming (a finger on the pad) no rebuild ever ran. The
 * pad came back only once the finger had been off it for two seconds and
 * then touched again: "dead for half a minute" on a battery-powered half,
 * where the radio's connection burst at boot can brown the IC out more than
 * once.
 */
static void iqs9151_request_recovery(struct iqs9151_data *data) {
    const int64_t now = k_uptime_get();
    int64_t wait_ms = 0;

    if (data->recover_count > 0U &&
        (now - data->recover_last_ms) < IQS9151_RECOVERY_MIN_INTERVAL_MS) {
        wait_ms = IQS9151_RECOVERY_MIN_INTERVAL_MS - (now - data->recover_last_ms);
        LOG_WRN("Trackpad reset again within %dms; rebuilding in %lldms",
                IQS9151_RECOVERY_MIN_INTERVAL_MS, wait_ms);
    }

    if (!iqs9151_recovery_q_started) {
        LOG_ERR("Recovery queue is not running; trackpad stays on its defaults");
        atomic_set(&data->recovering, 0);
        return;
    }

    /* The recovering flag stays set until the rebuild is done: frames until
     * then describe a pad on its power-on defaults and are ignored. */
    k_work_schedule_for_queue(&iqs9151_recovery_q, &data->recover_work, K_MSEC(wait_ms));
}

static int iqs9151_init(const struct device *dev) {
    const struct iqs9151_config *cfg = dev->config;
    struct iqs9151_data *data = dev->data;
    int ret;
    data->dev = dev;

    LOG_DBG("Initialization Start");

    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    if (!cfg->irq_gpio.port) {
        LOG_ERR("IRQ GPIO not defined");
        return -ENODEV;
    }
    if (!device_is_ready(cfg->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
    if (ret) {
        return ret;
    }

    /*
     * A freshly powered IC is streaming and pulls RDY low within a few tens
     * of milliseconds. One that kept its configuration across an MCU-only
     * reset is in event mode and will not, with nothing on the pad -- see
     * iqs9151_force_comms. Give it the short wait, then ask.
     */
    iqs9151_wait_for_ready(dev, 300);
    if (!gpio_pin_get_dt(&cfg->irq_gpio)) {
        LOG_WRN("No RDY after power-up: pad kept its state across an MCU reset; "
                "requesting a communication window");
        ret = iqs9151_force_comms(dev);
        if (ret != 0) {
            LOG_WRN("Communication window request refused (%d)", ret);
        }
        iqs9151_wait_for_ready(dev, 1200);
    }
    
    // Check Product Number
    ret = iqs9151_check_product_number(dev);
    if (ret != 0) {
        return ret;
    }

    iqs9151_wait_for_ready(dev, 500);

    // SW Reset (Show Reset wait + ACK)
    ret = iqs9151_sw_reset(dev);
    if (ret) {
        LOG_ERR("SW Reset failed (%d)", ret);
        return ret;
    }
    LOG_DBG("SW Reset complete");

    iqs9151_wait_for_ready(dev, 500);

    // ACK Reset
    ret = iqs9151_ack_reset(dev);
    if (ret) {
        LOG_ERR("Reset flag clear failed (%d)", ret);
        return ret;
    }
    LOG_DBG("ACK Reset complete");

    iqs9151_wait_for_ready(dev, 500);

    // Setup Initial Config
    ret = iqs9151_configure(dev);
    if (ret != 0) {
        LOG_ERR("Device configuration failed: %d", ret);
        return ret;
    }
    LOG_DBG("Setup Initial Config complete");

    iqs9151_wait_for_ready(dev, 100);

    ret = iqs9151_apply_kconfig_overrides(dev);
    if (ret != 0) {
        LOG_ERR("Kconfig override apply failed: %d", ret);
        return ret;
    }
    LOG_DBG("Kconfig overrides applied");

    iqs9151_wait_for_ready(dev, 100);

    // ATI auto-tune
    ret = iqs9151_auto_tune_ati(dev);
    if (ret != 0) {
        LOG_ERR("ATI auto-tune failed (%d)", ret);
        return ret;
    }
    LOG_DBG("ATI complete");

    /*
     * One recovery queue for every IQS9151 on this half. Started here, on the
     * first instance to get this far, because a work queue needs a thread and
     * a thread needs somewhere to run: doing it at init keeps the rebuild off
     * the system work queue that ZMK's watchdog feeds from.
     */
    if (!iqs9151_recovery_q_started) {
        k_work_queue_start(&iqs9151_recovery_q, iqs9151_recovery_stack,
                           K_THREAD_STACK_SIZEOF(iqs9151_recovery_stack),
                           CONFIG_INPUT_IQS9151_RECOVERY_THREAD_PRIORITY, NULL);
        (void)k_thread_name_set(&iqs9151_recovery_q.thread, "iqs9151_recover");
        iqs9151_recovery_q_started = true;
    }
    k_work_init_delayable(&data->recover_work, iqs9151_recover_work_handler);
    k_work_init(&data->map_save_work, iqs9151_map_save_work_handler);

    // Setup IRQ Call Back
    k_work_init(&data->work, iqs9151_work_cb);
    k_work_init_delayable(&data->one_finger_click_work, iqs9151_one_finger_click_work_cb);
    k_work_init_delayable(&data->two_finger_click_work, iqs9151_two_finger_click_work_cb);
    k_work_init_delayable(&data->three_finger_click_work, iqs9151_three_finger_click_work_cb);
    k_work_init_delayable(&data->inertia_scroll_work, iqs9151_inertia_scroll_work_cb);
    k_work_init_delayable(&data->inertia_cursor_work, iqs9151_inertia_cursor_work_cb);
    iqs9151_inertia_state_reset(&data->inertia_scroll);
    iqs9151_inertia_state_reset(&data->inertia_cursor);
    iqs9151_ema_reset(&data->scroll_ema_x_fp, &data->scroll_ema_y_fp);
    iqs9151_ema_reset(&data->cursor_ema_x_fp, &data->cursor_ema_y_fp);
    iqs9151_motion_history_reset(&data->scroll_motion_history);
    iqs9151_motion_history_reset(&data->cursor_motion_history);
    iqs9151_one_finger_reset(&data->one_finger);
    iqs9151_two_finger_reset(&data->two_finger);
    iqs9151_clear_one_finger_click_pending(data);
    iqs9151_clear_two_finger_click_pending(data);
    iqs9151_clear_three_finger_click_pending(data);
    data->two_finger_one_lead_valid = false;
    data->two_finger_tail_suppresses_cursor = false;
    data->three_finger_one_lead_valid = false;
    data->three_finger_two_lead_valid = false;
    iqs9151_three_finger_reset(data);
    data->hold_button = 0U;
    iqs9151_reset_finger_history(data);
    gpio_init_callback(&data->gpio_cb, iqs9151_gpio_cb,
                        BIT(cfg->irq_gpio.pin));
    ret = gpio_add_callback(cfg->irq_gpio.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to set DR callback: %d", ret);
        return -EIO;
    }

    iqs9151_wait_for_ready(dev, 100);

    // Set Event Mode
    ret = iqs9151_set_event_mode(dev);
    if (ret) {
        LOG_ERR("Set Event Mode failed (%d)", ret);
        return ret;
    }
    LOG_DBG("Set Event Mode complete complete");

    // start IRQ
    iqs9151_set_interrupt(dev, true);
    LOG_DBG("Initialization complete");
    return 0;
}

#ifdef CONFIG_INPUT_IQS9151_TEST
size_t iqs9151_test_context_size(void) {
    return sizeof(struct iqs9151_data);
}

void iqs9151_test_context_init(void *ctx, const struct device *dev) {
    struct iqs9151_data *data = (struct iqs9151_data *)ctx;

    memset(data, 0, sizeof(*data));
    data->dev = dev;
    k_work_init_delayable(&data->recover_work, iqs9151_recover_work_handler);
    k_work_init(&data->map_save_work, iqs9151_map_save_work_handler);
    k_work_init(&data->work, iqs9151_work_cb);
    k_work_init_delayable(&data->one_finger_click_work, iqs9151_one_finger_click_work_cb);
    k_work_init_delayable(&data->two_finger_click_work, iqs9151_two_finger_click_work_cb);
    k_work_init_delayable(&data->three_finger_click_work, iqs9151_three_finger_click_work_cb);
    k_work_init_delayable(&data->inertia_scroll_work, iqs9151_inertia_scroll_work_cb);
    k_work_init_delayable(&data->inertia_cursor_work, iqs9151_inertia_cursor_work_cb);
    iqs9151_inertia_state_reset(&data->inertia_scroll);
    iqs9151_inertia_state_reset(&data->inertia_cursor);
    iqs9151_ema_reset(&data->scroll_ema_x_fp, &data->scroll_ema_y_fp);
    iqs9151_ema_reset(&data->cursor_ema_x_fp, &data->cursor_ema_y_fp);
    iqs9151_motion_history_reset(&data->scroll_motion_history);
    iqs9151_motion_history_reset(&data->cursor_motion_history);
    iqs9151_one_finger_reset(&data->one_finger);
    iqs9151_two_finger_reset(&data->two_finger);
    iqs9151_clear_one_finger_click_pending(data);
    iqs9151_clear_two_finger_click_pending(data);
    iqs9151_clear_three_finger_click_pending(data);
    data->two_finger_one_lead_valid = false;
    data->two_finger_tail_suppresses_cursor = false;
    data->three_finger_one_lead_valid = false;
    data->three_finger_two_lead_valid = false;
    iqs9151_three_finger_reset(data);
    data->hold_button = 0U;
    iqs9151_reset_finger_history(data);
}

void iqs9151_test_cancel_pending_work(void *ctx) {
    struct iqs9151_data *data = (struct iqs9151_data *)ctx;

    (void)k_work_cancel_delayable(&data->one_finger_click_work);
    (void)k_work_cancel_delayable(&data->two_finger_click_work);
    (void)k_work_cancel_delayable(&data->three_finger_click_work);
    (void)k_work_cancel_delayable(&data->inertia_scroll_work);
    (void)k_work_cancel_delayable(&data->inertia_cursor_work);
    (void)k_work_cancel(&data->work);
}

void iqs9151_test_process_frame(void *ctx,
                                const struct iqs9151_test_frame *frame,
                                int64_t now_ms) {
    struct iqs9151_data *data = (struct iqs9151_data *)ctx;
    const struct iqs9151_frame internal_frame = {
        .rel_x = frame->rel_x,
        .rel_y = frame->rel_y,
        .info_flags = frame->info_flags,
        .trackpad_flags = frame->trackpad_flags,
        .finger_count = frame->finger_count,
        .finger1_x = frame->finger1_x,
        .finger1_y = frame->finger1_y,
        .finger2_x = frame->finger2_x,
        .finger2_y = frame->finger2_y,
    };

    iqs9151_process_frame(data, &internal_frame, now_ms);
}

void iqs9151_test_set_event_hook(iqs9151_test_event_hook_t hook, void *user_data) {
    iqs9151_test_hook.hook = hook;
    iqs9151_test_hook.user_data = user_data;
}

uint16_t iqs9151_test_hold_button(const void *ctx) {
    const struct iqs9151_data *data = (const struct iqs9151_data *)ctx;

    return data->hold_button;
}

void iqs9151_test_force_hold_button(void *ctx, uint16_t button) {
    struct iqs9151_data *data = (struct iqs9151_data *)ctx;

    data->hold_button = button;
}

uint8_t iqs9151_test_prev_finger_count(const void *ctx) {
    const struct iqs9151_data *data = (const struct iqs9151_data *)ctx;

    return data->prev_frame.finger_count;
}

bool iqs9151_test_cursor_inertia_active(const void *ctx) {
    const struct iqs9151_data *data = (const struct iqs9151_data *)ctx;

    return data->inertia_cursor.active;
}

bool iqs9151_test_scroll_inertia_active(const void *ctx) {
    const struct iqs9151_data *data = (const struct iqs9151_data *)ctx;

    return data->inertia_scroll.active;
}

void iqs9151_test_force_pinch_session(void *ctx, bool active) {
    struct iqs9151_data *data = (struct iqs9151_data *)ctx;

    data->two_finger.active = active;
    data->two_finger.mode = active ? IQS9151_2F_MODE_PINCH : IQS9151_2F_MODE_NONE;
}
#endif

#define IQS9151_INIT(inst)                                                \
    static const struct iqs9151_config iqs9151_config_##inst = {    \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                      \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),                     \
  };                                                                          \
  static struct iqs9151_data iqs9151_data_##inst;                 \
  DEVICE_DT_INST_DEFINE(inst, iqs9151_init, NULL,                       \
                        &iqs9151_data_##inst,                           \
                        &iqs9151_config_##inst, POST_KERNEL,            \
                        CONFIG_INPUT_IQS9151_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IQS9151_INIT);
