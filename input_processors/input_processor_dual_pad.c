/*
 * Fuses one finger on each half of a split trackpad into a single gesture.
 *
 * A split keyboard relays processed input events, not raw coordinates, so
 * neither half can recognise a gesture that spans both pads. This processor
 * sits on both listeners of the central half and does that fusion: while both
 * pads report a touch, their relative movement stops being two cursors and
 * becomes one two-finger gesture - moving both the same way scrolls, moving
 * them apart or together zooms.
 *
 * The two references share one device instance, so the state below is common
 * to both sides; param1 says which side an event came from.
 */

#define DT_DRV_COMPAT zmk_input_processor_dual_pad

#include <zephyr/dt-bindings/input/input-event-codes.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/atomic.h>
#include <drivers/input_processor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/keymap.h>
#include <zmk/behavior.h>
#include <zmk/virtual_key_position.h>

#define DUAL_PAD_SIDES 2

/*
 * param2 flags. The devicetree properties set the board's baseline; these flip
 * it per reference, so a listener layer override can hand the same processor a
 * different orientation - which is how the scroll direction follows the OS
 * without a rebuild.
 */
#define DUAL_PAD_FLAG_INVERT_SCROLL BIT(0)
#define DUAL_PAD_FLAG_INVERT_ZOOM BIT(1)

enum dual_pad_mode {
    DUAL_PAD_MODE_NONE = 0,
    DUAL_PAD_MODE_SCROLL,
    DUAL_PAD_MODE_ZOOM,
};

struct dual_pad_config {
    uint8_t index;
    uint16_t touch_code;
    uint16_t scroll_start;
    uint16_t zoom_start;
    uint16_t scroll_divisor;
    uint16_t zoom_divisor;
    bool invert_scroll;
    bool invert_zoom;
    bool has_zoom_binding;
    struct zmk_behavior_binding zoom_binding;
};

struct dual_pad_data {
    /*
     * The two references run in different contexts - the local pad's events
     * arrive on the driver's work queue, the other half's come off the split
     * link - so every access to the state below has to be serialised. Behavior
     * invocation stays outside the lock, since it reaches the keymap and the
     * HID stack and must not run with interrupts masked.
     */
    struct k_spinlock lock;
    bool touched[DUAL_PAD_SIDES];
    int32_t acc_x[DUAL_PAD_SIDES];
    int32_t acc_y[DUAL_PAD_SIDES];
    enum dual_pad_mode mode;
    atomic_t zoom_held;
};

static inline int32_t dual_pad_abs(int32_t v) { return v < 0 ? -v : v; }

/*
 * Suppress an event without dropping it. Returning STOP would take the event's
 * sync flag with it, and the listener only pushes the report it has been
 * accumulating when it sees sync - so a suppressed sync event would strand the
 * wheel value built from the events before it, and scrolling would stall after
 * a notch or two. INPUT_REL_MISC is a code the listener ignores, so the value
 * goes nowhere while the sync flag still arrives.
 */
static int dual_pad_swallow(struct input_event *event) {
    event->code = INPUT_REL_MISC;
    event->value = 0;
    return ZMK_INPUT_PROC_CONTINUE;
}

static void dual_pad_set_zoom_modifier(const struct device *dev, const struct dual_pad_config *cfg,
                                       struct dual_pad_data *data,
                                       struct zmk_input_processor_state *state, bool pressed) {
    if (!cfg->has_zoom_binding) {
        return;
    }

    /* Claim the transition, so two contexts cannot both press or both release. */
    if (!atomic_cas(&data->zoom_held, pressed ? 0 : 1, pressed ? 1 : 0)) {
        return;
    }

    struct zmk_behavior_binding_event behavior_event = {
        .position = ZMK_VIRTUAL_KEY_POSITION_BEHAVIOR_INPUT_PROCESSOR(
            state ? state->input_device_index : 0, cfg->index),
        .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    int ret = zmk_behavior_invoke_binding(&cfg->zoom_binding, behavior_event, pressed);
    if (ret < 0) {
        LOG_WRN("Failed to %s the zoom modifier (%d)", pressed ? "press" : "release", ret);
        atomic_set(&data->zoom_held, pressed ? 0 : 1);
    }
}

/* Caller holds the lock. */
static void dual_pad_reset_locked(struct dual_pad_data *data) {
    data->mode = DUAL_PAD_MODE_NONE;
    for (size_t i = 0; i < DUAL_PAD_SIDES; i++) {
        data->acc_x[i] = 0;
        data->acc_y[i] = 0;
    }
}

static int dual_pad_handle_event(const struct device *dev, struct input_event *event,
                                 uint32_t param1, uint32_t param2,
                                 struct zmk_input_processor_state *state) {
    const struct dual_pad_config *cfg = dev->config;
    struct dual_pad_data *data = dev->data;
    const uint8_t side = param1 ? 1 : 0;
    const bool invert_scroll =
        cfg->invert_scroll != ((param2 & DUAL_PAD_FLAG_INVERT_SCROLL) != 0);
    const bool invert_zoom = cfg->invert_zoom != ((param2 & DUAL_PAD_FLAG_INVERT_ZOOM) != 0);

    if (event->type == INPUT_EV_KEY && event->code == cfg->touch_code) {
        const bool touched = event->value != 0;
        bool release_modifier = false;

        K_SPINLOCK(&data->lock) {
            if (data->touched[side] != touched) {
                data->touched[side] = touched;
                /* Either finger leaving ends the combined gesture. */
                if (!touched) {
                    dual_pad_reset_locked(data);
                    release_modifier = true;
                }
            }
        }

        if (release_modifier) {
            dual_pad_set_zoom_modifier(dev, cfg, data, state, false);
        }

        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->type != INPUT_EV_REL ||
        (event->code != INPUT_REL_X && event->code != INPUT_REL_Y)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    enum { DUAL_PAD_PASS, DUAL_PAD_SWALLOW, DUAL_PAD_EMIT } action = DUAL_PAD_PASS;
    uint16_t out_code = 0;
    int32_t out_value = 0;
    bool press_modifier = false;

    k_spinlock_key_t key = k_spin_lock(&data->lock);

    /* Only one pad in use: ordinary pointer movement. */
    if (data->touched[0] && data->touched[1]) {
        if (event->code == INPUT_REL_X) {
            data->acc_x[side] += event->value;
        } else {
            data->acc_y[side] += event->value;
        }

        /* Movement both fingers share is a scroll; movement that separates them is a zoom. */
        const int32_t common_x = (data->acc_x[0] + data->acc_x[1]) / 2;
        const int32_t common_y = (data->acc_y[0] + data->acc_y[1]) / 2;
        const int32_t separation = data->acc_x[1] - data->acc_x[0];
        const int32_t common_max = MAX(dual_pad_abs(common_x), dual_pad_abs(common_y));

        if (data->mode == DUAL_PAD_MODE_NONE) {
            if (dual_pad_abs(separation) >= cfg->zoom_start &&
                dual_pad_abs(separation) > common_max) {
                data->mode = DUAL_PAD_MODE_ZOOM;
            } else if (common_max >= cfg->scroll_start) {
                data->mode = DUAL_PAD_MODE_SCROLL;
            }
        }

        if (data->mode == DUAL_PAD_MODE_ZOOM) {
            const int32_t out = separation / (int32_t)cfg->zoom_divisor;
            if (out != 0) {
                const int32_t consumed = out * (int32_t)cfg->zoom_divisor;
                data->acc_x[1] -= consumed / 2;
                data->acc_x[0] += consumed - (consumed / 2);

                press_modifier = true;
                out_code = INPUT_REL_WHEEL;
                out_value = invert_zoom ? -out : out;
                action = DUAL_PAD_EMIT;
            } else {
                action = DUAL_PAD_SWALLOW;
            }
        } else if (data->mode == DUAL_PAD_MODE_SCROLL) {
            const bool vertical = dual_pad_abs(common_y) >= dual_pad_abs(common_x);
            const int32_t along = vertical ? common_y : common_x;
            const int32_t out = along / (int32_t)cfg->scroll_divisor;
            if (out != 0) {
                const int32_t consumed = out * (int32_t)cfg->scroll_divisor;
                if (vertical) {
                    data->acc_y[0] -= consumed;
                    data->acc_y[1] -= consumed;
                } else {
                    data->acc_x[0] -= consumed;
                    data->acc_x[1] -= consumed;
                }

                out_code = vertical ? INPUT_REL_WHEEL : INPUT_REL_HWHEEL;
                out_value = invert_scroll ? -out : out;
                action = DUAL_PAD_EMIT;
            } else {
                action = DUAL_PAD_SWALLOW;
            }
        } else {
            /* Not enough to tell yet - swallow it so the pointer stays put. */
            action = DUAL_PAD_SWALLOW;
        }
    }

    k_spin_unlock(&data->lock, key);

    switch (action) {
    case DUAL_PAD_SWALLOW:
        return dual_pad_swallow(event);
    case DUAL_PAD_EMIT:
        if (press_modifier) {
            dual_pad_set_zoom_modifier(dev, cfg, data, state, true);
        }
        event->code = out_code;
        event->value = out_value;
        return ZMK_INPUT_PROC_CONTINUE;
    default:
        return ZMK_INPUT_PROC_CONTINUE;
    }
}

static struct zmk_input_processor_driver_api dual_pad_driver_api = {
    .handle_event = dual_pad_handle_event,
};

static int dual_pad_init(const struct device *dev) { return 0; }

#define DUAL_PAD_ZOOM_BINDING(n)                                                                   \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, bindings),                                                \
                (ZMK_KEYMAP_EXTRACT_BINDING(0, DT_DRV_INST(n))), ({0}))

#define DUAL_PAD_INST(n)                                                                           \
    static struct dual_pad_data dual_pad_data_##n = {0};                                           \
    static const struct dual_pad_config dual_pad_config_##n = {                                    \
        .index = n,                                                                                \
        .touch_code = DT_INST_PROP_OR(n, touch_code, INPUT_BTN_8),                                 \
        .scroll_start = DT_INST_PROP_OR(n, scroll_start, 30),                                      \
        .zoom_start = DT_INST_PROP_OR(n, zoom_start, 60),                                          \
        .scroll_divisor = DT_INST_PROP_OR(n, scroll_divisor, 16),                                  \
        .zoom_divisor = DT_INST_PROP_OR(n, zoom_divisor, 24),                                      \
        .invert_scroll = DT_INST_PROP(n, invert_scroll),                                           \
        .invert_zoom = DT_INST_PROP(n, invert_zoom),                                               \
        .has_zoom_binding = DT_INST_NODE_HAS_PROP(n, bindings),                                    \
        .zoom_binding = DUAL_PAD_ZOOM_BINDING(n),                                                  \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, &dual_pad_init, NULL, &dual_pad_data_##n, &dual_pad_config_##n,       \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &dual_pad_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DUAL_PAD_INST)
