/*
 * Drag click behavior: a mouse button that stays pressed while a finger is on
 * the trackpad.
 *
 * Pressing the key presses the button (and optionally activates a layer) like
 * &mkp + &mo. Releasing the key releases the button right away when no pad is
 * being touched. When a finger is on either pad, the release is deferred until
 * every finger has left, so a drag started with a physical switch keeps going
 * as long as the finger keeps moving. This is meant for a switch mounted under
 * the trackpad; keys used for ordinary clicking should stay on &mkp.
 *
 * Touch presence comes from the IQS9151 driver's raw touch state event
 * (CONFIG_INPUT_IQS9151_TOUCH_STATE_ENABLE), which the split link relays from
 * the peripheral half, so a finger on either pad counts. The tracker registers a
 * raw input callback for every device, so it sees the event before any
 * listener chain modifies it, and it needs no entry in the processor lists.
 */

#define DT_DRV_COMPAT zmk_behavior_drag_click

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/keymap.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#ifdef CONFIG_INPUT_IQS9151_TOUCH_STATE_CODE
#define DRAG_CLICK_TOUCH_CODE CONFIG_INPUT_IQS9151_TOUCH_STATE_CODE
#else
#define DRAG_CLICK_TOUCH_CODE INPUT_BTN_8
#endif

#define DRAG_CLICK_MAX_PADS 4
#define DRAG_CLICK_NO_LAYER -1

struct drag_click_config {
    uint8_t button;
    int8_t layer;
};

struct drag_click_data {
    const struct device *dev;
    struct k_work release_work;
    bool held;   /* the key is physically down */
    bool sticky; /* key released, button kept for the finger */
    bool down;   /* the button (and layer) we own is pressed */
};

/* --- touch presence, shared by all instances --- */

static struct k_spinlock drag_click_lock;
static const struct device *drag_click_pads[DRAG_CLICK_MAX_PADS];
static uint8_t drag_click_touched; /* bit per slot of drag_click_pads */

static void drag_click_all_lifted(void);

static int drag_click_pad_slot(const struct device *dev) {
    int free_slot = -1;

    for (int i = 0; i < DRAG_CLICK_MAX_PADS; i++) {
        if (drag_click_pads[i] == dev) {
            return i;
        }
        if (drag_click_pads[i] == NULL && free_slot < 0) {
            free_slot = i;
        }
    }
    if (free_slot >= 0) {
        drag_click_pads[free_slot] = dev;
    }
    return free_slot;
}

static void drag_click_input_cb(struct input_event *evt, void *user_data) {
    ARG_UNUSED(user_data);

    if (evt->type != INPUT_EV_KEY || evt->code != DRAG_CLICK_TOUCH_CODE) {
        return;
    }

    k_spinlock_key_t key = k_spin_lock(&drag_click_lock);
    const int slot = drag_click_pad_slot(evt->dev);
    bool lifted = false;

    if (slot >= 0) {
        const uint8_t before = drag_click_touched;

        WRITE_BIT(drag_click_touched, slot, evt->value != 0);
        lifted = (before != 0U) && (drag_click_touched == 0U);
    }
    k_spin_unlock(&drag_click_lock, key);

    if (lifted) {
        drag_click_all_lifted();
    }
}

INPUT_CALLBACK_DEFINE(NULL, drag_click_input_cb, NULL);

/* --- button and layer --- */

static void drag_click_apply(const struct device *dev, bool pressed) {
    const struct drag_click_config *cfg = dev->config;

    if (pressed) {
        if (cfg->layer != DRAG_CLICK_NO_LAYER) {
            zmk_keymap_layer_activate(cfg->layer, false);
        }
        input_report_key(dev, INPUT_BTN_0 + cfg->button, 1, true, K_FOREVER);
    } else {
        input_report_key(dev, INPUT_BTN_0 + cfg->button, 0, true, K_FOREVER);
        if (cfg->layer != DRAG_CLICK_NO_LAYER) {
            zmk_keymap_layer_deactivate(cfg->layer, false);
        }
    }
}

static void drag_click_release_work_cb(struct k_work *work) {
    struct drag_click_data *data = CONTAINER_OF(work, struct drag_click_data, release_work);

    drag_click_apply(data->dev, false);
}

static int drag_click_pressed(struct zmk_behavior_binding *binding,
                              struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct drag_click_data *data = dev->data;
    bool press = false;

    ARG_UNUSED(event);

    k_spinlock_key_t key = k_spin_lock(&drag_click_lock);
    data->held = true;
    data->sticky = false; /* a deferred drag is now held by the key again */
    if (!data->down) {
        data->down = true;
        press = true;
    }
    k_spin_unlock(&drag_click_lock, key);

    if (press) {
        drag_click_apply(dev, true);
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

static int drag_click_released(struct zmk_behavior_binding *binding,
                               struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct drag_click_data *data = dev->data;
    bool release = false;

    ARG_UNUSED(event);

    k_spinlock_key_t key = k_spin_lock(&drag_click_lock);
    data->held = false;
    if (drag_click_touched != 0U) {
        data->sticky = true;
    } else if (data->down) {
        data->down = false;
        release = true;
    }
    k_spin_unlock(&drag_click_lock, key);

    if (release) {
        drag_click_apply(dev, false);
    } else {
        LOG_DBG("drag click: finger on a pad, keeping the button");
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

#define DRAG_CLICK_DECLARE(n) static struct drag_click_data drag_click_data_##n;
DT_INST_FOREACH_STATUS_OKAY(DRAG_CLICK_DECLARE)

#define DRAG_CLICK_DATA_PTR(n) &drag_click_data_##n,

static struct drag_click_data *const drag_click_instances[] = {
    DT_INST_FOREACH_STATUS_OKAY(DRAG_CLICK_DATA_PTR)};

/* Runs in the input thread: hand the actual release to the system work queue
 * so it happens where behaviors normally run. */
static void drag_click_all_lifted(void) {
    for (size_t i = 0; i < ARRAY_SIZE(drag_click_instances); i++) {
        struct drag_click_data *data = drag_click_instances[i];
        bool release = false;

        k_spinlock_key_t key = k_spin_lock(&drag_click_lock);
        if (data->sticky && !data->held) {
            data->sticky = false;
            if (data->down) {
                data->down = false;
                release = true;
            }
        }
        k_spin_unlock(&drag_click_lock, key);

        if (release) {
            k_work_submit(&data->release_work);
        }
    }
}

static int drag_click_init(const struct device *dev) {
    struct drag_click_data *data = dev->data;

    data->dev = dev;
    k_work_init(&data->release_work, drag_click_release_work_cb);
    return 0;
}

static const struct behavior_driver_api drag_click_driver_api = {
    .binding_pressed = drag_click_pressed,
    .binding_released = drag_click_released,
};

#define DRAG_CLICK_INST(n)                                                                         \
    static const struct drag_click_config drag_click_config_##n = {                                \
        .button = DT_INST_PROP_OR(n, button, 0),                                                   \
        .layer = DT_INST_PROP_OR(n, layer, DRAG_CLICK_NO_LAYER),                                   \
    };                                                                                             \
    BUILD_ASSERT(DT_INST_PROP_OR(n, button, 0) < 5, "button must be 0..4 (MB1..MB5)");             \
    BEHAVIOR_DT_INST_DEFINE(n, drag_click_init, NULL, &drag_click_data_##n,                        \
                            &drag_click_config_##n, POST_KERNEL,                                   \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &drag_click_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DRAG_CLICK_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
