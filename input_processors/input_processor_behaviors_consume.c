/*
 * Input processor that invokes behaviors for matched events and then
 * neutralizes the event to avoid mouse button output.
 */

#define DT_DRV_COMPAT zmk_input_processor_behaviors_consume

#include <zephyr/dt-bindings/input/input-event-codes.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/keymap.h>
#include <zmk/behavior.h>
#include <zmk/virtual_key_position.h>
#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>

/* The keymap -- layers, and what is pressed on them -- exists on the central
 * or a non-split board. A peripheral has neither, and never instantiates this
 * processor with held-codes. */
#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#define IP_BEHAVIORS_HAS_KEYMAP 1
#else
#define IP_BEHAVIORS_HAS_KEYMAP 0
#endif

/*
 * Code the matched event is rewritten to. It must be one nothing downstream
 * reacts to: not BTN_0..4 or BTN_TOUCH (the listener turns those into mouse
 * buttons) and not BTN_8/BTN_9 (the IQS9151 driver's touch state events,
 * which dual_pad and the drag click behavior read - rewriting to BTN_8 with
 * value 0 looked like "all fingers lifted" to them). 0x10F sits in the gap
 * between BTN_9 and BTN_MOUSE that Linux leaves undefined.
 */
#define IP_BEHAVIORS_CONSUME_NEUTRAL_CODE 0x10F

/*
 * "held-codes" are state events rather than gestures: the IQS9151's touch
 * state (BTN_8, 1 while any finger is on that pad, 0 when the last one
 * leaves). Two things differ for them. They are not neutralized, because
 * dual_pad further down the chain reads the same event. And the binding is
 * pressed when the first pad reports a touch and released when the last pad
 * reports none -- both pads go through one of these processors, and without
 * that a finger landing on the second pad would press the binding a second
 * time and the first finger lifting would release it while the other is
 * still there.
 *
 * A held binding follows the layer, not just the finger. It is resolved on
 * the layer that is on top when the touch begins -- that is what makes it
 * per-layer -- and the keymap releases whatever was pressed even after the
 * layers change. Left at that, letting go of a momentary layer key with a
 * finger still on the pad kept the modifier and button of that layer down
 * until the finger lifted: Shift + middle button meant for panning, held
 * into whatever was done next. So the top layer is watched, and when it
 * changes under a touch the binding is released at once, and the new top
 * layer's binding pressed in its place (nothing, if that layer has none).
 * The finger's state is not what the host sees; the layer is.
 *
 * "held-skip-default-layer" keeps the default layer out of it: a touch that
 * begins there presses nothing, whatever the default layer's binding says.
 * Every touch passes through that layer, so a binding there acts on every
 * pointer movement; the momentary layers are where a hold belongs.
 */
#define IP_BEHAVIORS_MAX_HELD 4
#define IP_BEHAVIORS_MAX_DEVICES 8
#define IP_BEHAVIORS_NO_LAYER 0xFF

struct ip_behaviors_config {
    uint8_t index;
    size_t size;
    uint16_t type;

    const uint16_t *codes;
    const struct zmk_behavior_binding *bindings;

    size_t held_size;
    const uint16_t *held_codes;
    bool held_skip_default_layer;
};

struct ip_behaviors_held {
    /* One bit per input device currently reporting a touch. */
    uint8_t by;
    /* The binding is pressed, on `layer` (a layer id). */
    bool pressed;
    uint8_t layer;
    /* Which of cfg->bindings, and the event it was pressed with, for the
     * release the layer listener may have to send. */
    uint8_t binding;
    struct zmk_behavior_binding_event event;
};

struct ip_behaviors_data {
    const struct device *dev;
    struct k_spinlock lock;
    struct ip_behaviors_held held[IP_BEHAVIORS_MAX_HELD];
};

#if IP_BEHAVIORS_HAS_KEYMAP
static uint8_t ip_behaviors_top_layer(void) {
    return zmk_keymap_layer_index_to_id(zmk_keymap_highest_layer_active());
}

static bool ip_behaviors_layer_allowed(const struct ip_behaviors_config *cfg, uint8_t layer) {
    return !cfg->held_skip_default_layer || layer != zmk_keymap_layer_default();
}
#else
static uint8_t ip_behaviors_top_layer(void) { return 0; }
static bool ip_behaviors_layer_allowed(const struct ip_behaviors_config *cfg, uint8_t layer) {
    ARG_UNUSED(cfg);
    ARG_UNUSED(layer);
    return true;
}
#endif

/*
 * Bring one held slot to `want` (pressed on the current top layer, or
 * released). Called with the lock held; the behavior call itself is made
 * outside it by the caller, which is why this only decides.
 */
static bool ip_behaviors_held_plan(const struct ip_behaviors_config *cfg,
                                   struct ip_behaviors_held *slot, bool want,
                                   bool *do_release, bool *do_press) {
    const uint8_t top = ip_behaviors_top_layer();

    *do_release = false;
    *do_press = false;

    if (slot->pressed && (!want || slot->layer != top)) {
        *do_release = true;
        slot->pressed = false;
        slot->layer = IP_BEHAVIORS_NO_LAYER;
    }
    if (want && !slot->pressed && ip_behaviors_layer_allowed(cfg, top)) {
        *do_press = true;
        slot->pressed = true;
        slot->layer = top;
    }
    return *do_release || *do_press;
}

static int ip_behaviors_held_slot(const struct ip_behaviors_config *cfg, uint16_t code) {
    for (size_t i = 0; i < cfg->held_size && i < IP_BEHAVIORS_MAX_HELD; i++) {
        if (cfg->held_codes[i] == code) {
            return (int)i;
        }
    }
    return -1;
}

static int ip_behaviors_consume_handle_event(const struct device *dev, struct input_event *event,
                                             uint32_t param1, uint32_t param2,
                                             struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);

    const struct ip_behaviors_config *cfg = dev->config;
    struct ip_behaviors_data *data = dev->data;

    if (event->type != cfg->type) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    for (size_t i = 0; i < cfg->size; i++) {
        if (cfg->codes[i] == event->code) {
            struct zmk_behavior_binding_event behavior_event = {
                .position = ZMK_VIRTUAL_KEY_POSITION_BEHAVIOR_INPUT_PROCESSOR(
                    state->input_device_index, cfg->index),
                .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
                .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
            };

            const int held = ip_behaviors_held_slot(cfg, event->code);
            if (held >= 0) {
                struct ip_behaviors_held *slot = &data->held[held];
                const unsigned int bit =
                    MIN(state->input_device_index, IP_BEHAVIORS_MAX_DEVICES - 1);
                bool do_release = false;
                bool do_press = false;

                k_spinlock_key_t key = k_spin_lock(&data->lock);
                WRITE_BIT(slot->by, bit, event->value != 0);
                slot->binding = (uint8_t)i;
                if (slot->by != 0U) {
                    slot->event = behavior_event;
                }
                ip_behaviors_held_plan(cfg, slot, slot->by != 0U, &do_release, &do_press);
                const struct zmk_behavior_binding_event pressed_event = slot->event;
                k_spin_unlock(&data->lock, key);

                if (do_release) {
                    LOG_DBG("HELD, release %s", cfg->bindings[i].behavior_dev);
                    (void)zmk_behavior_invoke_binding(&cfg->bindings[i], pressed_event, false);
                }
                if (do_press) {
                    LOG_DBG("HELD, press %s", cfg->bindings[i].behavior_dev);
                    int ret = zmk_behavior_invoke_binding(&cfg->bindings[i], pressed_event, true);
                    if (ret < 0) {
                        return ret;
                    }
                }
                /* Left as it is: others down the chain read this event. */
                return ZMK_INPUT_PROC_CONTINUE;
            }

            LOG_DBG("MATCH, invoke %s for position %d", cfg->bindings[i].behavior_dev,
                    behavior_event.position);
            int ret = zmk_behavior_invoke_binding(&cfg->bindings[i], behavior_event, event->value);
            if (ret < 0) {
                return ret;
            }

            /* Neutralize the event so the listener doesn't emit a mouse click. */
            event->type = INPUT_EV_KEY;
            event->code = IP_BEHAVIORS_CONSUME_NEUTRAL_CODE;
            event->value = 0;

            return ZMK_INPUT_PROC_CONTINUE;
        }
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api ip_behaviors_consume_driver_api = {
    .handle_event = ip_behaviors_consume_handle_event,
};


static int ip_behaviors_consume_init(const struct device *dev) {
    struct ip_behaviors_data *data = dev->data;

    data->dev = dev;
    for (size_t h = 0; h < IP_BEHAVIORS_MAX_HELD; h++) {
        data->held[h].layer = IP_BEHAVIORS_NO_LAYER;
    }
    return 0;
}

#define ENTRY(i, node) ZMK_KEYMAP_EXTRACT_BINDING(i, node)

#define IP_BEHAVIORS_CONSUME_INST(n)                                                               \
    static const uint16_t ip_behaviors_codes_##n[] = DT_INST_PROP(n, codes);                       \
    static const struct zmk_behavior_binding ip_behaviors_bindings_##n[] = {                       \
        LISTIFY(DT_INST_PROP_LEN(n, bindings), ZMK_KEYMAP_EXTRACT_BINDING, (, ), DT_DRV_INST(n))}; \
    BUILD_ASSERT(ARRAY_SIZE(ip_behaviors_codes_##n) == ARRAY_SIZE(ip_behaviors_bindings_##n),      \
                 "codes and bindings need to be the same length");                                 \
    static const uint16_t ip_behaviors_held_codes_##n[] =                                          \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(n, held_codes), (DT_INST_PROP(n, held_codes)), ({0}));   \
    BUILD_ASSERT(ARRAY_SIZE(ip_behaviors_held_codes_##n) <= IP_BEHAVIORS_MAX_HELD,                 \
                 "too many held-codes");                                                           \
    static struct ip_behaviors_data ip_behaviors_data_##n;                                         \
    static const struct ip_behaviors_config ip_behaviors_config_##n = {                            \
        .index = n,                                                                                \
        .type = DT_INST_PROP_OR(n, type, INPUT_EV_KEY),                                            \
        .size = DT_INST_PROP_LEN(n, codes),                                                        \
        .codes = ip_behaviors_codes_##n,                                                           \
        .bindings = ip_behaviors_bindings_##n,                                                     \
        .held_size = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, held_codes),                             \
                                 (DT_INST_PROP_LEN(n, held_codes)), (0)),                          \
        .held_codes = ip_behaviors_held_codes_##n,                                                 \
        .held_skip_default_layer = DT_INST_PROP(n, held_skip_default_layer),                       \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, &ip_behaviors_consume_init, NULL, &ip_behaviors_data_##n,             \
                          &ip_behaviors_config_##n,                                                \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                        \
                          &ip_behaviors_consume_driver_api);

DT_INST_FOREACH_STATUS_OKAY(IP_BEHAVIORS_CONSUME_INST)

#if IP_BEHAVIORS_HAS_KEYMAP && DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
#define IP_BEHAVIORS_DATA_PTR(n) &ip_behaviors_data_##n,
static struct ip_behaviors_data *const ip_behaviors_instances[] = {
    DT_INST_FOREACH_STATUS_OKAY(IP_BEHAVIORS_DATA_PTR)};

/* The top layer changed. Every held slot with fingers on it is brought in
 * line: released if it was pressed on another layer, pressed on the new one
 * if that layer allows it. Runs where behaviors run, so the calls are made
 * directly. */
static int ip_behaviors_layer_listener(const zmk_event_t *eh) {
    if (as_zmk_layer_state_changed(eh) == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    for (size_t n = 0; n < ARRAY_SIZE(ip_behaviors_instances); n++) {
        struct ip_behaviors_data *data = ip_behaviors_instances[n];
        const struct ip_behaviors_config *cfg = data->dev->config;

        for (size_t h = 0; h < cfg->held_size && h < IP_BEHAVIORS_MAX_HELD; h++) {
            struct ip_behaviors_held *slot = &data->held[h];
            bool do_release = false;
            bool do_press = false;

            k_spinlock_key_t key = k_spin_lock(&data->lock);
            const bool touched = slot->by != 0U;
            const uint8_t binding = slot->binding;
            const struct zmk_behavior_binding_event pressed_event = slot->event;
            const bool any = touched && ip_behaviors_held_plan(cfg, slot, true, &do_release,
                                                               &do_press);
            k_spin_unlock(&data->lock, key);

            if (!any) {
                continue;
            }
            if (do_release) {
                LOG_DBG("HELD, layer changed: release %s", cfg->bindings[binding].behavior_dev);
                (void)zmk_behavior_invoke_binding(&cfg->bindings[binding], pressed_event, false);
            }
            if (do_press) {
                LOG_DBG("HELD, layer changed: press %s", cfg->bindings[binding].behavior_dev);
                (void)zmk_behavior_invoke_binding(&cfg->bindings[binding], pressed_event, true);
            }
        }
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(ip_behaviors_consume_layers, ip_behaviors_layer_listener);
ZMK_SUBSCRIPTION(ip_behaviors_consume_layers, zmk_layer_state_changed);
#endif /* IP_BEHAVIORS_HAS_KEYMAP && DT_HAS_COMPAT_STATUS_OKAY */
