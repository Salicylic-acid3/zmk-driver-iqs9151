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

/*
 * Code the matched event is rewritten to. It must be one nothing downstream
 * reacts to: not BTN_0..4 or BTN_TOUCH (the listener turns those into mouse
 * buttons) and not BTN_8/BTN_9 (the IQS9151 driver's touch state events,
 * which dual_pad and the drag click behavior read - rewriting to BTN_8 with
 * value 0 looked like "all fingers lifted" to them). 0x10F sits in the gap
 * between BTN_9 and BTN_MOUSE that Linux leaves undefined.
 */
#define IP_BEHAVIORS_CONSUME_NEUTRAL_CODE 0x10F

struct ip_behaviors_config {
    uint8_t index;
    size_t size;
    uint16_t type;

    const uint16_t *codes;
    const struct zmk_behavior_binding *bindings;
};

static int ip_behaviors_consume_handle_event(const struct device *dev, struct input_event *event,
                                             uint32_t param1, uint32_t param2,
                                             struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);

    const struct ip_behaviors_config *cfg = dev->config;

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

static int ip_behaviors_consume_init(const struct device *dev) { return 0; }

#define ENTRY(i, node) ZMK_KEYMAP_EXTRACT_BINDING(i, node)

#define IP_BEHAVIORS_CONSUME_INST(n)                                                               \
    static const uint16_t ip_behaviors_codes_##n[] = DT_INST_PROP(n, codes);                       \
    static const struct zmk_behavior_binding ip_behaviors_bindings_##n[] = {                       \
        LISTIFY(DT_INST_PROP_LEN(n, bindings), ZMK_KEYMAP_EXTRACT_BINDING, (, ), DT_DRV_INST(n))}; \
    BUILD_ASSERT(ARRAY_SIZE(ip_behaviors_codes_##n) == ARRAY_SIZE(ip_behaviors_bindings_##n),      \
                 "codes and bindings need to be the same length");                                 \
    static const struct ip_behaviors_config ip_behaviors_config_##n = {                            \
        .index = n,                                                                                \
        .type = DT_INST_PROP_OR(n, type, INPUT_EV_KEY),                                            \
        .size = DT_INST_PROP_LEN(n, codes),                                                        \
        .codes = ip_behaviors_codes_##n,                                                           \
        .bindings = ip_behaviors_bindings_##n,                                                     \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, &ip_behaviors_consume_init, NULL, NULL, &ip_behaviors_config_##n,     \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                        \
                          &ip_behaviors_consume_driver_api);

DT_INST_FOREACH_STATUS_OKAY(IP_BEHAVIORS_CONSUME_INST)
