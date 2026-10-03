#define DT_DRV_COMPAT zmk_behavior_trackpad_to_pos

#include <zephyr/kernel.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/matrix.h>

static int behavior_trackpad_to_pos_raise(bool state,
                                          struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    struct zmk_position_state_changed ev = {
        .position = binding->param1,
        .state = state,
        .timestamp = k_uptime_get(),
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
    };

    ARG_UNUSED(event);
    return raise_zmk_position_state_changed(ev);
}

static int behavior_trackpad_to_pos_binding_pressed(
    struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event) {
    return behavior_trackpad_to_pos_raise(true, binding, event);
}

static int behavior_trackpad_to_pos_binding_released(
    struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event) {
    return behavior_trackpad_to_pos_raise(false, binding, event);
}

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)

/* The parameter is a key position. This is wired to gestures in the shield
 * overlay rather than put on keys, but Studio lists every behavior, and one
 * with no metadata is refused outright if it is ever assigned. */
static const struct behavior_parameter_value_metadata tp_to_pos_values[] = {
    {.display_name = "Key position",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_RANGE,
     .range = {.min = 0, .max = ZMK_KEYMAP_LEN - 1}},
};

static const struct behavior_parameter_metadata_set tp_to_pos_metadata_set = {
    .param1_values = tp_to_pos_values,
    .param1_values_len = ARRAY_SIZE(tp_to_pos_values),
};

static const struct behavior_parameter_metadata tp_to_pos_metadata = {
    .sets_len = 1,
    .sets = &tp_to_pos_metadata_set,
};

#endif /* IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA) */

static const struct behavior_driver_api behavior_trackpad_to_pos_driver_api = {
    .binding_pressed = behavior_trackpad_to_pos_binding_pressed,
    .binding_released = behavior_trackpad_to_pos_binding_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &tp_to_pos_metadata,
#endif
};

// Allow multiple instances if needed.
#define TP_TO_POS_INST(n)                                                                          \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL,                                \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                  \
                            &behavior_trackpad_to_pos_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TP_TO_POS_INST)
