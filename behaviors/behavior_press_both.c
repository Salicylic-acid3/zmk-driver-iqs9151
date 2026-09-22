/*
 * Press-both behavior: one binding that holds two behaviors at once.
 *
 * `&tp_hold LSHIFT MB3` presses the first child behavior with param1 and the
 * second with param2, and releases both (second first) when the key is
 * released. Written for the trackpad's "held while a finger is on the pad"
 * slot, where what people want to hold is a modifier together with a mouse
 * button -- Shift + middle button to pan in a CAD program, say -- and no
 * single stock behavior can hold a keycode and a mouse button together:
 * &kp cannot take a mouse button, &mkp cannot take a modifier, and the
 * runtime macro plays its whole body on press with nothing left to release.
 *
 * The children are ordinary one-parameter behaviors named in `bindings`,
 * &kp and &mkp for the case above. A parameter of 0 skips that child, so
 * the same binding also does "key only" or "button only".
 *
 * Parameter metadata is borrowed from the children, the way hold-tap does
 * it, so the app offers a keycode for the first parameter and a mouse
 * button for the second without knowing anything about this behavior.
 */

#define DT_DRV_COMPAT zmk_behavior_press_both

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct press_both_config {
    const char *first_dev;
    const char *second_dev;
};

struct press_both_data {
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    struct behavior_parameter_metadata_set set;
#endif
};

static int press_both_invoke(const char *behavior_dev, uint32_t param,
                             struct zmk_behavior_binding_event event, bool pressed) {
    if (param == 0U) {
        return 0;
    }

    struct zmk_behavior_binding child = {
        .behavior_dev = behavior_dev,
        .param1 = param,
        .param2 = 0,
    };

    return zmk_behavior_invoke_binding(&child, event, pressed);
}

static int press_both_pressed(struct zmk_behavior_binding *binding,
                              struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct press_both_config *cfg = dev->config;
    int ret;

    ret = press_both_invoke(cfg->first_dev, binding->param1, event, true);
    if (ret < 0) {
        return ret;
    }
    ret = press_both_invoke(cfg->second_dev, binding->param2, event, true);
    if (ret < 0) {
        return ret;
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

static int press_both_released(struct zmk_behavior_binding *binding,
                               struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct press_both_config *cfg = dev->config;

    /* Released in reverse: the button goes before the modifier, so the host
     * never sees the button without the modifier it was pressed with. */
    (void)press_both_invoke(cfg->second_dev, binding->param2, event, false);
    (void)press_both_invoke(cfg->first_dev, binding->param1, event, false);
    return ZMK_BEHAVIOR_OPAQUE;
}

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
static int press_both_parameter_metadata(const struct device *dev,
                                         struct behavior_parameter_metadata *param_metadata) {
    const struct press_both_config *cfg = dev->config;
    struct press_both_data *data = dev->data;
    struct behavior_parameter_metadata child_meta;
    int err;

    err = behavior_get_parameter_metadata(zmk_behavior_get_binding(cfg->first_dev), &child_meta);
    if (err < 0) {
        LOG_WRN("Failed to get the first behavior's parameter metadata: %d", err);
        return err;
    }
    if (child_meta.sets_len > 0) {
        data->set.param1_values = child_meta.sets[0].param1_values;
        data->set.param1_values_len = child_meta.sets[0].param1_values_len;
    }

    err = behavior_get_parameter_metadata(zmk_behavior_get_binding(cfg->second_dev), &child_meta);
    if (err < 0) {
        LOG_WRN("Failed to get the second behavior's parameter metadata: %d", err);
        return err;
    }
    if (child_meta.sets_len > 0) {
        data->set.param2_values = child_meta.sets[0].param1_values;
        data->set.param2_values_len = child_meta.sets[0].param1_values_len;
    }

    param_metadata->sets = &data->set;
    param_metadata->sets_len = 1;
    return 0;
}
#endif /* CONFIG_ZMK_BEHAVIOR_METADATA */

static int press_both_init(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

static const struct behavior_driver_api press_both_driver_api = {
    .binding_pressed = press_both_pressed,
    .binding_released = press_both_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = press_both_parameter_metadata,
#endif
};

#define PRESS_BOTH_INST(n)                                                                         \
    static struct press_both_data press_both_data_##n;                                             \
    static const struct press_both_config press_both_config_##n = {                                \
        .first_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(n, bindings, 0)),                       \
        .second_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(n, bindings, 1)),                      \
    };                                                                                             \
    BEHAVIOR_DT_INST_DEFINE(n, press_both_init, NULL, &press_both_data_##n,                       \
                            &press_both_config_##n, POST_KERNEL,                                   \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &press_both_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PRESS_BOTH_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
