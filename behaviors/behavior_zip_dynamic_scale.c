#define DT_DRV_COMPAT zmk_behavior_zip_dynamic_scale

#include <drivers/behavior.h>
#include <zmk/behavior.h>

#include <dt-bindings/zmk/zip_dynamic_scale.h>
#include <zmk/zip_dynamic_scaler.h>

static int behavior_zip_dynamic_scale_binding_pressed(struct zmk_behavior_binding *binding,
                                                      struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);
    int ret = 0;

    uint8_t group = (uint8_t)binding->param1;

    switch (binding->param2) {
    case ZDS_INC:
        if (group == ZDS_ALL) {
            ret = zmk_zip_dynamic_scaler_step(ZDS_XY, 1);
            if (ret >= 0) {
                ret = zmk_zip_dynamic_scaler_step(ZDS_SC, 1);
            }
        } else {
            ret = zmk_zip_dynamic_scaler_step(group, 1);
        }
        break;
    case ZDS_DEC:
        if (group == ZDS_ALL) {
            ret = zmk_zip_dynamic_scaler_step(ZDS_XY, -1);
            if (ret >= 0) {
                ret = zmk_zip_dynamic_scaler_step(ZDS_SC, -1);
            }
        } else {
            ret = zmk_zip_dynamic_scaler_step(group, -1);
        }
        break;
    case ZDS_RST:
        if (group == ZDS_ALL) {
            ret = zmk_zip_dynamic_scaler_reset(ZDS_XY);
            if (ret >= 0) {
                ret = zmk_zip_dynamic_scaler_reset(ZDS_SC);
            }
        } else {
            ret = zmk_zip_dynamic_scaler_reset(group);
        }
        break;
    default:
        return -EINVAL;
    }

    return ret < 0 ? ret : ZMK_BEHAVIOR_OPAQUE;
}

static int behavior_zip_dynamic_scale_binding_released(struct zmk_behavior_binding *binding,
                                                       struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return 0;
}

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)

/* What Studio offers for each parameter. Without this the keymap RPC refuses
 * every assignment (no metadata reads as "no parameters allowed"), and the app
 * has nothing to name the values with. */
static const struct behavior_parameter_value_metadata zds_group_values[] = {
    {.display_name = "Cursor", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = ZDS_XY},
    {.display_name = "Scroll", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = ZDS_SC},
    {.display_name = "Cursor and scroll",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = ZDS_ALL},
};

static const struct behavior_parameter_value_metadata zds_step_values[] = {
    {.display_name = "Faster", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = ZDS_INC},
    {.display_name = "Slower", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = ZDS_DEC},
    {.display_name = "Reset to default",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = ZDS_RST},
};

static const struct behavior_parameter_metadata_set zds_metadata_set = {
    .param1_values = zds_group_values,
    .param1_values_len = ARRAY_SIZE(zds_group_values),
    .param2_values = zds_step_values,
    .param2_values_len = ARRAY_SIZE(zds_step_values),
};

static const struct behavior_parameter_metadata zds_metadata = {
    .sets_len = 1,
    .sets = &zds_metadata_set,
};

#endif /* IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA) */

static const struct behavior_driver_api behavior_zip_dynamic_scale_driver_api = {
    .binding_pressed = behavior_zip_dynamic_scale_binding_pressed,
    .binding_released = behavior_zip_dynamic_scale_binding_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &zds_metadata,
#endif
};

#define ZIP_DYN_SCALE_INST(n)                                                                      \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL,                                \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                  \
                            &behavior_zip_dynamic_scale_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ZIP_DYN_SCALE_INST)
