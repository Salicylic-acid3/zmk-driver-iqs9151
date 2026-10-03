#define DT_DRV_COMPAT zmk_behavior_zip_dynamic_scale_set

#include <drivers/behavior.h>
#include <zmk/behavior.h>

#include <dt-bindings/zmk/zip_dynamic_scale.h>
#include <zmk/zip_dynamic_scaler.h>

static int behavior_zip_dynamic_scale_set_binding_pressed(struct zmk_behavior_binding *binding,
                                                          struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);
    int ret = 0;

    uint8_t group = (uint8_t)binding->param1;
    uint16_t value = (uint16_t)binding->param2;

    if (group == ZDS_ALL) {
        ret = zmk_zip_dynamic_scaler_set_scale_x10(ZDS_XY, value);
        if (ret >= 0) {
            ret = zmk_zip_dynamic_scaler_set_scale_x10(ZDS_SC, value);
        }
    } else {
        ret = zmk_zip_dynamic_scaler_set_scale_x10(group, value);
    }

    return ret < 0 ? ret : ZMK_BEHAVIOR_OPAQUE;
}

static int behavior_zip_dynamic_scale_set_binding_released(struct zmk_behavior_binding *binding,
                                                           struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return 0;
}

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)

/* See behavior_zip_dynamic_scale.c: without metadata Studio refuses the
 * assignment. The value is the multiplier times ten, within the clamp the
 * scaler itself applies. */
static const struct behavior_parameter_value_metadata zds_set_group_values[] = {
    {.display_name = "Cursor", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = ZDS_XY},
    {.display_name = "Scroll", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE, .value = ZDS_SC},
    {.display_name = "Cursor and scroll",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
     .value = ZDS_ALL},
};

static const struct behavior_parameter_value_metadata zds_set_scale_values[] = {
    {.display_name = "Speed x10 (10 = normal)",
     .type = BEHAVIOR_PARAMETER_VALUE_TYPE_RANGE,
     .range =
         {
             .min = CONFIG_ZMK_INPUT_PROCESSOR_DYNAMIC_SCALER_MIN_SCALE_X10,
             .max = CONFIG_ZMK_INPUT_PROCESSOR_DYNAMIC_SCALER_MAX_SCALE_X10,
         }},
};

static const struct behavior_parameter_metadata_set zds_set_metadata_set = {
    .param1_values = zds_set_group_values,
    .param1_values_len = ARRAY_SIZE(zds_set_group_values),
    .param2_values = zds_set_scale_values,
    .param2_values_len = ARRAY_SIZE(zds_set_scale_values),
};

static const struct behavior_parameter_metadata zds_set_metadata = {
    .sets_len = 1,
    .sets = &zds_set_metadata_set,
};

#endif /* IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA) */

static const struct behavior_driver_api behavior_zip_dynamic_scale_set_driver_api = {
    .binding_pressed = behavior_zip_dynamic_scale_set_binding_pressed,
    .binding_released = behavior_zip_dynamic_scale_set_binding_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &zds_set_metadata,
#endif
};

#define ZIP_DYN_SCALE_SET_INST(n)                                                                  \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL,                                \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                  \
                            &behavior_zip_dynamic_scale_set_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ZIP_DYN_SCALE_SET_INST)
