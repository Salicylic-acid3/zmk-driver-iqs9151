/*
 * IQS9151 register map helpers
 */

#ifndef ZEPHYR_DRIVERS_INPUT_IQS9151_REGS_H_
#define ZEPHYR_DRIVERS_INPUT_IQS9151_REGS_H_

#include <zephyr/sys/util.h>

#define IQS9151_PRODUCT_NUMBER                 0x09BC

/* Read-only data */
#define IQS9151_ADDR_PRODUCT_NUMBER            0x1000
#define IQS9151_ADDR_RELATIVE_X                0x1014
#define IQS9151_ADDR_RELATIVE_Y                0x1016
#define IQS9151_INFO_FLAGS                     0x1020
#define IQS9151_ADDR_SINGLE_GESTURES           0x101C
#define IQS9151_ADDR_TWO_FINGER_GESTURES       0x101E
#define IQS9151_ADDR_INFO_FLAGS                0x1020
#define IQS9151_ADDR_TRACKPAD_FLAGS            0x1022
#define IQS9151_ADDR_FINGER_DATA               0x1024
#define IQS9151_ADDR_FINGER1_X                 0x1024
#define IQS9151_ADDR_FINGER1_Y                 0x1026
#define IQS9151_ADDR_FINGER2_X                 0x102C
#define IQS9151_ADDR_FINGER2_Y                 0x102E
#define IQS9151_ADDR_TOUCH_STATUS              0x105C

#define IQS9151_COORD_BLOCK_START              IQS9151_ADDR_RELATIVE_X
#define IQS9151_COORD_BLOCK_LENGTH             0x48
#define IQS9151_FINGER_STRIDE                  0x08

/* Configuration addresses */
#define IQS9151_ADDR_ALP_COMPENSATION          0x115C
#define IQS9151_ADDR_I2C_UPDATE_KEY            0x1176
#define IQS9151_ADDR_I2C_ADDR                  0x1177
#define IQS9151_ADDR_SETTINGS_MINOR            0x1178
#define IQS9151_ADDR_SETTINGS_MAJOR            0x1179
#define IQS9151_ADDR_ATI_MULTIPLIERS           0x117A
#define IQS9151_ADDR_ATI_SETTINGS              0x1196
#define IQS9151_ADDR_TRACKPAD_ATI_TARGET       0x1196
#define IQS9151_ADDR_DEVICE_CONFIG             0x11A2
#define IQS9151_ADDR_SYSTEM_CONTROL            0x11BC
#define IQS9151_ADDR_CONFIG_SETTINGS           0x11BE
#define IQS9151_ADDR_OTHER_SETTINGS            0x11C0
#define IQS9151_ADDR_TRACKPAD_SETTINGS         0x11E2
#define IQS9151_ADDR_X_RESOLUTION              0x11E6
#define IQS9151_ADDR_Y_RESOLUTION              0x11E8
#define IQS9151_ADDR_XY_DYNAMIC_FILTER_BOTTOM_SPEED 0x11EA
#define IQS9151_ADDR_XY_DYNAMIC_FILTER_TOP_SPEED 0x11EC
#define IQS9151_ADDR_XY_DYNAMIC_FILTER_BOTTOM_BETA 0x11EE
#define IQS9151_ADDR_XY_STATIC_FILTER_BETA     0x11EF
#define IQS9151_ADDR_STATIONARY_TOUCH_MOV_THRESHOLD 0x11F0
#define IQS9151_ADDR_FINGER_SPLIT_FACTOR       0x11F1
#define IQS9151_ADDR_X_TRIM_VALUE              0x11F2
#define IQS9151_ADDR_Y_TRIM_VALUE              0x11F3
#define IQS9151_ADDR_JITTER_FILTER_DELTA       0x11F4
#define IQS9151_ADDR_FINGER_CONFIDENCE_THRESHOLD 0x11F5
#define IQS9151_ADDR_GESTURE_ENABLE            0x11F6
#define IQS9151_ADDR_TWO_FINGER_GESTURE_ENABLE 0x11F8
#define IQS9151_ADDR_RX_TX_MAPPING             0x1218
#define IQS9151_ADDR_CHANNEL_DISABLE           0x1246
#define IQS9151_ADDR_SNAP_ENABLE               0x129E
#define IQS9151_ADDR_TOUCH_THRESHOLD_ADJUST    0x12F6
#define IQS9151_ADDR_ATI_RESULT_BASE           0xD000

/* Info Flags bits */
#define IQS9151_INFO_SHOW_RESET                BIT(7)
#define IQS9151_INFO_GLOBAL_TP_TOUCH           BIT(9)
#define IQS9151_INFO_TP_TOUCH_TOGGLED          BIT(13)

/* Trackpad Flags bits */
#define IQS9151_TP_MOVEMENT_DETECTED           BIT(4)
#define IQS9151_TP_FINGER_COUNT_MASK           BIT_MASK(4)
#define IQS9151_TP_FINGER1_CONFIDENCE          BIT(8)
#define IQS9151_TP_FINGER2_CONFIDENCE          BIT(9)

/* Trackpad Settings bits (IQS9151_ADDR_TRACKPAD_SETTINGS) */
#define IQS9151_TRACKPAD_SETTING_FLIP_X        BIT(0)
#define IQS9151_TRACKPAD_SETTING_FLIP_Y        BIT(1)
#define IQS9151_TRACKPAD_SETTING_SWITCH_XY     BIT(2)

/* Single-Finger Gesture Enable bits (IQS9151_ADDR_GESTURE_ENABLE) */
#define IQS9151_SFG_SINGLE_TAP                 BIT(0)
#define IQS9151_SFG_PRESS_HOLD                 BIT(3)

/* Two-Finger Gestures bits (IQS9151_ADDR_TWO_FINGER_GESTURES) */
#define IQS9151_TFG_TWO_TAP                    BIT(0)
#define IQS9151_TFG_TWO_PRESS_HOLD             BIT(3)
#define IQS9151_TFG_ZOOM_IN                    BIT(4)
#define IQS9151_TFG_ZOOM_OUT                   BIT(5)
#define IQS9151_TFG_VSCROLL                    BIT(6)
#define IQS9151_TFG_HSCROLL                    BIT(7)

/* System Control bits */
#define IQS9151_SYS_CTRL_SW_RESET              BIT(9)
#define IQS9151_SYS_CTRL_ACK_RESET             BIT(7)
#define IQS9151_SYS_CTRL_ALP_RE_ATI            BIT(6)
#define IQS9151_SYS_CTRL_TP_RE_ATI             BIT(5)
#define IQS9151_SYS_CTRL_TP_RESEED             BIT(3)

/* Configuration Settings bits */
#define IQS9151_CFG_SNAP_EVENT_EN              BIT(15)
#define IQS9151_CFG_SWITCH_EVENT_EN            BIT(14)
#define IQS9151_CFG_TP_TOUCH_EVENT_EN          BIT(13)
#define IQS9151_CFG_ALP_EVENT_EN               BIT(12)
#define IQS9151_CFG_RE_ATI_EVENT_EN            BIT(11)
#define IQS9151_CFG_TP_EVENT_EN                BIT(10)
#define IQS9151_CFG_GESTURE_EVENT_EN           BIT(9)
#define IQS9151_CFG_EVENT_MODE                 BIT(8)
#define IQS9151_CFG_MANUAL_MODE                BIT(7)

#endif /* ZEPHYR_DRIVERS_INPUT_IQS9151_REGS_H_ */
