/*
 * Copyright (c) 2026 Salicylic_acid3
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Registers the trackpad as a Studio custom subsystem.
 *
 * There is no RPC here and there is not meant to be one. The two settings in
 * iqs9151_settings.c travel over the ordinary settings RPC; this file exists
 * because of how a setting reaches the app at all.
 *
 * Every setting carries a `custom_subsystem_id` string, and the settings
 * handler turns that string into the index the app sees by looking it up among
 * the registered Studio subsystems. A setting whose id matches no registered
 * subsystem cannot be given an index, so it is dropped from ListSettings --
 * silently, and from every listing. Without the registration below the two
 * settings would exist on the keyboard and be invisible to the app, which is
 * the same trap that cost a day on the tap dance module and another on the
 * battery one.
 *
 * Only the central needs this: it is the half Studio talks to, and it is the
 * half that relays a write to the other one.
 */

#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zmk/studio/custom.h>

LOG_MODULE_DECLARE(iqs9151, CONFIG_INPUT_IQS9151_LOG_LEVEL);

/* Declared before the registration because the macro takes the handler by
 * name; defining it afterwards keeps the "why" next to the refusal. */
static bool iqs9151_rpc_handle_request(const zmk_custom_CallRequest *req, pb_callback_t *res);

/*
 * Unsecured, matching the settings themselves: reading how a pad is set up is
 * not a secret, and the app should be able to show it without being unlocked.
 * Writing is still gated, by the settings' own write permission.
 */
static struct zmk_rpc_custom_subsystem_meta iqs9151_meta = {
    ZMK_RPC_CUSTOM_SUBSYSTEM_UI_URLS(),
    .security = ZMK_STUDIO_RPC_HANDLER_UNSECURED,
};

ZMK_RPC_CUSTOM_SUBSYSTEM(keebon__trackpad, &iqs9151_meta, iqs9151_rpc_handle_request);

static bool iqs9151_rpc_handle_request(const zmk_custom_CallRequest *req, pb_callback_t *res) {
    ARG_UNUSED(req);
    ARG_UNUSED(res);

    /* This subsystem exists to be named, not to be called. */
    LOG_WRN("trackpad has no RPC of its own; use the settings RPC");
    return false;
}
