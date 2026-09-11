zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_INPUT_IQS9151 iqs9151.c)

# The runtime settings pull in ZMK's custom-settings module, so they are only
# compiled when a build asks for them. Without them the driver still builds
# against plain Zephyr -- that is what the inline fallbacks in
# iqs9151_settings.h are for.
zephyr_library_sources_ifdef(CONFIG_INPUT_IQS9151_RUNTIME_SETTINGS iqs9151_settings.c)
zephyr_library_sources_ifdef(CONFIG_INPUT_IQS9151_RUNTIME_SETTINGS_STUDIO_RPC
                             iqs9151_settings_subsystem.c)
