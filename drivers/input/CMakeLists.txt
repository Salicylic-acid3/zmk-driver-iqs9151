zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_INPUT_IQS9151 iqs9151.c)

# The driver reads the runtime settings through <keebon/iqs9151/settings.h>,
# whose directory the module's top-level CMakeLists puts on every library's
# path. The header pulls in nothing but Zephyr on purpose: this is Zephyr's own
# input driver library, which this module only amends, and it has no ZMK
# includes at all. The ZMK side lives in settings/, compiled into app.
