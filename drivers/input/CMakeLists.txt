zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_INPUT_IQS9151 iqs9151.c)

# For <keebon/iqs9151/settings.h>, which the driver reads the runtime settings
# through. The header itself pulls in nothing but Zephyr -- the ZMK side of it
# is compiled in settings/, which is a library that has ZMK's includes. This
# library does not, and cannot be given them: it is Zephyr's own input driver
# library, which this module only amends.
zephyr_library_include_directories(${CMAKE_CURRENT_LIST_DIR}/../../include)
