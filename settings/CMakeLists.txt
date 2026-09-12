# Compiled into `app`, not into a zephyr_library() of our own.
#
# <zmk/studio/custom.h> includes <proto/zmk/custom.pb.h>, which nanopb
# generates into the build tree during the build. That generated directory is
# on the app target's include path and on nothing else, so a library that adds
# ZMK's app/include by hand gets the header and then fails inside it -- which
# is exactly what the second attempt at this did.
#
# Every other module here that registers a Studio subsystem (battery-report,
# runtime-combo, runtime-macro) puts that file in `app` for the same reason.
# Both files go there rather than one, so there is one rule instead of two:
# `app` has every include path a ZMK module could want.
#
# Absolute paths: `app` is defined in another directory, so a relative one
# would depend on CMP0076 being NEW to be resolved against this directory
# rather than app's.
if(CONFIG_INPUT_IQS9151_RUNTIME_SETTINGS)
  target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/iqs9151_settings.c)
endif()

if(CONFIG_INPUT_IQS9151_RUNTIME_SETTINGS_STUDIO_RPC)
  target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/iqs9151_settings_subsystem.c)
endif()
