zephyr_library()

# These sources include ZMK headers, and ZMK's app/include is not on the path
# of the Zephyr driver library the rest of this driver amends. That is why they
# live here rather than next to iqs9151.c: the first attempt put them there and
# the build failed on <zmk/event_manager.h>, before a board had ever been
# flashed. Same preamble as behaviors/ and input_processors/ for the same
# reason.
set(_zmk_app_include_candidates
  ${ZEPHYR_BASE}/../app/include
  ${ZEPHYR_BASE}/../zmk/app/include
)
foreach(_inc ${_zmk_app_include_candidates})
  if(EXISTS ${_inc})
    zephyr_library_include_directories(${_inc})
    break()
  endif()
endforeach()
zephyr_library_include_directories(${CMAKE_CURRENT_LIST_DIR}/../include)

zephyr_library_sources_ifdef(CONFIG_INPUT_IQS9151_RUNTIME_SETTINGS iqs9151_settings.c)
zephyr_library_sources_ifdef(CONFIG_INPUT_IQS9151_RUNTIME_SETTINGS_STUDIO_RPC
                             iqs9151_settings_subsystem.c)
