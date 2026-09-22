zephyr_library()


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

zephyr_library_sources_ifdef(CONFIG_ZMK_BEHAVIOR_TRACKPAD_TO_POS behavior_trackpad_to_pos.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_BEHAVIOR_ZIP_DYNAMIC_SCALE behavior_zip_dynamic_scale.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_BEHAVIOR_ZIP_DYNAMIC_SCALE_SET behavior_zip_dynamic_scale_set.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_BEHAVIOR_DRAG_CLICK behavior_drag_click.c)
zephyr_library_sources_ifdef(CONFIG_ZMK_BEHAVIOR_PRESS_BOTH behavior_press_both.c)
