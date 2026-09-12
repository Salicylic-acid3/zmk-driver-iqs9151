# <keebon/iqs9151/settings.h>, which both the driver and settings/ read. Global
# rather than per-library: the driver is part of Zephyr's input library and
# settings/ is part of app, and this is the one line that reaches both.
zephyr_include_directories(include)

add_subdirectory(drivers)

if(EXISTS "${APPLICATION_SOURCE_DIR}/include/drivers/behavior.h")
  add_subdirectory(behaviors)
endif()

if(EXISTS "${APPLICATION_SOURCE_DIR}/include/drivers/input_processor.h")
  add_subdirectory(input_processors)
endif()

# The runtime settings need ZMK's event manager, so they are only built in a
# ZMK application. Which of the two files are compiled is then Kconfig's
# business; see settings/CMakeLists.txt.
if(EXISTS "${APPLICATION_SOURCE_DIR}/include/zmk/event_manager.h")
  add_subdirectory(settings)
endif()
