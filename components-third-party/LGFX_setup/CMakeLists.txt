idf_component_register(
    SRCS
		  "LGFX_setup.cpp"
    INCLUDE_DIRS
		  "inc"
    REQUIRES
      driver esp_timer
)

target_include_directories(${COMPONENT_LIB} PRIVATE
    "${CMAKE_SOURCE_DIR}/.pio/libdeps/esp32dev/LovyanGFX/src"
)