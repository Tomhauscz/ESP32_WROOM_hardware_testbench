FILE(GLOB app_sources
            ${CMAKE_SOURCE_DIR}/src/*.c
            ${CMAKE_SOURCE_DIR}/src/*.cpp)

idf_component_register(SRCS
                            ${app_sources}
                       INCLUDE_DIRS 
                            "${CMAKE_SOURCE_DIR}/include"
                       REQUIRES
                            driver freertos esp_timer RotaryEncoder)