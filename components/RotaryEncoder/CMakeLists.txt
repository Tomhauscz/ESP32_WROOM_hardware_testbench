idf_component_register(
    SRCS
		  "RotaryEncoder.cpp"
    INCLUDE_DIRS
		  "inc"
    REQUIRES
		  driver freertos esp_timer
)