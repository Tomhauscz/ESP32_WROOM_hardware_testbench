idf_component_register(
	SRCS 
		"vl53l0x.c"
	INCLUDE_DIRS 
		"inc"
	REQUIRES
		driver esp_timer
)
