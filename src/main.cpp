/*
 * Copyright © 2025 Tomas Hak
 *
 * MIT Licensed as described in the file LICENSE
 */

#include "main.hpp"


/* GLOBAL VARIABLES */
// I2C bus
i2c_dev_t i2c_dev_bus_num_0;

// VL53L0X sensor instance
static vl53l0x_t* TOF_device;
volatile uint16_t TOF_distance = 0;

// ST7789 TFT display instance
static LGFX disp;
uint8_t bitmap[7*50];
volatile bool bitmaps_draw[3] = {true, true, true};

// Rotary encoder
static RotaryEncoder* rotenc;
int64_t rot_counter = 0;

// PCA9685 16-channel PWM driver
static uint16_t PCA9685_PWM_freq_actual = 0;
volatile uint16_t servo_pulse_width = 0;	// [counts]
volatile bool servo_enable = false;

// Queue handlers
static QueueHandle_t gpio_btn_evt_queue = NULL;
static volatile int64_t btn_last_press_time[3] = {0};
const int64_t buttons_debounce_time_us = 100000; 	// 100 ms

volatile bool btns_latch_state[3] = {true, true, true};


extern "C" void app_main(void)
{
	/* ====== Peripheral initialization ====== */
	/* init the i2c_dev and scan for available devices */
	ESP_ERROR_CHECK(i2cdev_init());
	i2c_scan();

	/* === setup the I2C bus */
	i2c_dev_bus_num_0.port = TOF_sensor_I2C_PORT_NUM;
	i2c_dev_bus_num_0.addr = TOF_sensor_I2C_address;
	i2c_dev_bus_num_0.addr_bit_len = I2C_ADDR_BIT_LEN_7;
	i2c_dev_bus_num_0.cfg.sda_io_num = GPIO_NUM_21;
	i2c_dev_bus_num_0.cfg.scl_io_num = GPIO_NUM_22;
	i2c_dev_bus_num_0.cfg.sda_pullup_en = true;
	i2c_dev_bus_num_0.cfg.scl_pullup_en = true;
	i2c_dev_bus_num_0.cfg.master.clk_speed = I2C_FREQ_HZ;

	/* === init the TOF device and create I2C mutex */
	ESP_ERROR_CHECK(i2c_dev_create_mutex(&i2c_dev_bus_num_0));
	
	TOF_device = vl53l0x_config(&i2c_dev_bus_num_0, GPIO_NUM_5, 0);
	esp_err_t tof_init_err = vl53l0x_init(TOF_device);

	char tof_init_err_txt[140];
	if (tof_init_err != ESP_OK) {
		vl53l0x_error_t tof_init_err_code = vl53l0x_getError(TOF_device);
		sprintf(tof_init_err_txt, "FAIL 0x%02X (%s)", tof_init_err_code, vl53l0x_err_to_name(tof_init_err_code));
	} else {
		sprintf(tof_init_err_txt, "Success");
	}
	
	ESP_LOGI(TOF_tag, "Init return value: %s", tof_init_err_txt);
	
	// start continuous measurements
	esp_err_t tof_cont_meas_err = vl53l0x_startContinuous(TOF_device, TOF_SENSOR_MEAS_TASK_PERIOD);
	ESP_LOGI(TOF_tag, "Starting continuous measurements: %s", (tof_cont_meas_err != ESP_OK) ? "FAIL" : "Success");
	

	/* === init TFT display */
	disp.LGFX_config(GPIO_NUM_18, GPIO_NUM_23, GPIO_NUM_19, GPIO_NUM_25, PANEL_ST7796);		// My wrapper config
	disp.LGFX_init();		// My wrapper init
	bool disp_init_ret = disp.init();	// LovyanGFX Library init
	ESP_LOGI(DISP_tag, "Init return value: %s", (disp_init_ret) ? "Success" : "FAIL");
	ESP_LOGI(DISP_tag, "Display resolution: %dx%d", (int)disp.width(), (int)disp.height());
	disp.setColorDepth(24);
	disp.setRotation(2);

	/* === init Buttons */
	gpio_config_t btn_conf = {
		.pin_bit_mask = (1ULL << BTN1_PIN) | (1ULL << BTN2_PIN) | (1ULL << BTN3_PIN),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,  // pull-down resistor connected externally
		.intr_type = GPIO_INTR_POSEDGE          // trigger interrupt on rising edge
	};
	gpio_config(&btn_conf);
	
	// event queue for capturing button presses
	gpio_btn_evt_queue = xQueueCreate(10, sizeof(button_event_t));
	// enable ISR service
	gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
	gpio_isr_handler_add(BTN1_PIN, btn_gpio_isr_handler, (void*)BTN1);
	gpio_isr_handler_add(BTN2_PIN, btn_gpio_isr_handler, (void*)BTN2);
	gpio_isr_handler_add(BTN3_PIN, btn_gpio_isr_handler, (void*)BTN3);

	/* === init rotary encoder */
	rotenc = new RotaryEncoder(GPIO_NUM_33, GPIO_NUM_32, GPIO_NUM_35);
	rotenc->init();

	/* === init PCA9685 driver for Servo motors */
	ESP_ERROR_CHECK(pca9685_init_desc(
		&i2c_dev_bus_num_0,
		Servo_PWM_driver_I2C_write_address,
		Servo_PWM_driver_I2C_PORT_NUM,
		i2c_dev_bus_num_0.cfg.sda_io_num,
		i2c_dev_bus_num_0.cfg.scl_io_num,
		i2c_dev_bus_num_0.cfg.master.clk_speed)
	);
	ESP_ERROR_CHECK(pca9685_init(&i2c_dev_bus_num_0));
	// restart the device
	ESP_ERROR_CHECK(pca9685_restart(&i2c_dev_bus_num_0));
	// setup frequency
	
	ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&i2c_dev_bus_num_0, Servo_PWM_driver_I2C_PWM_freq));
	ESP_ERROR_CHECK(pca9685_get_pwm_frequency(&i2c_dev_bus_num_0, &PCA9685_PWM_freq_actual));

	ESP_LOGI(PCA9685_tag, "Setup freq: %dHz, actual: %dHz", Servo_PWM_driver_I2C_PWM_freq, PCA9685_PWM_freq_actual);


	/* ====== Other initial setup ====== */

	// test 
	disp.fillScreen(TFT_BLACK);
	disp.setTextColor(TFT_WHITE, TFT_BLACK);
	disp.setTextSize(2);
	disp.setCursor(10, 5);
	disp.print("Hello ESP32!");

	for (uint8_t y = 0; y < 50; y++) {
		for (uint8_t x = 0; x < 7; x++) {
			bitmap[y * 7 + x] = (y % 4 >= 2) ? 0x33 : 0xCC;
		}
	}

	/* ====== Tasks creation ====== */
	xTaskCreate(Timer_task, "Timer_task", 2048, NULL, 8, NULL);
	xTaskCreate(Buttons_task, "Buttons_task", 4096, NULL, 10, NULL);
	xTaskCreate(Rotary_encoder_task, "Rotary_encoder_task", 4096, NULL, 6, NULL);
	xTaskCreate(TOF_sensor_meas_task, "TOF_sensor_meas_task", 4096, NULL, 5, NULL);
	xTaskCreate(Servo_motors_task, "Servo_motors_task", 4096, NULL, 7, NULL);
	xTaskCreate(TFT_ST7789_display_task, "TFT_display_task", 4096, NULL, 9, NULL);
}

/* FreeRTOS task functions */
void TOF_sensor_meas_task(void* pvParameter)
{
	static uint8_t cycle_counter = 0;

	uint16_t measured_distance = 0x0000;

	for (;;) {
		esp_err_t read_ret = vl53l0x_readRangeContinuousMillimeters(TOF_device, &measured_distance);

		if (read_ret != ESP_OK) {
			vl53l0x_error_t tof_err = vl53l0x_getError(TOF_device);
			ESP_LOGE(TOF_tag, "Reading range error: 0x%02X (%s)", tof_err, vl53l0x_err_to_name(tof_err));

			// Recovery: restart continuous measurement
			vl53l0x_stopContinuous(TOF_device);
			vTaskDelay(pdMS_TO_TICKS(10));
			vl53l0x_startContinuous(TOF_device, TOF_SENSOR_MEAS_TASK_PERIOD);

			cycle_counter = 0;
		} else {
			TOF_distance = measured_distance;
		}

		if (++cycle_counter >= TOF_SENSOR_RESTART_CYCLE_COUNT) {
			cycle_counter = 0;
			ESP_LOGW(TOF_tag, "Continuous measurement restart");

			// Periodic restart of continuous measurement
			vl53l0x_stopContinuous(TOF_device);
			vTaskDelay(pdMS_TO_TICKS(10));
			vl53l0x_startContinuous(TOF_device, TOF_SENSOR_MEAS_TASK_PERIOD);
		}

		vTaskDelay(pdMS_TO_TICKS(TOF_SENSOR_MEAS_TASK_PERIOD));
	}
}

void TFT_ST7789_display_task(void* pvParameter)
{
	// init to some arbitrary values so the refresh will trigger first time
	int64_t rot_counter_prev = INT64_MAX;
	uint16_t TOF_distance_prev = UINT16_MAX;

	for (;;) {
		if (bitmaps_draw[0]) {
			if (btns_latch_state[0])
				disp.drawBitmap(10, 25, bitmap, 50, 50, TFT_RED, TFT_BLACK);
			else
				disp.fillRect(10, 25, 50, 50, TFT_BLACK);

			bitmaps_draw[0] = false;
		}
		if (bitmaps_draw[1]) {
			if (btns_latch_state[1])
				disp.drawBitmap(65, 25, bitmap, 50, 50, TFT_GREEN, TFT_BLACK);
			else
				disp.fillRect(65, 25, 50, 50, TFT_BLACK);

			bitmaps_draw[1] = false;
		}
		if (bitmaps_draw[2]) {
			if (btns_latch_state[2])
				disp.drawBitmap(120, 25, bitmap, 50, 50, TFT_BLUE, TFT_BLACK);
			else
				disp.fillRect(120, 25, 50, 50, TFT_BLACK);
			
			bitmaps_draw[2] = false;
		}

		
		if (rot_counter != rot_counter_prev) {
			rot_counter_prev = rot_counter;		// so the next time the display does not refresh the value if it did not change

			disp.fillRect(10, 85, 230, 20, TFT_BLACK);		// clear the previous value
			
			disp.setTextColor(TFT_WHITE, TFT_BLACK);
			disp.setTextSize(2);
			disp.setCursor(10, 85);
			char counter_txt[20];
			sprintf(counter_txt, "Counter: %lld", rot_counter);
			disp.print(counter_txt);
		}

		if (TOF_distance != TOF_distance_prev) {
			TOF_distance_prev = TOF_distance;

			disp.fillRect(10, 105, 230, 20, TFT_BLACK);		// clear the previous value
			disp.setTextColor(TFT_WHITE, TFT_BLACK);
			disp.setTextSize(2);
			disp.setCursor(10, 105);
			char tof_distance_txt[20];
			sprintf(tof_distance_txt, "Dist: %dmm", TOF_distance);
			disp.print(tof_distance_txt);
		}
		
		vTaskDelay(pdMS_TO_TICKS(TFT_ST7789_DISPLAY_TASK_PERIOD));
	}
}

void Buttons_task(void* pvParameter)
{
	button_event_t ev;

	for (;;) {
		if (xQueueReceive(gpio_btn_evt_queue, &ev, portMAX_DELAY)) {
			ESP_LOGI(BTNS_tag, "Button %d pressed!", ev.btn_num);

			btns_latch_state[ev.btn_num] ^= 1;
			bitmaps_draw[ev.btn_num] = true;

		}

		vTaskDelay(pdMS_TO_TICKS(BUTTONS_TASK_PERIOD));
	}
}

void Rotary_encoder_task(void* pvParameter)
{
	rotary_event_t ev;
	int last_btn = -1;

	for (;;) {
		if (xQueueReceive(rotenc->event_queue, &ev, portMAX_DELAY)) {
			switch (ev.ev_type) {
				case ROTARY_EVENT:
					//ESP_LOGI(ROTENC_tag, "Rotary encoder direction: %s", (ev.ev_data.encoder.delta > 0) ? "Clockwise" : "Counter-clockwise");
					rot_counter += ev.ev_data.encoder.delta;
					break;

				case BUTTON_EVENT:
					if (ev.ev_data.button.button_state == last_btn && ev.ev_data.button.button_state == 1)
						continue;

					//ESP_LOGI(ROTENC_tag, "Rotary encoder button PRESSED!");
					servo_enable = !servo_enable;
\
					last_btn = ev.ev_data.button.button_state;
					break;

				default:
					break;
			}
		}
		vTaskDelay(pdMS_TO_TICKS(3));
	}
}

void Servo_motors_task(void* pvParameter)
{
	uint16_t servo_pwm_val_prev = UINT16_MAX;

	for (;;) {
		// update servo_pulse_width based on Rotary encoder counter
		servo_pulse_width = getServoCounts((float)rot_counter);

		if (servo_pulse_width != servo_pwm_val_prev) {
			servo_pwm_val_prev = servo_pulse_width;

			ESP_LOGI(PCA9685_tag, "Servo PWM value: %d", servo_pulse_width);

			if (pca9685_set_pwm_value(&i2c_dev_bus_num_0, Servo_PWM_driver_channel, servo_pulse_width) != ESP_OK)
				ESP_LOGE(PCA9685_tag, "Could not set PWM value to CH%d", Servo_PWM_driver_channel);	
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

void Timer_task(void* pvParameter)
{
	int64_t time_now, time_now_unit_ms, time_now_unit_s, time_now_unit_min;

	for (;;) {
		time_now = esp_timer_get_time();
		time_now_unit_ms = (time_now / 1000) % 1000;
		time_now_unit_s = (time_now / 1000000) % 60;
		time_now_unit_min = (time_now / 60000000);

		ESP_LOGI(Timer_tag, "Time: %lld:%02lld.%03lld", time_now_unit_min, time_now_unit_s, time_now_unit_ms);

		vTaskDelay(pdMS_TO_TICKS(TIMER_TASK_PERIOD));
	}
}


/* Non-task functions */

// I2C bus scan
void i2c_scan(void)
{
    ESP_LOGI(I2C_scan_tag, "Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; addr++) {
		i2c_dev_t dev;
		dev.port = I2C_NUM_0;
		dev.addr = addr;
		dev.addr_bit_len = I2C_ADDR_BIT_LEN_7;
		dev.cfg = {
			.sda_io_num = GPIO_NUM_21,
			.scl_io_num = GPIO_NUM_22,
			.sda_pullup_en = true,
			.scl_pullup_en = true,
			.master = {
				.clk_speed = 100000
			},
		};

		if (i2c_dev_probe(&dev, I2C_DEV_WRITE) == ESP_OK) {
            ESP_LOGI(I2C_scan_tag, "Found device at 0x%02X", addr);
        }
    }
}

uint16_t getServoCounts(float angle_deg)
{
	float angle_tmp = angle_deg;
	if (angle_deg > DS2330_servo_max_angle_deg)
		angle_tmp = DS2330_servo_max_angle_deg;

	if (angle_deg < DS2330_servo_min_angle_deg)
		angle_tmp = DS2330_servo_min_angle_deg;
	
	float duty_us = DS3230_servo_pwm_min_value_us + (1 - angle_tmp / 270.0f) * (DS3230_servo_pwm_max_value_us - DS3230_servo_pwm_min_value_us);
	return (uint16_t)(duty_us * 1e-6f * 4096.0f * PCA9685_PWM_freq_actual);
}

// button interrput handler
static void IRAM_ATTR btn_gpio_isr_handler(void* arg)
{
	int btn_num = (int)arg;
	int64_t time_now = esp_timer_get_time();

	if (time_now - btn_last_press_time[btn_num] > buttons_debounce_time_us) {
		btn_last_press_time[btn_num] = time_now;

		button_event_t ev = {
			.btn_num = btn_num,
			.timestamp = time_now
		};

		// send the acquired event outside of this handler
		xQueueSendFromISR(gpio_btn_evt_queue, &ev, NULL);
	}
}