/*
 * Copyright © 2025 Tomas Hak
 *
 * MIT Licensed as described in the file LICENSE
 */

extern "C" {
    /* common C includes */
    #include <stdio.h>

    /* hardware specific includes */
    #include "driver/gpio.h"
    // externally connected devices
    #include "vl53l0x.h"        // TOF sensor
    #include "pca9685.h"        // 16-channel 12-bit driver for servo motors

    /* logging includes */
    #include "esp_log.h"

    /* FreeRTOS includes */
    #include "freertos/FreeRTOS.h"
    #include "freertos/queue.h"
    #include "freertos/task.h"

    /* ESP includes */
    #include "esp_timer.h"
}
/* C++ libraries */
#include <LovyanGFX.hpp>            // ST7789 display library
#include <LGFX_setup.hpp>

#include <RotaryEncoder.hpp>        // My custom rotary encoder library

#define DISPLAY_DRIVER_ST7789       0
#define DISPLAY_DRIVER_ST7796       1

/* Global defines */
#define I2C_FREQ_HZ                         100000 // 100 kHz
// TOF sensor
#define TOF_sensor_I2C_PORT_NUM				I2C_NUM_0
#define TOF_sensor_I2C_address				0x29

// Buttons
#define BTN1_PIN    GPIO_NUM_36
#define BTN2_PIN    GPIO_NUM_39
#define BTN3_PIN    GPIO_NUM_34

// 16-channel PWM driver PCA9685
#define Servo_PWM_driver_I2C_PORT_NUM		I2C_NUM_0
#define Servo_PWM_driver_I2C_write_address	(PCA9685_ADDR_BASE | 0x14)       // (0b01 010100) - lower 6 bit are setup in hardware
#define Servo_PWM_driver_I2C_PWM_freq       100        // Frequency of PWM outputs. The maximum PWM frequency is 1526 Hz, and the minimum PWM frequency is 24 Hz. See "7.3.5 PWM frequency PRE_SCALE" in the datasheet.
#define Servo_PWM_driver_channel            PCA9685_CHANNEL_0

// DS3230 Pro-270 values
#define DS3230_servo_pwm_min_value_us       380     // ~500us
#define DS3230_servo_pwm_max_value_us       2670    // ~2500us
#define DS2330_servo_min_angle_deg          0       // [°]
#define DS2330_servo_max_angle_deg          270     // [°]


// RTOS Task periods [ms]
#define TOF_SENSOR_MEAS_TASK_PERIOD         500
#define TFT_ST7789_DISPLAY_TASK_PERIOD      250
#define BUTTONS_TASK_PERIOD                 20
#define ROTARY_ENCODER_TASK_PERIOD          0
#define TIMER_TASK_PERIOD                   500

#define TOF_SENSOR_RESTART_PERIOD           30000
#define TOF_SENSOR_RESTART_CYCLE_COUNT      ((uint16_t)(TOF_SENSOR_RESTART_PERIOD / TOF_SENSOR_MEAS_TASK_PERIOD))

/* TYPEDEFS */
typedef enum {
    BTN1 = 0,
    BTN2 = 1,
    BTN3 = 2
} BTN_nums;

static const char *I2C_scan_tag     = "[I2C scan]";
static const char* TOF_tag          = TAG_VL53L0X;
static const char* DISP_tag         = "[TFT display]";
static const char* BTNS_tag         = "[Buttons]";
static const char* ROTENC_tag       = TAG_rotary_encoder;
static const char* Timer_tag        = "[Timer]";
static const char *PCA9685_tag      = "[PCA9685]";

typedef struct {
    int btn_num;
    int64_t timestamp;
} button_event_t;

/* function declarations */
// FreeRTOS tasks
void TOF_sensor_meas_task(void *pvParameter);
void TFT_ST7789_display_task(void *pvParameter);
void Counter_disp_task(void *pvParameter);
void Buttons_task(void *pvParameter);
void Rotary_encoder_task(void *pvParameter);
void Servo_motors_task(void* pvParameter);
void Timer_task(void *pvParameter);

/* Non-task functions */
// other functions
void i2c_scan(void);
uint16_t getServoCounts(float angle_deg);

// interrupt functions
static void btn_gpio_isr_handler(void *arg);