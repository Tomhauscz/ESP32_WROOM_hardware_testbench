/*
 * Copyright © 2025 Tomas Hak
 *
 * MIT Licensed as described in the file LICENSE
 */

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_timer.h"

static const char* __attribute__((unused)) TAG_rotary_encoder = "[Rotary encoder]";

#define ROTARY_ENCODER_DEBOUNCE_TIME_US             2000    // 2ms
#define ROTARY_ENCODER_BUTTON_DEBOUNCE_TIME_US      100000   // 100 ms

typedef enum {
    ROTARY_EVENT,
    BUTTON_EVENT
} rotary_event_type_t;

typedef struct {
    int delta;
} rotary_enc_event_t;

typedef struct {
    int button_state;
} rotary_btn_event_t;


typedef struct {
    rotary_event_type_t ev_type;
    union {
        rotary_enc_event_t encoder;
        rotary_btn_event_t button;
    } ev_data;
    int64_t timestamp;
} rotary_event_t;

class RotaryEncoder {
    public:
        RotaryEncoder(gpio_num_t enc_pin_a, gpio_num_t enc_pin_b, gpio_num_t button_pin);
        void init();
        static void IRAM_ATTR rotary_encoder_enc_isr_handler(void *arg);
        static void IRAM_ATTR rotary_encoder_button_isr_handler(void *arg);
        static QueueHandle_t event_queue;
    private:
        gpio_num_t enc_pin_a;       // sometimes called also CLK pin
        gpio_num_t enc_pin_b;       // sometimes called also DT pin
        gpio_num_t button_pin;
        uint8_t last_enc_state = 0x00;
        int64_t last_enc_event_time = 0;
        int64_t last_btn_event_time = 0;
};