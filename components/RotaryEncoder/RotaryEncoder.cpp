/*
 * Copyright © 2025 Tomas Hak
 *
 * MIT Licensed as described in the file LICENSE
 */

#include "RotaryEncoder.hpp"

// look-up table quad_table maps (old << 2 | new) -> delta 
static const int8_t quad_table[16] = {
    0, -1, +1, 0,
   +1,  0,  0, -1,
   -1,  0,  0, +1,
    0, +1, -1, 0
};

QueueHandle_t RotaryEncoder::event_queue = nullptr;

RotaryEncoder::RotaryEncoder(gpio_num_t enc_pin_a, gpio_num_t enc_pin_b, gpio_num_t button_pin)
    : enc_pin_a(enc_pin_a), enc_pin_b(enc_pin_b), button_pin(button_pin) {}

void RotaryEncoder::init()
{
    gpio_config_t pinAB_io_conf = {
        .pin_bit_mask = (1ULL << enc_pin_a) | (1ULL << enc_pin_b),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&pinAB_io_conf);

    gpio_config_t pinBtn_io_conf = {
        .pin_bit_mask = (1ULL << button_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&pinBtn_io_conf);

    // init encoder state
    last_enc_state = (gpio_get_level(enc_pin_a) << 1) | gpio_get_level(enc_pin_b);

    event_queue = xQueueCreate(20, sizeof(rotary_event_t));
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(enc_pin_a, rotary_encoder_enc_isr_handler, this);
    gpio_isr_handler_add(enc_pin_b, rotary_encoder_enc_isr_handler, this);
    gpio_isr_handler_add(button_pin, rotary_encoder_button_isr_handler, this);
}

void IRAM_ATTR RotaryEncoder::rotary_encoder_enc_isr_handler(void* arg)
{
    auto *rot_enc = reinterpret_cast<RotaryEncoder*>(arg);
    int64_t time_now = esp_timer_get_time();

    if (time_now - rot_enc->last_enc_event_time < ROTARY_ENCODER_DEBOUNCE_TIME_US)
        return;

    rot_enc->last_enc_event_time = time_now;

    uint8_t a_state = gpio_get_level(rot_enc->enc_pin_a);
    uint8_t b_state = gpio_get_level(rot_enc->enc_pin_b);
    uint8_t new_state = (a_state << 1) | b_state;

    uint8_t idx = (rot_enc->last_enc_state << 2) | new_state;
    int8_t delta = quad_table[idx];

    if (delta != 0) {
        rotary_event_t evt = {
            .ev_type = ROTARY_EVENT,
            .ev_data = {
                .encoder = {
                    .delta = delta,
                },
            },
            .timestamp = time_now,
        };

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(event_queue, &evt, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();

    }

    rot_enc->last_enc_state = new_state;
}

void IRAM_ATTR RotaryEncoder::rotary_encoder_button_isr_handler(void* arg)
{
    auto *rot_enc = reinterpret_cast<RotaryEncoder*>(arg);
    int64_t time_now = esp_timer_get_time();

    if (time_now - rot_enc->last_btn_event_time < ROTARY_ENCODER_BUTTON_DEBOUNCE_TIME_US)
        return;     // ignore bounce

    rot_enc->last_btn_event_time = time_now;

    if (gpio_get_level(rot_enc->button_pin) != 0) {
        return; // ignoring bounce or release
    }   

    rotary_event_t evt = {
        .ev_type = BUTTON_EVENT,
        .ev_data = { .button = { .button_state = 0 }},
        .timestamp = time_now,
    };

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(event_queue, &evt, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}