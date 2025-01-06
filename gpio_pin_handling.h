#ifndef GPIO_PIN_HANDLING
#define GPIO_PIN_HANDLING

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "hal/ledc_types.h"

void init_pwm_pin(uint32_t pwm_pin);
void init_input_pin(uint32_t input_pin);
void init_input_pin_isr(uint32_t input_pin, void (*isr_func)(void* arg));
void init_logic_pin(uint32_t logic_pin);
void set_logic_pin(uint32_t logic_pin, bool logic_level);
bool get_logic_pin(uint32_t logic_pin);
void set_pwm_level(uint32_t pin, uint32_t percent_level);
void init_ledc_timers();

#endif