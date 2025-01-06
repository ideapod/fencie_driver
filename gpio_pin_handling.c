#include "gpio_pin_handling.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "fencie_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "hal/ledc_types.h"

#define TAG "GPIO_PIN"



typedef struct ledc_map_s {
    uint32_t pwm_pin;
    int ledc_timer_num;
    int ledc_channel_num;
} ledc_map_t;

ledc_map_t pin_timer_set[LEDC_TIMER_MAX -1];
int allocated_timer_count = 0;

int allocate_timer_for_pin(int32_t pwm_pin) {
    int retVal = allocated_timer_count;
    pin_timer_set[retVal].ledc_timer_num = retVal;
    pin_timer_set[retVal].ledc_channel_num = retVal;
    pin_timer_set[retVal].pwm_pin = pwm_pin;
    allocated_timer_count++;
    return retVal;
}

int find_timer_for_pin(int32_t pwm_pin) {
    for (int i=0;i<=allocated_timer_count; i++) {
        if (pin_timer_set[i].pwm_pin == pwm_pin) {
            return i;
        }
    }
    return -1;
}

int get_timer_for_pin(int32_t pwm_pin) {
    int retVal = find_timer_for_pin(pwm_pin);

    if (retVal == -1) {
        retVal = allocate_timer_for_pin(pwm_pin); 
    }

    return retVal;
}

void init_ledc_timer_for_pin(int32_t pwm_pin) {
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = get_timer_for_pin(pwm_pin),
        .duty_resolution  = LEDC_DUTY_RESOLUTION,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
}

void config_channel_for_pin(int32_t pwm_pin) {
    int timer_num = get_timer_for_pin(pwm_pin);

    ledc_channel_config_t ledc_channel = {
            .channel    = pin_timer_set[timer_num].ledc_channel_num,
            .duty       = 0,
            .gpio_num   = pwm_pin,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = pin_timer_set[timer_num].ledc_timer_num
        };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

/**
 * @brief sets up the servo on given pin and initialises the mcpwm module on ESP32 for the pin
 *
 * @param  servo_pin the ESP32 pin for the servo
 *
 * @return
 *     - calculated pulse width
 */
void init_pwm_pin(uint32_t pwm_pin)
{
    ESP_LOGI(TAG,"initializing ledc pwm on pin %d",pwm_pin);

    init_ledc_timer_for_pin(pwm_pin); // each pwm_pin needs it's own ledc timer so we can control duty on each.
    config_channel_for_pin(pwm_pin);
}


void set_pwm_level(uint32_t pin, uint32_t percent_level)
{
    float percent_float = percent_level;
    float dcf = (percent_float / 100.0) * LEDC_MAX_PWM;
    int duty_cycle = (int) dcf;
    int pin_idx = get_timer_for_pin(pin);

    // ESP_LOGI(TAG,"setting pin: %d, channel num: %d, to percent level: %d, duty_cycle %d", 
    //     pin, pin_timer_set[pin_idx].ledc_channel_num, percent_level, duty_cycle);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, pin_timer_set[pin_idx].ledc_channel_num, duty_cycle);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, pin_timer_set[pin_idx].ledc_channel_num);
}


void init_input_pin(uint32_t input_pin) {
	gpio_config_t myGPIOconfig;

	myGPIOconfig.pin_bit_mask = 1ULL << input_pin;
    myGPIOconfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    myGPIOconfig.pull_up_en = GPIO_PULLUP_DISABLE;
    myGPIOconfig.mode = GPIO_MODE_INPUT;
    myGPIOconfig.intr_type = GPIO_INTR_DISABLE;

    gpio_config(&myGPIOconfig);
}

void init_input_pin_isr(uint32_t input_pin, void (*isr_func)(void* arg)) {
	gpio_config_t myGPIOconfig;

	//interrupt of rising edge
    myGPIOconfig.intr_type = GPIO_INTR_POSEDGE;
    myGPIOconfig.pin_bit_mask = 1ULL << input_pin;
    myGPIOconfig.mode = GPIO_MODE_INPUT; //set as input mode
    myGPIOconfig.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&myGPIOconfig);

    
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(input_pin, *isr_func, (void*) input_pin);
}

void init_logic_pin(uint32_t logic_pin) {

	gpio_config_t myGPIOconfig;

	// Configure Digital I/O for LEDs
    myGPIOconfig.pin_bit_mask = 1ULL << logic_pin;
    myGPIOconfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    myGPIOconfig.pull_up_en = GPIO_PULLUP_DISABLE;
    myGPIOconfig.mode = GPIO_MODE_OUTPUT;
    myGPIOconfig.intr_type = GPIO_INTR_DISABLE;

    gpio_config(&myGPIOconfig);
}

void set_logic_pin(uint32_t logic_pin, bool logic_level) {
	if (logic_level == true) {
		gpio_set_level(logic_pin, 1);
	}
	else {
		gpio_set_level(logic_pin, 0);
	}
}

bool get_logic_pin(uint32_t logic_pin) {
	// 1 = high
	// 0 = low.
	return (gpio_get_level(logic_pin) == GPIO_HIGH);
}

