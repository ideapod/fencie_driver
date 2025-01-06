#include <quadmotor_interfaces/msg/detail/quad_motor_encoder_counts__struct.h>
#include <rcl/publisher.h>
#include <rcl/types.h>
#include <stdio.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "fencie_config.h"
#include "gpio_pin_handling.h"
#include "fencie_encoder_driver.h"
#include "fencie_config.h"
#include <quadmotor_interfaces/msg/quad_motor_encoder_counts.h>

#define TAG "ENCODER DRIVER"


rcl_allocator_t * pAllocator; 
rcl_node_t * pNode; 
rclc_executor_t * pExecutor; 
rclc_support_t * pSupport;
rcl_publisher_t quad_publisher;
quadmotor_interfaces__msg__QuadMotorEncoderCounts quad_publish_msg;
quad_wheels_t *pWheels;

bool publishMsg = false;
/* 
 * we need to set up the pins, interrupts and ISR's to handle counting each encoder. 
 */


int numEncoders = 0;
DirectionSet directionCommand = directionStopped;
quadmotor_interfaces__msg__QuadMotorEncoderCounts msg;

void savePinState(wheel_sensor_t *pSensor) {
    pSensor->oldPinStateA= pSensor->pinStateA;
    pSensor->oldPinStateB= pSensor->pinStateB;
}

void getPinState(wheel_sensor_t *pSensor) {
    pSensor->pinStateA = (get_logic_pin(pSensor->pinA) == GPIO_HIGH);
    pSensor->pinStateB = (get_logic_pin(pSensor->pinB) == GPIO_HIGH);
}

void getPinStates() {

  for (int i=0; i<= rhs; i++) {
  	getPinState(&pWheels->sides[i].front_wheel.sensor);
  	getPinState(&pWheels->sides[i].rear_wheel.sensor);
  }
}


void pulseEncoder(wheel_sensor_t *pSensor) {
  getPinState(pSensor);

  if (!pSensor->oldPinStateA && pSensor->pinStateA) {
      // low to high on pinA 

  	  // work out if we've changed direation
      if (!pSensor->pinStateB && (pSensor->direction == directionForward)) {
        // direction changed
        pSensor->direction = directionReverse;
      } else if (pSensor->pinStateB && (pSensor->direction == directionReverse)) {
        // direction changed
        pSensor->direction = directionForward;
      }
    }

    // pulse the counter. 
    if (pSensor->direction == directionForward) {
      pSensor->current_pulse_count++;
    } else {
      pSensor->current_pulse_count--;
    }

  savePinState(pSensor);
}

/* 
 * find the phaseA pin in the encoder array. 
 */
wheel_sensor_t *get_sensor_from_phaseA_pin(uint32_t gpio_num) {

	for (int i=0;i<=rhs; i++) {
		wheel_sensor_t *pSensor = &pWheels->sides[i].front_wheel.sensor;
  	if (pSensor->pinA == gpio_num) {
  		return pSensor;
  	}

  	pSensor = &pWheels->sides[i].rear_wheel.sensor;
  	if (pSensor->pinA == gpio_num) {
  		return pSensor;
  	}
  }

  return NULL;
}

/* 
 * phase A interrupt handler. 
 * receives the pin that caused the interrupt. we find the encoder in question and pulse it. 
 */
static void IRAM_ATTR phase_a_handler(void* arg)
{
    int gpio_num = (int)arg;
    wheel_sensor_t *pSensor = get_sensor_from_phaseA_pin(gpio_num);
    if (pSensor == NULL) {
    	ESP_LOGE(TAG,"could not find gpio_num %d in wheels config", gpio_num);
    	return;
    }
    pulseEncoder(pSensor);
}

void init_wheel_sensor_gpio(wheel_sensor_t *pSensor) {
	init_input_pin_isr( // set up phase A pin as the interrupt trigger count.
			pSensor->pinA,
			phase_a_handler
		);
		init_input_pin(pSensor->pinB);
    pSensor->current_pulse_count = 0;
}

esp_err_t init_encoder_driver(quad_wheels_t *pWheels_in) {
	// set up phase A/B pins for each wheel
	// set up interrupt calls
	pWheels = pWheels_in;

	ESP_LOGI(TAG, "Setting up GPIO pins");
	for (int i=0;i<=rhs; i++) {
		wheel_sensor_t *pSensor = &pWheels->sides[i].front_wheel.sensor;
		init_wheel_sensor_gpio(pSensor);
  	pSensor = &pWheels->sides[i].rear_wheel.sensor;
  	init_wheel_sensor_gpio(pSensor);
  }

 	ESP_LOGI(TAG, "GPIO pins complete");
 	return ESP_OK;
}

esp_err_t init_encoder_ros(rcl_allocator_t * allocator, rcl_node_t * node, rclc_executor_t * executor, rclc_support_t * support) {
	// we will publish the quadencoder counts

	pAllocator = allocator;
	pNode = node;
	pExecutor = executor;
	pSupport = support;

	// create publisher

	RCCHECK(rclc_publisher_init_default(
		&quad_publisher,
		pNode,
		ROSIDL_GET_MSG_TYPE_SUPPORT(quadmotor_interfaces, msg, QuadMotorEncoderCounts),
		"/fencie/quad_motor_encoder_counts"));

	ESP_LOGI(TAG, "publisher created for /fencie/quad_motor_encoder_counts");

	return ESP_OK;	
}

void publish_encoder_message() {
	quad_publish_msg.front_right_encoder_count = pWheels->sides[rhs].front_wheel.sensor.current_pulse_count;
	quad_publish_msg.rear_right_encoder_count = pWheels->sides[rhs].rear_wheel.sensor.current_pulse_count;
	quad_publish_msg.front_left_encoder_count = pWheels->sides[lhs].front_wheel.sensor.current_pulse_count;
	quad_publish_msg.rear_left_encoder_count = pWheels->sides[lhs].rear_wheel.sensor.current_pulse_count;
	
	//ESP_LOGI(TAG, "Publishing quad_encoder counts message");
	rcl_ret_t ret = rcl_publish(&quad_publisher, &quad_publish_msg, NULL);	
	if (ret == RCL_RET_ERROR) {
		ESP_LOGE(TAG, "publishing encoder had error");
	}
}

esp_err_t do_encoder_work() {
	// every 50 counts dump state
	// ESP_LOGI(TAG, "doing encoder work, pulishMsg: %d",publishMsg);
	if (publishMsg == true) {
		publish_encoder_message();
		publishMsg = false;
	}
	return ESP_OK;
}


void dump_encoder_states() {
	ESP_LOGI(TAG, "encoder pulses %s: %ld, %s: %ld, %s: %ld, %s: %ld",
			pWheels->sides[lhs].front_wheel.wheel_name,
			pWheels->sides[lhs].front_wheel.sensor.current_pulse_count,
			pWheels->sides[lhs].rear_wheel.wheel_name,
			pWheels->sides[lhs].rear_wheel.sensor.current_pulse_count,
			pWheels->sides[rhs].front_wheel.wheel_name,
			pWheels->sides[rhs].front_wheel.sensor.current_pulse_count,
			pWheels->sides[rhs].rear_wheel.wheel_name,
			pWheels->sides[rhs].rear_wheel.sensor.current_pulse_count
		);	
}

esp_err_t do_encoder_timer_work(rcl_timer_t * timer, int64_t last_call_time, int timer_count) {
	// send encoder message at the required frequency

	// ESP_LOGI(TAG, "Doing Encoder timer work %d", timer_count);
	if ((timer_count % 1000) == 0) {
		dump_encoder_states();
	}

	// publishing may take too long on the timer call back. so do it in work time instead.
	publishMsg = true;
	
	return ESP_OK;
}

esp_err_t terminate_encoder() {
	RCSOFTCHECK(rcl_publisher_fini(&quad_publisher, pNode));
	return ESP_OK;
}


esp_err_t test_encoder_driver() {
	ESP_LOGI(TAG,"No tests for encoders");

	getPinStates();
	dump_encoder_states();

	return ESP_OK;
}