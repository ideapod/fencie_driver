


#include <math.h>
#include <rcl/subscription.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "quadmotor_interfaces/msg/quad_motor_velocities.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include <driver/gpio.h>

#include "fencie_motor_driver.h"
#include "fencie_config.h"
#include "gpio_pin_handling.h"
#include "pid.h"

#define TAG "MOTOR DRIVER"


rcl_allocator_t * pAllocator; 
rcl_node_t * pNode; 
rclc_executor_t * pExecutor; 
rclc_support_t * pSupport;

quad_wheels_t *pWheels;

rcl_subscription_t quad_motor_velocities_subscriber;
quadmotor_interfaces__msg__QuadMotorVelocities msg;

int64_t prev_call_time;

/*
 * initialise the control pins for all 4 motors
 * work out which joint is mapped to which motor
 * publish joint state messages
 * take joint commands
 */


void setForward(side_drive_t *pSide) {
	set_logic_pin(pSide->pin_forward, true);
	set_logic_pin(pSide->pin_reverse, false);
}

void setReverse(side_drive_t *pSide) {
	set_logic_pin(pSide->pin_forward, false);
	set_logic_pin(pSide->pin_reverse, true);
}

void setStop(side_drive_t *pSide) {
	if ((pSide->front_wheel.pid_set_point.target_pulses_per_frame != 0) ||
		(pSide->front_wheel.pid_set_point.target_pulses_per_frame != 0)) {
		return; // one of the wheels still wants to go, so leave this sides motors enabled. 
	}
	set_logic_pin(pSide->pin_forward, false);
	set_logic_pin(pSide->pin_reverse, false);
}

side_drive_t *find_side_for_wheel(wheel_config_t *pWheel) {
	for (int i=0; i <= rhs; i++) {
		if (&pWheels->sides[i].front_wheel == pWheel)
			return &pWheels->sides[i];
		if (&pWheels->sides[i].rear_wheel == pWheel)
			return &pWheels->sides[i];
	}
	return NULL;
}

void set_motor_to_pid_point(wheel_config_t *pWheel) {
	float percent_level = ((float) pWheel->pid_set_point.curr_output / LEDC_MAX_PWM) * 100;

	if (percent_level > 100) {
		percent_level = 100;
	}

	// ESP_LOGI(TAG, "setting pid curr output: %ld, max_pwm: %d, to pwm percent: %f for wheel: %s", 
	// 	pWheel->pid_set_point.curr_output,
	// 	LEDC_MAX_PWM,
	// 	percent_level, pWheel->wheel_name);
	set_pwm_level(pWheel->pin_enable, (int)percent_level);
}


void reset_motor_pids() {
	for (int i=0; i<=rhs; i++) {
		resetPID(&pWheels->sides[i].front_wheel.pid_set_point);
		resetPID(&pWheels->sides[i].rear_wheel.pid_set_point);
	}
}

void set_motor_pid_point(wheel_config_t *pWheel, int32_t velocity_pulses_per_second) {
	// a frame is x milliseconds. 
	// to convert to puluses per frame we  divide pulses per second by 1000 to get pulses per milli second
	// then multiply by the number of millis in a work period (aka frame)

	resetPID(&pWheel->pid_set_point);

	float pps = (float)velocity_pulses_per_second;
	int pulses_per_frame = (int)((pps / 1000) * TIMER_WORK_PERIOD_MILLIS);

	if (pWheel->pid_set_point.target_direction == directionReverse) {
		pulses_per_frame *= -1; // make it a positive value. 
	}

	pWheel->pid_set_point.target_pulses_per_frame = pulses_per_frame ;
	pWheel->pid_set_point.prev_frame_pulse_count = pWheel->sensor.current_pulse_count;
	ESP_LOGI(TAG, "setting direction %s, pid point %d pulses per frame for wheel %s, input %d velocity_pulses_per_second", 
		pWheel->pid_set_point.target_direction == directionForward ? "Forward": "Reverse",
		pulses_per_frame, pWheel->wheel_name, velocity_pulses_per_second );
}

void set_motor_velocity(wheel_config_t *pWheel, int32_t velocity_pulses_per_second) {
	side_drive_t *pSide = find_side_for_wheel(pWheel);

	if (velocity_pulses_per_second > 0) {
		setForward (pSide);
		pWheel->pid_set_point.target_direction = directionForward;
		set_motor_pid_point(pWheel, velocity_pulses_per_second);
	} else if (velocity_pulses_per_second < 0) {
		setReverse (pSide);
		pWheel->pid_set_point.target_direction = directionReverse;
		set_motor_pid_point(pWheel, velocity_pulses_per_second);
	} else { // == 0 
		ESP_LOGI(TAG, "setting motor %s to stop as velcoity is 0", pWheel->wheel_name);
		setStop (pSide);
		pWheel->pid_set_point.target_direction = directionStopped;
		set_motor_pid_point(pWheel, 0);
	}
}

float radians_to_pulses_per_second(wheel_config_t *pWheel, float velocity_rads_per_sec) {
	// radians per second is a per wheel thing. 
	// we know how many encoder ticks per wheel 
	// 2 pi is 1 radian per second. 
	// so to go to encoder ticks per second nta
	float radians_per_pulse = (float) (2 * M_PI) / (float) pWheel->sensor.calibrated_pulse_per_rev;
	float pulses_per_second = velocity_rads_per_sec / radians_per_pulse;
	return pulses_per_second;
}

void set_wheel_velocity_radians_per_second(wheel_config_t *pWheel, float velocity_rads_per_sec) {
	float pulses_per_second = radians_to_pulses_per_second(pWheel, velocity_rads_per_sec);
	set_motor_velocity(pWheel, (int )pulses_per_second);
}


void velocities_callback(const void * msgin)
{
	const quadmotor_interfaces__msg__QuadMotorVelocities * msg = (const quadmotor_interfaces__msg__QuadMotorVelocities *)msgin;

	ESP_LOGI(TAG, "motor message received: fr %f, rr %f, fl %f, rl %f", 
		msg->front_right_motor_velocity,
		msg->front_left_motor_velocity,
		msg->rear_right_motor_velocity,
		msg->rear_left_motor_velocity);

	// these velocities are radians per second
	set_wheel_velocity_radians_per_second(&pWheels->sides[rhs].front_wheel, (float)msg->front_right_motor_velocity);
	set_wheel_velocity_radians_per_second(&pWheels->sides[lhs].front_wheel, (float)msg->front_left_motor_velocity);
	set_wheel_velocity_radians_per_second(&pWheels->sides[rhs].rear_wheel, (float)msg->rear_right_motor_velocity);
	set_wheel_velocity_radians_per_second(&pWheels->sides[lhs].rear_wheel, (float)msg->rear_left_motor_velocity);
	ESP_LOGI(TAG, "motor speeds set");
	dump_pid_coefficients();
}

esp_err_t init_motor_driver(quad_wheels_t *p_wheels_in) {
	// set up pins to motor controllers
	// for each side there is - forward pin, backwards pin, enable left, enable right. 
	// set to 0 velocity

	pWheels = p_wheels_in;
	// set up the sides
	for (int i = 0; i < 2; i++) {
		init_logic_pin(pWheels->sides[i].pin_forward); // lhs forward logic pin
		init_logic_pin(pWheels->sides[i].pin_reverse); // lhs reverse logic pin
		init_pwm_pin(pWheels->sides[i].front_wheel.pin_enable); // enable is driven as PWM. 
		init_pwm_pin(pWheels->sides[i].rear_wheel.pin_enable); // enable is driven as PWM. 	
	}

	return ESP_OK;
}

esp_err_t init_motor_ros(rcl_allocator_t * allocator, rcl_node_t * node, rclc_executor_t * executor, rclc_support_t * support) {
	// subscribe to QuadMotorVelocities

	pAllocator = allocator;
	pNode = node;
	pExecutor = executor;
	pSupport = support;

	RCCHECK(rclc_subscription_init_default(
		&quad_motor_velocities_subscriber,
		pNode,
		ROSIDL_GET_MSG_TYPE_SUPPORT(quadmotor_interfaces, msg, QuadMotorVelocities),
		"/fencie/quad_motor_velocities"));

	RCCHECK(rclc_executor_add_subscription(pExecutor, &quad_motor_velocities_subscriber, &msg, &velocities_callback, ON_NEW_DATA));

	ESP_LOGI(TAG, "subscription created to: /fencie/quad_motor_velocities");
	return ESP_OK;
}

esp_err_t do_motor_work() {
	// every 50 counts dump state

	// check if there are any data errors or empty data. 
	// if (do_report  == true) {
	// 	ESP_LOGI(TAG, "%d seconds passed, no data returned %d times, errors %d times.",count_seconds, no_data, error_count);
	// 	no_data = 0;
	// 	error_count = 0;
	// 	do_report = false;
	// }



	return ESP_OK;
}

void set_direction_all_forward() {
	for (int i = 0; i <= rhs; i++) {
		setForward(&pWheels->sides[i]);
	}
}

void set_direction_all_back() {
	for (int i = 0; i <= rhs; i++) {
		setReverse(&pWheels->sides[i]);
	}
}

void set_direction_all_stop() {
	for (int i = 0; i <= rhs; i++) {
		setStop(&pWheels->sides[i]);
	}
}

void test_all_pwm_level( int percent_level) {
	for (int i = 0; i <= rhs; i++) {
		set_pwm_level(pWheels->sides[i].front_wheel.pin_enable, percent_level);
		set_pwm_level(pWheels->sides[i].rear_wheel.pin_enable, percent_level);
	}
}

esp_err_t test_motor_driver() {
	bool skip_test = true; // change this if you want a test of the motors

	if (skip_test) {
		ESP_LOGI(TAG, "Skipping tests");
		return;
	}
	// rotate wheels forward 1 revolution, back 1 revolution
	int twixDelay = 5000 / portTICK_PERIOD_MS; // 5000ms = 5s
	int duty_percent = 30;

	ESP_LOGI(TAG,"Starting motor tests");
	ESP_LOGI(TAG,"forward");
	set_direction_all_forward();
	test_all_pwm_level(duty_percent);
	vTaskDelay(twixDelay);
	ESP_LOGI(TAG,"stop");
	set_direction_all_stop();
	test_all_pwm_level(0);
	vTaskDelay(twixDelay);
	ESP_LOGI(TAG,"back");
	set_direction_all_back();
	test_all_pwm_level(duty_percent);
	vTaskDelay(twixDelay);
	ESP_LOGI(TAG,"stop");
	set_direction_all_stop();
	test_all_pwm_level(0);

	return ESP_OK;
}

/* Read the encoder values and call the PID routine */
void updatePID(wheel_config_t *pWheel) {

	// keep track of time per frame
	pWheel->pid_set_point.prev_time = pWheel->pid_set_point.curr_time;
  pWheel->pid_set_point.curr_time = esp_timer_get_time(); 

  // keep track of pulse counts for this wheel per frame. 
  pWheel->pid_set_point.prev_frame_pulse_count = 
  	pWheel->pid_set_point.curr_frame_pulse_count;
  pWheel->pid_set_point.curr_frame_pulse_count = 
  	pWheel->sensor.current_pulse_count;

  /* If we're not moving there is nothing more to do */
  if (pWheel->pid_set_point.target_pulses_per_frame == 0.0)
  {
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInput is considered a good proxy to detect
    * whether reset has already happened
    */
    if (pWheel->pid_set_point.prev_input != 0) {
    	ESP_LOGI(TAG, "Resetting PID for %s", 
    	pWheel->wheel_name);
  		resetPID(&pWheel->pid_set_point);	
    }
    
    return;
  }

  /* Compute PID update for each motor */
  doPID(pWheel);

  /* Set the motor speeds accordingly */
  set_motor_to_pid_point(pWheel);

}



esp_err_t do_motor_timer_work(rcl_timer_t * timer, int64_t last_call_time, int timer_count) {
	// do PD control here. 
	// timer_count is in useconds.


	// ESP_LOGI(TAG,"updating PID at each wheel - prev_call_time: %lld, curr_call_time: %lld, timer_count: %d",
	// 	prev_call_time, last_call_time, timer_count );

	for (int i =0; i<= rhs; i++) {
		updatePID(&pWheels->sides[i].front_wheel);	
		updatePID(&pWheels->sides[i].rear_wheel);	
	}

	prev_call_time = last_call_time;

	//  ESP_LOGI(TAG,"motor work done");
	return ESP_OK;
}

esp_err_t terminate_motor() {
	RCCHECK(rcl_subscription_fini(&quad_motor_velocities_subscriber, pNode));
	return ESP_OK;
}
