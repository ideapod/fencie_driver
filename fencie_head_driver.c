#include <stdint.h>
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

#include "fencie_head_driver.h"
#include "servo_pca9685.h"
#include "fencie_config.h"
#include <quadmotor_interfaces/msg/fencie_head_command.h>
#include <quadmotor_interfaces/msg/fencie_head_position.h>

#define TAG "HEAD DRIVER"

/*
 * initialise the pca9685 to control head
 * work out which joint is mapped to which servo
 * publish joint state messages
 * take joint commands
 */

rcl_allocator_t * pAllocator; 
rcl_node_t * pNode; 
rclc_executor_t * pExecutor; 
rclc_support_t * pSupport;
quadmotor_interfaces__msg__FencieHeadCommand cmd_msg;
quadmotor_interfaces__msg__FencieHeadPosition pos_msg;

rcl_subscription_t fencie_head_command_subscriber;
rcl_publisher_t fencie_head_position_publisher;

int twixDelay = 5000 / portTICK_PERIOD_MS; // 5000ms = 5s

typedef struct head_axes_state {
	int32_t assigned_angle_degrees;
	int32_t actual_angle_degrees;
	uint32_t min_pulse_us;
	uint32_t max_pulse_us;
	uint32_t max_degrees;
	char name[MAX_NAME_LENGTH];
} head_axes_state_t;

#define MAX_AXES 3
#define IDX_JAW 0
#define IDX_X_AXIS 1
#define IDX_Z_AXIS 2
head_axes_state_t axes[MAX_AXES];
int num_axes =0;

#define I2C_ADDRESS 0x40

/*
 * Initialise the servo control system. 
 */
void servo_control_initialise() {
	ESP_LOGI(TAG, "setting up pca9685");

	// set up servo on pin 18
	// servo_driver_initialize(SERVO_PIN);
	servo_pca9685_initialise(I2C_ADDRESS);

	// in fencie there is a neck with 2 servos
	// and a jaw mover. 
	// channel 0 - jaw mover
	// channel 1 - z axis rotation
	// channel 2 - x axis rotation
	for (int i=0; i< num_axes;i++ ) {
		set_channel_min_max_pulse_us(i, axes[i].min_pulse_us, axes[i].max_pulse_us, axes[i].max_degrees); 
	}	

	ESP_LOGI(TAG, "servo driver initialised");
}

void setup_axes() {
	strcpy(axes[IDX_JAW].name,"Jaw rotator"); // Channel 0 - CSPower DS-S006M 500us -> 2500us), 180 degrees
	axes[IDX_JAW].assigned_angle_degrees = 0;
	axes[IDX_JAW].actual_angle_degrees = 0;
	axes[IDX_JAW].min_pulse_us = 500;
	axes[IDX_JAW].max_pulse_us = 2500;
	axes[IDX_JAW].max_degrees = 180;

	strcpy(axes[IDX_X_AXIS].name,"Head X rotator"); // Channel 1 - Tower Pro MG995 - 544us -> 2400 us, 180 degrees
	axes[IDX_X_AXIS].assigned_angle_degrees = 0;
	axes[IDX_X_AXIS].actual_angle_degrees = 0;
	axes[IDX_X_AXIS].min_pulse_us = 544;
	axes[IDX_X_AXIS].max_pulse_us = 2400;
	axes[IDX_X_AXIS].max_degrees = 180;

	strcpy(axes[IDX_Z_AXIS].name,"Head Z rotator"); // Channel 2 - Tower Pro MG995 - 544us -> 2400 us, 180 degrees
	axes[IDX_Z_AXIS].assigned_angle_degrees = 0;
	axes[IDX_Z_AXIS].actual_angle_degrees = 0;
	axes[IDX_Z_AXIS].min_pulse_us = 544;
	axes[IDX_Z_AXIS].max_pulse_us = 2400;
	axes[IDX_Z_AXIS].max_degrees = 180;

	num_axes = 3;
}	


void rotate_to_assigned_angles()
{
	for (int i=0; i<num_axes; i++) {
		set_pca9685_servo_angle (i, axes[i].assigned_angle_degrees);
	}

	// cheat - say we've already hit the angle:
	for (int i=0; i<num_axes; i++) {
		axes[i].actual_angle_degrees = axes[i].assigned_angle_degrees;
	}
}
/*
 * process the ros message for the given servo
 */
void process_head_command(const quadmotor_interfaces__msg__FencieHeadCommand *msg) {
	
	// ESP_LOGI(TAG, "setting servo angle: jaw:%d, x:%d, and z: %d", 
	// 	msg->jaw_angle,
	// 	msg->arm_rot_x_angle,
	// 	msg->arm_rot_z_angle);

	axes[IDX_JAW].assigned_angle_degrees = msg->jaw_angle;
	axes[IDX_X_AXIS].assigned_angle_degrees = msg->arm_rot_x_angle;
	axes[IDX_Z_AXIS].assigned_angle_degrees = msg->arm_rot_z_angle;

	rotate_to_assigned_angles();
	
}


void fencie_head_command_callback(const void * msgin)
{
	const quadmotor_interfaces__msg__FencieHeadCommand * msg = (const quadmotor_interfaces__msg__FencieHeadCommand *)msgin;
	
	/* int32_t arm_rot_z_angle;
  	 * int32_t arm_rot_x_angle;
     * int32_t jaw_angle;
     */
	ESP_LOGI(TAG, "received fencie head command msg: z rot: %d, x_rot: %d, jaw_angle: %d", 
		msg->arm_rot_z_angle,
		msg->arm_rot_x_angle,
		msg->jaw_angle);

	process_head_command(msg); // in this configuration there is only 1 servo 
}


esp_err_t init_head_driver() {
	setup_axes();

	// wihtout the gui task, we need to set up the i2c master
	servo_control_initialise();
	return ESP_OK;
}

esp_err_t init_head_ros(rcl_allocator_t * allocator, rcl_node_t * node, rclc_executor_t * executor, rclc_support_t * support) {

	pAllocator = allocator;
	pNode = node;
	pExecutor = executor;
	pSupport = support;

	RCCHECK(rclc_subscription_init_default(
		&fencie_head_command_subscriber,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(quadmotor_interfaces, msg, FencieHeadCommand),
		"/fencie/fencie_head_command"));

	RCCHECK(
		rclc_executor_add_subscription(
			pExecutor, 
			&fencie_head_command_subscriber, 
			&cmd_msg, 
			&fencie_head_command_callback, ON_NEW_DATA));

	ESP_LOGI(TAG, "subscription created to: /fencie/fencie_head_command");

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&fencie_head_position_publisher,
		pNode,
		ROSIDL_GET_MSG_TYPE_SUPPORT(quadmotor_interfaces, msg, FencieHeadPosition),
		"/fencie/fencie_head_position"));

	ESP_LOGI(TAG, "publisher intialised to: /fencie/fencie_head_position");
	return ESP_OK;
}

esp_err_t do_head_work() {
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

void positions_please(int jaw, int x_axis, int y_axis) {
	ESP_LOGI(TAG, "Rotate jaw %d, x %d, z %d", jaw, x_axis, y_axis);
	axes[IDX_JAW].assigned_angle_degrees = jaw;
	axes[IDX_X_AXIS].assigned_angle_degrees = x_axis;
	axes[IDX_Z_AXIS].assigned_angle_degrees = y_axis;
	rotate_to_assigned_angles();
	vTaskDelay(twixDelay);
}

esp_err_t test_head_driver() {
	bool skip_test = true;

	if (skip_test) {
		ESP_LOGI(TAG, "Skipping head tests");
		return;
	}
	// rotate z 0-180, return 90
	// rotate  x 0-180, return 90
	// open and close jaw. 

	ESP_LOGI(TAG, "ABout to do Head Tests");

	ESP_LOGI(TAG, "jaw open");
	positions_please(135,45,30);
	ESP_LOGI(TAG, "jaw closed");
	positions_please(45,45,30);

	ESP_LOGI(TAG, "neck rotated right");
	positions_please(60,45,160);
	ESP_LOGI(TAG, "neck rotated left");
	positions_please(60,45,120);
	ESP_LOGI(TAG, "neck middle");
	positions_please(60,45,140);
	
	ESP_LOGI(TAG, "head tilt back");
	positions_please(60,60,140);
	ESP_LOGI(TAG, "head tilt forward");
	positions_please(60,30,140);
	ESP_LOGI(TAG, "head tilt middle");
	positions_please(60,45,140);

	ESP_LOGI(TAG,"All Head Tests complete");
	return ESP_OK;
}


esp_err_t do_head_timer_work(rcl_timer_t * timer, int64_t last_call_time, int timer_count) {
	// send state  message at the required frequency

	// publish the head position.
	if ((timer_count % 20) == 0) {
		pos_msg.arm_rot_x_angle = axes[IDX_X_AXIS].actual_angle_degrees;
		pos_msg.arm_rot_z_angle = axes[IDX_Z_AXIS].actual_angle_degrees;
		pos_msg.jaw_angle = axes[IDX_JAW].actual_angle_degrees;
		// ESP_LOGI(TAG, "published head position: x: %d, y: %d, jaw: %d",
		// 	pos_msg.arm_rot_x_angle,
		// 	pos_msg.arm_rot_z_angle,
		// 	pos_msg.jaw_angle);
		RCSOFTCHECK(rcl_publish(&fencie_head_position_publisher, &pos_msg, NULL));	
	}
	return ESP_OK;
}

esp_err_t terminate_head() {
	return ESP_OK;
}

