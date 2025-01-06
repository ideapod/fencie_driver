/*
 * micro ros tasks to drive fencie robot.
 * It should receive a QuadMotorvelocities message and set speeds
 * it should report encoder counts using Encodercounts message
 * It should set position of fencie head using HeadCommand message
 * It should report position of fencie head using HeadCommand message
 * ros wise - it subscribes to 2 messages, and publishes 2 messages.
 */

#include <rcl/types.h>
#include <stdio.h>
#include <sys/types.h>
#include <types.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/timer.h>

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "fencie_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "uros_task.h"
#include "app.h"
#include "fencie_encoder_driver.h"
#include "fencie_motor_driver.h"
#include "fencie_head_driver.h"
#include "lwip/err.h"

#define TAG "UROS"

#include "servo_pca9685.h"
#define I2C_ADDRESS 0x40

// uncomment if we need to do http calls for heartbeats.
// #define HTTP_HEARTBEAT 1

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status in %s on line %d: %d. Aborting.\n",__FILE__, __LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status %s on line %d: %d. Continuing.\n",__FILE__, __LINE__,(int)temp_rc);}}

/*************************
 * globals
 *************************/
rcl_subscription_t subscriber, subscriber0, subscriber1;

QueueHandle_t xDataQueue;

rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
quad_wheels_t quad_wheels_config;

rcl_publisher_t publisher;
std_msgs__msg__Int32 publish_msg;
int count_seconds;
rcl_timer_t timer;
bool do_http_heartbeat = false;
bool do_report = false;
rcl_timer_callback_t pCallback;
const unsigned int timer_timeout_milli = TIMER_WORK_PERIOD_MILLIS; // do work every 1 second. 
rcl_allocator_t allocator; 

/*****************************
Prototypes
******************************/
void send_queue_servo_angle(int servo_num, int32_t data);
void process_servo_msg(int servo_num, const std_msgs__msg__Int32 *msg);



/* 
 * timer_callback is for regularly publishing position messages
 */
void uros_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	esp_err_t err;
	RCLC_UNUSED(last_call_time);

	// ESP_LOGI(TAG, "Timer callback seconds: %d, call time: %lld",
	// 	count_seconds, last_call_time);


	if (timer != NULL) {

		count_seconds++;

		err = do_encoder_timer_work(timer, last_call_time, count_seconds);
		assert(err == ESP_OK);

		err = do_motor_timer_work(timer, last_call_time, count_seconds);
		assert(err == ESP_OK);

		err = do_head_timer_work(timer, last_call_time, count_seconds);
		assert(err == ESP_OK);

		if (count_seconds % 10 == 0) {
			do_report = true;
		}

	} else {
		ESP_LOGE(TAG, "timer callback on null timer instance");
	}
}


/* Config the i2c master
 *
 * This should init the i2c master to be used on display and touch controllers.
 * So we should be able to know if the display and touch controllers shares the
 * same i2c master.
 */
bool i2c_master_init(int port, int sda_pin, int scl_pin, int speed_hz)
{
    esp_err_t err;
    
    ESP_LOGI(TAG, "Initializing I2C master port %d...", port);
    ESP_LOGI(TAG, "SDA pin: %d, SCL pin: %d, Speed: %d (Hz)",
        sda_pin, scl_pin, speed_hz);
    
    i2c_config_t conf = {
        .mode               = I2C_MODE_MASTER,
        .sda_io_num         = sda_pin,
        .sda_pullup_en      = GPIO_PULLUP_ENABLE,
        .scl_io_num         = scl_pin,
        .scl_pullup_en      = GPIO_PULLUP_ENABLE,
        .master.clk_speed   = speed_hz,
    };

    ESP_LOGI(TAG, "Setting I2C master configuration...");
    err = i2c_param_config(port, &conf);
    assert(ESP_OK == err);

    ESP_LOGI(TAG, "Installing I2C master driver...");
    err = i2c_driver_install(port,
        I2C_MODE_MASTER,
        0, 0 /*I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE */,
        0 /* intr_alloc_flags */);
    assert(ESP_OK == err);

    return ESP_OK != err;
}

void set_wheel_config() {

	quad_wheels_config.sides[lhs].pin_forward = lFwd;
	quad_wheels_config.sides[lhs].pin_reverse = lRvr;
	quad_wheels_config.sides[rhs].pin_forward = rFwd;
	quad_wheels_config.sides[rhs].pin_reverse = rRvr;

	quad_wheels_config.sides[lhs].front_wheel.pin_enable = flEn;
	quad_wheels_config.sides[lhs].front_wheel.sensor.pinA = flPhA;
	quad_wheels_config.sides[lhs].front_wheel.sensor.pinB = flPhB;
	strcpy(quad_wheels_config.sides[lhs].front_wheel.wheel_name, "Front Left");
	quad_wheels_config.sides[lhs].front_wheel.sensor.calibrated_pulse_per_rev = fl_encoder_counts_per_rev;

	quad_wheels_config.sides[rhs].front_wheel.pin_enable = frEn;
	quad_wheels_config.sides[rhs].front_wheel.sensor.pinA = frPhA;
	quad_wheels_config.sides[rhs].front_wheel.sensor.pinB = frPhB;
	strcpy(quad_wheels_config.sides[rhs].front_wheel.wheel_name, "Front Right");
	quad_wheels_config.sides[rhs].front_wheel.sensor.calibrated_pulse_per_rev = fr_encoder_counts_per_rev;

	quad_wheels_config.sides[lhs].rear_wheel.pin_enable = rlEn;
	quad_wheels_config.sides[lhs].rear_wheel.sensor.pinA = rlPhA;
	quad_wheels_config.sides[lhs].rear_wheel.sensor.pinB = rlPhB;
	strcpy(quad_wheels_config.sides[lhs].rear_wheel.wheel_name, "Rear Left");
	quad_wheels_config.sides[lhs].rear_wheel.sensor.calibrated_pulse_per_rev = rl_encoder_counts_per_rev;

	quad_wheels_config.sides[rhs].rear_wheel.pin_enable = rrEn;
	quad_wheels_config.sides[rhs].rear_wheel.sensor.pinA = rrPhA;
	quad_wheels_config.sides[rhs].rear_wheel.sensor.pinB = rrPhB;
	strcpy(quad_wheels_config.sides[rhs].rear_wheel.wheel_name, "Rear Right");
	quad_wheels_config.sides[rhs].rear_wheel.sensor.calibrated_pulse_per_rev = rr_encoder_counts_per_rev;
}

/* 
 * going to use this function to initialise all the local hardware
 */ 
void init_local_systems() {
	
	set_wheel_config();


	//install gpio isr service - enables per pin interrupts
  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);

  // set up i2c master bus
  // from OLED display output:
	// I (2297) lvgl_helpers: Initializing I2C master port 0...
	// I (2297) lvgl_helpers: SDA pin: 21, SCL pin: 22, Speed: 400000 (Hz)
	//without the GUI task we need to install master
	int port = 0;
	int sda_pin = 21;
	int scl_pin = 22;
	int speed_hz = 40000;
	esp_err_t err;

	ESP_LOGI(TAG, "initialising i2c Master");
	err = i2c_master_init(port, sda_pin, scl_pin, speed_hz);
	assert(ESP_OK == err);

	i2c_scan();

	err = init_encoder_driver(&quad_wheels_config);
	assert(err == ESP_OK);

	err = init_motor_driver(&quad_wheels_config);
	assert(err == ESP_OK);

	err = init_head_driver();
	assert(err == ESP_OK);
}


void test_local_systems() {
	esp_err_t err;
	ESP_LOGI(TAG, "Test local systems");

	err = test_encoder_driver();
	assert(err == ESP_OK);

	err = test_motor_driver();
	assert(err == ESP_OK);

	err = test_head_driver();
	assert(err == ESP_OK);
}


void dump_support(rclc_support_t *pSupport) {
		ESP_LOGI(TAG, "Dumping support");
		// ESP_LOGI(TAG, "- num action clients: %zu", ;
		// ESP_LOGI(TAG, "- num events: %zu", executor.info.number_of_events);
		// ESP_LOGI(TAG, "- num services: %zu", executor.info.number_of_services);
		// ESP_LOGI(TAG, "- num action clients: %zu", executor.info.number_of_action_clients);
		// ESP_LOGI(TAG, "- num action servers: %zu", executor.info.number_of_action_servers);
		// ESP_LOGI(TAG, "- num guard conditions: %zu", executor.info.number_of_guard_conditions);
		// ESP_LOGI(TAG, "- num subscriptions: %zu", executor.info.number_of_subscriptions);
		// ESP_LOGI(TAG, "- num timers: %zu", executor.info.number_of_timers);
}

/*
 * this routine initialises all the micro-ros subscriptiosn, publishers, services etc.
 */
esp_err_t init_micro_ros() {

	rcl_ret_t ret;
	allocator = rcl_get_default_allocator();


	// create init_options
	ret = rclc_support_init(&support, 0, NULL, &allocator);
	if (ret == RCL_RET_ERROR) {
		ESP_LOGE(TAG, "Error initialising support object");
		dump_support(&support);
		return ESP_FAIL;
	}

	// create node
	node = rcl_get_zero_initialized_node();
	ret = rclc_node_init_default(&node, "fencie_driver", "", &support);
	if (ret == RCL_RET_ERROR) {
		ESP_LOGE(TAG, "Error initialising node");
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "Node created: fencie_driver");
	// create timer for publishing a message. 
	count_seconds = 0;

	// timer_timeout_milli = 500;
	ESP_LOGI(TAG, "creating timer, timout %d, num timers: %zu",
		timer_timeout_milli,
		executor.info.number_of_timers);

	timer = rcl_get_zero_initialized_timer();
	ret = rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout_milli),
		uros_timer_callback);

	if (ret != RCL_RET_OK) {
		ESP_LOGE(TAG,"Timer initialise failed: %d", ret);
	}

	// call each module to set up ros publishers/subscriptions:
	executor = rclc_executor_get_zero_initialized_executor();
	// create executor
	int num_handles = NUM_ENCODER_ROS_HANDLES + NUM_HEAD_ROS_HANDLES + NUM_MOTOR_ROS_HANDLES + 1; // what each component needs + timer.

	RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
	ESP_LOGI(TAG, "executor created with num_handles: %d", num_handles);

	ret = rclc_executor_add_timer(&executor, &timer);

	if (ret != RCL_RET_OK) {
		ESP_LOGE(TAG,"Timer add to executor failed: %d", ret);
	}

	ESP_LOGI(TAG, "timer created, num timers: %zu", executor.info.number_of_timers);

	ESP_LOGI(TAG, "modules starting");
	// get the modules to setup their ros integration
	esp_err_t err = init_head_ros(&allocator, &node, &executor, &support);
	assert(err == ESP_OK);

	err = init_motor_ros(&allocator, &node, &executor, &support);
	assert(err == ESP_OK);

	err = init_encoder_ros(&allocator, &node, &executor, &support);
	assert(err == ESP_OK);


	ESP_LOGI(TAG, "modules finished, num timers after: %zu", executor.info.number_of_timers);
	return ESP_OK;
}

void dump_executor(rclc_executor_t *pExecutor) {
		ESP_LOGI(TAG, "Dumping executor");
		ESP_LOGI(TAG, "- num action clients: %zu", executor.info.number_of_action_clients);
		ESP_LOGI(TAG, "- num events: %zu", executor.info.number_of_events);
		ESP_LOGI(TAG, "- num services: %zu", executor.info.number_of_services);
		ESP_LOGI(TAG, "- num action clients: %zu", executor.info.number_of_action_clients);
		ESP_LOGI(TAG, "- num action servers: %zu", executor.info.number_of_action_servers);
		ESP_LOGI(TAG, "- num guard conditions: %zu", executor.info.number_of_guard_conditions);
		ESP_LOGI(TAG, "- num subscriptions: %zu", executor.info.number_of_subscriptions);
		ESP_LOGI(TAG, "- num timers: %zu", executor.info.number_of_timers);

}

/*
 * main entry point for this module 
 * this routine taskes an interprocess (inter-core) queue handle created by the caller.
 * this is used to inform the UI of any updates
 * it initialises the hardware
 * then initialises the ros entities.
 * then spins as a task  
 */
void uros_start(QueueHandle_t inQueueHandle)
{

	rcl_ret_t ret;
	esp_err_t err;

	xDataQueue = inQueueHandle; // save the IPC queue

	
  init_local_systems();

  test_local_systems();

  err = init_micro_ros();
	if (err != ERR_OK) {
		ESP_LOGE(TAG, "micro-ros failed");
		return;
	}


	long loop_count = 0;
	int last_forced_callback = 0;

	long usecond_prev = esp_timer_get_time();
	long usecond_curr = 0;
	int loop_counted_seconds = 0;
	int no_data_count = 0;

	// main task loop - lets the executor do work and periodically does house keeping. 
	dump_executor(&executor);


	
	ESP_LOGI(TAG, "executor starting ");
	while(1){
			loop_count++;

			ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(200));
			// ret = rclc_executor_spin_one_period(&executor, RCL_MS_TO_NS(200));
			// ret = rclc_executor_spin(&executor);

			// TODO - dump executor at various points - before setup, after setup, after spin
			// implement toggles for each module so they can be on and off. (turn them all off, and just get the timer to fire.
			// add them back in one by one and see when it breaks) 

			if (ret == RCL_RET_TIMEOUT) {
				no_data_count++;
			}

			if (ret == RCL_RET_ERROR) { 
				ESP_LOGE(TAG, "executor spin returned error");
			}

			// if(do_report) {
			// 	ESP_LOGI(TAG, "10 seconds gone by: %d", count_seconds);
			// 	do_report = false;
			// }

			err = do_encoder_work();
			assert(err == ESP_OK);

			err = do_head_work();
			assert(err == ESP_OK);

			err = do_motor_work();
			assert(err == ESP_OK);

			
			// if ((loop_count % 5000) == 0) { // only check this every 5000 counts
				
			// 	usecond_curr = esp_timer_get_time();
			// 	long delta_usecond = usecond_curr - usecond_prev;
			// 	// ESP_LOGI(TAG,"looping count: %ld, current: %ld, previous: %ld, delta: %ld",
			// 	// 	loop_count, usecond_curr, usecond_prev, delta_usecond);

			// 	if ((((delta_usecond / 1000) >  timer_timeout_milli)) ) { // call every timer timeout period
			// 			loop_counted_seconds++;
			// 			pCallback(&timer, usecond_curr); // callback the timer.
			// 			last_forced_callback = loop_counted_seconds;
			// 			usecond_prev = usecond_curr;
			// 			ESP_LOGI(TAG, "Called back, no data count: %d", no_data_count);
			// 	}
			// }

			usleep(10000);
	}

	ESP_LOGI(TAG, "executor exited? cleaning up and dying (shouldnt happen)");
	// free resources
	terminate_head();
	terminate_encoder();
	terminate_motor();

	RCCHECK(rcl_node_fini(&node));
	
	vTaskDelete(NULL);
}
