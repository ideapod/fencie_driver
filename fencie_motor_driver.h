#ifndef FENCIE_MOTOR_DRIVER_H
#define FENCIE_MOTOR_DRIVER_H

#include "esp_err.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "fencie_config.h"

#define NUM_MOTOR_ROS_HANDLES 1 // only 1 - for subscribing to velocities
esp_err_t init_motor_driver(quad_wheels_t *p_wheels_in);
esp_err_t init_motor_ros(rcl_allocator_t * allocator, rcl_node_t * node, rclc_executor_t * executor, rclc_support_t * support);
esp_err_t do_motor_work();
esp_err_t do_motor_timer_work(rcl_timer_t * timer, int64_t last_call_time, int timer_count);
esp_err_t test_motor_driver();
esp_err_t terminate_motor();

#endif