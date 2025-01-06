#ifndef FENCIE_HEAD_DRIVER_H
#define FENCIE_HEAD_DRIVER_H

/*
 * fencie_head_driver controls the neck - a 3 DOF robot arm with a skull on top.
 * the skull has a jaw that is operated by a small servo.
 */

#include "esp_err.h"

#define NUM_HEAD_ROS_HANDLES 1 // subscribes to head position, handle not needed for publisher

esp_err_t init_head_driver();
esp_err_t init_head_ros(rcl_allocator_t * allocator, rcl_node_t * node, rclc_executor_t * executor, rclc_support_t * support);
esp_err_t do_head_work();
esp_err_t do_head_timer_work(rcl_timer_t * timer, int64_t last_call_time, int timer_count);
esp_err_t test_head_driver();
esp_err_t terminate_head();

#endif