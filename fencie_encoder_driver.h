#ifndef FENCIE_ENCODER_DRIVER_H
#define FENCIE_ENCODER_DRIVER_H

#include "esp_err.h"
#include <rcl/allocator.h>
#include <executor.h>
#include <rclc.h>
#include <rcl/rcl.h>
#include "fencie_config.h"

#define NUM_ENCODER_ROS_HANDLES 0 // no subscribers, only publishers
esp_err_t init_encoder_driver(quad_wheels_t *pWheels_in);
esp_err_t init_encoder_ros(rcl_allocator_t * allocator, rcl_node_t * node, rclc_executor_t * executor, rclc_support_t * support);
esp_err_t do_encoder_work();
esp_err_t do_encoder_timer_work(rcl_timer_t * timer, int64_t last_call_time, int timer_count);
esp_err_t terminate_encoder();
esp_err_t test_encoder_driver();


#endif