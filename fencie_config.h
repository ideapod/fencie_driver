#ifndef FENCIE_CONFIG_H
#define FENCIE_CONFIG_H

#include <driver/gpio.h>
#include "hal/ledc_types.h"

#define GPIO_HIGH 1
#define GPIO_LOW 0
#define MAX_NAME_LENGTH 15
#define TIMER_WORK_PERIOD_MILLIS 500
/* 
 * DEFINES the set of pins used with the motors, and encoders.
 * abbreviations - 
 * - fr = front right
 * - fl = front left
 * - rr = rear right
 * - rl = rear left 
 */
#define frEn GPIO_NUM_2 // front right enable
#define rFwd GPIO_NUM_13 // right side forward (one pin forward both rr and fr )
#define rRvr GPIO_NUM_12 // right side reverse
#define frPhA GPIO_NUM_26 
#define frPhB GPIO_NUM_25
#define rrEn GPIO_NUM_15 // rear  right enable
#define rrPhA GPIO_NUM_35
#define rrPhB  GPIO_NUM_34 // switched with Phase A

#define flEn GPIO_NUM_4
#define lFwd GPIO_NUM_17 
#define lRvr GPIO_NUM_16
#define flPhA GPIO_NUM_14 
#define flPhB GPIO_NUM_27 // switched with Phase A
#define rlEn GPIO_NUM_0
#define rlPhA GPIO_NUM_33 
#define rlPhB GPIO_NUM_32 // switched with Phase A

 #define PWRTWO(EXP) (1 << (EXP))
#define LEDC_DUTY_RESOLUTION LEDC_TIMER_13_BIT
#define LEDC_MAX_PWM PWRTWO(LEDC_DUTY_RESOLUTION)
#define LEDC_FREQUENCY 5000 // 5kHz - something about ripple current low.

#define fl_encoder_counts_per_rev 2445
#define fr_encoder_counts_per_rev 2593
#define rl_encoder_counts_per_rev 2183
#define rr_encoder_counts_per_rev 2211

typedef enum wheel_set_e {
	front_left,
	front_right,
	rear_left,
	rear_right
} wheel_set_t;

typedef enum side_set_e {
	lhs,
	rhs
} side_set_t;

typedef enum direction_set {
  directionForward,
  directionReverse,
  directionStopped
} DirectionSet;


typedef struct wheel_sensor_s {
  int pinA; // pin for the encoder phase A
  int pinB; // pin for the encoder phase B
  bool pinStateA; // current state of pin A
  bool pinStateB; // current state of pin B
  bool oldPinStateA; // old state of pin A
  bool oldPinStateB; // old state of pin B
  DirectionSet direction; // What can we detect for for this wheel? 
  long calibrated_pulse_per_rev; // how many pulses represent one revolution of the wheel
  long current_pulse_count; // current pulse count
  long pulse_counts_per_second; // velocity in pulse counts per second 
} wheel_sensor_t;


/* PID setpoint info For a Motor  - from diff_drive.h - articulated robotics. */
typedef struct set_point_s {
  DirectionSet target_direction;
  double target_pulses_per_frame;    // target speed in pulses per frame
  float accum_error; // accumulated error
  long curr_output;                    // last motor setting
  long curr_input;				
  int prev_input; // last input, not last error to avoid derivative kick, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  long prev_output;                    // last motor setting
  long prev_frame_pulse_count; // last frame pulse Count
  long curr_frame_pulse_count;
  float prev_err;
  float curr_err;
  long prev_time; // save previous time slice
  long curr_time;
} set_point_t;

typedef struct wheel_config_s {
	char wheel_name[MAX_NAME_LENGTH];
	int pin_enable;
	wheel_sensor_t sensor;
	set_point_t pid_set_point;
} wheel_config_t;

typedef struct side_drive_s {
	int pin_forward;
	int pin_reverse;
	wheel_config_t front_wheel;
	wheel_config_t rear_wheel;
} side_drive_t;

typedef struct quad_wheels_s {
	side_drive_t sides[2]; // use side_set_t to access;
} quad_wheels_t;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status in %s on line %d: %d. Aborting.\n",__FILE__, __LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status %s on line %d: %d. Continuing.\n",__FILE__, __LINE__,(int)temp_rc);}}

#endif

