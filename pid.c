#include <esp_log.h>
#include "esp_timer.h"
#include "fencie_config.h"
#include "pid.h"
#include <esp_err.h>

#define TAG "PID"

float Kp = Kp_initial;
float Ki = Ki_initial;
float Kd = Kd_initial;

void dump_pid_coefficients() {
  ESP_LOGI(TAG, "Kp = %f, Ki = %f, Kd = %f", Kp, Ki, Kd);
}

void set_coefficients(float Kp_in, float Ki_in, float Kd_in) {
  Kp = Kp_in;
  Ki = Ki_in;
  Kd = Kd_in;
  dump_pid_coefficients();
}

void resetPID(set_point_t *pSetPoint) {
  ESP_LOGI(TAG, "Resetting PID");
  pSetPoint->target_pulses_per_frame = 0;
  pSetPoint->curr_output = 0;
  pSetPoint->curr_input = 0;
  pSetPoint->prev_output = 0;
  pSetPoint->prev_input = 0;
  pSetPoint->accum_error = 0;
  pSetPoint->prev_err = 0;
  pSetPoint->curr_err = 0;
  // keep tracking these 
  // pSetPoint->prev_frame_pulse_count = 0;
  // pSetPoint->curr_frame_pulse_count = 0;
}


void save_prev_frame(wheel_config_t *pWh) {
  pWh->pid_set_point.prev_output = pWh->pid_set_point.curr_output; // save previous frames data
  pWh->pid_set_point.prev_input = pWh->pid_set_point.curr_input;
  pWh->pid_set_point.prev_err = pWh->pid_set_point.curr_err;
  // track outside of pID: pWh->pid_set_point.prev_frame_pulse_count = pWh->pid_set_point.curr_frame_pulse_count;

}

// pid input is the different in the pulse counts
// that is - how many pulses have we had in the time period? 
// if we are going forwards the curr frame is higher in value than previous
// but its the reverse for going in reverse. 
// make sure its ALWAYS positive. 
long calc_pid_input(wheel_config_t *pWh) {
  long retVal; 
  // input is the distance moved in pulses - current pulses - prev. 
  if (pWh->pid_set_point.target_direction == directionForward) {
    retVal =  // rolling forward the current frame is a higher number
      labs(pWh->pid_set_point.curr_frame_pulse_count) - 
      labs(pWh->pid_set_point.prev_frame_pulse_count);
  }
  else {
    retVal =   // but rolling backwards, the previous frame is a higher number
       labs(pWh->pid_set_point.prev_frame_pulse_count) - 
       labs(pWh->pid_set_point.curr_frame_pulse_count); 
  }

  return labs(retVal);
}

// ensure output is between 0 and max for PWM. 
void enforce_output_limit(wheel_config_t *pWh) {
  if (pWh->pid_set_point.curr_output > LEDC_MAX_PWM) {
    pWh->pid_set_point.curr_output = LEDC_MAX_PWM;
  }

  if (pWh->pid_set_point.curr_output < 0) {
    pWh->pid_set_point.curr_output = 0;
  }
}

/* PID routine to compute the next motor commands */
// see http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/ for basic implementation. 
void doPID(wheel_config_t *pWh) {
  float delta_t = (pWh->pid_set_point.curr_time - pWh->pid_set_point.prev_time) / 1000000.0; // convert to seconds

  save_prev_frame(pWh);

  // get the latest pulse count 
  // TRACK in parent call pWh->pid_set_point.curr_frame_pulse_count = pWh->sensor.current_pulse_count; 

  pWh->pid_set_point.curr_input = calc_pid_input(pWh);

  // determine the error term: curr_error = target pulses - current input; 
  pWh->pid_set_point.curr_err = pWh->pid_set_point.target_pulses_per_frame -
    pWh->pid_set_point.curr_input; 

  
  // accumulate the error * unit of time.
  pWh->pid_set_point.accum_error+= 
    (pWh->pid_set_point.curr_err * delta_t);

  float dErr = (pWh->pid_set_point.curr_err - pWh->pid_set_point.prev_err)/ 
    delta_t; // get the change in the error per unit of time.

  long kpTerm = Kp * pWh->pid_set_point.curr_err;
  long kiTerm = Ki * pWh->pid_set_point.accum_error; 
  long kdTerm = Kd * dErr; 

  // output = kp * error + ki * errsum + kd * derr;
  pWh->pid_set_point.curr_output = 
    kpTerm + 
    kiTerm +
    kdTerm;

  enforce_output_limit(pWh);

  ESP_LOGI(TAG, "PID algorithm: %s, target: %f, delta_t: %f, curr_pulses: %ld, prev_pulses: %ld, input: %ld, error: %f, accum error: %f, derr: %f, curr_output: %ld, prev_output: %ld, kpTerm: %ld, kiTerm: %ld, kdTerm: %ld", 
      pWh->wheel_name, 
      pWh->pid_set_point.target_pulses_per_frame,
      delta_t,
      pWh->pid_set_point.curr_frame_pulse_count,
      pWh->pid_set_point.prev_frame_pulse_count,
      pWh->pid_set_point.curr_input,
      pWh->pid_set_point.curr_err, 
      pWh->pid_set_point.accum_error, 
      dErr,
      pWh->pid_set_point.curr_output,
      pWh->pid_set_point.prev_output,
      kpTerm,
      kiTerm,
      kdTerm);
}

