#include "fencie_config.h"

/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID Parameters */
#define  Kp_initial 2.5
#define Ki_initial 0.5  // 0.5
#define Kd_initial 0.05  // 4


/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(set_point_t *pSetPoint);

/* PID routine to compute the next motor commands */
void doPID(wheel_config_t *pWheelConfig);

void dump_pid_coefficients();
void set_coefficients(float Kp_in, float Ki_in, float Kd_in);
