/*
 * pid_regulator.h
 *
 *  Created on: 20 avr. 2021
 *      Author: Steph
 */

#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

#define NB_SAMPLES 1024

typedef struct {
  float kp;
  float kd;
  float ki;
  
  float integral;
  float derivative;
} reg_param_t;

//start the PID regulator thread
void pid_regulator_start(void);


#endif /* PID_REGULATOR_H */
