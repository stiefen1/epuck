/*
 * proximity_sensor.c
 *
 *  Created on: 20 avr. 2021
 *      Author: Stephen
 */

#include <ch.h>
#include <hal.h>
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <pid_regulator.h>
#include "sensors/imu.h"
#include "sensors/proximity.h"

#include "motors.h"
#include "estimator.h"
#include "receive_data.h"
#include "send_data.h"
#include "proximity_sensor.h"
#include "leds.h"
#include "main_bus.h"

static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);
static float send_tab[2*NB_SAMPLES] = {0};
static reg_param_t reg_param;

static THD_WORKING_AREA(waPIDRegulator, 256);
static THD_FUNCTION(PIDRegulator, arg) {

	chRegSetThreadName(__FUNCTION__);
  (void)arg;

	systime_t time;

  // Control
	float commande = 0;
	float angle_error = 0.f;
  float angle_error_prev = 0.f;

  // Proximity sensors states
	int back = 0;
	int front = 0;

	while(1){
    for(uint16_t i=0; i<NB_SAMPLES; ++i) {
      time = chVTGetSystemTime();

      // Get state of promixity sensors
      back = isDetected(0);
      front = isDetected(1);

      if((back && front) || (!back && !front))
      {
        set_led(LED5, 0);
        set_led(LED1, 0);
      }

      else if(isDetected(0))
      {
        set_led(LED5, 1);
        set_led(LED1, 0);
        reg_param.setpoint += 0.1/10.f;
      }

      else if(isDetected(1))
      {
        set_led(LED1, 1);
        set_led(LED5, 0);
        reg_param.setpoint -= 0.1/10.f;
      }

      angle_error_prev = angle_error;
      angle_error = get_angle_x() - reg_param.setpoint;
      reg_param.derivative = angle_error_prev - angle_error; // Compute the derivative
      reg_param.integral += angle_error; // Compute the integral

      // Limit the integral value if the command is saturated
      if((reg_param.integral * reg_param.ki) > MOTOR_SPEED_LIMIT)
        reg_param.integral = MOTOR_SPEED_LIMIT / reg_param.ki;

      else if((reg_param.integral * reg_param.ki) < - MOTOR_SPEED_LIMIT)
        reg_param.integral = - MOTOR_SPEED_LIMIT / reg_param.ki;


      // Compute the speed command 
      commande = reg_param.kp * angle_error + reg_param.kd * reg_param.derivative + reg_param.ki * reg_param.integral;

      // limits the speed to the motors max speed
      if(commande > MOTOR_SPEED_LIMIT) {
        commande = MOTOR_SPEED_LIMIT;
      } else if(commande < -MOTOR_SPEED_LIMIT) {
        commande = -MOTOR_SPEED_LIMIT;
      }

      right_motor_set_speed(commande);
      left_motor_set_speed(commande);

      send_tab[i] = angle_error;
      send_tab[NB_SAMPLES+i] = commande;

      //20Hz
      chThdSleepUntilWindowed(time, time + MS2ST(50));

    }

    chBSemSignal(&sendToComputer_sem);
	}
}

static THD_WORKING_AREA(waPIDRegulatorReader, 256);
static THD_FUNCTION(PIDRegulatorReader, arg) {

	chRegSetThreadName(__FUNCTION__);
  (void) arg;

	float data[4];
	while(true) {
		ReceiveFloatFromComputer((BaseSequentialStream *) &SD3, data, 4);

		reg_param.kp = data[0];
		reg_param.kd = data[1];
		reg_param.ki = data[2];
		reg_param.setpoint = data[3];

    // Reset de l'intégrale au cas ou
		reg_param.integral = 0.f; 

    // Indicateur visuelle sur le robot que les valeures
    // ont été mise à jour
		set_front_led(1);
		chThdSleepMilliseconds(1000);
		set_front_led(0);
	}
}


// Thread to send the angle datas to the computer
static THD_WORKING_AREA(waPIDRegulatorSender, 256);
static THD_FUNCTION(PIDRegulatorSender, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	while(true) {
		chBSemWait(&sendToComputer_sem);

		set_body_led(1);
		SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, 2*NB_SAMPLES);
		set_body_led(0);
	}
}


void pid_regulator_start(void){
  reg_param.kp = 0.0;
  reg_param.kd = 0.0;
  reg_param.ki = 0.0;
  reg_param.integral = 0.0;
  reg_param.ref = 0.0;

	chThdCreateStatic(waPIDRegulator, sizeof(waPIDRegulator), HIGHPRIO, PIDRegulator, NULL);
	chThdCreateStatic(waPIDRegulatorReader, sizeof(waPIDRegulatorReader), NORMALPRIO, PIDRegulatorReader, NULL);
	chThdCreateStatic(waPIDRegulatorSender, sizeof(waPIDRegulatorSender), LOWPRIO, PIDRegulatorSender, NULL);
}
