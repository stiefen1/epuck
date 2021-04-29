#include <ch.h>
#include <hal.h>
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "sensors/imu.h"
#include "sensors/proximity.h"

#include "main.h"
#include "motors.h"
#include "pi_regulator.h"
#include "compute_data.h"
#include "estimator.h"

static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

static float angle_x_tab[NB_SAMPLES] = {0};
static float speed[NB_SAMPLES] = {0};
static uint16_t i = 0;

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    reg_param_t* reg_param = (reg_param_t*)arg;

    systime_t time;

    // Régulateur
    float commande = 0;

    float angle_error = 0.0;
    float angle_consigne = 0.0;

    while(1){
        time = chVTGetSystemTime();

       	angle_error = get_angle_x() - angle_consigne;
        reg_param->integral += angle_error;

       	// Set speed as the integral of acceleration
       	commande = reg_param->kp * angle_error; // + reg_param->ki * reg_param->integral;

       	if(abs(commande) > MOTOR_SPEED_LIMIT)
       	{
       		if(commande < 0)
       			commande = -MOTOR_SPEED_LIMIT;

       		else if(commande > 0)
       			commande = MOTOR_SPEED_LIMIT;
       	}

       	right_motor_set_speed(commande);
       	left_motor_set_speed(commande);

       	if(i < NB_SAMPLES)
       	{
       		angle_x_tab[i] = angle_error;
       		// speed[i] = commande;
       		i++;
       	}

       	else if(i == NB_SAMPLES)
       	{
       		SendFloatToComputer((BaseSequentialStream *) &SD3, angle_x_tab, NB_SAMPLES);
       		i = 0;
       	}

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

static THD_WORKING_AREA(waPiRegulatorReader, 256);
static THD_FUNCTION(PiRegulatorReader, arg) {
  float data[3];
  reg_param_t* reg_param = (reg_param_t*)arg;
  while(true) {
    ReceiveFloatFromComputer((BaseSequentialStream *) &SD3, data, 3);

    reg_param->kp = data[0];
    reg_param->kd = data[1];
    reg_param->ki = data[2];
    reg_param->integral = 0.f;

    set_front_led(1);
    chThdSleepMilliseconds(1000);
    set_front_led(0);
  }
}

void pi_regulator_start(reg_param_t* reg_param){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, reg_param);
	chThdCreateStatic(waPiRegulatorReader, sizeof(waPiRegulatorReader), NORMALPRIO, PiRegulatorReader, reg_param);
}
