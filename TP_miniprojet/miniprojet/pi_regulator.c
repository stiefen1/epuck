#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "sensors/imu.h"
#include "sensors/proximity.h"

#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <compute_data.h>

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    float speed = 0;

    float gyro_x = 0.0;
    float acc_error = 0.0;
    //float kp = 0.2; / Pour le gyro
    //float kd = 1;

    // kp = -3.0 et kd = -100 --> stable
    float kp = -3.0;
    float kd = -150; // -10
    float acc_error_derivative = 0.0;
    float acc_consigne = 0.0;

    float kp_pos = 20;
    float ki_pos = 0.2;
    float prox_consigne = 30; // Env. 6 cm
    float prox_error = 0;
    float speed_pi_commande = 0;
    float prox_error_integral = 0;

    int proximity_delta[2] = {0};

    calibrate_gyro();
    calibrate_acc();
    calibrate_ir();


    while(1){
        time = chVTGetSystemTime();

        // Convert speed from cm/s in pas/s
        /*
         * 1000 = nb pas/tour -> N*v/(2*Pi*dist_par_tour)
         */



       	// read x axis gyroscopic acceleration
       	acc_error_derivative = -acc_error; // d[k] = -e[k-1]


       	//gyro_x = get_computed_gyro(X_AXIS);
       	gyro_x = get_computed_acc(Y_AXIS); // Essai avec une consigne de 0 sur acc_y

        // read proximity sensor
        proximity_delta[0] = get_calibrated_prox(0); // front right
        proximity_delta[1] = get_calibrated_prox(7); // front left

       	//chprintf((BaseSequentialStream *)&SD3, " gx = %f ", gyro_x);
       	//chprintf((BaseSequentialStream *)&SD3, " F-R = %d ", proximity_delta[0]);
       	//chprintf((BaseSequentialStream *)&SD3, " F-L = %d ", proximity_delta[1]);


       	// Position regulator (PI)
       	if(proximity_delta[0] > 8 && proximity_delta[1] > 8) // Check if it's in a good sensibility range
       	{

       		prox_error = prox_consigne - 0.5*(proximity_delta[0] + proximity_delta[1]);

       		if(abs(ki_pos * prox_error_integral) < MOTOR_SPEED_LIMIT)
       		{
       			prox_error_integral += prox_error;
       			speed_pi_commande = kp_pos * prox_error + ki_pos * prox_error_integral;
       		}

       		else if(speed_pi_commande > 0)
       			speed_pi_commande = MOTOR_SPEED_LIMIT;

       		else if(speed_pi_commande < 0)
       			speed_pi_commande = -MOTOR_SPEED_LIMIT;
       	}

       	else
       		speed_pi_commande = 0;

       	// Inverse Pendulum regulator (PD)
       	acc_error = gyro_x - acc_consigne;
       	acc_error_derivative += acc_error; // d[k] = e[k] - e[k-1]

       	// Set speed as the integral of acceleration
       	speed += kp*acc_error + kd*acc_error_derivative;

       	if(abs(speed) > MOTOR_SPEED_LIMIT)
       	{
       		if(speed < 0)
       			speed = -MOTOR_SPEED_LIMIT;

       		else if(speed > 0)
       			speed = MOTOR_SPEED_LIMIT;
       	}

       	//speed = 1000.0*5/(2.0*3.14159*13.0);
       	//speed_l = speed;
       	//speed_r = speed;
       	//speed_l = 1000.0*speed_l/(2.0*3.14159*13.0);
       	//speed_r = 1000.0*speed_r/(2.0*3.14159*13.0);

       	right_motor_set_speed(speed);// + speed_pi_commande);//speed+speed_r);
       	left_motor_set_speed(speed);// + speed_pi_commande);//speed+speed_l);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
