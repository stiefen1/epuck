#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    float distance_consigne = 10;
    float position_consigne = 340;
    float Kp = 5;
    float Ki = 0.01;
    float Kp_pos = 0.1;
    float Ki_pos = 0;
    float integral_pos = 0;
    float error_pos = 0;
    float position = 0;
    float integral = 0;
    float error = 0;
    float distance = 0;
    float speed_l = 0;
    float speed_r = 0;

    float speed = 0;

    while(1){
        time = chVTGetSystemTime();

        position = get_position_pxl();
        distance = get_distance_cm();

        // If the distance = 0 it means that the line has not been detected
        if(distance != 0)
        {
        	// Compute the error between distance (measured) and position goal
        	error = distance - distance_consigne;
        	error_pos = position - position_consigne;

        	// Anti-windup
        	/*
        	 * Si le terme intégral dépasse 5, sa valeur est limitée à 5
        	 */
        	if(integral > 5)
        		integral = 5;

        	else if(integral < -5)
        		integral = -5;

        	else
        		integral += error;

        	// Compute the speed
        	speed = Kp*error+Ki*integral;
        	speed_l = Kp_pos * error_pos;
        	speed_r = -Kp_pos * error_pos;
        }

        else
        {
        	speed = 0.0;
        	speed_l = 10;
        	speed_r = -10;
        }

        // Convert speed from cm/s in pas/s
        /*
         * 1000 = nb pas/tour -> N*v/(2*Pi*dist_par_tour)
         */
        speed = 1000.0*speed/(2.0*3.14159*13.0);
        speed_l = 1000.0*speed_l/(2.0*3.14159*13.0);
        speed_r = 1000.0*speed_r/(2.0*3.14159*13.0);

		right_motor_set_speed(speed+speed_r);
		left_motor_set_speed(speed+speed_l);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
