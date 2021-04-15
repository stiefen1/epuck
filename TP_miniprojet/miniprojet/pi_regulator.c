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

    float speed = 0, speed_l = 0, speed_r = 0;

    while(1){
        time = chVTGetSystemTime();

        // Convert speed from cm/s in pas/s
        /*
         * 1000 = nb pas/tour -> N*v/(2*Pi*dist_par_tour)
         */
        speed = 1000.0*5/(2.0*3.14159*13.0);
        speed_l = speed;
        speed_r = speed;
        speed_l = 1000.0*speed_l/(2.0*3.14159*13.0);
        speed_r = 1000.0*speed_r/(2.0*3.14159*13.0);
//
		right_motor_set_speed(speed+speed_r);
		left_motor_set_speed(speed+speed_l);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
