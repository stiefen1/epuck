#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "sensors/imu.h"
#include "sensors/proximity.h"
#include <leds.h>

#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <compute_data.h>

#define NB_SAMPLES 1024

static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

static float acc[NB_SAMPLES] = {0};
static float speed[NB_SAMPLES] = {0};
static uint16_t i = 0;

void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size)
{
	chSequentialStreamWrite(out, (uint8_t*)"START", 5);
	chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite(out, (uint8_t*)data, sizeof(float) * size);
}

void SendFloatToComputerFast(BaseSequentialStream* out, float data)
{
	chSequentialStreamWrite(out, (uint8_t*)&data, sizeof(float));
}


uint16_t ReceiveFloatFromComputer(BaseSequentialStream* in, float* data, uint16_t size){

	volatile uint8_t c1, c2;
	volatile uint16_t temp_size = 0;
	uint16_t i=0;

	volatile uint8_t state = 0;
	while(state != 5){

        c1 = chSequentialStreamGet(in);

        //State machine to detect the string EOF\0S in order synchronize
        //with the frame received
        switch(state){
        	case 0:
        		if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
          break;
        	case 1:
        		if(c1 == 'T')
        			state = 2;
        		else if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
          break;
        	case 2:
        		if(c1 == 'A')
        			state = 3;
        		else if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
          break;
        	case 3:
        		if(c1 == 'R')
        			state = 4;
        		else if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
          break;
        	case 4:
        		if(c1 == 'T')
        			state = 5;
        		else if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
          break;
        }

	}

	c1 = chSequentialStreamGet(in);
	c2 = chSequentialStreamGet(in);

	// The first 2 bytes is the length of the datas
	// -> number of int16_t data
	temp_size = (int16_t)((c1 | c2<<8));

	if(temp_size/4 == size){
    chSequentialStreamRead(in, (uint8_t*)data, temp_size);
	}
	return temp_size;

}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);

    systime_t time;

    float speed_pd_commande = 0;

    float acc_y = 0.0;
    float acc_error = 0.0;
    float acc_error_derivative = 0.0;
    float acc_consigne = 0.0;

    // PD Regulator
    float kp = -3.0;
    float kd = -200; // -10
    RegParam* reg_param = (RegParam*)arg;

    float prox_consigne = 30; // Env. 6 cm
    float prox_error = 0;
    float speed_pi_commande = 0;
    float prox_error_integral = 0;

    // PI Regulator
    float kp_pos = 20;
    float ki_pos = 0.2;

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
       	acc_y = get_computed_acc(Y_AXIS); // Essai avec une consigne de 0 sur acc_y


        // read proximity sensor
        // proximity_delta[0] = get_calibrated_prox(0); // front right
        // proximity_delta[1] = get_calibrated_prox(7); // front left

       	//chprintf((BaseSequentialStream *)&SD3, " gx = %f ", gyro_x);
       	//chprintf((BaseSequentialStream *)&SD3, " F-R = %d ", proximity_delta[0]);
       	//chprintf((BaseSequentialStream *)&SD3, " F-L = %d ", proximity_delta[1]);


       	// Position regulator (PI)
       	// if(proximity_delta[0] > 8 && proximity_delta[1] > 8) // Check if it's in a good sensibility range
       	// {
// 
       		// prox_error = prox_consigne - 0.5*(proximity_delta[0] + proximity_delta[1]);
// 
       		// if(abs(ki_pos * prox_error_integral) < MOTOR_SPEED_LIMIT)
       		// {
       			// prox_error_integral += prox_error;
       			// speed_pi_commande = kp_pos * prox_error + ki_pos * prox_error_integral;
       		// }
// 
       		// else if(speed_pi_commande > 0)
       			// speed_pi_commande = MOTOR_SPEED_LIMIT;
// 
       		// else if(speed_pi_commande < 0)
       			// speed_pi_commande = -MOTOR_SPEED_LIMIT;
       	// }
// 
       	// else
       		// speed_pi_commande = 0;

       	// Inverse Pendulum regulator (PD)
       	acc_error = acc_y - acc_consigne;
       	acc_error_derivative += acc_error; // d[k] = e[k] - e[k-1]

       	// Set speed as the integral of acceleration
       	speed_pd_commande += reg_param->kp*acc_error + reg_param->kd*acc_error_derivative;

       	if(abs(speed_pd_commande) > MOTOR_SPEED_LIMIT)
       	{
       		if(speed_pd_commande < 0)
       			speed_pd_commande = -MOTOR_SPEED_LIMIT;

       		else if(speed_pd_commande > 0)
       			speed_pd_commande = MOTOR_SPEED_LIMIT;
       	}

       	//speed = 1000.0*5/(2.0*3.14159*13.0);
       	//speed_l = 1000.0*speed_l/(2.0*3.14159*13.0);
       	//speed_r = 1000.0*speed_r/(2.0*3.14159*13.0);

       	//right_motor_set_speed(speed_pd_commande);// + speed_pi_commande);//speed+speed_r);
       	//left_motor_set_speed(speed_pd_commande);// + speed_pi_commande);//speed+speed_l);

        acc[i++] = acc_y;

        if(i == NB_SAMPLES) {
          SendFloatToComputer((BaseSequentialStream *)&SD3, acc, NB_SAMPLES);
          i = 0;
        }


        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));

    }
}

static THD_WORKING_AREA(waPiRegulatorReader, 256);
static THD_FUNCTION(PiRegulatorReader, arg) {
  float data[2];
  RegParam* reg_param = (RegParam*)arg;

  while(true) {
    ReceiveFloatFromComputer((BaseSequentialStream *)&SD3, data, 2);
    reg_param->kp = data[0];
    reg_param->kd = data[1];

    set_front_led(1);
    chThdSleepMilliseconds(1000);
    set_front_led(0);
  }
}

void pi_regulator_start(RegParam* reg_param) {
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, (void*)reg_param);
	chThdCreateStatic(waPiRegulatorReader, sizeof(waPiRegulatorReader), NORMALPRIO+1, PiRegulatorReader, (void*)reg_param);
}
