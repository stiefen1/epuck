#include <ch.h>
#include <hal.h>
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <sensors/imu.h>
#include <sensors/proximity.h>
#include <leds.h>
#include <motors.h>

#include "main.h"
#include "pi_regulator.h"
#include "auto_regulator.h"
#include "compute_data.h"
#include "send_data.h"
#include "receive_data.h"

static float acc[NB_SAMPLES] = {0};

static void run_regulator_test(reg_param_t* reg_param, float perturbation)
{
    systime_t time;

    float acc_y = 0.0;
    float acc_error = 0.0;
    float acc_error_derivative = 0.0;
    float acc_consigne = 0.0;
    float acc_error_integral = 0;

    float speed_pd_commande = 0;

    for(int i=0; i<NB_SAMPLES; ++i) {
        time = chVTGetSystemTime();

       	acc_error_derivative = -acc_error; // d[k] = -e[k-1]
       	acc_y = get_computed_acc(Y_AXIS); // Essai avec une consigne de 0 sur acc_y
       	// Inverse Pendulum regulator (PD)
       	acc_error = acc_y - acc_consigne;
       	acc_error_derivative += acc_error; // d[k] = e[k] - e[k-1]
        acc_error_integral += acc_error;

       	// Set speed as the integral of acceleration
       	speed_pd_commande = reg_param->kp*acc_error + reg_param->kd*acc_error_derivative + reg_param->ki*acc_error_integral;

        // Addition de la perturbation au début
        if(i < NB_SAMPLES/8) {
          speed_pd_commande += perturbation;
        }

       	if(abs(speed_pd_commande) > MOTOR_SPEED_LIMIT)
       	{
       		if(speed_pd_commande < 0)
       			speed_pd_commande = -MOTOR_SPEED_LIMIT;

       		else if(speed_pd_commande > 0)
       			speed_pd_commande = MOTOR_SPEED_LIMIT;
       	}

       	right_motor_set_speed(speed_pd_commande);
       	left_motor_set_speed(speed_pd_commande);

        acc[i] = acc_y;
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

static void stop_motor()
{
  right_motor_set_speed(0);
  left_motor_set_speed(0);
}

static bool check_epuck_stability()
{
  // TODO: tester si le position est arrivé en position stable
  return true;
}

static THD_WORKING_AREA(waAutoRegulator, 256);
static THD_FUNCTION(AutoRegulator, arg) {
  chRegSetThreadName(__FUNCTION__);

  // Paramètres du régulateur
  reg_param_t* reg_param = (reg_param_t*)arg;
  float data[3];

  // Réception des paramètres du régulateur
  // C'est aussi une astuce pour démarrer le robot
  // depuis le script python
  set_front_led(1);
  ReceiveFloatFromComputer((BaseSequentialStream *)&SD3, data, 3);
  set_front_led(0);
  reg_param->kp = data[0];
  reg_param->kd = data[1];
  reg_param->ki = data[2];

  // Calibration du tout
  calibrate_gyro();
  calibrate_acc();
  calibrate_ir();

  // Boucle d'autorégulation
  //  1. Rétablissement du epuck en position d'équilibre.
  //  2. Réception des nouveaux paramètres du régulateur du script python.
  //  3. Création d'une perturbation (mis en marche des moteurs pendant un court laps de temps).
  //  4. Régulation avec les paramètres reçu. Acquisition des données.
  //  5. Envoi des données au script python.
  //
  // L'ordre des opérations est changé pour faciliter la synchronisation entre le script
  // python et l'epuck
  while(1){
    // Régulation avec les nouveaux paramètres. Une perturbation est ajouté
    // à la commande
    run_regulator_test(reg_param, 100.f);

    // Stopper les moteurs
    stop_motor();

    // Envoi des donnèes
    SendFloatToComputer((BaseSequentialStream *)&SD3, acc, NB_SAMPLES);

    // Attente de la réception des paramètres pour le régulateur
    // du script python
    ReceiveFloatFromComputer((BaseSequentialStream *)&SD3, data, 3);

    // Rétablissement du robot à une position d'équilibre
    // avec des valeurs de régulateur par défaut
    reg_param->kp = -3.0;
    reg_param->kd = -200.0;
    reg_param->ki = 0.0;

    set_front_led(1);
    for(int i=0; i<3; ++i) {
      run_regulator_test(reg_param, 0.f);
      if(check_epuck_stability()) {
        break;
      }
    }
    set_front_led(0);

    // TODO: Rétabilir le robot aussi en position horizontale
    // La mesure de la distance peut-elle se faire avec les
    // moteurs?
    
    // Modification des paramètres pour le prochain test avec
    // ceux du script python
    reg_param->kp = data[0];
    reg_param->kd = data[1];
    reg_param->ki = data[2];
  }
}

void auto_regulator_start(reg_param_t* reg_param) {
	chThdCreateStatic(waAutoRegulator, sizeof(waAutoRegulator), NORMALPRIO, AutoRegulator, (void*)reg_param);
}
