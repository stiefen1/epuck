#include <ch.h>
#include <chvt.h>
#include <hal.h>
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "sensors/imu.h"
#include "sensors/proximity.h"

#include "main.h"
#include "motors.h"
#include "compute_data.h"
#include "pi_regulator.h"
#include "estimator.h"

static estimator_t estimator_values;
static float theta[NB_SAMPLES] = {0};

float get_angle_x()
{
  // return estimator_values.angle_x;
  return estimator_values.angle_x;
}

static THD_WORKING_AREA(waEstimator, 256);
static THD_FUNCTION(Estimator, arg) {
  messagebus_topic_t* imu_topic = (messagebus_topic_t*)arg;
  systime_t start, elapsed;
  float gyro_x;
  float dt;
  imu_msg_t imu_values;
  int32_t elapsed_us;

  calibrate_gyro();
  calibrate_acc();
  calibrate_ir();

  while(true) {
    start = chVTGetSystemTime();

    // Attente des valeurs des prochaines valeurs de l'IMU
    messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

    // Calcul du temps écoulé depuis la dernière itération
    elapsed = chVTTimeElapsedSinceX(start);
    elapsed_us = (int32_t)ST2US(elapsed);

    // Intégration toute simple pour l'instant
    dt = (float)elapsed_us/1000000.f;
    estimator_values.angle_x += dt * imu_values.gyro_rate[0];
  }
}

static THD_WORKING_AREA(waEstimatorWrite, 256);
static THD_FUNCTION(EstimatorWrite, arg) {
  systime_t time;
  int i = 0;

  while(true) {
    time = chVTGetSystemTime();

    if(i == NB_SAMPLES) {
      SendFloatToComputer((BaseSequentialStream *)&SD3, theta, NB_SAMPLES);
      i = 0;
    }

    theta[i++] = get_angle_x();
    chThdSleepUntilWindowed(time, time + MS2ST(10));
  }
}

void estimator_start(void){
  messagebus_topic_t *imu_topic;

  estimator_values.angle_x = 0.f;
  estimator_values.omega_x = 0.f;

  imu_topic = messagebus_find_topic_blocking(&bus, "/imu");

	chThdCreateStatic(waEstimator, sizeof(waEstimator), NORMALPRIO, Estimator, (void*)imu_topic);
	chThdCreateStatic(waEstimatorWrite, sizeof(waEstimatorWrite), NORMALPRIO, EstimatorWrite, NULL);
}
