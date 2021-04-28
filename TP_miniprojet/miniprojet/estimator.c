#include <ch.h>
#include <hal.h>
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "sensors/imu.h"
#include "sensors/proximity.h"

#include <main.h>
#include <motors.h>
#include <compute_data.h>

static estimator_t estimator_values;

void get_angle_y()
{
  return estimator_values.angle_y;
}

static THD_WORKING_AREA(waEstimator, 256);
static THD_FUNCTION(Estimator, arg) {
  messagebus_topic_t imu_topic = (messagebus_topic_t*)arg;
  systime_t start, elapsed;
  int16_t gyro_y;
  float dt;

  while(true) {
    start = chVTGetSystemTime();

    // Attente des valeurs des prochaines valeurs de l'IMU
    messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

    // Calcul du temps écoulé depuis la dernière itération
    elapsed = chVTTimeElapsedSinceX(start);
    time_msecs_t elapsed_ms = TIME_I2US(elapsed);

    // Calcul de l'accélération angulaire sur l'axe Y
    gyro_y = imu_values.gyro_raw[1] - imu_values.gyro_offset[1];

    // Intégration toute simple pour l'instant
    dt = (float)elaped_ms/1000000.f;
    estimator_values.omega_y += dt * gyro_y;
    estimator_values.angle_y += dt * estimator_values.omega_y;
  }
}

void estimator_start(void){
  estimator_values.angle_y = 0.f;
  estimator_values.omega_y = 0.f;

  messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu")

	chThdCreateStatic(waEstimator, sizeof(waEstimator), NORMALPRIO, Estimator, (void*)imu_topic);
}
