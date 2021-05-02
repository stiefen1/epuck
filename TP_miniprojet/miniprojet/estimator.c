#include <ch.h>
#include <chvt.h>
#include <hal.h>
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <compute_imu_data.h>
#include <pid_regulator.h>
#include "sensors/imu.h"
#include "sensors/proximity.h"

#include "main.h"
#include "motors.h"
#include "estimator.h"
#include "send_data.h"

// Erreur lin�aire
#define LIN_ERR (1.0/30.0)/(1024.0*0.01) 
#define STANDARD_GRAVITY    9.80665f 

static estimator_t estimator_values;

float get_angle_x()
{
  // return estimator_values.angle_x;
  return estimator_values.angle_x;
}

static THD_WORKING_AREA(waEstimator, 256);
static THD_FUNCTION(Estimator, arg) {

  messagebus_topic_t* imu_topic = (messagebus_topic_t*)arg;
  imu_msg_t imu_values;

  float dt;
  float alpha = 0.5f;
  float angle_acc;

  uint32_t elapsed_us;

  while(true) {
    // Attente des valeurs des prochaines valeurs de l'IMU
    messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

    // Calcul du temps écoulé depuis la dernière itération
    elapsed_us = 4000;
    // Temps de rafraichissement de l'IMU (mesur�)

    // Calcul de l'angle grâce à l'accéléromètre
    angle_acc = atan2f(imu_values.acceleration[1]/STANDARD_GRAVITY, -imu_values.acceleration[2]/STANDARD_GRAVITY);

    // Sensor fusion
    dt = (float)elapsed_us/1000000.f;
    estimator_values.angle_x = alpha*(estimator_values.angle_x + dt * imu_values.gyro_rate[0]) + (1.f - alpha) * angle_acc;
  }
}

void estimator_start(void){

  messagebus_topic_t *imu_topic;
  imu_topic = messagebus_find_topic_blocking(&bus, "/imu");

  estimator_values.angle_x = 0.f;

  chThdCreateStatic(waEstimator, sizeof(waEstimator), NORMALPRIO+2, Estimator, (void*)imu_topic);
}
