/*
 * estimator.c
 *
 *  Created on: 21 avr. 2021
 *      Author: Julien
 */

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

#include "motors.h"
#include "estimator.h"

#include "main_bus.h"
#include "send_data.h"

#define STANDARD_GRAVITY    9.80665f 
// Le temps de rafraichissement de l'IMU est quasi-constant (determiné empiriquement) 
// donc le temps est directement fixé à une constante pour réduire le nombre d'instructions
#define IMU_REFRESH_TIME 4.f/1000.f
#define SENSOR_FUSION_ALPHA 0.7f

static estimator_t estimator_values;

float get_angle_x()
{
  return estimator_values.angle_x;
}

static THD_WORKING_AREA(waEstimator, 256);
static THD_FUNCTION(Estimator, arg) {

  messagebus_topic_t* imu_topic = (messagebus_topic_t*)arg;
  imu_msg_t imu_values;

  float dt;
  float angle_acc;

  while(true) {
    // Attente des valeurs des prochaines valeurs de l'IMU
    messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

    // Calcul de l'angle grace a  l'accelerometre
    angle_acc = atan2f(imu_values.acceleration[1], -imu_values.acceleration[2]);

    // Sensor fusion.
    //  1. Angle de l'accéleromètre
    //  2. Angle du gyroscope (Intégration)
    // Combinaison linéaire
    estimator_values.angle_x = SENSOR_FUSION_ALPHA*(estimator_values.angle_x + IMU_REFRESH_TIME * imu_values.gyro_rate[0]) + (1.f - SENSOR_FUSION_ALPHA) * angle_acc;
  }
}

void estimator_start(void){
  messagebus_topic_t *imu_topic;
  imu_topic = messagebus_find_topic_blocking(&bus, "/imu");

  estimator_values.angle_x = 0.f;

  chThdCreateStatic(waEstimator, sizeof(waEstimator), NORMALPRIO+2, Estimator, (void*)imu_topic);
}
