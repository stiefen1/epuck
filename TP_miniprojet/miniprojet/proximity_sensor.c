/*
 * proximity_sensor.c
 *
 *  Created on: 1 mai 2021
*   Author: Stephen
 *
 *  Description: 
 *    Thread qui gère la détection d'objet à 
 *    proximité des capteurs avant et arrière.
 *
 */

#include <ch.h>
#include <hal.h>
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "sensors/proximity.h"
#include "leds.h"

#include "proximity_sensor.h"
#include "main_bus.h"

static prox_group_t proximity_sensors;

// Thread to compute the proximity sensors datas
static THD_WORKING_AREA(waProximitySensorCompute, 256);
static THD_FUNCTION(ProximitySensorCompute, arg) {

	(void) arg;
	// Initalization of the proximity_sensors datas
	prox_struct_init(IR_DETECTION_LIMIT);

	while(true) {
    // FRONT_RIGHT & FRONT_LEFT
    if(get_calibrated_prox(0) + get_calibrated_prox(7) > proximity_sensors.detection_limit) {
      // The sum of 2 values needs to be > threshold to consider an object to be detected
      proximity_sensors.front_detection = 1;
    } else {
      proximity_sensors.front_detection = 0;
    }

    // BACK_LEFT & BACK_RIGHT
    if(get_calibrated_prox(4) + get_calibrated_prox(3) > proximity_sensors.detection_limit) {
      // The sum of 2 values needs to be > threshold to consider an object to be detected
      proximity_sensors.back_detection = 1;
    } else {
      proximity_sensors.back_detection = 0;
    }

		chThdSleepMilliseconds(200);
  }
}

void prox_compute_start(void)
{
	// Start the thread to compute proximity sensors data
	chThdCreateStatic(waProximitySensorCompute, sizeof(waProximitySensorCompute), NORMALPRIO, ProximitySensorCompute, NULL);
}

void prox_struct_init(uint16_t limit)
{
	// Initialize the proximity sensors structure values
	proximity_sensors.detection_limit = limit;
	proximity_sensors.front_detection = 0;
	proximity_sensors.back_detection = 0;
}

// Return 1 if something has been detected in the direction
int isDetected(int direction)
{
	// The value of direction can only be 1 (front) or 0 (back)
	if(direction != 0 && direction != 1) // return 0 if wrong direction value appears
		return 0;

	switch(direction)
	{
	case 0:
		return proximity_sensors.back_detection;
		break;

	case 1:
		return proximity_sensors.front_detection;
		break;
	}

	return 0;
}
