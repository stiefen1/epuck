/*
 * proximity_sensor.c
 *
 *  Created on: 1 mai 2021
 *      Author: Stephen
 */

#include <ch.h>
#include <hal.h>
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <compute_imu_data.h>
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

		// read the values from the 4 sensors that we use
		proximity_sensors.prox_value[FRONT_RIGHT] = get_calibrated_prox(0);
		proximity_sensors.prox_value[FRONT_LEFT] = get_calibrated_prox(7);
		proximity_sensors.prox_value[BACK_LEFT] = get_calibrated_prox(4);
		proximity_sensors.prox_value[BACK_RIGHT] = get_calibrated_prox(3);

		// Check the proximity sensors values to detect an object
		for(int i=0; i<NB_SENSOR_USED; i++)
		{
			if(i == 1 || i == 3)
			{
				// The sum of 2 values needs to be > threshold to consider an object to be detected
				if((proximity_sensors.prox_value[i] + proximity_sensors.prox_value[i-1]) > proximity_sensors.detection_limit)
				{
					// Check on which side the object has been detected
					if(i==1)
						proximity_sensors.front_detection = 1;

					else if(i==3)
						proximity_sensors.back_detection = 1;
				}
				else
				{
					// Check on which side nothing has been detected
					if(i==1)
						proximity_sensors.front_detection = 0;

					else if(i==3)
						proximity_sensors.back_detection = 0;
				}
			}
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
