/*
 * proximity_sensor.h
 *
 *  Created on: 1 mai 2021
 *      Author: Stephen
 */

#ifndef PROXIMITY_SENSOR_H_
#define PROXIMITY_SENSOR_H_

#define IR_DETECTION_LIMIT 200

typedef struct
{
	uint16_t detection_limit;
	uint8_t front_detection;
	uint8_t back_detection;
} prox_group_t;

void prox_compute_start(void);
void prox_struct_init(uint16_t limit);
int isDetected(int direction);

#endif /* PROXIMITY_SENSOR_H_ */
