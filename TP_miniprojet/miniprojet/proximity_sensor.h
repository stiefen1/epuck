/*
 * proximity_sensor.h
 *
 *  Created on: 1 mai 2021
 *      Author: Stephen
 */

#ifndef PROXIMITY_SENSOR_H_
#define PROXIMITY_SENSOR_H_

#define NB_SENSOR_USED 4
#define IR_DETECTION_LIMIT 200

enum prox_sensor_pos{FRONT_RIGHT, FRONT_LEFT, BACK_LEFT, BACK_RIGHT};

typedef struct
{
	uint16_t prox_value[NB_SENSOR_USED];
	uint16_t detection_limit;
	uint8_t front_detection;
	uint8_t back_detection;
}prox_group_t;

void prox_compute_start(void);
void prox_struct_init(uint16_t limit);
int isDetected(int direction);

#endif /* PROXIMITY_SENSOR_H_ */
