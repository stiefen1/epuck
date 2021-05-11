/*
 * compute_imu_data.c
 *
 *  Created on: 15 avr. 2021
 *      Author: Steph
 */

#include <math.h>

#include "sensors/imu.h"
#include "sensors/proximity.h"

#define STANDARD_GRAVITY    9.80665f
#define DEG2RAD(deg) ((deg) / 180.0 * M_PI)
#define HALF_UINT16 16384

float get_computed_gyro(uint8_t axis){
	return DEG2RAD((float)get_gyro(axis) - (float)get_gyro_offset(axis));// - get_gyro_offset(axis);
}

float get_computed_acc(uint8_t axis){
	return (STANDARD_GRAVITY*(get_acc(axis) - get_acc_offset(axis))) / HALF_UINT16;
}


