/*
 * estimator.h
 *
 *  Created on: 21 avr. 2021
 *      Author: Julien
 */

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

typedef struct
{
  float angle_x;
} estimator_t;

void estimator_start(void);
float get_angle_x(void);

#endif /* ESTIMATOR_H */
