#ifndef ESTIMATOR_H
#define ESTIMATOR_H

typedef struct
{
  float angle_y;
  float omega_y;
} estimator_t;

void estimator_start(void);
void get_angle_y();

#endif
