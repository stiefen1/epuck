#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#define RES_250DPS 250.0f
#define MAX_INT16 32768.0f
#define GYRO_RAW2DPS (RES_250DPS / MAX_INT16) //250DPS (degrees per second) scale for 32768

typedef struct
{
  float angle_x;
  float omega_x;
  float dt;
} estimator_t;

void estimator_start(void);
float get_angle_x();

#endif
