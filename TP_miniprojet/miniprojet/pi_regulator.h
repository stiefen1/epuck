#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

#define NB_SAMPLES 1024

typedef struct {
  float kp;
  float kd;
  float ki;
} RegParam;

//start the PI regulator thread
void pi_regulator_start(RegParam* reg_param);


#endif /* PI_REGULATOR_H */
