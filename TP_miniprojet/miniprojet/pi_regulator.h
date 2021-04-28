#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

#define NB_SAMPLES 1024

typedef struct {
  float kp;
  float kd;
  float ki;
} reg_param_t;

//start the PI regulator thread
void pi_regulator_start(reg_param_t* reg_param);


#endif /* PI_REGULATOR_H */
