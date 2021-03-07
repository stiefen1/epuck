#ifndef MOTOR_H
#define MOTOR_H

#define MOTOR_NB_STEPS 20 // Number of step/360� of the motor
#define REDUCTION_FACTOR 50 // Reduction factor of the reductor
#define WHEEL_PERIMETER 13 // in centimeters

void motor_init(void);
void motor_set_speed(float speed_r, float speed_l);
void motor_set_position(float position_r, float position_l, float speed_r, float speed_l);
void motor_stop(void);

#endif /* MOTOR_H */