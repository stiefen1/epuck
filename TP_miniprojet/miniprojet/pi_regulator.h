#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//start the PI regulator thread
void pi_regulator_start(void);
void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size);
uint16_t ReceiveInt16FromComputer(BaseSequentialStream* in, float* data, uint16_t size);

#endif /* PI_REGULATOR_H */
