/*
 * receive_data.h
 *
 *  Created on: 21 avr. 2021
 *      Author: Julien
 */

#ifndef RECEIVE_DATA_H
#define RECEIVE_DATA_H

uint16_t ReceiveFloatFromComputer(BaseSequentialStream* in, float* data, uint16_t size);

#endif /* RECEIVE_DATA_H */
