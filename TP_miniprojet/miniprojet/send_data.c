#include <ch.h>
#include <hal.h>
#include <usbcfg.h>

#include "send_data.h"

void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size)
{
	chSequentialStreamWrite(out, (uint8_t*)"START", 5);
	chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite(out, (uint8_t*)data, sizeof(float) * size);
}

void SendFloatToComputerFast(BaseSequentialStream* out, float data)
{
	chSequentialStreamWrite(out, (uint8_t*)&data, sizeof(float));
}


