/*
 * send_data.c
 *
 *  Created on: 21 avr. 2021
 *  Author: Julien
 *
 *  Description:
 *    Fonctions liés à l'envoi des données au script
 *    python.
 *
 */

#include <ch.h>
#include <hal.h>
#include <usbcfg.h>

#include "send_data.h"

// Fonction reprise tel quel du TP5
void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size)
{
	chSequentialStreamWrite(out, (uint8_t*)"START", 5);
	chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite(out, (uint8_t*)data, sizeof(float) * size);
}

// Fonction reprise tel quel du TP5
void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}
