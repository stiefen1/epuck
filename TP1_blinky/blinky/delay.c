/*
 * delay.c
 *
 *
 *  Created on: 25 f�vr. 2021
 *      Author: Steph
 */

void mydelay(unsigned int us)
{
	unsigned int i = 0;
	for(;i<us;i++)
	{
		asm("nop");
	}
}
