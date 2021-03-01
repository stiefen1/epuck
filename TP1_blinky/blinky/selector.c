/*
 * selector.c
 *
 *  Created on: 1 mars 2021
 *      Author: Steph
 */
#include <stm32f4xx.h>
#include <selector.h>
#include <main.h>

void SelectorConfiguration(void)
{
	/*
	 * Configure the 16pos selector
	 */

	// Input Mode
	GPIOD->MODER &= ~(3 << (2*4)); // Pins 8&9 du registre D mise à 0
	GPIOC->MODER &= ~(63 << (2*13)); // Pins 26 -> 31 du registre C mise à 0

	// Pull-down resistor on pin D4, C13, C14, C15
	GPIOD->PUPDR |= (2 << (2*4));
	GPIOD->PUPDR &= ~(1 << (2*4));
	GPIOC->PUPDR |= (42 << (2*13));
	GPIOC->PUPDR &= ~(21 << (2*13));
}

unsigned int SelectorReadPosition(void)
{
	/*
	 * Send back the position of the selector (number between 0 and 15)
	 */

	unsigned int s[4] = {0, 0, 0, 0};

	// Read value of s0 -> s3 (0 or 1)
	s[3] = ((GPIOC->IDR) & (1 << SEL3)) >> SEL3;
	s[2] = ((GPIOC->IDR) & (1 << SEL2)) >> SEL2;
	s[1] = ((GPIOC->IDR) & (1 << SEL1)) >> SEL1;
	s[0] = ((GPIOD->IDR) & (1 << SEL0)) >> SEL0;

	// Conversion bin -> dec
	return (s[3]+s[2]*2+s[1]*4+s[0]*8);
}

