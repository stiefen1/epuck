#include <stm32f4xx.h>
#include <system_clock_config.h>
#include <gpio.h>
#include <main.h>
#include <delay.h>
#include <timer.h>
#include <selector.h>


void blink_circle(unsigned int blink_time, unsigned int count);

// Init function required by __libc_init_array
void _init(void) {}

int main(void)
{
	unsigned int blink_time = 0, count = 0, selector_position = 0;

    SystemClock_Config();

    // Enable GPIOD & GPIOB & GPIOC peripheral clock
    RCC->AHB1ENR    |= (RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);

    // LED used init
    gpio_config_output_opendrain(LED1);
    gpio_config_output_opendrain(LED3);
    gpio_config_output_opendrain(LED5);
    gpio_config_output_opendrain(LED7);

    // Configure the 4 input pins of the selector
	SelectorConfiguration();
	timer7_start();

    while (1) {
    	/*selector_position = SelectorReadPosition();
		blink_time = (1+selector_position)*250000;

    	blink_circle(blink_time, count);
    	count++;*/

    }
}


void blink_circle(unsigned int blink_time, unsigned int count)
{
	/*
	 * Toggle LEDs 1-3-5-7 with circular pattern
	 * Each LEDs lights up then goes out one after the other
	 */

	if(count>0)
		gpio_toggle(LED7);

	gpio_toggle(LED1);
	mydelay(blink_time);

	gpio_toggle(LED1);
	gpio_toggle(LED3);
	mydelay(blink_time);

	gpio_toggle(LED3);
	gpio_toggle(LED5);
	mydelay(blink_time);

	gpio_toggle(LED5);
	gpio_toggle(LED7);
	mydelay(blink_time);

}

