#include <stm32f4xx.h>
#include <system_clock_config.h>
#include <gpio.h>
#include <main.h>
#include <timer.h>
#include <motor.h>
#include <selector.h>
#include <stdio.h>
#include <stdlib.h>

#define PI                  3.1415926536f
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

// Init function required by __libc_init_array
void _init(void) {}

// Simple delay function
void delay(unsigned int n)
{
    while (n--) {
        __asm__ volatile ("nop");
    }
}


int main(void)
{
    delay(10000000);
    SystemClock_Config();

    // Enable GPIOD and GPIOE peripheral clock
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN;

    int sylvain = 0;

    init_selector();
    gpio_config_output_af_pushpull(FRONT_LED);
    timer4_start();

    motor_init();

    moveL(50, -50, 10);

    while (1) {
        //if(isFinished())
        	//set_PWM_Frequency(6*get_selector());
/*
    	if(isFinished())
    	{
    		switch(sylvain)
    		{
    			case 0:
    				motor_forward(20, 13);
    			break;

    			case 1:
    				motor_turn(270, 13);
    			break;

    			case 2:
    				motor_forward(-20,13);
    			break;

    			case 3:
    				motor_turn(-90, 13);
    			break;

    			case 4:
    				motor_forward(10, 13);
    			break;

    			case 5:
    				motor_curve(10,50,90);
    			break;

    			case 6:
    				motor_forward(10, 13);
    			break;

    			case 7:
    				motor_turn(90, 13);
    			break;

    			default:
    				sylvain = -1;
    			break;
    		}

    		sylvain++;
    	}*/
    }
}

