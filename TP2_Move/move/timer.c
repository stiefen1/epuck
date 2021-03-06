#include <stm32f4xx.h>
#include <gpio.h>
#include <main.h>

#define TIMER_CLOCK         84000000    // APB1 clock
//#define PRESCALER_TIM7      840        // timer frequency: 10kHz
//#define COUNTER_MAX_TIM7    100000       // timer max counter -> 1Hz

//#define PRESCALER_TIM6      8400        // timer frequency: 10kHz
//#define COUNTER_MAX_TIM6    100000       // timer max counter -> 1Hz

#define PRESCALER_TIM4		8400
#define COUNTER_MAX_TIM4	100
/*
void timer7_start(void)
{
    // Enable TIM7 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    // Enable TIM7 interrupt vector
    NVIC_EnableIRQ(TIM7_IRQn);

    // Configure TIM7
    TIM7->PSC = PRESCALER_TIM7 - 1;      // Note: final timer clock  = timer clock / (prescaler + 1)
    TIM7->ARR = COUNTER_MAX_TIM7 - 1;    // Note: timer reload takes 1 cycle, thus -1
    TIM7->DIER |= TIM_DIER_UIE;          // Enable update interrupt
    TIM7->CR1 |= TIM_CR1_CEN;            // Enable timer
}

void timer6_start(void)
{
    // Enable TIM6 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    // Enable TIM6 interrupt vector
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    // Configure TIM6
    TIM6->PSC = PRESCALER_TIM6 - 1;      // Note: final timer clock  = timer clock / (prescaler + 1)
    TIM6->ARR = COUNTER_MAX_TIM6 - 1;    // Note: timer reload takes 1 cycle, thus -1
    TIM6->DIER |= TIM_DIER_UIE;          // Enable update interrupt
    TIM6->CR1 |= TIM_CR1_CEN;            // Enable timer
}

*/
void timer4_start(void)
{
    // Enable TIM4 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // Configure TIM7
    TIM4->PSC = PRESCALER_TIM4 - 1;      // Note: final timer clock  = timer clock / (prescaler + 1)
    TIM4->ARR = COUNTER_MAX_TIM4 - 1;    // Note: timer reload takes 1 cycle, thus -1

    TIM4->CR1 |= TIM_CR1_CEN;            // Enable timer

    //TIM4->CCMR2
	TIM4->CCR3 = 90;
	TIM4->CCER |= (1 | (1 << 8));
	TIM4->CCMR2 = ((TIM4->CCMR2 & ~(7 << 4)) | (6 << 4)); // PWM Mode 1

}

void set_PWM_Frequency(unsigned int ratio)
{
	TIM4->CCR3 = ratio;
}
/*
*   Commented because used for the motors
*/

// // Timer 7 Interrupt Service Routine
/*void TIM7_IRQHandler(void)
{
	/*
	*
	*   BEWARE !!
	*   Based on STM32F40x and STM32F41x Errata sheet - 2.1.13 Delay after an RCC peripheral clock enabling
	*
	*   As there can be a delay between the instruction of clearing of the IF (Interrupt Flag) of corresponding register (named here CR) and
	*   the effective peripheral IF clearing bit there is a risk to enter again in the interrupt if the clearing is done at the end of ISR.
	*
	*   As tested, only the workaround 3 is working well, then read back of CR must be done before leaving the ISR
	*


//     /* do something ...
//     gpio_toggle(BODY_LED);

//     // Clear interrupt flag
     TIM7->SR &= ~TIM_SR_UIF;
     TIM7->SR;	// Read back in order to ensure the effective IF clearing
}

// Timer 6 Interrupt Service Routine
void TIM6_IRQHandler(void)
{
    // Clear interrupt flag
    TIM6->SR &= ~TIM_SR_UIF;
    TIM6->SR;	// Read back in order to ensure the effective IF clearing

}
*/
