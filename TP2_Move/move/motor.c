#include <stdlib.h>
#include <stdint.h>
#include <stm32f4xx.h>
#include <gpio.h>
#include <motor.h>
#include <math.h>

#define PI                  3.1415926536f

#define TIMER_CLOCK         84000000
#define TIMER_FREQ          100000 // [Hz]
#define MOTOR_SPEED_LIMIT   13 // [cm/s]
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define NSTEP_ONE_EL_TURN   4  //number of steps to do 1 electrical turn
#define NB_OF_PHASES        4  //number of phases of the motors
#define WHEEL_PERIMETER     13.f // [cm]
#define REDUCTION_FACTOR	50.f // Réducteur, facteur 50:1
#define WHEELS_DISTANCE		5.3f // Ecart entre les roues en cm

//timers to use for the motors
#define MOTOR_RIGHT_TIMER       TIM6
#define MOTOR_RIGHT_TIMER_EN    RCC_APB1ENR_TIM6EN
#define MOTOR_RIGHT_IRQHandler  TIM6_DAC_IRQHandler
#define MOTOR_RIGHT_IRQ         TIM6_DAC_IRQn

#define MOTOR_LEFT_TIMER        TIM7
#define MOTOR_LEFT_TIMER_EN     RCC_APB1ENR_TIM7EN
#define MOTOR_LEFT_IRQ          TIM7_IRQn
#define MOTOR_LEFT_IRQHandler   TIM7_IRQHandler

#define PRESCALER			840 // Prescaler to obtain a timer frequency of 10 kHz

#define PRESCALER_TIM7      PRESCALER // timer frequency: 10kHz left motor
#define COUNTER_MAX_TIM7    100       // timer max counter -> 100Hz

#define PRESCALER_TIM6      PRESCALER // timer frequency: 10kHz right motor
#define COUNTER_MAX_TIM6    100       // timer max counter -> 100Hz

// Motor 2
#define MOTOR_RIGHT_A	GPIOE, 13
#define MOTOR_RIGHT_B	GPIOE, 12
#define MOTOR_RIGHT_C	GPIOE, 14
#define MOTOR_RIGHT_D	GPIOE, 15

// Motor 1
#define MOTOR_LEFT_A	GPIOE, 9
#define MOTOR_LEFT_B	GPIOE, 8
#define MOTOR_LEFT_C	GPIOE, 11
#define MOTOR_LEFT_D	GPIOE, 10

// Facteur constant utilisé pour calculer counter_max à partir de la vitesse
const float speed_factor = WHEEL_PERIMETER*TIMER_CLOCK/(PRESCALER*20.0*REDUCTION_FACTOR);

static const uint8_t step_halt[NB_OF_PHASES] = {0, 0, 0, 0};
static const uint8_t step_table[NSTEP_ONE_EL_TURN][NB_OF_PHASES] = {
    {1, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 1},
    {1, 0, 0, 1},
};

// Nb de pas à réaliser (consigne)
int step_goal_right = 0;
int step_goal_left = 0;

// Nb de pas effectué
unsigned int step_count_right = 0;
unsigned int step_count_left = 0;


void motor_init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; // Enable GPIOE Clock to allow GPIOE use

    // Enable TIM6 clock
    RCC->APB1ENR |= MOTOR_RIGHT_TIMER_EN;

    // Enable TIM6 interrupt vector
    NVIC_EnableIRQ(MOTOR_RIGHT_IRQ);

    // Configure TIM6
    TIM6->PSC = PRESCALER_TIM6 - 1;      // Note: final timer clock  = timer clock / (prescaler + 1)
    TIM6->ARR = COUNTER_MAX_TIM6 - 1;    // Note: timer reload takes 1 cycle, thus -1
    TIM6->DIER |= TIM_DIER_UIE;          // Enable update interrupt
    TIM6->CR1 |= TIM_CR1_CEN;            // Enable timer

    // Enable TIM7 clock
    RCC->APB1ENR |= MOTOR_LEFT_TIMER_EN;

    // Enable TIM7 interrupt vector
    NVIC_EnableIRQ(MOTOR_LEFT_IRQ);

    // Configure TIM7
    TIM7->PSC = PRESCALER_TIM7 - 1;      // Note: final timer clock  = timer clock / (prescaler + 1)
    TIM7->ARR = COUNTER_MAX_TIM7 - 1;    // Note: timer reload takes 1 cycle, thus -1
    TIM7->DIER |= TIM_DIER_UIE;          // Enable update interrupt
    TIM7->CR1 |= TIM_CR1_CEN;            // Enable timer

    // Set Output mode for pin 8 -> 15 of GPIOE (Motor)
	GPIOE->MODER = ((GPIOE->MODER & ~(65535 << 16)) | (21845 << 16));

	// Set every control pins of driver motor to 0 as init
	GPIOE->ODR &= ~(255 << 8); // force 0 to pin8 -> 15

	// At this point, Timer 6&7 have been initialized with interrupt
	// Interrupt from each timer has been linked to the corresponding ISR

}


static void right_motor_update(const uint8_t *out)
{
	// Controle des bobines (right) : met en sortie les valeurs indiquées par la machine d'état
	GPIOE->ODR = (GPIOE->ODR & ~(1 << 13)) | (out[0] << 13); // A1
	GPIOE->ODR = (GPIOE->ODR & ~(1 << 12)) | (out[1] << 12); // B1
	GPIOE->ODR = (GPIOE->ODR & ~(1 << 14)) | (out[2] << 14); // C1
	GPIOE->ODR = (GPIOE->ODR & ~(1 << 15)) | (out[3] << 15); // D1
}

/*
*
*   TO COMPLETE
*
*   Updates the state of the gpios of the left motor given an array of 4 elements
*   describing the state. For exeample step_table[0] which gives the first step.
*/
static void left_motor_update(const uint8_t *out)
{
	// Contrôle des bobines (left) : mets en sortie les valeurs indiquées par la machine d'état
	GPIOE->ODR = (GPIOE->ODR & ~(1 << 9)) | (out[0] << 9); // A1
	GPIOE->ODR = (GPIOE->ODR & ~(1 << 8)) | (out[1] << 8); // B1
	GPIOE->ODR = (GPIOE->ODR & ~(1 << 11)) | (out[2] << 11); // C1
	GPIOE->ODR = (GPIOE->ODR & ~(1 << 10)) | (out[3] << 10); // D1
}

/*
*
*   TO COMPLETE
*
*   Stops the motors (all the gpio must be clear to 0) and set 0 to the ARR register of the timers to prevent
*   the interrupts of the timers (because it never reaches 0 after an increment)
*/
void motor_stop(void)
{
	// IL FAUT ENCORE SET à 0 LE REGISTRE ARR POUR EVITER LES INTERRUPTIONS
	// IL FAUDRA LE REACTIVER QUAND ON VOUDRA SE DEPLACER A NOUVEAU

	// Arrête les deux moteurs
	right_motor_update(step_halt);
	left_motor_update(step_halt);

	// Remise à zéro des goals & compteurs
	step_goal_right = 0;
	step_goal_left = 0;
	step_count_right = 0;
	step_count_left = 0;
}


void motor_set_position(float position_r, float position_l, float speed_r, float speed_l)
{
	// POSITION SIGN SET THE SENS OF ROTATION

	motor_set_speed(speed_r, speed_l);

	step_goal_right = round(position_r*NSTEP_ONE_TURN/WHEEL_PERIMETER); // Position in cm
	step_goal_left = round(position_l*NSTEP_ONE_TURN/WHEEL_PERIMETER); // Position in cm
}


void motor_set_speed(float speed_r, float speed_l)
{
	if(speed_r>MOTOR_SPEED_LIMIT)
		speed_r = MOTOR_SPEED_LIMIT;

	if(speed_l>MOTOR_SPEED_LIMIT)
		speed_l = MOTOR_SPEED_LIMIT;

	TIM6->ARR = round(speed_factor/speed_r) - 1; // Speed in cm/s
	TIM7->ARR = round(speed_factor/speed_l) -1; // Speed in cm/s
}

void motor_curve(float speed, float radius, float angle)
{
	float speed_l, speed_r, distance_r, distance_l, distance;

	distance = angle * PI * radius / 180;

	if(radius != 0) // Evite la division par 0
	{
		speed_l = abs((radius-(WHEELS_DISTANCE/2.0))*speed/radius);
		speed_r = abs((radius+(WHEELS_DISTANCE/2.0))*speed/radius);
		distance_l = (radius-(WHEELS_DISTANCE/2.0))*distance/radius;
		distance_r = (radius+(WHEELS_DISTANCE/2.0))*distance/radius;

		motor_set_position(distance_r, distance_l, speed_r, speed_l);
	}
}

void MOTOR_RIGHT_IRQHandler(void)
{
	step_count_right++; // Compte le nb de pas réalisés

	if(step_count_right < abs(step_goal_right)) // Test si le nb de pas (goal) a été atteint
	{
		if(step_goal_right>0) // Si le goal est + la roue tourne dans le sens positif
			right_motor_update(step_table[3-(step_count_right % 4)]);

		else // Si le goal est négatif la roue tourne dans le sens négatif
			right_motor_update(step_table[(step_count_right % 4)]);
	}

	else // Si le goal a été atteint, le moteur s'arrête
		right_motor_update(step_halt);

	// Clear interrupt flag
	MOTOR_RIGHT_TIMER->SR &= ~TIM_SR_UIF;
	MOTOR_RIGHT_TIMER->SR;	// Read back in order to ensure the effective IF clearing
}

void MOTOR_LEFT_IRQHandler(void)
{
	step_count_left++; // Compte le nb de pas réalisés

	if(step_count_left < abs(step_goal_left)) // Test si le nb de pas (goal) a été atteint
	{
		if(step_goal_left>0) // Si le goal est + la roue tourne dans le sens positif
			left_motor_update(step_table[((step_count_left) % 4)]);

		else // Si le goal est négatif la roue tourne dans le sens négatif
			left_motor_update(step_table[3-(step_count_left % 4)]);
	}

	else // Si le goal a été atteint, le moteur s'arrête
		left_motor_update(step_halt);

	// Clear interrupt flag
    MOTOR_LEFT_TIMER->SR &= ~TIM_SR_UIF;
    MOTOR_LEFT_TIMER->SR;	// Read back in order to ensure the effective IF clearing
}

void motor_turn(float angle, float speed)
{
  float distance = WHEELS_DISTANCE*PI*angle/360.f;

  if(speed == 0.f) {
    // (Julien) vitesse nulle, on devrait afficher
    // une erreur, mais ici je me contente de 
    // rien faire
    return;
  }

  motor_set_position(distance, -distance, speed, speed);
}
