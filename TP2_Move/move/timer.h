#ifndef TIMER_H
#define TIMER_H

void timer7_start(void);
void timer4_start(void);
//void TIM6_IRQHandler(void);
//void TIM7_IRQHandler(void);
void set_PWM_Frequency(unsigned int ratio);

#endif /* TIMER_H */
