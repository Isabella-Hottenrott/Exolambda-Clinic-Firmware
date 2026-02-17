#ifndef TIM15_H
#define TIM15_H

#include <stdint.h>

// Function declarations
void TIM15_Init_Complementary_PWM(uint32_t arr, uint32_t duty_ticks, uint8_t deadtime);
void GPIO_Init_TIM15_Outputs(void);

#endif // TIM15_H
