#ifndef TIM1_H
#define TIM1_H

#include <stdint.h>

// Function declarations
void TIM1PWMinit(uint32_t PSC, uint32_t ARR, uint32_t CCR, uint8_t DTencoded, uint8_t phase_deg_value, uint32_t CCR3, uint32_t CCR4);
void TIM1_Configure_TRGO(void);
void GPIO_Init_TIM1_Outputs(void);

#endif // TIM1_H
