#ifndef CLAUDE_H
#define CLAUDE_H

#include <stdint.h>

// Structure to hold CCR3 and CCR4 values for asymmetric PWM
typedef struct {
    uint32_t CCR3;
    uint32_t CCR4;
} TIM1_PhaseShift_t;

// Function declarations
// Note: tim1_phase_shift is static (internal to Claude.c only)
void TIM1PWMinit(uint32_t PSC, uint32_t ARR, uint32_t CCR, uint8_t DTencoded, uint8_t phase_deg_value, uint32_t CCR3, uint32_t CCR4);
void TIM1_Configure_TRGO(void);
void TIM2_Init_Phase_Delay(uint32_t phase_delay_ticks);
void DMA1_CH5_Init_TIM2_to_TIM15(void);
void TIM15_Init_Complementary_PWM(uint32_t arr, uint32_t duty_ticks, uint8_t deadtime);
void GPIO_Init_TIM15_Outputs(void);
void GPIO_Init_TIM1_Outputs(void);
void Calculate_Timing_Parameters(void);
void Init_Phase_Shifted_PWM_System(void);
void Update_Phase_Shift(float new_phase_deg);
void Update_PWM_Frequency(uint32_t new_freq_hz);
void Update_TIM15_Duty(float duty_percent);
void Debug_Enable_TIM2_Output(void);

#endif // CLAUDE_H
