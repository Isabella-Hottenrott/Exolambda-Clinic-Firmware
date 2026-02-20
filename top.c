#include "stm32l4xx.h"
#include <stdint.h>
#include "top.h"
#include "calculations.h"
#include "tim1.h"
#include "tim2.h"
#include "tim15.h"
#include "main.h"
#include "STM32L432KC_RCC.h"

// DMA transfer value for triggering TIM15
const uint16_t trigger_value = TIM_EGR_UG;


void DMA1_CH5_Init_TIM2_to_TIM15(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;

    DMA1_CSELR->CSELR &= ~DMA_CSELR_C5S_Msk;
    DMA1_CSELR->CSELR |= (0x04 << DMA_CSELR_C5S_Pos);

    DMA1_Channel5->CCR = 0;
    DMA1_Channel5->CCR |= DMA_CCR_DIR;         // Read from memory
    DMA1_Channel5->CCR |= DMA_CCR_CIRC;        // Circular mode
    DMA1_Channel5->CCR |= DMA_CCR_MSIZE_0;     // Memory size: 16-bit
    DMA1_Channel5->CCR |= DMA_CCR_PSIZE_0;     // Peripheral size: 16-bit

    DMA1_Channel5->CMAR = (uint32_t)&trigger_value;  // Source address
    DMA1_Channel5->CPAR = (uint32_t)&TIM15->EGR;     // Destination address

    DMA1_Channel5->CNDTR = 1;  // One transfer per trigger
    DMA1_Channel5->CCR |= DMA_CCR_EN;
}


void Init_Phase_Shifted_PWM_System(void)
{
    Calculate_Timing_Parameters();
    GPIO_Init_TIM1_Outputs();   // TIM1 pins
    GPIO_Init_TIM15_Outputs();  // TIM15 pins

    uint32_t duty_ticks = ((2*ARR)-1) / 2;  // 50% duty cycle
    TIM15_Init_Complementary_PWM(ARR, duty_ticks, DTencoded);
    DMA1_CH5_Init_TIM2_to_TIM15();
    TIM2_Init_Phase_Delay(phase_ticks);

    uint32_t PSC = 0;                 // No prescaler
    uint32_t CCR = ARR / 2;           // 50% duty for Pair A

    TIM1_PhaseShift_t phase_b;
    uint32_t halfwave = ARR + 1U;
    uint32_t period = 2 * halfwave;
    float phase_ticks_f = (PHASE_DEG_B_CFG / 360.0f) * (float)period;
    uint32_t pb_ticks = (uint32_t)(phase_ticks_f + 0.5f);
    phase_b.CCR3 = pb_ticks;
    phase_b.CCR4 = ARR - pb_ticks;

    TIM1PWMinit(PSC, ARR, CCR, DTencoded, PHASE_DEG_B_CFG, phase_b.CCR3, phase_b.CCR4);

}

void Update_PrimTwo_Phase(float new_phase_deg)
{
    uint32_t period_ticks = 2 * ARR;
    float phase_with_offset = new_phase_deg + 180.0f;
    if (phase_with_offset >= 360.0f)
        phase_with_offset -= 360.0f;

    float phase_ticks_f = (phase_with_offset / 360.0f) * (float)period_ticks;
    uint32_t new_ticks = (uint32_t)(phase_ticks_f + 0.5f);

    TIM2->CCR1 = new_ticks;
}

void Update_Secondary_Shift(float new_phase_deg)
{
    uint32_t halfwave = ARR + 1U;
    uint32_t period = 2 * halfwave;
    float phase_ticks_f = (new_phase_deg / 360.0f) * (float)period;
    uint32_t pb_ticks = (uint32_t)(phase_ticks_f + 0.5f);

    TIM1->CCR3 = pb_ticks;
    TIM1->CCR4 = ARR - pb_ticks;
}
