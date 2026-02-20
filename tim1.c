/**
 * tim1.c
 * TIM1 configuration for asymmetric center-aligned PWM
 */

#include "stm32l4xx.h"
#include <stdint.h>
#include <stdio.h>
#include "tim1.h"
#include "calculations.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_RCC.h"

/**
 * @brief Calculate phase shift CCR values for TIM1 (internal helper)
 */
static TIM1_PhaseShift_t tim1_phase_shift(uint32_t ARR, float phase_deg_value){
    TIM1_PhaseShift_t result;
    uint32_t halfwave = ARR + 1U;
    uint32_t period = 2 * halfwave;
    float phase_ticks_f = (phase_deg_value / 360.0f) * (float)period;
    uint32_t phase_ticks = (uint32_t)(phase_ticks_f + 0.5f); // round

    result.CCR3 = phase_ticks;
    result.CCR4 = ARR - phase_ticks;

    return result;
}

/**
 * @brief Initialize TIM1 for asymmetric center-aligned PWM
 */
void TIM1PWMinit(uint32_t PSC, uint32_t ARR, uint32_t CCR, uint8_t DTencoded, uint8_t phase_deg_value, uint32_t CCR3, uint32_t CCR4){
    TIM1->CR1 &= ~TIM_CR1_CEN;                    //disable for config
    TIM1->CCMR1 = 0;                             // clearing just for OC1PE later in case
    TIM1->CCMR2 = 0;                             // clearing just for OC1PE later in case

    TIM1->PSC = PSC;
    TIM1->ARR = ARR;
    printf("TIM1 ARR= %d \n", ARR);
    TIM1->CR1 |=  TIM_CR1_ARPE;                   // ARPE = 1 (ARR preload)

    TIM1->CR1 |= _VAL2FLD(TIM_CR1_CMS, 1);        // CMS = 01 (center-aligned)
    TIM1->CR1 &= ~TIM_CR1_CKD_Msk;        // ensure dead timer to same clock division as others

    TIM1->CCMR1 |= _VAL2FLD(TIM_CCMR1_CC1S, 0); // (output)
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE; // Output compare preload en
    TIM1->CCMR1 |= _VAL2FLD(TIM_CCMR1_OC1M, 0b110); // PWM mode 1
    TIM1->CCMR1 |= (1 << 16); // get that last top bit
    TIM1->CCMR1 |= _VAL2FLD(TIM_CCMR1_CC2S, 0); // (output)
    TIM1->CCMR1 |= TIM_CCMR1_OC2PE; // Output compare preload en
    TIM1->CCMR1 |= _VAL2FLD(TIM_CCMR1_OC2M, 0b110); // PWM mode 1
    TIM1->CCMR1 |= (1 << 24); // get that last top bit
    TIM1->CCR1 = 0; // was calculated above
    TIM1->CCR2 = ARR; // was calculated above

    TIM1->CCMR2 |= _VAL2FLD(TIM_CCMR2_CC3S, 0); // (output)
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE; // Output compare preload en
    TIM1->CCMR2 |= _VAL2FLD(TIM_CCMR2_OC3M, 0b110); // PWM mode 1
    TIM1->CCMR2 |= (1 << 16); // get that last top bit
    TIM1->CCMR2 |= _VAL2FLD(TIM_CCMR2_CC4S, 0); // (output)
    TIM1->CCMR2 |= TIM_CCMR2_OC4PE; // Output compare preload en
    TIM1->CCMR2 |= _VAL2FLD(TIM_CCMR2_OC4M, 0b110); // PWM mode 1
    TIM1->CCMR2 |= (1 << 24); // get that last top bit
    TIM1_PhaseShift_t phase = tim1_phase_shift(ARR, phase_deg_value);
    TIM1->CCR3 = phase.CCR3;
    TIM1->CCR4 = phase.CCR4;

    TIM1->CCER = 0; // start from a clean state
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE ); // Capture compare en for both channels on CH1
    TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE );// Capture compare en for both channels on CH2
    TIM1->EGR  |= TIM_EGR_UG;
    TIM1->EGR  &= ~TIM_EGR_UG;
    TIM1->CR2 &= ~TIM_CR2_MMS;
    TIM1->CR2 |= (2U << TIM_CR2_MMS_Pos); // MMS = 05: Trigger on Update Even
    TIM1->RCR = 1;
    TIM1->BDTR = 0;
    TIM1->BDTR |= (DTencoded << TIM_BDTR_DTG_Pos); // for dead time generator setup
    TIM1->EGR  |= TIM_EGR_TG;
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM1->BDTR  |= TIM_BDTR_MOE;
}

/**
 * @brief Configure TIM1 TRGO output (alternative configuration method)
 */
void TIM1_Configure_TRGO(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    // Configure TIM1 as master: TRGO = Update event
    // MMS[2:0] = 010 (Update event selected as trigger output)
    TIM1->CR2 &= ~TIM_CR2_MMS_Msk;
    TIM1->CR2 |= (0x02 << TIM_CR2_MMS_Pos);  // TRGO on update
    // NOTE: TIM1 continues to run in center-aligned mode with existing configuration
}

/**
 * @brief Initialize GPIO pins for TIM1 outputs
 */
void GPIO_Init_TIM1_Outputs(void)
{
    gpioEnable(GPIO_PORT_A);
    gpioEnable(GPIO_PORT_B);

    pinMode(PA8, GPIO_ALT);     //TIM1_CH1    rn blue
    pinMode(PA7, GPIO_ALT);      //TIM1_CH1N A6

    pinMode(PA10, GPIO_ALT);     //TIM1_CH3 D0
    pinMode(PB1, GPIO_ALT);     //TIM1_CH13N A1 D6

    GPIOA->AFR[1]  |=  (1U << GPIO_AFRH_AFSEL8_Pos);          // AF1 = TIM1_CH1
    GPIOA->AFR[0]  |=  (1U << GPIO_AFRL_AFSEL7_Pos);          // AF1 = TIM1_CH1N

    GPIOA->AFR[1]  |=  (1U << GPIO_AFRH_AFSEL10_Pos);          // AF1 = TIM1_CH3
    GPIOB->AFR[0]  |=  (1U << GPIO_AFRL_AFSEL1_Pos);          // AF1 = TIM1_CH3N

    GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED7_Msk);
    GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED8_Msk);
    GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED10_Msk);
    GPIOB->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED1_Msk);

    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
}
