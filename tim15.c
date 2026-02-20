/**
 * tim15.c
 * TIM15 configuration for complementary PWM with dead-time
 */

#include "stm32l4xx.h"
#include <stdint.h>
#include <stdio.h>
#include "tim15.h"
#include "calculations.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_RCC.h"

/**
 * @brief Initialize TIM15 for complementary PWM with dead-time
 *
 * Configuration:
 *   - Triggered by DMA write to EGR (UG bit)
 *   - Slave mode: Trigger mode (SMS=0110) - starts on TRGI rising edge
 *   - PWM mode 1 on CH1
 *   - Complementary outputs: CH1 and CH1N
 *   - Dead-time insertion
 *   - Same frequency as TIM1 (same ARR value)
 */
void TIM15_Init_Complementary_PWM(uint32_t arr, uint32_t duty_ticks, uint8_t deadtime)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;

    // Disable counter during configuration
    TIM15->CR1 &= ~TIM_CR1_CEN;

    // ===== SLAVE MODE CONFIGURATION =====
    // For DMA-triggered mode, we DISABLE slave mode and rely on manual UG trigger
    // SMS[3:0] = 0000 (Slave mode disabled)
    // DMA will write UG bit to EGR to manually reset and restart counter
    TIM15->SMCR &= ~(TIM_SMCR_TS_Msk | TIM_SMCR_SMS_Msk);
    // SMS = 0x00 (disabled) - counter is controlled by CEN and manual EGR writes

    // ===== TIMEBASE CONFIGURATION =====
    TIM15->PSC = 0;              // No prescaler (same as TIM1)
    TIM15->ARR = (2*ARR)-1;
    TIM15->RCR = 0;              // Repetition counter = 0

    // ===== CAPTURE/COMPARE CHANNEL 1 (PWM) =====
    TIM15->CCR1 = duty_ticks;    // 50% duty cycle

    // CC1S = 00 (output)
    // OC1M = 0110 (PWM mode 1: active when CNT < CCR1)
    // OC1PE = 1 (preload enable)
    TIM15->CCMR1 &= ~(TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1M_Msk);
    TIM15->CCMR1 |= (0x07 << TIM_CCMR1_OC1M_Pos);
    TIM15->CCMR1 |= TIM_CCMR1_OC1PE;

    // ===== OUTPUT CONFIGURATION =====
    // Enable CH1 and CH1N outputs
    // CC1E = 1 (CH1 enable)
    // CC1NE = 1 (CH1N enable)
    // CC1P = 0 (CH1 active high)
    // CC1NP = 0 (CH1N active high)
    TIM15->CCER = 0;
    TIM15->CCER |= TIM_CCER_CC1E;   // Enable CH1
    TIM15->CCER |= TIM_CCER_CC1NE;  // Enable CH1N

    // ===== BREAK AND DEAD-TIME REGISTER (BDTR) =====
    // MOE = 1 (Main Output Enable)
    // OSSR = 0 (Off-state selection for Run mode)
    // OSSI = 0 (Off-state selection for Idle mode)
    // DTG[7:0] = deadtime encoding
    TIM15->BDTR = 0;
    TIM15->BDTR |= TIM_BDTR_MOE;                    // Main output enable
    TIM15->BDTR |= (deadtime << TIM_BDTR_DTG_Pos);  // Dead-time

    // ===== CONTROL REGISTER =====
    TIM15->CR1 |= TIM_CR1_ARPE;  // Auto-reload preload enable
    // NOTE: OPM removed - TIM15 runs freely to avoid resetting mid-cycle

    // ===== GENERATE UPDATE EVENT =====
    TIM15->EGR = TIM_EGR_UG;

    // ===== ENABLE COUNTER =====
    // TIM15 runs freely - phase offset set once at initialization
    TIM15->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Configure GPIO pins for TIM15 outputs
 *
 * TIM15 pins on STM32L432KC:
 *   - TIM15_CH1:  PA2 (AF14)
 *   - TIM15_CH1N: PB15 (AF14)
 */
void GPIO_Init_TIM15_Outputs(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    pinMode(PA2, GPIO_ALT);     //TIM15_CH1 A7  //rn pink
    pinMode(PA1, GPIO_ALT);     //TIM15_CH1N A1

    GPIOA->AFR[0]  |=  (14U << GPIO_AFRL_AFSEL2_Pos);          // AF1 = TIM15_CH1
    GPIOA->AFR[0]  |=  (14U << GPIO_AFRL_AFSEL1_Pos);          // AF1 = TIM15_CH1N

    GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED1_Msk);
    GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED2_Msk);
}
