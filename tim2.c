/**
 * tim2.c
 * TIM2 configuration for phase delay generation
 */

#include "stm32l4xx.h"
#include <stdint.h>
#include <stdio.h>
#include "tim2.h"
#include "calculations.h"
#include "STM32L432KC_RCC.h"

/**
 * @brief Initialize TIM2 for phase delay generation
 *
 * TIM2 operates in trigger mode, slaved to TIM1 TRGO
 * Generates DMA request when CCR1 matches (phase delay point)
 *
 * @param phase_delay_ticks Delay in timer ticks before triggering TIM15
 */
void TIM2_Init_Phase_Delay(uint32_t phase_delay_ticks)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    printf("phase delay ticks = %d \n", phase_delay_ticks);

    // ===== SLAVE MODE CONFIGURATION =====
    // TS[2:0] = 000 (ITR0 = TIM1 TRGO)
    // SMS[3:0] = 0100 (Reset Mode: TRGI resets counter to 0)
    // Counter runs continuously (CEN=1), resets on each TIM1 trigger
    TIM2->SMCR &= ~(TIM_SMCR_TS_Msk | TIM_SMCR_SMS_Msk);
    TIM2->SMCR |= (0x00 << TIM_SMCR_TS_Pos);   // ITR0 = TIM1
    TIM2->SMCR |= (0x04 << TIM_SMCR_SMS_Pos);  // Reset mode
    TIM2->PSC = 0;
    TIM2->ARR = (2*ARR)-1;  // Full period to allow phase shifts up to 360°
    printf("TIM2 ARR= %d \n", ((2 *ARR)-1));

    TIM2->CCR1 = phase_delay_ticks;

    // CC1S = 00 (output mode)
    // OC1M = 0111 (PWM mode 2: OC1REF high when CNT >= CCR1)
    TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1M_Msk);
    TIM2->CCMR1 |= (0x07 << TIM_CCMR1_OC1M_Pos);

    TIM2->CCMR1 |= TIM_CCMR1_OC1PE; // Enable output compare 1 preload
    TIM2->CCER |= TIM_CCER_CC1E;

    // Enable CC1 DMA request (triggers DMA1_CH5 when CCR1 matches)
    TIM2->DIER |= TIM_DIER_CC1DE;

    // Enable auto-reload preload
    TIM2->CR1 |= TIM_CR1_ARPE;

    // Generate update event to load preload registers
    TIM2->EGR = TIM_EGR_UG;

    // Enable counter - it will reset and restart on each TIM1 trigger
    TIM2->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Optional: Configure TIM2_CH1 output for debugging
 * This allows you to see the phase delay timer output on scope
 *
 * TIM2_CH1 can be mapped to PA0, PA5, or PA15 (check datasheet)
 * Example: PA0 = AF1
 */
void Debug_Enable_TIM2_Output(void)
{
    // Enable GPIOA clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // Configure PA0 as TIM2_CH1 (AF1)
    GPIOA->MODER &= ~GPIO_MODER_MODE0_Msk;
    GPIOA->MODER |= (0x02 << GPIO_MODER_MODE0_Pos);  // AF mode
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT0;               // Push-pull
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED0;          // High speed
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL0_Msk;
    GPIOA->AFR[0] |= (1 << GPIO_AFRL_AFSEL0_Pos);    // AF1

    // TIM2 CH1 is already configured as output compare in TIM2_Init_Phase_Delay
    // This will output a pulse when CCR1 matches (at phase offset point)
}
