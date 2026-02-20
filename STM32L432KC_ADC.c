

#include "STM32L432KC_ADC.h"
#include "STM32L432KC_GPIO.h"

/**
 * @brief Initialize ADC1 for continuous single-channel polling on PA0 (IN5)
 */
void ADC1_Init(void)
{    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
    gpioEnable(GPIO_PORT_A);
    pinMode(PA0, GPIO_ANALOG);
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0_Msk;

    /* ── 3. ADC common clock: synchronous HCLK/1 (= 80 MHz) ─────────── */
    ADC1_COMMON->CCR &= ~ADC_CCR_CKMODE_Msk;
    ADC1_COMMON->CCR |=  (1U << ADC_CCR_CKMODE_Pos);  // CKMODE = 01

    /* ── 4. Exit deep power-down ─────────────────────────────────────── */
    ADC1->CR &= ~ADC_CR_DEEPPWD;

    ADC1->CR |= ADC_CR_ADVREGEN;
    // 80 MHz → 12.5 ns/cycle; 20 us = 1600 cycles minimum; 2000 gives margin
    for (volatile uint32_t i = 0U; i < 2000U; i++);

    ADC1->CR &= ~ADC_CR_ADCALDIF;  // single-ended mode
    ADC1->CR |=  ADC_CR_ADCAL;     // start calibration
    while (ADC1->CR & ADC_CR_ADCAL);  // wait until hardware clears ADCAL

    ADC1->ISR |= ADC_ISR_ADRDY;    // clear stale ADRDY
    ADC1->CR  |= ADC_CR_ADEN;       // en
    while (!(ADC1->ISR & ADC_ISR_ADRDY));  // wait until ADC ready

    ADC1->SQR1 = 0U;                                      // L = 0 (1 conv)
    ADC1->SQR1 |= (ADC_CH_PA0 << ADC_SQR1_SQ1_Pos);      // SQ1 = IN5

    ADC1->SMPR1 &= ~(7U << (3U * ADC_CH_PA0)); // fiel stats at bit 3N
    ADC1->SMPR1 |=  (ADC_SMP_92_5 << (3U * ADC_CH_PA0)); //Sampling time for channel 5: 92.5 cycles

    ADC1->CFGR = ADC_CFGR_CONT     // continuous conversion mode
               | ADC_CFGR_OVRMOD;  // overwrite DR with newest result on overrun

    ADC1->CR |= ADC_CR_ADSTART;
}


uint16_t ADC1_Read(void)
{
    while (!(ADC1->ISR & ADC_ISR_EOC));
    return (uint16_t)(ADC1->DR & 0x0FFFU);
}


uint32_t ADC1_ToMillivolts(uint16_t raw)
{
    return ((uint32_t)raw * ADC_VREF_MV) / ADC_FULL_SCALE;
}
