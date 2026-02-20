#ifndef STM32L432KC_ADC_H
#define STM32L432KC_ADC_H

#include <stdint.h>
#include <stm32l432xx.h>

///////////////////////////////////////////////////////////////////////////////
// Configuration
///////////////////////////////////////////////////////////////////////////////

// ADC1 channel number for PA0 (ADC1_IN5)
#define ADC_CH_PA0          5U

// Sampling time encoding (written into SMPRx, 3-bit field)
#define ADC_SMP_2_5         0U
#define ADC_SMP_6_5         1U
#define ADC_SMP_12_5        2U
#define ADC_SMP_24_5        3U
#define ADC_SMP_47_5        4U
#define ADC_SMP_92_5        5U   // ~1.3 us at 80 MHz -- good for moderate source Z
#define ADC_SMP_247_5       6U
#define ADC_SMP_640_5       7U

// Voltage reference (mV) and 12-bit full-scale count
#define ADC_VREF_MV         3300U
#define ADC_FULL_SCALE      4095U

///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

void     ADC1_Init(void);
uint16_t ADC1_Read(void);
uint32_t ADC1_ToMillivolts(uint16_t raw);

#endif // STM32L432KC_ADC_H
