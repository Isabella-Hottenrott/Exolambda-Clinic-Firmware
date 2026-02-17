/**
 * calculations.c
 * Timing calculations and parameter generation for phase-shifted PWM system
 */

#include "stm32l4xx.h"
#include <stdint.h>
#include "calculations.h"

// Global timing variables
uint32_t ARR;              // Auto-reload value for TIM1/TIM15
uint32_t phase_ticks;      // Phase delay in timer ticks
uint8_t DTencoded;         // Encoded dead-time value

/**
 * @brief Calculate phase shift CCR values for TIM1
 * @param ARR Auto-reload value
 * @param phase_deg_value Phase shift in degrees
 * @return TIM1_PhaseShift_t structure with CCR3 and CCR4 values
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
 * @brief Generate dead-time encoding from nanoseconds
 * @param dead_us Dead-time in nanoseconds (despite parameter name)
 * @param tim_freq Timer frequency in Hz
 * @return 8-bit encoded dead-time value
 */
static uint8_t dead_time_generator(float dead_us, uint32_t tim_freq){
    const double  t_dts = 1.0 / (double)tim_freq;
    uint32_t ticks = (uint32_t)((dead_us * 1e-9f) / t_dts);
    if (ticks <= 127)                   return (uint8_t)ticks;
    if (ticks   <= (127*2))             return (uint8_t)(0x80 | ((ticks/2)  - 64));
    if (ticks   <= (504))               return (uint8_t)(0xC0 | ((ticks/8)  - 32));
    if (ticks/16  <= (1008))            return (uint8_t)(0xE0 | ((ticks/16) - 32));
    return 0xFF; // clamp otherwise
}

/**
 * @brief Calculate timing parameters from PWM frequency
 * Uses same formula as user's tim_compute_edge() for consistency
 */
void Calculate_Timing_Parameters(void)
{
    // Match user's tim_compute_edge() formula (no -1, consistent with their TIM1 setup)
    uint32_t PSC = 0;  // Prescaler = 1 (same as user's code)
    ARR = (F_TIM_CLK / (2 * (PSC + 1U) * F_PWM_HZ));

    // Calculate phase ticks
    uint32_t period_ticks = 2 * ARR;
    float phase_with_offset = PHASE2_DEG_CFG + 180.0f;   

    // Wrap around if > 360                                
    if (phase_with_offset >= 360.0f) {                  
        phase_with_offset -= 360.0f;                      
    }

    float phase_ticks_f = (phase_with_offset / 360.0f) * (float)period_ticks;  
    phase_ticks = (uint32_t)(phase_ticks_f + 0.5f);  // Round to nearest
    DTencoded = dead_time_generator(DEADTIME_NS_CFG, F_TIM_CLK);
}
