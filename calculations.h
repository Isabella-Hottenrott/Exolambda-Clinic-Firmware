#ifndef CALCULATIONS_H
#define CALCULATIONS_H

#include <stdint.h>

// Structure to hold CCR3 and CCR4 values for asymmetric PWM
typedef struct {
    uint32_t CCR3;
    uint32_t CCR4;
} TIM1_PhaseShift_t;

// Configuration parameters (shared across modules)
#define F_TIM_CLK           80000000UL  // 80 MHz timer clock
#define F_PWM_HZ            100000UL     // 100 kHz PWM frequency
#define DEADTIME_NS_CFG     100         // 100 ns dead-time
#define PHASE_DEG_B_CFG     0.0f       // Phase shift for Pair B in degrees [0..360)
#define PHASE2_DEG_CFG      0.0f       // Phase shift for Pair C in degrees [0..360)

// Global timing variables (defined in calculations.c)
extern uint32_t ARR;              // Auto-reload value for TIM1/TIM15
extern uint32_t phase_ticks;      // Phase delay in timer ticks
extern uint8_t DTencoded;         // Encoded dead-time value

// Function declarations
void Calculate_Timing_Parameters(void);

#endif // CALCULATIONS_H
