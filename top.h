#ifndef TOP_H
#define TOP_H

#include <stdint.h>
#include "calculations.h"
#include "tim1.h"
#include "tim2.h"
#include "tim15.h"

// DMA transfer value for triggering TIM15
extern const uint16_t trigger_value;

// Function declarations (DMA and main system functions)
void DMA1_CH5_Init_TIM2_to_TIM15(void);
void Init_Phase_Shifted_PWM_System(void);
void Update_Phase_Shift(float new_phase_deg);
void Update_PWM_Frequency(uint32_t new_freq_hz);
void Update_TIM15_Duty(float duty_percent);

// Verification and debug functions
typedef struct {
    uint32_t tim1_cr2_mms;      
    uint32_t tim2_smcr_ts;      
    uint32_t tim2_smcr_sms;    
    uint32_t tim2_cr1_opm;      
    uint32_t tim2_cr2_mms;     
    uint32_t tim2_ccr1;        
    uint32_t dma_cselr_c5s;    
    uint32_t dma_ch5_enabled;  
    uint32_t tim15_smcr_sms;    
    uint32_t tim15_bdtr_moe;   
    uint32_t tim15_arr;         
    uint32_t tim15_ccr1;        
} Verification_Registers_t;

Verification_Registers_t Read_Verification_Registers(void);
void Verify_Configuration(void);
void Example_Usage(void);

#endif 
