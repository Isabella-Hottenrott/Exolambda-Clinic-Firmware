

// main.c
// Exolamba Clinic
// email
// data
//
// This is the main c code for the phase modulation for the dual active bridge
// Below is the Arduino code in the process to be configured to STM32L432KC MCU

#include "main.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_RCC.h"
#include "STM32L432KC_SPI.h"
#include "STM32L432KC.h"
#include "STM32L432KC_TIM.h"

int F_TIM_HZ = 80000000;
int F_PWM_HZ = 100000;
int DT_us = 100;
int phase_deg = 90;
int PHASE2_DEG = 30;


static uint8_t dead_time_generator(float dead_us, uint32_t tim_freq){
    const double  t_dts = 1.0 / (double)tim_freq;
    uint32_t ticks = (uint32_t)((dead_us * 1e-9f) / t_dts);
    if (ticks <= 127)                   return (uint8_t)ticks; // 0 to 127 because 127 is biggest 6 bit number
    if (ticks   <= (127*2))             return (uint8_t)(0x80 | ((ticks/2)  - 64)); // for ticks = (64+k)*2 where k = {0,63} since 63 is biggest 6 bit number
    if (ticks   <= (504))              return (uint8_t)(0xC0 | ((ticks/8)  - 32));     // for ticks = (32+k)*8 where k = {0,32} since 32 is biggest 5 bit number
    if (ticks/16  <= (1008))              return (uint8_t)(0xE0 | ((ticks/16) - 32));     // for ticks = (32+k)*16 where k = {0,32} since 32 is biggest 5 bit number
    return 0xFF; // clamp otherwise
}



void TIM1GPIOinit(void){

gpioEnable(GPIO_PORT_A);
gpioEnable(GPIO_PORT_B);

//GPIO channels for TIM1
pinMode(PA8, GPIO_ALT);   //TIM1_CH1 D9
pinMode(PA7, GPIO_ALT);   //TIM1_CH1N A6
pinMode(PA10, GPIO_ALT);     //TIM1_CH3 D0
pinMode(PB1, GPIO_ALT);     //TIM1_CH3N D6

GPIOA->AFR[1]  |=  (1U << GPIO_AFRH_AFSEL8_Pos);          // AF1 = TIM1_CH1
GPIOA->AFR[0]  |=  (1U << GPIO_AFRL_AFSEL7_Pos);          // AF1 = TIM1_CH1N
GPIOA->AFR[1]  |=  (1U << GPIO_AFRH_AFSEL10_Pos);          // AF1 = TIM1_CH2
GPIOB->AFR[0]  |=  (1U << GPIO_AFRL_AFSEL1_Pos);          // AF1 = TIM1_CH2N

//setting GPIOs to push pull
GPIOA->OTYPER &= ~(1U << 7);
GPIOA->OTYPER &= ~(1U << 8);
GPIOA->OTYPER &= ~(1U << 10);
GPIOB->OTYPER &= ~(1U << 1);

// Setting all GPIO to high speed
GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED7_Msk);
GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED8_Msk);
GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED10_Msk);
GPIOB->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED1_Msk);

// Enable clks to Timers
RCC->APB2ENR |= (RCC_APB2ENR_TIM1EN);
}



void TIM1PWMinit(uint32_t PSC, uint32_t ARR, uint32_t CCR, uint8_t DTencoded, uint8_t phase_deg, uint32_t CCR3, uint32_t CCR4){
// Making all changes to TIM1
TIM1->CR1 &= ~TIM_CR1_CEN;                    //disable for config
TIM1->CCMR1 = 0;                             // clearing just for OC1PE later in case
TIM1->CCMR2 = 0;                             // clearing just for OC1PE later in case

TIM1->PSC = PSC;
TIM1->ARR = ARR;
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
TIM1->CCR3 = CCR3; // from fn 
TIM1->CCR4 = CCR4; // from fn


TIM1->CCER = 0; // start from a clean state
TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE ); // Capture compare en for both channels on CH1
TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE );// Capture compare en for both channels on CH2

TIM1->BDTR = 0;
TIM1->BDTR |= (DTencoded << TIM_BDTR_DTG_Pos); // for dead time generator setup

TIM1->BDTR &= ~TIM_BDTR_MOE;      
TIM1->BDTR |= TIM_BDTR_OSSR;  // When inactive, OC and OCN outputs enabled with their inactive level. 
// ^^Used when MOE=1 on channels w complementary outputs


TIM1->EGR  |= TIM_EGR_UG;  
TIM1->CR1 |= TIM_CR1_CEN; //enable slave second                
}

static void tim_compute_edge(uint32_t f_tim_hz, uint32_t f_pwm_hz,
                             uint32_t *PSC, uint32_t *ARR, uint32_t *CCR){
    *PSC = 0; // prescaler = 1
    *ARR = (f_tim_hz / (2 * ( *PSC + 1U) * f_pwm_hz)) - 1U;
    *CCR = (*ARR + 1U)/2; // always half of ARR for 50%
}

static void tim_phase_shift(uint32_t ARR, float phase_deg, uint32_t *CCR3, uint32_t *CCR4)
{
    uint32_t halfwave = ARR+ 1U;
    uint32_t period = 2*halfwave;

    float phase_ticks_f = (phase_deg / 360.0f) * (float)period;
    uint32_t phase_ticks = (uint32_t)(phase_ticks_f + 0.5f); // round

    *CCR3 = phase_ticks;
    *CCR4 = ARR - phase_ticks;
}




void TIM1PWMramp(uint8_t ramp){

TIM1->BDTR = 0;
TIM1->BDTR |= (ramp << TIM_BDTR_DTG_Pos); // for dead time generator setup

TIM1->BDTR &= ~TIM_BDTR_MOE;      
TIM1->BDTR |= TIM_BDTR_OSSR;  // When inactive, OC and OCN outputs enabled with their inactive level. 
// ^^Used when MOE=1 on channels w complementary outputs

TIM1->EGR  |= TIM_EGR_UG;     
}




static inline void TIM1_ConfigPhaseTRGO_CH4(uint32_t ARR, uint16_t phase_deg)
{
    // --- compute phase_ticks in [0 .. 2*(ARR+1)] ---
    uint32_t T = 2u * (ARR + 1u);

    // clamp
    if (phase_deg > 360) phase_deg = 360;

    // integer ticks (rounded)
    uint32_t phase_ticks = (uint32_t)((((uint64_t)phase_deg) * T + 180u) / 360u);

    // Special-case: 360° == 0°
    if (phase_ticks >= T) phase_ticks = 0;

    // --- program CH4 to create exactly one rising edge per full cycle ---
    // We'll use OC4PE so updates are synchronous.
    TIM1->CCMR2 &= ~(TIM_CCMR2_CC4S_Msk | TIM_CCMR2_OC4M_Msk);
    TIM1->CCMR2 |= (0u << TIM_CCMR2_CC4S_Pos);     // CC4 as output
    TIM1->CCMR2 |= TIM_CCMR2_OC4PE;                // preload enable for CCR4/OC4M

    uint32_t ccr4;

    if (phase_ticks <= (ARR + 1u)) {
        // First half: rising edge on upcount compare => PWM2
        // PWM2 = 0b111 in OCxM (plus OCxM_3 for extended modes if needed)
        // On STM32L4, PWM1/PWM2 are 110/111, extended bit is bit 16/24 for CH3/4.
        ccr4 = phase_ticks;                // 0..ARR+1 (CCR=ARR+1 will never match; you may clamp)
        if (ccr4 > ARR) ccr4 = ARR;        // ensure match occurs
        // OC4M = PWM2 (0b111)
        TIM1->CCMR2 &= ~(TIM_CCMR2_OC4M_Msk);
        TIM1->CCMR2 |= (0b111u << TIM_CCMR2_OC4M_Pos);
        // No need to set the "extended" bit for basic PWM modes on L4.
    } else {
        // Second half: rising edge on downcount compare => PWM1
        uint32_t delta = phase_ticks - (ARR + 1u);   // 1..ARR+1
        // phase = (ARR+1) + (ARR - CCR4)  => CCR4 = ARR - delta
        if (delta > ARR) delta = ARR;
        ccr4 = ARR - delta;

        TIM1->CCMR2 &= ~(TIM_CCMR2_OC4M_Msk);
        TIM1->CCMR2 |= (0b110u << TIM_CCMR2_OC4M_Pos); // PWM1
    }

    TIM1->CCR4 = ccr4;

    // Ensure CH4 is not output to a pin (but OC4REF still exists internally):
    TIM1->CCER &= ~(TIM_CCER_CC4E);

    // Select TRGO = OC4REF (MMS = 111 per typical STM32 mapping; verify in RM if you want)
    // AN4013 confirms OC4Ref is a valid TRGO selection.  [oai_citation:4‡STMicroelectronics](https://www.st.com/resource/en/application_note/an4013-introduction-to-timers-for-stm32-mcus-stmicroelectronics.pdf)
    TIM1->CR2 &= ~(TIM_CR2_MMS_Msk);
    TIM1->CR2 |= (0b111u << TIM_CR2_MMS_Pos);  // TRGO = OC4REF

    // Optional but recommended: master/slave mode enable for tighter sync
    TIM1->SMCR |= TIM_SMCR_MSM;
}



void TIM15_PWM_Compl_SlaveInit(uint32_t PSC, uint32_t ARR, uint8_t DTencoded)
{
    // Enable TIM15 clock in RCC before calling this (not shown).

    TIM15->CR1 &= ~TIM_CR1_CEN;

    TIM15->PSC = PSC;
    TIM15->ARR = ARR;
    TIM15->CR1 |= TIM_CR1_ARPE;

    // Match TIM1 counting style to keep edges symmetric:
    TIM15->CR1 &= ~TIM_CR1_DIR;
    TIM15->CR1 &= ~TIM_CR1_CMS_Msk;
    TIM15->CR1 |= (1u << TIM_CR1_CMS_Pos);   // center-aligned mode 1

    // CH1 PWM, 50% duty
    TIM15->CCMR1 &= ~(TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1M_Msk);
    TIM15->CCMR1 |= (0u << TIM_CCMR1_CC1S_Pos);  // output
    TIM15->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM15->CCMR1 |= (0b110u << TIM_CCMR1_OC1M_Pos); // PWM1
    TIM15->CCR1 = ARR / 2u;

    // Enable CH1 and CH1N
    TIM15->CCER = 0;
    TIM15->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);

    // Dead-time and MOE (TIM15 has BDTR-style register set on STM32L4)
    TIM15->BDTR = 0;
    TIM15->BDTR |= ((uint32_t)DTencoded << TIM_BDTR_DTG_Pos);
    TIM15->BDTR |= TIM_BDTR_MOE;
    TIM15->BDTR |= TIM_BDTR_OSSR;

    // ---- Slave mode: TRGI = ITR0, SMS = Reset mode ----
    // For TIM15, ITR0 selects TIM1 internally on STM32L4.  [oai_citation:6‡ManualsLib](https://www.manualslib.com/manual/1317428/St-Stm32l4x6.html?page=818)
    TIM15->SMCR = 0;
    TIM15->SMCR |= (0b000u << TIM_SMCR_TS_Pos);   // TS = ITR0
    TIM15->SMCR |= (0b100u << TIM_SMCR_SMS_Pos);  // SMS = Reset mode

    // Generate update to load preloads
    TIM15->EGR = TIM_EGR_UG;

    // Enable counter: it will continuously run, but will be *reset* each phase trigger.
    TIM15->CR1 |= TIM_CR1_CEN;
}



int main(void){

configureFlash();
configureClock();
RCC->APB2ENR |= (1 << 16);
RCC->APB2ENR |= (1 << 17);
initTIM(TIM15);
initTIM(TIM16);
TIM1GPIOinit();


uint32_t PSC, ARR, CCR, CCR3, CCR4;
tim_compute_edge(F_TIM_HZ, F_PWM_HZ, &PSC, &ARR, &CCR);
tim_phase_shift(ARR, phase_deg, &CCR3, &CCR4);

uint8_t DTencoded = dead_time_generator(DT_us, F_TIM_HZ);

//used to be 0xCF
TIM1PWMinit(PSC, ARR, CCR, DTencoded, phase_deg, CCR3, CCR4); // initial deadtime is 0xFF -> or clamped
TIM1_ConfigPhaseTRGO_CH4(ARR, PHASE2_DEG);
TIM1->BDTR &= ~TIM_BDTR_MOE; 
TIM1->BDTR  |= TIM_BDTR_MOE;  
TIM15_PWM_Compl_SlaveInit(PSC, ARR, DTencoded);




 
//delay_millis(TIM15, 1000);




/*
for (uint8_t ramp = 0xCF; ramp >= DTencoded; ramp--)
{   
    TIM1PWMramp(ramp);
    delay_millis(TIM16, 750);
    delay_millis(TIM16, 750);
    delay_millis(TIM16, 750);
    delay_millis(TIM16, 750);
}
*/


while (1) {
}
} 
