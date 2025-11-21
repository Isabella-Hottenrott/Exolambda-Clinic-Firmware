// main.c
// Exolamba Clinic
// email
// data
//
// This is the main c code for the phase modulation for the dual active bridge
// Below is the Arduino code in the process to be configured to STM32L432KC MCU

#include <string.h>
#include <stdio.h>
#include "main.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_RCC.h"
#include "STM32L432KC_SPI.h"
#include "STM32L432KC.h"
#include "STM32L432KC_TIM.h"

int F_TIM_HZ = 80000000;
int F_PWM_HZ = 100000;
int DT_us = 100;
int phase_deg = 180;


static uint8_t dead_time_generator(float dead_us, uint32_t tim_freq){
    const double  t_dts = 1.0 / (double)tim_freq;
    uint32_t ticks = (uint32_t)((dead_us * 1e-9f) / t_dts);
    if (ticks <= 127)                   return (uint8_t)ticks; // 0to127 because 127 is biggest 6 bit number
    if (ticks   <= (127*2))             return (uint8_t)(0x80 | ((ticks/2)  - 64)); // for ticks = (64+k)*2 where k = {0,63} since 63 is biggest 6 bit number
    if (ticks   <= (504))              return (uint8_t)(0xC0 | ((ticks/8)  - 32));     // for ticks = (32+k)*8 where k = {0,32} since 32 is biggest 5 bit number
    if (ticks/16  <= (1008))              return (uint8_t)(0xE0 | ((ticks/16) - 32));     // for ticks = (32+k)*16 where k = {0,32} since 32 is biggest 5 bit number
    return 0xFF; // clamp otherwise
}



void TIM1GPIOinit(void){

gpioEnable(GPIO_PORT_A);
gpioEnable(GPIO_PORT_B);

//GPIO channels for TIM1
pinMode(PA8, GPIO_ALT);   //TIM1_CH1
pinMode(PA7, GPIO_ALT);   //TIM1_CH1N
pinMode(PA9, GPIO_ALT);     //TIM1_CH2
pinMode(PB0, GPIO_ALT);     //TIM1_CH2N

GPIOA->AFR[1]  |=  (1U << GPIO_AFRH_AFSEL8_Pos);          // AF1 = TIM1_CH1
GPIOA->AFR[0]  |=  (1U << GPIO_AFRL_AFSEL7_Pos);          // AF1 = TIM1_CH1N
GPIOA->AFR[1]  |=  (1U << GPIO_AFRH_AFSEL9_Pos);          // AF1 = TIM1_CH2
GPIOB->AFR[0]  |=  (1U << GPIO_AFRL_AFSEL0_Pos);          // AF1 = TIM1_CH2N

//setting GPIOs to push pull
GPIOA->OTYPER &= ~(1U << 7);
GPIOA->OTYPER &= ~(1U << 8);
GPIOB->OTYPER &= ~(1U << 0);
GPIOB->OTYPER &= ~(1U << 1);

// Setting all GPIO to high speed
GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED7_Msk);
GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED8_Msk);
GPIOB->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED0_Msk);
GPIOB->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED1_Msk);

// Enable clks to Timers
RCC->APB2ENR |= (RCC_APB2ENR_TIM1EN);
}



void TIM1PWMinit(uint32_t PSC, uint32_t ARR, uint32_t CCR, uint8_t DTencoded, uint8_t phase_deg){
// Making all changes to TIM1
TIM1->CR1 &= ~TIM_CR1_CEN;                    //disable for config
TIM1->CCMR1 = 0;                             // clearing just for OC1PE later in case
TIM1->CCMR2 = 0;                             // clearing just for OC1PE later in case

TIM1->PSC = PSC;
TIM1->ARR = ARR;
TIM1->CR1 |=  TIM_CR1_ARPE;                   // ARPE = 1 (ARR preload)

//TIM1->CR1 &= ~TIM_CR1_DIR;            // DIR = 0 (up)
TIM1->CR1 |= _VAL2FLD(TIM_CR1_CMS, 1);        // CMS = 01 (center-aligned)
TIM1->CR1 &= ~TIM_CR1_CKD_Msk;        // ensure dead timer to same clock division as others

TIM1->CCMR1 |= _VAL2FLD(TIM_CCMR1_CC1S, 0); // (output)
TIM1->CCMR1 |= TIM_CCMR1_OC1PE; // Output compare preload en
TIM1->CCMR1 |= (0b110 << TIM_CCMR1_OC1M_Pos); //PWM assymetric. 
TIM1->CCMR1 |= (1 << 16); // get that last top bit
TIM1->CCR1 = 0; // was calculated above

TIM1->CCMR1 |= _VAL2FLD(TIM_CCMR1_CC2S, 0); // (output)
TIM1->CCMR1 |= TIM_CCMR1_OC2PE; // Output compare preload en
TIM1->CCMR1 |= (0b110 << TIM_CCMR1_OC2M_Pos); //PWM assymetric. 
TIM1->CCMR1 |= (1 << 24); // get that last top bit
TIM1->CCR2 = CCR; // was calculated above

TIM1->CCMR1 |= _VAL2FLD(TIM_CCMR1_CC1S, 0); // (output)
TIM1->CCMR1 |= TIM_CCMR1_OC1PE; // Output compare preload en
TIM1->CCR1 = CCR3; // from fn 

TIM1->CCMR1 |= _VAL2FLD(TIM_CCMR1_CC1S, 0); // (output)
TIM1->CCMR1 |= TIM_CCMR1_OC1PE; // Output compare preload en
TIM1->CCR1 = CCR4; // from fn




TIM1->CCER = 0; // start from a clean state
TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE ); // Capture compare en for both channels on CH1
TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE );// Capture compare en for both channels on CH2

TIM1->BDTR = 0;
TIM1->BDTR |= (DTencoded << TIM_BDTR_DTG_Pos); // for dead time generator setup

TIM1->BDTR &= ~TIM_BDTR_MOE;      
TIM1->BDTR |= TIM_BDTR_OSSR;  // When inactive, OC and OCN outputs enabled with their inactive level. 
// ^^Used when MOE=1 on channels w complementary outputs

TIM1->BDTR |= TIM_BDTR_BKE;   // enables break protection? Just disables everything instantly if we need
TIM1->BDTR |= TIM_BDTR_BKP;   // break input BRK is active high

TIM1->EGR  |= TIM_EGR_UG;  
TIM1->CR1 |= TIM_CR1_CEN; //enable slave second                
}

static void tim_compute_edge(uint32_t f_tim_hz, uint32_t f_pwm_hz,
                             uint32_t *PSC, uint32_t *ARR, uint32_t *CCR){
    *PSC = 0; // prescaler = 1
    *ARR = (f_tim_hz / (2 * ( *PSC + 1U) * f_pwm_hz)) - 1U;
    *CCR = (*ARR + 1U) / 2U; // always half of ARR for 50%
}

static void tim_phase_shift(uint32_t ARR, float phase_deg, uint32_t *CCR3, uint32_t *CCR4)
{
    uint32_t halfwave = ARR + 1U;
    uint32_t period = 2*halfwave;

    float phase_ticks_f = (phase_deg / 360.0f) * (float)period;
    uint32_t phase_ticks = (uint32_t)(phase_ticks_f + 0.5f); // round

    *CCR3 = phase_ticks;
    *CCR4 = phase_ticks + period;
}


int main(void){
//Just in case
configureFlash();

//Set RCC= 80MHz
configureClock();

TIM1GPIOinit();
TIM15GPIOinit();


uint32_t PSC, ARR, CCR, CCR3, CCR4;
tim_compute_edge(F_TIM_HZ, F_PWM_HZ, &PSC, &ARR, &CCR);
tim_phase_shift(ARR, phase_deg, &CCR3, &CCR4)
uint8_t DTencoded = dead_time_generator(DT_us, F_TIM_HZ);

TIM1PWMinit(PSC, ARR, CCR, DTencoded, phase_deg);

TIM1->BDTR &= ~TIM_BDTR_MOE;  
TIM1->BDTR  |= TIM_BDTR_MOE;   // master first or either order—both are locked now


while (1) {
}
} 