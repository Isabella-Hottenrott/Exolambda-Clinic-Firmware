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
int F_PWM_HZ = 200000;
int DT_us = 100;
int phase_deg = 90;


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
pinMode(PA10, GPIO_ALT);     //TIM1_CH3
pinMode(PB1, GPIO_ALT);     //TIM1_CH3N

GPIOA->AFR[1]  |=  (1U << GPIO_AFRH_AFSEL8_Pos);          // AF1 = TIM1_CH1
GPIOA->AFR[0]  |=  (1U << GPIO_AFRL_AFSEL7_Pos);          // AF1 = TIM1_CH1N
GPIOA->AFR[1]  |=  (1U << GPIO_AFRH_AFSEL9_Pos);          // AF1 = TIM1_CH2
GPIOB->AFR[0]  |=  (1U << GPIO_AFRL_AFSEL0_Pos);          // AF1 = TIM1_CH2N
GPIOA->AFR[1]  |=  (1U << GPIO_AFRH_AFSEL10_Pos);         // AF1 = TIM1_CH3
GPIOB->AFR[0]  |=  (1U << GPIO_AFRL_AFSEL1_Pos);          // AF1 = TIM1_CH3N


//setting GPIOs to push pull
GPIOA->OTYPER &= ~(1U << 7);
GPIOA->OTYPER &= ~(1U << 8);
GPIOA->OTYPER &= ~(1U << 9);
GPIOA->OTYPER &= ~(1U << 10);
GPIOB->OTYPER &= ~(1U << 0);
GPIOB->OTYPER &= ~(1U << 1);

// Setting all GPIO to high speed
GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED7_Msk);
GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED8_Msk);
GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED9_Msk);
GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED10_Msk);
GPIOB->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED0_Msk);
GPIOB->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED1_Msk);

// Enable clks to Timers
RCC->APB2ENR |= (RCC_APB2ENR_TIM1EN);
}


void TIM15GPIOinit(void){
gpioEnable(GPIO_PORT_A);
//GPIO channels for TIM15
pinMode(PA2, GPIO_ALT);   // TIM15_CH1
pinMode(PA1, GPIO_ALT);   // TIM15_CH1N


GPIOA->AFR[0]  |=  (14U << GPIO_AFRL_AFSEL2_Pos);      // AF14
GPIOA->AFR[0]  |=  (14U << GPIO_AFRL_AFSEL1_Pos);      // AF14


GPIOA->OTYPER &= ~(1U << 1);
GPIOA->OTYPER &= ~(1U << 2);

GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED1_Msk);
GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED2_Msk);

RCC->APB2ENR |= (RCC_APB2ENR_TIM15EN);
}

void TIM1PWMinit(uint32_t PSC, uint32_t ARR, uint32_t CCR, uint8_t DTencoded, uint8_t phase_deg){
// Making all changes to TIM1
TIM1->CR1 &= ~TIM_CR1_CEN;                    //disable for config
TIM1->CCMR1 = 0;                             // clearing just for OC1PE later in case
TIM1->CCMR2 = 0;                             // clearing just for OC1PE later in case

TIM1->PSC = PSC;
TIM1->ARR = ARR;
TIM1->CR1 |=  TIM_CR1_ARPE;                   // ARPE = 1 (ARR preload)

// MSM bit in CR1 to enable master slave mode

TIM1->CR1 &= ~TIM_CR1_DIR;            // DIR = 0 (up)
TIM1->CR1 |= _VAL2FLD(TIM_CR1_CMS, 0b01);  // center aligned
TIM1->CR1 &= ~TIM_CR1_CKD_Msk;        // ensure dead timer to same clock division as others

// Fn to calculate the CCR value
TIM1->CCMR1 |= TIM_CCMR1_OC1PE; // Output compare preload en on channel 1
TIM1->CCMR1 |= _VAL2FLD(TIM_CCMR1_OC1M, 0b110); //normal PWM mode 1
TIM1->CCR1 = (ARR + 1)/2;

TIM1->CCMR1 |= TIM_CCMR1_OC2PE; // Output compare preload en on channel 2
TIM1->CCMR1 |= _VAL2FLD(TIM_CCMR1_OC2M, 0b110); //normal PWM mode 1
TIM1->CCR2 = (ARR + 1)/2;

TIM1->CCMR2 |= TIM_CCMR2_OC3PE; // Output compare preload en on channel 3
TIM1->CCMR2 |= _VAL2FLD(TIM_CCMR2_OC3M, 0b110); //Asymmetric PWM mode 1
TIM1->CCMR2 |= TIM_CCMR2_OC3M_3;                // extended bit

uint32_t period2 = 2*(ARR + 1);     // up and down count
uint32_t center0 = (ARR + 1);  // center
uint32_t halfON = (ARR + 1) / 2;  // half of ON

uint32_t left0  = (center0 >= halfON) ? (center0 - halfON) : (center0 + period2 - halfON);
uint32_t right0 = (center0 + halfON) % period2;
uint32_t phase_counts = (uint32_t)((phase_deg / 360.0f) * (float)period2 + 0.5f);

// shift both edges and wrap
uint32_t leftB  = (left0  + phase_counts) % period2;
uint32_t rightB = (right0 + phase_counts) % period2;
TIM1->CCR3 = (leftB  % (ARR + 1u));   // up-count edge → CCR3
TIM1->CCR4 = (rightB % (ARR + 1u));   // down-count edge → CCR4

TIM1->CCER |= TIM_CCER_CC1E;  // CH1
TIM1->CCER |= TIM_CCER_CC2E;  // CH2
TIM1->CCER |= TIM_CCER_CC3E;  // CH3
TIM1->CCER |= TIM_CCER_CC2NE;  // CH2
TIM1->CCER |= TIM_CCER_CC1NE;  // CH1
TIM1->CCER |= TIM_CCER_CC3NE;  // CH3

TIM1->BDTR = 0;
TIM1->BDTR |= (DTencoded << TIM_BDTR_DTG_Pos); // for dead time generator setup

//THESE
TIM1->SMCR |= TIM_SMCR_MSM;     // slave master mode enabled
TIM1->CR2 |=  (1 << TIM_CR2_MMS_Pos);  // update event is in TRGO YOU WANT SOMETHING ESLE??

TIM1->BDTR &= ~TIM_BDTR_MOE;      
TIM1->BDTR |= TIM_BDTR_OSSR;  // When inactive, OC and OCN outputs enabled with their inactive level. 
// ^^Used when MOE=1 on channels w complementary outputs

TIM1->BDTR |= TIM_BDTR_BKE;   // enables break protection? Just disables everything instantly if we need
TIM1->BDTR |= TIM_BDTR_BKP;   // break input BRK is active high

TIM1->EGR  |= TIM_EGR_UG;
TIM1->CR1 |= TIM_CR1_CEN; //enable master second
TIM1->BDTR &= ~TIM_BDTR_MOE;                    
}




void TIM15PWMinit(uint32_t PSC, uint32_t ARR, uint32_t CCR, uint8_t DTencoded) {
TIM15->CR1 &= ~TIM_CR1_CEN;                    //disable for config
TIM15->CR1 |= (0U << TIM_CR1_DIR_Pos);         // DIR = 0 (up)
TIM15->CR1 |= (1U << TIM_CR1_CMS_Pos);        // CMS = 01 (center aligned)
TIM15->CR1 |= (0U << TIM_CR1_CKD_Pos);        // ensure dead timer to
TIM15->CR1 |=  TIM_CR1_ARPE;                   // ARPE = 1 (ARR preload)

TIM15->PSC = PSC;
TIM15->ARR = ARR;
TIM15->CCR1 = CCR;


TIM15->CCMR1 |= TIM_CCMR1_OC1PE; // Output compare preload en
TIM15->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos); //PWM mode 1 for TIM15. 

TIM15->CCER |= TIM_CCER_CC1E;
TIM15->CCER |= TIM_CCER_CC1NE;

TIM15->BDTR |= TIM_BDTR_OSSR; 
TIM15->BDTR |= (DTencoded << TIM_BDTR_DTG_Pos); // for dead time

//configs for slave mode
TIM15->SMCR |= (0b110 << TIM_SMCR_SMS_Pos);      // SMS = 110 (Reset mode)
TIM15->SMCR |= (0x0U << TIM_SMCR_TS_Pos);       // TS = ITR0 (TIM1 TRGO on most L4s) ← verify in RM
// Need this to be a different one

uint32_t period2      = 2u * (ARR + 1u);
uint32_t phase_counts = (uint32_t)((phase_deg / 360.0f) * (float)period2 + 0.5f);
uint32_t cnt_phase    = phase_counts % (ARR + 1u);
TIM15->CNT = cnt_phase;     // position the carrier phase for the up-half

TIM15->EGR |= TIM_EGR_UG;   // latch PSC/ARR/CCR
TIM15->CR1 |= TIM_CR1_CEN; //enable slave second
TIM15->BDTR &= ~TIM_BDTR_MOE;                    // enable CH/CHN driving when CC1E/CC1NE set
}

static void tim_compute_edge(uint32_t f_tim_hz, uint32_t f_pwm_hz,
                             uint32_t *PSC, uint32_t *ARR, uint32_t *CCR){
    *PSC = 0; // prescaler = 1
    *ARR = (f_tim_hz / (2U * ( *PSC + 1U) * f_pwm_hz)) - 1U;
    *CCR = (*ARR + 1U) / 2U; // always half of ARR
}



int main(void){
//Just in case
configureFlash();

//Set RCC= 80MHz
configureClock();

TIM1GPIOinit();
TIM15GPIOinit();

//fn to calculate the prescaler and arr values
//put that fn inside TIM1PWMinit fn

uint32_t PSC, ARR, CCR;
tim_compute_edge(F_TIM_HZ, F_PWM_HZ, &PSC, &ARR, &CCR);

uint8_t DTencoded = dead_time_generator(DT_us, F_TIM_HZ);

TIM1PWMinit(PSC, ARR, CCR, DTencoded, phase_deg);
TIM15PWMinit(PSC, ARR, CCR, DTencoded);


TIM1->BDTR  |= TIM_BDTR_MOE;   // master first or either order—both are locked now
TIM15->BDTR |= TIM_BDTR_MOE;

while (1) {
}
}