
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
int PHASE2_DEG = 20;
const uint32_t cnt_rst = 17;
// ^ right now its not taking the bit 16


static uint8_t dead_time_generator(float dead_us, uint32_t tim_freq){
    const double  t_dts = 1.0 / (double)tim_freq;
    uint32_t ticks = (uint32_t)((dead_us * 1e-9f) / t_dts);
    if (ticks <= 127)                   return (uint8_t)ticks; // 0 to 127 because 127 is biggest 6 bit number
    if (ticks   <= (127*2))             return (uint8_t)(0x80 | ((ticks/2)  - 64)); // for ticks = (64+k)*2 where k = {0,63} since 63 is biggest 6 bit number
    if (ticks   <= (504))              return (uint8_t)(0xC0 | ((ticks/8)  - 32));     // for ticks = (32+k)*8 where k = {0,32} since 32 is biggest 5 bit number
    if (ticks/16  <= (1008))              return (uint8_t)(0xE0 | ((ticks/16) - 32));     // for ticks = (32+k)*16 where k = {0,32} since 32 is biggest 5 bit number
    return 0xFF; // clamp otherwise
}



void TIMERGPIOinit(void){

gpioEnable(GPIO_PORT_A);
gpioEnable(GPIO_PORT_B);
gpioEnable(GPIO_PORT_C);

//GPIO channels for TIM1
pinMode(PA8, GPIO_ALT);   //TIM1_CH1 D9
pinMode(PA7, GPIO_ALT);   //TIM1_CH1N A6
pinMode(PA10, GPIO_ALT);     //TIM1_CH3 D0
pinMode(PB1, GPIO_ALT);     //TIM1_CH3N D6
pinMode(PA2, GPIO_ALT);     //TIM15_CH1 A7
pinMode(PA1, GPIO_ALT);     //TIM15_CH1N A1


GPIOA->AFR[1]  |=  (1U << GPIO_AFRH_AFSEL8_Pos);          // AF1 = TIM1_CH1
GPIOA->AFR[0]  |=  (1U << GPIO_AFRL_AFSEL7_Pos);          // AF1 = TIM1_CH1N
GPIOA->AFR[1]  |=  (1U << GPIO_AFRH_AFSEL10_Pos);          // AF1 = TIM1_CH2
GPIOB->AFR[0]  |=  (1U << GPIO_AFRL_AFSEL1_Pos);          // AF1 = TIM1_CH2N
GPIOA->AFR[0]  |=  (14U << GPIO_AFRL_AFSEL2_Pos);          // AF1 = TIM15_CH1
GPIOA->AFR[0]  |=  (14U << GPIO_AFRL_AFSEL1_Pos);          // AF1 = TIM15_CH1N

//setting GPIOs to push pull
GPIOA->OTYPER &= ~(1U << 7);
GPIOA->OTYPER &= ~(1U << 8);
GPIOA->OTYPER &= ~(1U << 10);
GPIOB->OTYPER &= ~(1U << 1);
GPIOA->OTYPER &= ~(1U << 1);
GPIOA->OTYPER &= ~(1U << 2);


// Setting all GPIO to high speed
GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED7_Msk);
GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED8_Msk);
GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED10_Msk);
GPIOB->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED1_Msk);
GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED1_Msk);
GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED2_Msk);


// Enable clks to Timers
RCC->APB2ENR |= (RCC_APB2ENR_TIM1EN);
RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
}



void TIM1PWMinit(uint32_t PSC, uint32_t ARR, uint32_t CCR, uint8_t DTencoded, uint8_t phase_deg, uint32_t CCR3, uint32_t CCR4){
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

TIM1->CR2 &= ~TIM_CR2_MMS;
TIM1->CR2 |= (2U << TIM_CR2_MMS_Pos); // MMS = 05: Trigger on Update Even

TIM1->BDTR = 0;
TIM1->BDTR |= (DTencoded << TIM_BDTR_DTG_Pos); // for dead time generator setup
TIM1->EGR  |= TIM_CR1_UDIS;
TIM1->EGR  |= TIM_EGR_UG;  
TIM1->EGR  |= TIM_EGR_TG; 
TIM1->BDTR  |= TIM_BDTR_MOE;         
}

static void tim_compute_edge(uint32_t f_tim_hz, uint32_t f_pwm_hz,
                             uint32_t *PSC, uint32_t *ARR, uint32_t *CCR){
    *PSC = 0; // prescaler = 1
    *ARR = (f_tim_hz / (2 * ( *PSC + 1U) * f_pwm_hz));
    *CCR = (*ARR)/2; // always half of ARR for 50%
}

static void tim_phase_shift(uint32_t ARR, float phase_deg, uint32_t *CCR3, uint32_t *CCR4){
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
TIM1->EGR  |= TIM_EGR_UG;     
}

void TIM15PWMramp(uint8_t ramp){
TIM15->BDTR = 0;
TIM15->BDTR |= (ramp << TIM_BDTR_DTG_Pos); // for dead time generator setup
TIM15->BDTR &= ~TIM_BDTR_MOE;      
TIM15->BDTR |= TIM_BDTR_OSSR;  
TIM15->EGR  |= TIM_EGR_UG;     
}



void TIM15_PWM(uint32_t PSC, uint32_t tim15arr, uint8_t DT_encoded){
    TIM15->PSC = PSC;
    TIM15->ARR = tim15arr;
    TIM15->CR1 |= TIM_CR1_ARPE;
    TIM15->CCMR1 |= (6u << TIM_CCMR1_OC1M_Pos);
    TIM15->CCMR1 |=TIM_CCMR1_OC1PE;
    TIM15->CCR1 = (tim15arr)/2U;
    TIM15->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE;
    TIM15->CR1 &= ~TIM_CR1_CKD_Msk;        // ensure dead timer to same clock division as others
    TIM15->BDTR = 0;
    TIM15->BDTR |= (DT_encoded << TIM_BDTR_DTG_Pos);
    TIM15->EGR = TIM_EGR_UG;
    TIM15->BDTR  |= TIM_BDTR_MOE; 
}


//DMA Code

void initDMA2(void){
    RCC->AHB1ENR |= (RCC_AHB1ENR_DMA1EN);
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;

    // Reset DMA1 Channel 2
    DMA1_Channel5->CCR  &= ~(0xFFFFFFFF);
    DMA1_Channel5->CCR  |= (_VAL2FLD(DMA_CCR_PL,0b10) |
                            _VAL2FLD(DMA_CCR_MINC, 0b0) |
                            _VAL2FLD(DMA_CCR_CIRC, 0b1) |
                            _VAL2FLD(DMA_CCR_DIR, 0b1));
    
    // Set DMA source and destination addresses.
    // Source: Address of the character array buffer in memory.
    DMA1_Channel5->CMAR = _VAL2FLD(DMA_CMAR_MA, (uint32_t) &cnt_rst);
    DMA1_Channel5->CPAR = _VAL2FLD(DMA_CPAR_PA, (uint32_t) &(TIM15->CR1));
    DMA1_Channel5->CNDTR  |= 0b1;   // Set DMA data transfer length (# of samples).
    DMA1_CSELR->CSELR |= _VAL2FLD(DMA_CSELR_C5S, 4);  // Select 4th option for mux to channel 5 (TIM2_Channel)
      
    TIM1->CR1 |= TIM_CR1_CEN;  
    configureFlash();
    TIM1->CR1 |= TIM_CR1_CEN;
    DMA1_Channel5->CCR  |= DMA_CCR_EN;
}


void TIM2Init(uint32_t PSC, uint32_t ARR, uint32_t CCR1){
  RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN);
  TIM2->PSC = PSC;
  TIM2->ARR = ARR;
  TIM2->CR2 &= ~(TIM_CR2_CCDS); // Set DMA request when CCx event occurs

  TIM2->SMCR &= ~(0u << TIM_SMCR_TS_Pos); // TS = ITR0
  TIM2->SMCR |=(6u << TIM_SMCR_SMS_Pos);

  TIM2->DIER |= TIM_DIER_CC1DE; // for DMA
  TIM2->CCMR1 |= (3u << TIM_CCMR1_OC1M_Pos); // toggle when match
  TIM2->CCMR1 |=TIM_CCMR1_OC1PE;
  TIM2->CCER |= TIM_CCER_CC1E;
  TIM2->CCR1 = CCR1;

  TIM2->EGR |= TIM_EGR_CC1G;
  TIM2->EGR |= 1;
}


int main(void){
configureFlash();
configureClock();
TIMERGPIOinit();

uint32_t PSC, ARR, CCR, CCR3, CCR4;
tim_compute_edge(F_TIM_HZ, F_PWM_HZ, &PSC, &ARR, &CCR);
tim_phase_shift(ARR, phase_deg, &CCR3, &CCR4);

uint8_t DTencoded = dead_time_generator(DT_us, F_TIM_HZ); 
uint32_t tim15arr = 2U * (ARR)-1;
uint32_t phaseshiftCCR = (uint32_t)((((float)PHASE2_DEG) * (float)tim15arr) / 360.0f + 0.5f);

TIM1PWMinit(PSC, ARR, CCR, DTencoded, phase_deg, CCR3, CCR4);
TIM15_PWM(PSC, tim15arr, DTencoded);
TIM2Init(PSC, tim15arr, phaseshiftCCR);
initDMA2();


while (1) {
}
} 







// start value for DT sweep is 0xCF


