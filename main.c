
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
#include "top.h"


static uint8_t dead_time_generator(float dead_us, uint32_t tim_freq){
    const double  t_dts = 1.0 / (double)tim_freq;
    uint32_t ticks = (uint32_t)((dead_us * 1e-9f) / t_dts);
    if (ticks <= 127)                   return (uint8_t)ticks;
    if (ticks <= 255)   return (uint8_t)(0x80 | ((ticks/2) - 64));
    if (ticks   <= (504))               return (uint8_t)(0xC0 | ((ticks/8)  - 32));
    if (ticks/16  <= (1008))            return (uint8_t)(0xE0 | ((ticks/16) - 32));
    return 0xFF; // clamp otherwise
}

int main(void){
Init_Phase_Shifted_PWM_System();
TIM15->BDTR  |= TIM_BDTR_MOE; 

initTIM(TIM16);

float ns = 5000.0f;
uint8_t deadt = dead_time_generator(ns, 80000000UL);
    TIM15->BDTR = (TIM15->BDTR & ~TIM_BDTR_DTG_Msk) | (deadt << TIM_BDTR_DTG_Pos);
    TIM1->BDTR  = (TIM1->BDTR  & ~TIM_BDTR_DTG_Msk) | (deadt << TIM_BDTR_DTG_Pos);


delay_millis(TIM16, 100);
delay_millis(TIM16, 100);
delay_millis(TIM16, 100);
delay_millis(TIM16, 100);
delay_millis(TIM16, 100);
delay_millis(TIM16, 100);
delay_millis(TIM16, 100);
delay_millis(TIM16, 100);

printf("dt = %d\n", DTencoded);

float step = 10.0f;


delay_millis(TIM16, 100);
delay_millis(TIM16, 100);
delay_millis(TIM16, 100);
delay_millis(TIM16, 100);
delay_millis(TIM16, 100);
delay_millis(TIM16, 100);
delay_millis(TIM16, 100);
delay_millis(TIM16, 100);
delay_millis(TIM16, 100);

float i=10.0f;

for (float phasei=10.0f; phasei<90.0f; phasei=phasei+i){
    Update_Secondary_Shift(phasei);
    delay_millis(TIM16, 100);
}

for (float phasej=10.0f; phasej<180.0f; phasej=phasej+i){
    Update_PrimTwo_Phase(phasej);
    delay_millis(TIM16, 100);
}

for (ns = 5000.0f; ns >= 100.0f; ns -= step) {
    deadt = dead_time_generator(ns, 80000000UL);
    TIM15->BDTR = (TIM15->BDTR & ~TIM_BDTR_DTG_Msk) | (deadt << TIM_BDTR_DTG_Pos);
    TIM1->BDTR  = (TIM1->BDTR  & ~TIM_BDTR_DTG_Msk) | (deadt << TIM_BDTR_DTG_Pos);
    delay_millis(TIM16, 5);
}

while (1) {
}
} 




