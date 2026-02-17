
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


int main(void){
Init_Phase_Shifted_PWM_System();
TIM15->BDTR  |= TIM_BDTR_MOE; 
TIM2->BDTR  |= TIM_BDTR_MOE; 
Debug_Enable_TIM2_Output();

while (1) {}
} 




