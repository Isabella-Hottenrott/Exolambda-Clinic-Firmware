
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
#include "Claude.h"


int main(void){
Init_Phase_Shifted_PWM_System();
TIM15->BDTR  |= TIM_BDTR_MOE; 
TIM2->BDTR  |= TIM_BDTR_MOE; 
Debug_Enable_TIM2_Output();

while (1) {
}
} 

// 0 gives 180
// 10 gives phase delay ticks = 22 and appearing 135 
// 20 gives phase delay ticks = 45 and appearing 135
// 30 gives phase delay ticks = 67 and appearing 135 
// 40 gives phase delay ticks = 89 and appearing 135  diff up until now has been 25.2 uS

// 50 gives phase delay ticks = 111 and appearing 90  time diff here 44.8 uS
// 60 gives phase delay ticks = 134 and appearing 90
// 70 gives phase delay ticks = 156 and appearing 90
// 80 gives phase delay ticks = 178 and appearing 90
// 90 gives phase delay ticks = 201 and appearing 90ish time diff here 42 uS
// 100 gives phase delay ticks = 223 and appearing 90 time diff here 44 uS
// 120 gives phase delay ticks = 267 and appearing 90 time diff 44.8
// 140 gives phase delay ticks = 312 and appearing 90
// 160 gives phase delay ticks = 356  and appearing  90
// 180 gives phase delay ticks = 401  and appearing 90ish 41.6 uS
// 200 gives phase delay ticks = 446 and appearing  90ish and 46uS
// 220 gives phase delay ticks = 490 and appearing 90
// 240 gives phase delay ticks = 535 and appearing  90
// 260 gives phase delay ticks = 579 and appearing 90
// 280 gives phase delay ticks = 624 and appearing  90
// 300 gives phase delay ticks = 668 and appearing 90
// 320 gives phase delay ticks = 713 and appearing  90
// 340 gives phase delay ticks = 757 and appearing 90
// 360 gives phase delay ticks = 802 and appearing 90 42.4 uS
// 450 gives phase delay ticks = 1003 and appearing 45



//4.8 uS with 10
//10.8 uS with 10


// start value for DT sweep is 0xCF


