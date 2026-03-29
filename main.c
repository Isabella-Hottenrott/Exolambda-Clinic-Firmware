
// main.c
// Exolamba Clinic
// email



#include "main.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_RCC.h"
#include "STM32L432KC_SPI.h"
#include "STM32L432KC.h"
#include "STM32L432KC_TIM.h"
#include "top.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "SEGGER_RTT.h"


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

// ── RTT CLI ───────────────────────────────────────────────────────────
uint16_t cur_secondary = 90;   // last values from the ramps above
uint16_t cur_primtwo   = 180;
uint16_t cur_dead_ns   = 100;

char rxbuf[64];
uint8_t rxidx = 0;

SEGGER_RTT_WriteString(0, "\r\n--- Phase-Shift CLI ---\r\n");
SEGGER_RTT_WriteString(0, "Commands:\r\n");
SEGGER_RTT_WriteString(0, "  s <deg>   Set secondary shift\r\n");
SEGGER_RTT_WriteString(0, "  p <deg>   Set primary-two phase\r\n");
SEGGER_RTT_WriteString(0, "  d <ns>    Set dead time (ns)\r\n");
SEGGER_RTT_WriteString(0, "  ?         Show current values\r\n");
SEGGER_RTT_WriteString(0, "> ");

while (1) {
    if (!SEGGER_RTT_HasKey()) continue;
    char c = (char)SEGGER_RTT_GetKey();

    if (c == '\b' || c == 127) {
        if (rxidx > 0) rxidx--;
        continue;
    }

    if (c != '\r' && c != '\n') {
        if (rxidx < sizeof(rxbuf) - 1) rxbuf[rxidx++] = c;
        continue;
    }

    // Enter pressed — parse command
    SEGGER_RTT_WriteString(0, "\r\n");
    rxbuf[rxidx] = '\0';
    rxidx = 0;

    char cmd = rxbuf[0];
    uint16_t val = 0;
    char msg[80];

    if (strlen(rxbuf) >= 3) val = (uint16_t)atoi(&rxbuf[2]);

    switch (cmd) {
    case 's':
        cur_secondary = val;
        Update_Secondary_Shift((float)val);
        snprintf(msg, sizeof(msg), "Secondary shift -> %u deg\r\n", val);
        SEGGER_RTT_WriteString(0, msg);
        break;
    case 'p':
        cur_primtwo = val;
        Update_PrimTwo_Phase((float)val);
        snprintf(msg, sizeof(msg), "PrimTwo phase -> %u deg\r\n", val);
        SEGGER_RTT_WriteString(0, msg);
        break;
    case 'd': {
        cur_dead_ns = val;
        uint8_t dt = dead_time_generator((float)val, 80000000UL);
        TIM15->BDTR = (TIM15->BDTR & ~TIM_BDTR_DTG_Msk) | (dt << TIM_BDTR_DTG_Pos);
        TIM1->BDTR  = (TIM1->BDTR  & ~TIM_BDTR_DTG_Msk) | (dt << TIM_BDTR_DTG_Pos);
        snprintf(msg, sizeof(msg), "Dead time -> %u ns (DTG=0x%02X)\r\n", val, dt);
        SEGGER_RTT_WriteString(0, msg);
        break;
    }
    case '?':
        snprintf(msg, sizeof(msg), "Secondary: %u deg\r\n", cur_secondary);
        SEGGER_RTT_WriteString(0, msg);
        snprintf(msg, sizeof(msg), "PrimTwo:   %u deg\r\n", cur_primtwo);
        SEGGER_RTT_WriteString(0, msg);
        snprintf(msg, sizeof(msg), "Dead time: %u ns\r\n", cur_dead_ns);
        SEGGER_RTT_WriteString(0, msg);
        break;
    default:
        if (strlen(rxbuf) > 0)
            SEGGER_RTT_WriteString(0, "Unknown cmd. Type ? for help\r\n");
        break;
    }
    SEGGER_RTT_WriteString(0, "> ");
}
}




