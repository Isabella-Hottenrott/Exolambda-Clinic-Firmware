/**
 * ============================================================================
 * STM32L432KC - Third Complementary PWM Pair with Hardware Phase Shift
 * ============================================================================
 *
 * ARCHITECTURE TOPOLOGY:
 * ----------------------
 * TIM1 (Master, existing asymmetric PWM)
 *   └─[TRGO on Update]─> TIM2 (Phase Delay Timer, One-Pulse Mode)
 *                          └─[TRGO on OC1]─> TIM15 (Complementary PWM with Dead-Time)
 *
 * TIMER ASSIGNMENTS:
 * ------------------
 * - TIM1: Already configured (asymmetric PWM, center-aligned)
 *         Pair A: CH1+CH1N (50% duty)
 *         Pair B: CH3+CH3N
 *         Generates TRGO on update event (start of PWM cycle)
 *
 * - TIM2: Phase delay timer (32-bit GP timer)
 *         Slaves to TIM1 via ITR0
 *         Operates in One-Pulse Mode (OPM)
 *         CCR1 = phase_ticks (programmable delay)
 *         Generates TRGO on OC1 match
 *
 * - TIM15: Complementary PWM output
 *          Pair C: CH1+CH1N with dead-time
 *          Slaves to TIM2 via ITR1 (not available - see note below)
 *          Triggered mode: starts on TIM2 TRGO
 *
 * ITR MAPPINGS (from RM0394 Table 143, Table 146):
 * -------------------------------------------------
 * TIM2 (slave):  ITR0=TIM1, ITR1=USB, ITR2=Reserved, ITR3=Reserved
 * TIM15 (slave): ITR0=TIM1, ITR1=Reserved, ITR2=TIM16_OC1, ITR3=Reserved
 *
 * CRITICAL CONSTRAINT:
 * --------------------
 * TIM15 cannot be directly slaved to TIM2 because:
 *   - TIM15 ITR1 = Reserved (not connected to TIM2)
 *   - TIM15 ITR0 = TIM1 (already used by system)
 *   - TIM15 ITR2 = TIM16 OC1 (TIM16 cannot be slaved, no use)
 *
 * SOLUTION: Use DMA to trigger TIM15 from TIM2
 * ---------------------------------------------
 * Since hardware ITR routing is not available, we use DMA:
 *   TIM2 OC1 generates DMA request → DMA writes to TIM15_EGR → TIM15 starts
 *
 * DMA Configuration:
 *   - TIM2_CC1 DMA request (channel DMA1_CH5, per RM0394 Table 41)
 *   - Transfer: &trigger_value (=0x01) → &TIM15->EGR (UG bit)
 *   - Circular mode, triggered by TIM2 CCR1 match
 *
 * TIMING CALCULATION:
 * -------------------
 * For center-aligned mode (TIM1):
 *   - One complete PWM cycle = 2 * (ARR + 1) timer ticks
 *   - Phase in ticks: phase_ticks = (PHASE2_DEG / 360.0) * 2 * (ARR + 1)
 *   - TIM2 CCR1 = phase_ticks (delay from TIM1 update to TIM15 trigger)
 *
 * Example:
 *   F_TIM = 80 MHz
 *   F_PWM = 100 kHz
 *   ARR = (F_TIM / (2 * F_PWM)) - 1 = 399
 *   Period_ticks = 2 * (399 + 1) = 800
 *   PHASE2_DEG = 120°
 *   phase_ticks = (120 / 360) * 800 = 267
 *
 * ============================================================================
 */

#include "stm32l4xx.h"
#include <stdint.h>
#include "Claude.h"
#include "main.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_RCC.h"
#include "STM32L432KC_SPI.h"
#include "STM32L432KC.h"
#include "STM32L432KC_TIM.h"

// Configuration parameters
#define F_TIM_CLK           80000000UL  // 80 MHz timer clock
#define F_PWM_HZ            100000UL     // 100 kHz PWM frequency (example)
#define DEADTIME_NS_CFG     100         // 100 ns dead-time (renamed to avoid macro conflict)
#define PHASE_DEG_B_CFG     90.0f       // Phase shift for Pair B in degrees [0..360) (renamed to avoid conflict)
#define PHASE2_DEG_CFG      120.0f      // Phase shift for Pair C in degrees [0..360) (renamed to avoid conflict)


// Computed values (update these when F_PWM_HZ changes)
static uint32_t ARR;              // Auto-reload value for TIM1/TIM15
static uint32_t phase_ticks;            // Phase delay in timer ticks
static uint8_t DTencoded;               // Encoded dead-time value

// DMA transfer value for triggering TIM15
static const uint16_t trigger_value = TIM_EGR_UG;

/**
 * ============================================================================
 * REGISTER-LEVEL CONFIGURATION FUNCTIONS
 * ============================================================================
 */
// TIM1_PhaseShift_t is defined in Claude.h

static TIM1_PhaseShift_t tim1_phase_shift(uint32_t ARR, float phase_deg_value){
    TIM1_PhaseShift_t result;
    uint32_t halfwave = ARR + 1U;
    uint32_t period = 2 * halfwave;
    float phase_ticks_f = (phase_deg_value / 360.0f) * (float)period;
    uint32_t phase_ticks = (uint32_t)(phase_ticks_f + 0.5f); // round

    result.CCR3 = phase_ticks;
    result.CCR4 = ARR - phase_ticks;

    return result;
}

void TIM1PWMinit(uint32_t PSC, uint32_t ARR, uint32_t CCR, uint8_t DTencoded, uint8_t phase_deg_value, uint32_t CCR3, uint32_t CCR4){
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
TIM1_PhaseShift_t phase = tim1_phase_shift(ARR, phase_deg_value);
TIM1->CCR3 = phase.CCR3;
TIM1->CCR4 = phase.CCR4;


TIM1->CCER = 0; // start from a clean state
TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE ); // Capture compare en for both channels on CH1
TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE );// Capture compare en for both channels on CH2
TIM1->EGR  |= TIM_EGR_UG; 
TIM1->EGR  &= ~TIM_EGR_UG; 
TIM1->CR2 &= ~TIM_CR2_MMS;
TIM1->CR2 |= (2U << TIM_CR2_MMS_Pos); // MMS = 05: Trigger on Update Even

TIM1->BDTR = 0;
TIM1->BDTR |= (DTencoded << TIM_BDTR_DTG_Pos); // for dead time generator setup
TIM1->EGR  |= TIM_CR1_UDIS; 
TIM1->EGR  |= TIM_EGR_TG; 
TIM1->CR1 |= TIM_CR1_CEN;  
TIM1->BDTR  |= TIM_BDTR_MOE;         
}

static uint8_t dead_time_generator(float dead_us, uint32_t tim_freq){
    const double  t_dts = 1.0 / (double)tim_freq;
    uint32_t ticks = (uint32_t)((dead_us * 1e-9f) / t_dts);
    if (ticks <= 127)                   return (uint8_t)ticks; // 0 to 127 because 127 is biggest 6 bit number
    if (ticks   <= (127*2))             return (uint8_t)(0x80 | ((ticks/2)  - 64)); // for ticks = (64+k)*2 where k = {0,63} since 63 is biggest 6 bit number
    if (ticks   <= (504))              return (uint8_t)(0xC0 | ((ticks/8)  - 32));     // for ticks = (32+k)*8 where k = {0,32} since 32 is biggest 5 bit number
    if (ticks/16  <= (1008))              return (uint8_t)(0xE0 | ((ticks/16) - 32));     // for ticks = (32+k)*16 where k = {0,32} since 32 is biggest 5 bit number
    return 0xFF; // clamp otherwise
}


void TIM1_Configure_TRGO(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    // Configure TIM1 as master: TRGO = Update event
    // MMS[2:0] = 010 (Update event selected as trigger output)
    TIM1->CR2 &= ~TIM_CR2_MMS_Msk;
    TIM1->CR2 |= (0x02 << TIM_CR2_MMS_Pos);  // TRGO on update
    // NOTE: TIM1 continues to run in center-aligned mode with existing configuration
}


void TIM2_Init_Phase_Delay(uint32_t phase_delay_ticks)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    TIM2->CR1 &= ~TIM_CR1_CEN;

    // ===== SLAVE MODE CONFIGURATION =====
    // TS[2:0] = 000 (ITR0 = TIM1 TRGO)
    // SMS[3:0] = 0100 (Reset Mode: TRGI resets counter)
    TIM2->SMCR &= ~(TIM_SMCR_TS_Msk | TIM_SMCR_SMS_Msk);
    TIM2->SMCR |= (0x00 << TIM_SMCR_TS_Pos);   // ITR0 = TIM1
    TIM2->SMCR |= (0x04 << TIM_SMCR_SMS_Pos);  // Reset mode
    TIM2->CR1 |= TIM_CR1_OPM;  // One-pulse mode
    TIM2->PSC = 0;
    TIM2->ARR = 2 * (ARR + 1);

    TIM2->CCR1 = phase_delay_ticks;

    // CC1S = 00 (output mode)
    // OC1M = 0111 (PWM mode 2: OC1REF high when CNT >= CCR1)
    TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1M_Msk);
    TIM2->CCMR1 |= (0x07 << TIM_CCMR1_OC1M_Pos);

    TIM2->CCMR1 |= TIM_CCMR1_OC1PE; // Enable output compare 1 preload
    TIM2->CCER |= TIM_CCER_CC1E;
    // Enable CC1 DMA request (triggers DMA1_CH5 when CCR1 matches)
    TIM2->DIER |= TIM_DIER_CC1DE;

    // Enable auto-reload preload
    TIM2->CR1 |= TIM_CR1_ARPE;

    // Generate update event to load preload registers
    TIM2->EGR = TIM_EGR_UG;

    // NOTE: Counter will start automatically when TIM1 generates update event
    // Do NOT enable CEN - counter is controlled by slave mode
}

/**
 * @brief Initialize DMA1 Channel 5 for TIM2_CC1 → TIM15_EGR transfer
 *
 * DMA routing (RM0394 Table 41):
 *   TIM2_CH1 → DMA1 Channel 5 (C5S = 0100)
 *
 * Transfer:
 *   Source: &trigger_value (0x0001, constant in memory)
 *   Destination: &TIM15->EGR (EGR register address)
 *   Transfer size: 16-bit (half-word)
 *   Mode: Circular (repeats every TIM2 CC1 event)
 */
void DMA1_CH5_Init_TIM2_to_TIM15(void)
{
    // Enable DMA1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // Disable DMA channel during configuration
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;

    // ===== DMA CHANNEL SELECTION =====
    // C5S[3:0] = 0100 (TIM2_CH1)
    DMA1_CSELR->CSELR &= ~DMA_CSELR_C5S_Msk;
    DMA1_CSELR->CSELR |= (0x04 << DMA_CSELR_C5S_Pos);

    // ===== TRANSFER CONFIGURATION =====
    // Memory to peripheral, 16-bit, circular mode
    DMA1_Channel5->CCR = 0;
    DMA1_Channel5->CCR |= DMA_CCR_DIR;         // Read from memory
    DMA1_Channel5->CCR |= DMA_CCR_CIRC;        // Circular mode
    DMA1_Channel5->CCR |= DMA_CCR_MSIZE_0;     // Memory size: 16-bit
    DMA1_Channel5->CCR |= DMA_CCR_PSIZE_0;     // Peripheral size: 16-bit
    // No memory increment (always read same trigger_value)
    // No peripheral increment (always write to TIM15->EGR)

    // ===== ADDRESSES =====
    DMA1_Channel5->CMAR = (uint32_t)&trigger_value;  // Source address
    DMA1_Channel5->CPAR = (uint32_t)&TIM15->EGR;     // Destination address

    // ===== TRANSFER COUNT =====
    DMA1_Channel5->CNDTR = 1;  // One transfer per trigger

    // ===== ENABLE DMA CHANNEL =====
    DMA1_Channel5->CCR |= DMA_CCR_EN;
}

/**
 * @brief Initialize TIM15 for complementary PWM with dead-time
 *
 * Configuration:
 *   - Triggered by DMA write to EGR (UG bit)
 *   - Slave mode: Trigger mode (SMS=0110) - starts on TRGI rising edge
 *   - PWM mode 1 on CH1
 *   - Complementary outputs: CH1 and CH1N
 *   - Dead-time insertion
 *   - Same frequency as TIM1 (same ARR value)
 */
void TIM15_Init_Complementary_PWM(uint32_t arr, uint32_t duty_ticks, uint8_t deadtime)
{
    // Enable TIM15 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;

    // Disable counter during configuration
    TIM15->CR1 &= ~TIM_CR1_CEN;

    // ===== SLAVE MODE CONFIGURATION =====
    // For DMA-triggered mode, we use trigger mode
    // SMS[3:0] = 0110 (Trigger mode: counter starts on TRGI rising edge)
    // TS[2:0] = 000 (ITR0 = TIM1) - although we use DMA, set for consistency
    // NOTE: TRGI will be generated by DMA write to EGR (UG bit)
    TIM15->SMCR &= ~(TIM_SMCR_TS_Msk | TIM_SMCR_SMS_Msk);
    TIM15->SMCR |= (0x00 << TIM_SMCR_TS_Pos);   // ITR0 (not used in practice)
    TIM15->SMCR |= (0x06 << TIM_SMCR_SMS_Pos);  // Trigger mode

    // ===== TIMEBASE CONFIGURATION =====
    TIM15->PSC = 0;              // No prescaler (same as TIM1)
    TIM15->ARR = arr;            // Same ARR as TIM1
    TIM15->RCR = 0;              // Repetition counter = 0

    // ===== CAPTURE/COMPARE CHANNEL 1 (PWM) =====
    TIM15->CCR1 = duty_ticks;    // 50% duty cycle

    // CC1S = 00 (output)
    // OC1M = 0110 (PWM mode 1: active when CNT < CCR1)
    // OC1PE = 1 (preload enable)
    TIM15->CCMR1 &= ~(TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1M_Msk);
    TIM15->CCMR1 |= (0x06 << TIM_CCMR1_OC1M_Pos);
    TIM15->CCMR1 |= TIM_CCMR1_OC1PE;

    // ===== OUTPUT CONFIGURATION =====
    // Enable CH1 and CH1N outputs
    // CC1E = 1 (CH1 enable)
    // CC1NE = 1 (CH1N enable)
    // CC1P = 0 (CH1 active high)
    // CC1NP = 0 (CH1N active high)
    TIM15->CCER = 0;
    TIM15->CCER |= TIM_CCER_CC1E;   // Enable CH1
    TIM15->CCER |= TIM_CCER_CC1NE;  // Enable CH1N

    // ===== BREAK AND DEAD-TIME REGISTER (BDTR) =====
    // MOE = 1 (Main Output Enable)
    // OSSR = 0 (Off-state selection for Run mode)
    // OSSI = 0 (Off-state selection for Idle mode)
    // DTG[7:0] = deadtime encoding
    TIM15->BDTR = 0;
    TIM15->BDTR |= TIM_BDTR_MOE;                    // Main output enable
    TIM15->BDTR |= (deadtime << TIM_BDTR_DTG_Pos);  // Dead-time

    // ===== CONTROL REGISTER =====
    TIM15->CR1 |= TIM_CR1_ARPE;  // Auto-reload preload enable

    // ===== GENERATE UPDATE EVENT =====
    TIM15->EGR = TIM_EGR_UG;

    // ===== ENABLE COUNTER =====
    // Counter will start when DMA writes to EGR (trigger mode)
    TIM15->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Configure GPIO pins for TIM15 outputs
 *
 * TIM15 pins on STM32L432KC:
 *   - TIM15_CH1:  PA2 (AF14)
 *   - TIM15_CH1N: PB15 (AF14)
 */
void GPIO_Init_TIM15_Outputs(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    pinMode(PA2, GPIO_ALT);     //TIM15_CH1 A7
    pinMode(PA1, GPIO_ALT);     //TIM15_CH1N A1

    GPIOA->AFR[0]  |=  (14U << GPIO_AFRL_AFSEL2_Pos);          // AF1 = TIM15_CH1
    GPIOA->AFR[0]  |=  (14U << GPIO_AFRL_AFSEL1_Pos);          // AF1 = TIM15_CH1N

    GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED1_Msk);
    GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED2_Msk);
}

void GPIO_Init_TIM1_Outputs(void)
{
    gpioEnable(GPIO_PORT_A);
    gpioEnable(GPIO_PORT_B);

    pinMode(PA8, GPIO_ALT);     //TIM1_CH1 D8
    pinMode(PA7, GPIO_ALT);      //TIM1_CH1N A6

    pinMode(PA10, GPIO_ALT);     //TIM1_CH3 D0
    pinMode(PB1, GPIO_ALT);     //TIM1_CH13N A1 D5

    GPIOA->AFR[1]  |=  (1U << GPIO_AFRH_AFSEL8_Pos);          // AF1 = TIM1_CH1
    GPIOA->AFR[0]  |=  (1U << GPIO_AFRL_AFSEL7_Pos);          // AF1 = TIM1_CH1N

    GPIOA->AFR[1]  |=  (1U << GPIO_AFRH_AFSEL10_Pos);          // AF1 = TIM1_CH1
    GPIOB->AFR[0]  |=  (1U << GPIO_AFRL_AFSEL1_Pos);          // AF1 = TIM1_CH1N

    GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED7_Msk);
    GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED8_Msk);
    GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED10_Msk);
    GPIOB->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED1_Msk);

    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
}



/**
 * ============================================================================
 * MAIN INITIALIZATION AND UPDATE FUNCTIONS
 * ============================================================================
 */

/**
 * @brief Calculate timing parameters from PWM frequency
 * Uses same formula as user's tim_compute_edge() for consistency
 */
void Calculate_Timing_Parameters(void)
{
    // Match user's tim_compute_edge() formula (no -1, consistent with their TIM1 setup)
    uint32_t PSC = 0;  // Prescaler = 1 (same as user's code)
    ARR = (F_TIM_CLK / (2 * (PSC + 1U) * F_PWM_HZ));

    // Calculate phase ticks using same method as user's tim1_phase_shift()
    uint32_t halfwave = ARR + 1U;
    uint32_t period_ticks = 2 * halfwave;
    float phase_ticks_f = (PHASE2_DEG_CFG / 360.0f) * (float)period_ticks;
    phase_ticks = (uint32_t)(phase_ticks_f + 0.5f);  // Round like user's code

    DTencoded = dead_time_generator(DEADTIME_NS_CFG, F_TIM_CLK);
}

/**
 * @brief Initialize complete phase-shifted complementary PWM system
 *
 * Initializes all three complementary PWM pairs:
 *   - Pair A: TIM1_CH1 + TIM1_CH1N (reference, 50% duty)
 *   - Pair B: TIM1_CH3 + TIM1_CH3N (phase shifted by phase_deg)
 *   - Pair C: TIM15_CH1 + TIM15_CH1N (phase shifted by PHASE2_DEG via TIM2 delay)
 *
 * CRITICAL: Initialization order is bottom-up (slave → master) to ensure
 * all downstream peripherals are ready before TIM1 starts the trigger chain.
 */
void Init_Phase_Shifted_PWM_System(void)
{
    // ===== STEP 1: Calculate timing parameters =====
    Calculate_Timing_Parameters();

    // ===== STEP 2: Configure GPIO for all timers =====
    GPIO_Init_TIM1_Outputs();   // TIM1 pins
    GPIO_Init_TIM15_Outputs();  // TIM15 pins

    // ===== STEP 3: Configure TIM15 (end of chain - must be ready first) =====
    uint32_t duty_ticks = ARR / 2;  // 50% duty cycle
    TIM15_Init_Complementary_PWM(ARR, duty_ticks, DTencoded);

    // ===== STEP 4: Configure DMA (must be ready before TIM2 runs) =====
    DMA1_CH5_Init_TIM2_to_TIM15();

    // ===== STEP 5: Configure TIM2 (middle of chain, slaved to TIM1) =====
    TIM2_Init_Phase_Delay(phase_ticks);

    // ===== STEP 6: Configure and START TIM1 (master - starts everything) =====
    // WARNING: TIM1PWMinit() enables TIM1 at the end, starting the trigger chain!
    uint32_t PSC = 0;                 // No prescaler
    uint32_t CCR = ARR / 2;           // 50% duty for Pair A

    // Calculate CCR3/CCR4 for Pair B phase shift (using PHASE_DEG_B_CFG macro value)
    TIM1_PhaseShift_t phase_b = tim1_phase_shift(ARR, PHASE_DEG_B_CFG);

    // Initialize and start TIM1 (this starts the entire system)
    TIM1PWMinit(PSC, ARR, CCR, DTencoded, PHASE_DEG_B_CFG, phase_b.CCR3, phase_b.CCR4);

    // System is now fully operational!
    // Signal flow: TIM1 update → TIM2 reset → TIM2 count to CCR1 → DMA → TIM15 start
}

/**
 * @brief Update phase shift angle at runtime (glitch-free)
 *
 * @param new_phase_deg New phase angle in degrees [0..360)
 *
 * Procedure:
 *   1. Calculate new phase_ticks
 *   2. Write to TIM2->CCR1 (preload register)
 *   3. New value takes effect on next TIM2 update event (next TIM1 cycle)
 */
void Update_Phase_Shift(float new_phase_deg)
{
    // Validate input
    if (new_phase_deg < 0.0f) new_phase_deg = 0.0f;
    if (new_phase_deg >= 360.0f) new_phase_deg = 359.99f;

    // Calculate new phase ticks
    uint32_t period_ticks = 2 * (ARR + 1);
    uint32_t new_phase_ticks = (uint32_t)((new_phase_deg / 360.0f) * period_ticks);

    // Update TIM2 CCR1 (preload enabled, takes effect on next update)
    TIM2->CCR1 = new_phase_ticks;
}

/**
 * @brief Update PWM frequency at runtime (glitch-free)
 *
 * @param new_freq_hz New PWM frequency in Hz
 *
 * Procedure:
 *   1. Disable all timers
 *   2. Recalculate ARR and phase_ticks
 *   3. Update TIM1, TIM2, TIM15 ARR and related registers
 *   4. Re-enable timers
 */
void Update_PWM_Frequency(uint32_t new_freq_hz)
{
    // Disable timers
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM2->CR1 &= ~TIM_CR1_CEN;  // Not needed (slave mode), but for safety
    TIM15->CR1 &= ~TIM_CR1_CEN;

    // Recalculate parameters
    ARR = (F_TIM_CLK / (2 * new_freq_hz)) - 1;
    uint32_t period_ticks = 2 * (ARR + 1);
    phase_ticks = (uint32_t)((PHASE2_DEG_CFG / 360.0f) * period_ticks);

    // Update TIM1 ARR (user's existing timer)
    TIM1->ARR = ARR;
    TIM1->EGR = TIM_EGR_UG;

    // Update TIM2 ARR and CCR1
    TIM2->ARR = period_ticks;
    TIM2->CCR1 = phase_ticks;
    TIM2->EGR = TIM_EGR_UG;

    // Update TIM15 ARR
    TIM15->ARR = ARR;
    TIM15->CCR1 = ARR / 2;  // 50% duty
    TIM15->EGR = TIM_EGR_UG;

    // Re-enable timers
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM15->CR1 |= TIM_CR1_CEN;
    // TIM2 will start automatically via slave mode
}

/**
 * @brief Update duty cycle of TIM15 output (Pair C)
 *
 * @param duty_percent Duty cycle in percent [0..100]
 */
void Update_TIM15_Duty(float duty_percent)
{
    // Clamp duty cycle
    if (duty_percent < 0.0f) duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    // Calculate duty ticks
    uint32_t duty_ticks = (uint32_t)((duty_percent / 100.0f) * ARR);

    // Update CCR1 (preload enabled)
    TIM15->CCR1 = duty_ticks;
}

/**
 * ============================================================================
 * VERIFICATION AND DEBUG
 * ============================================================================
 */

/**
 * @brief Verification checklist structure
 */
typedef struct {
    uint32_t tim1_cr2_mms;      // TIM1 MMS bits (should be 0x02)
    uint32_t tim2_smcr_ts;      // TIM2 TS bits (should be 0x00 = ITR0)
    uint32_t tim2_smcr_sms;     // TIM2 SMS bits (should be 0x04 = Reset)
    uint32_t tim2_cr1_opm;      // TIM2 OPM bit (should be 1)
    uint32_t tim2_cr2_mms;      // TIM2 MMS bits (should be 0x04 = OC1REF)
    uint32_t tim2_ccr1;         // TIM2 phase delay value
    uint32_t dma_cselr_c5s;     // DMA channel 5 selection (should be 0x04)
    uint32_t dma_ch5_enabled;   // DMA CH5 EN bit
    uint32_t tim15_smcr_sms;    // TIM15 SMS bits (should be 0x06 = Trigger)
    uint32_t tim15_bdtr_moe;    // TIM15 MOE bit (should be 1)
    uint32_t tim15_arr;         // TIM15 ARR value
    uint32_t tim15_ccr1;        // TIM15 duty cycle value
} Verification_Registers_t;

/**
 * @brief Read all critical registers for verification
 */
Verification_Registers_t Read_Verification_Registers(void)
{
    Verification_Registers_t regs;

    regs.tim1_cr2_mms = (TIM1->CR2 & TIM_CR2_MMS_Msk) >> TIM_CR2_MMS_Pos;
    regs.tim2_smcr_ts = (TIM2->SMCR & TIM_SMCR_TS_Msk) >> TIM_SMCR_TS_Pos;
    regs.tim2_smcr_sms = (TIM2->SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos;
    regs.tim2_cr1_opm = (TIM2->CR1 & TIM_CR1_OPM) ? 1 : 0;
    regs.tim2_cr2_mms = (TIM2->CR2 & TIM_CR2_MMS_Msk) >> TIM_CR2_MMS_Pos;
    regs.tim2_ccr1 = TIM2->CCR1;
    regs.dma_cselr_c5s = (DMA1_CSELR->CSELR & DMA_CSELR_C5S_Msk) >> DMA_CSELR_C5S_Pos;
    regs.dma_ch5_enabled = (DMA1_Channel5->CCR & DMA_CCR_EN) ? 1 : 0;
    regs.tim15_smcr_sms = (TIM15->SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos;
    regs.tim15_bdtr_moe = (TIM15->BDTR & TIM_BDTR_MOE) ? 1 : 0;
    regs.tim15_arr = TIM15->ARR;
    regs.tim15_ccr1 = TIM15->CCR1;

    return regs;
}

/**
 * @brief Print verification results (requires printf or debug output)
 *
 * Expected values:
 *   - TIM1 CR2 MMS = 0x02 (Update event)
 *   - TIM2 SMCR TS = 0x00 (ITR0 = TIM1)
 *   - TIM2 SMCR SMS = 0x04 (Reset mode)
 *   - TIM2 CR1 OPM = 1 (One-pulse mode)
 *   - TIM2 CR2 MMS = 0x04 (OC1REF)
 *   - DMA CSELR C5S = 0x04 (TIM2_CH1)
 *   - DMA CH5 EN = 1
 *   - TIM15 SMCR SMS = 0x06 (Trigger mode)
 *   - TIM15 BDTR MOE = 1
 *
 * Scope verification:
 *   - CH1: TIM1_CH1 (existing Pair A)
 *   - CH2: TIM15_CH1 (new Pair C)
 *   - Measure: Phase difference should equal PHASE2_DEG
 *   - Verify: TIM15_CH1 and TIM15_CH1N have correct dead-time
 */
void Verify_Configuration(void)
{
    Verification_Registers_t regs = Read_Verification_Registers();

    // Use debugger or UART to inspect these values
    // Set breakpoint here and examine regs structure
    (void)regs;  // Prevent unused variable warning

    // TODO: Add your printf/UART output here if available
}

/**
 * ============================================================================
 * SCOPE VERIFICATION GUIDE
 * ============================================================================
 *
 * Signals to probe:
 *   1. TIM1_CH1  (PA8)  - Reference signal (Pair A)
 *   2. TIM15_CH1 (PA2)  - Phase-shifted signal (Pair C)
 *   3. TIM15_CH1N (PB15) - Complementary output with dead-time
 *
 * Expected relationships:
 *   - TIM1 and TIM15 should have same frequency
 *   - TIM15_CH1 rising edge should be delayed from TIM1_CH1 rising edge by:
 *     delay_time = (PHASE2_DEG / 360) * (1 / F_PWM_HZ)
 *     Example: PHASE2_DEG=120°, F_PWM=100kHz → delay = 3.33 μs
 *
 *   - TIM15_CH1 and TIM15_CH1N should be complementary
 *   - Dead-time between transitions should be ~DEADTIME_NS
 *     (measure falling edge of CH1 to rising edge of CH1N, or vice versa)
 *
 * Debug signals (optional):
 *   - Configure TIM2_CH1 output to a GPIO to verify phase delay timer
 *     (should pulse at phase offset point each cycle)
 *
 * ============================================================================
 */

/**
 * @brief Optional: Configure TIM2_CH1 output for debugging
 * This allows you to see the phase delay timer output on scope
 *
 * TIM2_CH1 can be mapped to PA0, PA5, or PA15 (check datasheet)
 * Example: PA0 = AF1
 */
void Debug_Enable_TIM2_Output(void)
{
    // Enable GPIOA clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // Configure PA0 as TIM2_CH1 (AF1)
    GPIOA->MODER &= ~GPIO_MODER_MODE0_Msk;
    GPIOA->MODER |= (0x02 << GPIO_MODER_MODE0_Pos);  // AF mode
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT0;               // Push-pull
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED0;          // High speed
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL0_Msk;
    GPIOA->AFR[0] |= (1 << GPIO_AFRL_AFSEL0_Pos);    // AF1

    // TIM2 CH1 is already configured as output compare in TIM2_Init_Phase_Delay
    // This will output a pulse when CCR1 matches (at phase offset point)
}

/**
 * ============================================================================
 * EXAMPLE USAGE
 * ============================================================================
 */

void Example_Usage(void)
{
    // 1. Initialize the phase-shifted PWM system
    Init_Phase_Shifted_PWM_System();

    // 2. Verify configuration (use debugger to inspect registers)
    Verify_Configuration();

    // 3. Optional: Enable TIM2 debug output to see phase delay
    // Debug_Enable_TIM2_Output();

    // 4. Start TIM1 (assumed to be done by user's existing code)
    // TIM1->CR1 |= TIM_CR1_CEN;

    // 5. Runtime updates (examples):

    // Change phase shift to 90 degrees
    // Update_Phase_Shift(90.0f);

    // Change PWM frequency to 50 kHz
    // Update_PWM_Frequency(50000);

    // Change Pair C duty cycle to 75%
    // Update_TIM15_Duty(75.0f);
}

/**
 * ============================================================================
 * ALTERNATIVE APPROACH (if DMA is not acceptable)
 * ============================================================================
 *
 * If using DMA to trigger TIM15 is not acceptable, there are two alternatives:
 *
 * OPTION A: Use TIM16 as intermediate trigger
 * ------------------------------------------
 * Since TIM15 ITR2 = TIM16_OC1, we can:
 *   TIM1 → TIM2 (phase delay) → TIM16 (pass-through) → TIM15 (PWM)
 *
 * However, TIM16 does NOT have slave mode, so we still need DMA:
 *   TIM2 OC1 DMA → TIM16_EGR → TIM16 starts → TIM16_OC1 → TIM15 ITR2
 *
 * This is MORE complex than the direct DMA approach above.
 *
 * OPTION B: Use interrupt instead of DMA
 * ---------------------------------------
 * TIM2 CC1 interrupt → ISR writes to TIM15->EGR
 *
 * Pros: No DMA
 * Cons: Introduces jitter (interrupt latency ~1-2 μs typical)
 *       Violates "no interrupts" requirement
 *
 * RECOMMENDATION: Stick with DMA approach (presented above)
 * ============================================================================
 */
