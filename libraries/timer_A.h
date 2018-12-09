/******************************************************************************/
/**
*      @file : timer_A.h
*   @version : 0
*      @date : December 9, 2018
*    @author : Enzo IGLESIS
* @copyright : Copyright (c) 2018 Enzo IGLESIS, MIT License (MIT)
*******************************************************************************/

/*******************************     BRIEF      *******************************/
/**
*
* 1. Introduction
* Timer_A is a 16-bit timer/counter with up to seven capture/compare registers.
* Timer_A can support multiple capture/compares, PWM outputs, and interval
* timing. Timer_A also has extensive interrupt capabilities. Interrupts may be
* generated from the counter on overflow conditions and from each of the
* capture/compare registers. Timer_A features include:
*   - Asynchronous 16-bit timer/counter with four operating modes
*   - Selectable and configurable clock source
*   - Up to seven configurable capture/compare registers
*   - Configurable outputs with pulse width modulation (PWM) capability
*   - Asynchronous input and output latching
*
* 2. Modes
*   - `Stop`: The timer is halted.
*   - `Up`: The timer repeatedly counts from zero to the loaded value.
*   - `Continuous`: The timer repeatedly counts from zero to 0FFFFh.
*   - `Up/down`: The timer repeatedly counts from zero up to the value loaded
*                and back down to zero.
*
*******************************************************************************/

#ifndef TIMER_A_H_
#define TIMER_A_H_

/*******************************    LIBRARYS    *******************************/
#include "msp432p401r.h"

/*******************************     MACROS     *******************************/

/**** Timer A ****/
/**
TIMER_A0: timer A (0) (Timer_A_Type *)
TIMER_A1: timer A (1) (Timer_A_Type *)
TIMER_A2: timer A (2) (Timer_A_Type *)
TIMER_A3: timer A (3) (Timer_A_Type *)
**/
    /**** divider ****/
#define TIMER_A_DIVIDER_1   TIMER_A_CTL_ID__1
#define TIMER_A_DIVIDER_2   TIMER_A_CTL_ID__2
#define TIMER_A_DIVIDER_4   TIMER_A_CTL_ID__4
#define TIMER_A_DIVIDER_8   TIMER_A_CTL_ID__8
    /**** divider extended ****/
#define TIMER_A_DIVIDER_EXTENDED_1  TIMER_A_EX0_IDEX__1
#define TIMER_A_DIVIDER_EXTENDED_2  TIMER_A_EX0_IDEX__2
#define TIMER_A_DIVIDER_EXTENDED_3  TIMER_A_EX0_IDEX__3
#define TIMER_A_DIVIDER_EXTENDED_4  TIMER_A_EX0_IDEX__4
#define TIMER_A_DIVIDER_EXTENDED_5  TIMER_A_EX0_IDEX__5
#define TIMER_A_DIVIDER_EXTENDED_6  TIMER_A_EX0_IDEX__6
#define TIMER_A_DIVIDER_EXTENDED_7  TIMER_A_EX0_IDEX__7
#define TIMER_A_DIVIDER_EXTENDED_8  TIMER_A_EX0_IDEX__8
    /**** edge ****/
#define TIMER_A_CAP_RISING_EDGE     TIMER_A_CCTLN_CM_1
#define TIMER_A_CAP_FALLING_EDGE    TIMER_A_CCTLN_CM_2
#define TIMER_A_CAP_BOTH_EDGES      TIMER_A_CCTLN_CM_3
    /**** input signal ****/
#define TIMER_A_INPUT_CCIA          TIMER_A_CCTLN_CCIS__CCIA
#define TIMER_A_INPUT_CCIB          TIMER_A_CCTLN_CCIS__CCIB
#define TIMER_A_INPUT_GND           TIMER_A_CCTLN_CCIS__GND
#define TIMER_A_INPUT_VCC           TIMER_A_CCTLN_CCIS__VCC
    /**** sync ****/
#define TIMER_A_ASYNC_CAP           0
#define TIMER_A_SYNC_CAP            TIMER_A_CCTLN_SCS
    /**** output mode ****/
#define TIMER_A_OUT_BIT             TIMER_A_CCTLN_OUTMOD_0 /* OUT bit value */
#define TIMER_A_OUT_SET             TIMER_A_CCTLN_OUTMOD_1 /* Set */
#define TIMER_A_OUT_TOGGLE_RESET    TIMER_A_CCTLN_OUTMOD_2 /* Toggle/reset */
#define TIMER_A_OUT_SET_RESET       TIMER_A_CCTLN_OUTMOD_3 /* Set/reset */
#define TIMER_A_OUT_TOGGLE          TIMER_A_CCTLN_OUTMOD_4 /* Toggle */
#define TIMER_A_OUT_RESET           TIMER_A_CCTLN_OUTMOD_5 /* Reset */
#define TIMER_A_OUT_TOGGLE_SET      TIMER_A_CCTLN_OUTMOD_6 /* Toggle/set */
#define TIMER_A_OUT_RESET_SET       TIMER_A_CCTLN_OUTMOD_7 /* Reset/set */

#ifndef freq_to_us
#   define freq_to_us(freq) ((uint64_t)(1000000/(freq)))
#endif

#ifndef ms_to_us
#   define ms_to_us(ms) ((uint64_t)(ms*1000))
#endif

#ifndef s_to_us
#   define s_to_us(s)  ((uint64_t)(s*1000000))
#endif

/*******************************     TYPES      *******************************/
typedef struct timer_A_t
{
    /**** variables ****/
        /**** timer ****/
    Timer_A_Type * timer_A;
        /**** attributes ****/
    uint32_t period_us;
    uint32_t mode_control;
    /**** functions ****/
        /**** Main modes ****/
    int (* up_mode) (struct timer_A_t * this, uint32_t period_us);
    int (* continuous_mode) (struct timer_A_t * this, uint8_t divider, uint8_t divider_extended);
    int (* updown_mode) (struct timer_A_t * this, uint32_t period_us);
        /**** Secondary modes ****/
    int (* capture_mode) (struct timer_A_t * this, uint8_t channel, uint8_t edge, uint8_t input_signal, uint8_t sync);
    int (* compare_mode) (struct timer_A_t * this, uint8_t channel);
        /**** Output ****/
    int (* set_output_mode) (struct timer_A_t * this, uint8_t channel, uint8_t output_mode);
    int (* set_channel_event_us) (struct timer_A_t * this, uint8_t channel, uint32_t event_us);
    int (* set_channel_event_percent) (struct timer_A_t * this, uint8_t channel, float percent_of_period);
    int (* out_bit_set) (struct timer_A_t * this, uint8_t channel);
    int (* out_bit_clear) (struct timer_A_t * this, uint8_t channel);
    int (* out_bit_toggle) (struct timer_A_t * this, uint8_t channel);
    int (* out_bit) (struct timer_A_t * this, uint8_t channel, uint8_t state);
    void (* pause) (struct timer_A_t * this);
    void (* start) (struct timer_A_t * this);
    uint16_t (* raw_value) (struct timer_A_t * this);
    uint32_t (* value_us) (struct timer_A_t * this);
    float (* value_percent) (struct timer_A_t * this);
    int (* set_period_us) (struct timer_A_t * this, uint32_t period_us);
        /****  Interrupt ****/
    int (* enable_interrupt) (struct timer_A_t * this, void (*interrupt_handler)(void), uint32_t priority);
    int (* enable_capture_compare_interrupt) (struct timer_A_t * this, uint8_t channel, void (*interrupt_handler)(void), uint32_t priority);
    int (* disable_interrupt) (struct timer_A_t * this);
    int (* disable_capture_compare_interrupt) (struct timer_A_t * this, uint8_t channel);
}timer_A_t;

/*******************************   FUNCTIONS    *******************************/
/**** Timer A ****/
    /**** Initialization ****/
void timer_A_initialize(timer_A_t * timer, Timer_A_Type * timer_A);
    /**** Main modes ****/
int timer_A_up_mode(timer_A_t * this, uint32_t period_us);
int timer_A_continuous_mode(timer_A_t * this, uint8_t divider, uint8_t divider_extended);
int timer_A_updown_mode(timer_A_t * this, uint32_t period_us);
    /**** Secondary modes ****/
int timer_A_capture_mode(timer_A_t * this, uint8_t channel, uint8_t edge, uint8_t input_signal, uint8_t sync);
int timer_A_compare_mode(timer_A_t * this, uint8_t channel);
    /**** Output ****/
int timer_A_set_output_mode(timer_A_t * this, uint8_t channel, uint8_t output_mode);
int timer_A_set_channel_event_us(timer_A_t * this, uint8_t channel, uint32_t event_us);
int timer_A_set_channel_event_percent(timer_A_t * this, uint8_t channel, float percent_of_period);
int timer_A_out_bit_set(timer_A_t * this, uint8_t channel);
int timer_A_out_bit_clear(timer_A_t * this, uint8_t channel);
int timer_A_out_bit_toggle(timer_A_t * this, uint8_t channel);
int timer_A_out_bit(timer_A_t * this, uint8_t channel, uint8_t state);
void timer_A_pause(timer_A_t * this);
void timer_A_start(timer_A_t * this);
uint16_t timer_A_raw_value(timer_A_t * this);
uint32_t timer_A_value_us(timer_A_t * this);
float timer_A_value_percent(timer_A_t * this);
int timer_A_set_period_us(timer_A_t * this, uint32_t period_us);
    /**** Interrupt ****/
int timer_A_enable_interrupt(timer_A_t * this, void (*interrupt_handler)(void), uint32_t priority);
int timer_A_enable_capture_compare_interrupt(timer_A_t * this, uint8_t channel, void (*interrupt_handler)(void), uint32_t priority);
int timer_A_disable_interrupt(timer_A_t * this);
int timer_A_disable_capture_compare_interrupt(timer_A_t * this, uint8_t channel);

#endif /* TIMER_A_H_ */
