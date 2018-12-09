/*******************************************************************************/
/**
*      @file : timer32.h
*   @version : 0
*      @date : December 9, 2018
*    @author : Enzo IGLESIS
* @copyright : Copyright (c) 2018 Enzo IGLESIS, MIT License (MIT)
*******************************************************************************/

/*******************************     BRIEF      *******************************/
/**
*
* 1. Introduction
* The Timer32 is an AMBA compliant peripheral developed, tested, and licensed by
* ARM Limited. The Timer32 consists of two programmable 32-bit or 16-bit down
* counters that can generate interrupts on reaching zero.
* The key features of Timer32 include:
* - Two independent counters each configurable as 32-bit or 16-bit counter size
* - Three different timer modes supported for each counter
* - Prescale unit to divide the input clock by 1, 16 or 256
* - Independent Interrupts from each of the counter, as well as, a combined
* interrupt from both the counters
*
* 2. Modes
*   - `Free running`: The counter wraps after reaching its zero value, and
*   continues to count down from the maximum value.
*   - `Periodic timer`: The counter generates an interrupt at a constant
*   interval, reloading the original value after wrapping past zero.
*   - `One-shot timer`: The counter generates an interrupt once. When the
*   counter reaches zero, it halts until reprogrammed by the user.
*******************************************************************************/

#ifndef TIMER32_H_
#define TIMER32_H_

/*******************************    LIBRARYS    *******************************/
#include "msp432p401r.h"

/*******************************     MACROS     *******************************/

/**** Timer 32 ****/
/**
TIMER32_1: timer 32 (1) (Timer32_Type *)
TIMER32_2: timer 32 (2) (Timer32_Type *)
**/
    /**** Free running****/
        /**** Size ****/
#define TIMER32_SIZE_16     0
#define TIMER32_SIZE_32     TIMER32_CONTROL_SIZE
        /**** Prescale ****/
#define TIMER32_PRESCALE_1  TIMER32_CONTROL_PRESCALE_0
#define TIMER32_PRESCALE_16 TIMER32_CONTROL_PRESCALE_1
#define TIMER32_PRESCALE_32 TIMER32_CONTROL_PRESCALE_2

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
typedef struct timer32_t
{
    /**** varaibles ****/
    Timer32_Type * timer32;
    /**** functions ****/
        /**** Configuration ****/
    int (*free_running) (struct timer32_t * this, uint8_t size, uint16_t prescale);
    int (*periodic_timer) (struct timer32_t * this, uint64_t period_us);
    int (*one_shot_timer) (struct timer32_t * this, uint64_t period_us);
        /**** Manipulation ****/
    void (*start) (struct timer32_t * this);
    void (*pause) (struct timer32_t * this);
    uint32_t (*value) (struct timer32_t * this);
    void (*load) (struct timer32_t * this, uint32_t value);
    int (*period_us) (struct timer32_t * this, uint64_t period_us);
        /**** interrupt ****/
    int (*enable_interrupt) (struct timer32_t * this, void (*interrupt_handler)(void), uint32_t priority);
    int (*disable_interrupt) (struct timer32_t * this);
} timer32_t;

/*******************************   FUNCTIONS    *******************************/
/**** Timer 32 ****/
    /**** Initialization ****/
void timer32_initialize(timer32_t * timer, Timer32_Type * timer32);
    /**** Configuration ****/
int timer32_free_running(timer32_t * this, uint8_t size, uint16_t prescale);
int timer32_periodic_timer(timer32_t * this, uint64_t period_us);
int timer32_one_shot_timer(timer32_t * this, uint64_t period_us);
    /**** Manipulation ****/
void timer32_start(timer32_t * this);
void timer32_pause(timer32_t * this);
uint32_t timer32_value(timer32_t * this);
void timer32_load(timer32_t * this, uint32_t value);
int timer32_period_us(timer32_t * this, uint64_t period_us);
    /**** interrupt ****/
int timer32_disable_interrupt(timer32_t * this);
int timer32_enable_interrupt(timer32_t * this, void (*interrupt_handler)(void), uint32_t priority);

#endif /* TIMER32_H_ */
