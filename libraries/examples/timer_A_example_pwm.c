/******************************************************************************/
/**
*      @file : timer_A_example_pwm.c
*   @version : 0
*      @date : December 16, 2018
*    @author : Enzo IGLESIS
* @copyright : Copyright (c) 2018 Enzo IGLESIS, MIT License (MIT)
*******************************************************************************/

/******************************************************************************/
/**
*   MSP432P401R Demo
*
*   Description: drive a HS-311 servo motor
*
*                   MSP432P401R
*                -----------------
*            /|\|                 |
*             | |                 |
*             --|RST              |
*               |                 |
*          S1-->|P1.1         P1.0|-->LED1
*         |     |                 |
*        ///    |             P2.4|-->HS_311_PWM
*
*******************************************************************************/


/*******************************   LIBRARIES    *******************************/
#include "msp432p401r.h"
#include "timer_A.h"
#include "gpio.h"

/*******************************     MACROS     *******************************/
#define HS_311_PERIOD       20000 /* µs (20 ms) */
#define HS_311_0_DEG_US     600  /* µs (1.25 ms) */
#define HS_311_180_DEG_US   2300  /* µs (1.75 ms) */

/*******************************   VARIABLES    *******************************/
static timer_A_t hs_311; /* global declaration for interrupt use */
static gpio_t led1; /* global declaration for interrupt use */

/*******************************   FUNCTIONS    *******************************/

static inline void interrupt_s1(void)
{
    /* local declaration */
    static uint32_t hs_311_status = HS_311_0_DEG_US;
    /* program statement */
    hs_311_status = (hs_311_status == HS_311_0_DEG_US)? HS_311_180_DEG_US : HS_311_0_DEG_US;
    led1.toggle(&led1);
    hs_311.pwm_time_on(&hs_311, hs_311_status);
}

/*******************************      MAIN      *******************************/
int main(void)
{
    /* local declaration */
    static gpio_t s1;
    /* Initialization */
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; /* stop watchdog timer */
        /* led1 */
    gpio_initialize_odd(&led1, P1, PIN0); /* associate led1 to P1.0 */
    led1.output(&led1); /* configure led1 pin as output */
    led1.clear(&led1);
        /* s1 */
    gpio_initialize_odd(&s1, P1, PIN1); /* associate s1 to P1.1 */
    s1.input_pullup(&s1); /* config s1 as input with pullup resistor */
    s1.enable_interrupt_falling_edge(&s1, interrupt_s1, 35);/* enable
    interrupt on s1 button for falling edge and associate to interrupt_s1
    function with a priority of 35 */
        /* TIMER A0 PWM */
    timer_A_initialize(&hs_311, TIMER_A0);  /* associate hs_311 to TIMER_A0 */
    hs_311.pwm(&hs_311, 1, HS_311_PERIOD, HS_311_0_DEG_US);
        /* system */
    __enable_interrupts();
    /* program statement */
    hs_311.start(&hs_311); /* start the timer */
    while(!0) /* infinite loop */
    {
        ; /* NOP */
    }
}
