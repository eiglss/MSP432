/******************************************************************************/
/**
*      @file : timer_32_example.c
*   @version : 0
*      @date : December 9, 2018
*    @author : Enzo IGLESIS
* @copyright : Copyright (c) 2018 Enzo IGLESIS, MIT License (MIT)
*******************************************************************************/

/******************************************************************************/
/**
*   MSP432P401R Demo
*
*   Description: Toggle the led (P1.0) every second.
*
*                   MSP432P401R
*                -----------------
*            /|\|                 |
*             | |                 |
*             --|RST              |
*               |                 |
*               |             P1.0|-->RED LED
*               |                 |
*
*******************************************************************************/


/*******************************   LIBRARIES    *******************************/
#include "msp432p401r.h"
#include "timer32.h"
#include "gpio.h"

/*******************************     MACROS     *******************************/

/*******************************   VARIABLES    *******************************/
static gpio_t led_red; /* global variable for interrupt use */
/*******************************   FUNCTIONS    *******************************/

/******************************************************************************/
/**
* @biref    Interrupt handler for the timer_1s. Toggle the led status.
*
* @param    None.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
static inline void blink_led(void)
{
    led_red.toggle(&led_red); /* toggle led status */
}

/*******************************      MAIN      *******************************/
int main(void)
{
    /* local declaration */
    timer32_t timer_1s;
    /* Initialization */
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; /* stop watchdog timer */
        /* GPIO */
    gpio_initialize_odd(&led_red, P1, PIN0); /* associate led_red to P1.0 */
    led_red.output(&led_red); /* configure led_red pin as output */
    led_red.clear(&led_red);
        /* TIMER 32 */
    timer32_initialize(&timer_1s, TIMER32_1);  /* associate timer_1s to TIMER32_1 */
    timer_1s.periodic_timer(&timer_1s, s_to_us(1)); /* configure timer_1s as
    periodic timer, with a period of 1 second */
    timer_1s.enable_interrupt(&timer_1s, blink_led, 25);
    __enable_interrupts(); /* enable interrupts globally */
    /* program statement */
    timer_1s.start(&timer_1s); /* start the timer */
    while(!0) /* infinite loop */
    {
        ; /* NOP */
    }
}
