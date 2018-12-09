/******************************************************************************/
/**
*      @file : gpio_example.c
*   @version : 0
*      @date : December 8, 2018
*    @author : Enzo IGLESIS
* @copyright : Copyright (c) 2018 Enzo IGLESIS, MIT License (MIT)
*******************************************************************************/

/******************************************************************************/
/**
*   MSP432P401R Demo
*
*   Description: change RGB light status when push button 1 is pressed.
*
*                   MSP432P401R
*                -----------------
*            /|\|                 |
*             | |                 |
*             --|RST              |
*               |                 |
*     BUTTON1-->|P1.1         P2.0|-->RED LED
*         |     |             P2.1|-->RED GREEN
*        ///    |             P2.1|-->RED BLUE
*
*******************************************************************************/


/*******************************   LIBRARIES    *******************************/
#include "msp432p401r.h"
#include "gpio.h"

/*******************************     MACROS     *******************************/

/*******************************   VARIABLES    *******************************/
static gpio_t rgb_r, rgb_g, rgb_b; /* declared as global for interrupt use */

/*******************************   FUNCTIONS    *******************************/

/******************************************************************************/
/**
* @biref    Control red green and blue led
*
* @param    red is the status of the red led (0: OFF, 1: ON)
* @param    green is the status of the green led (0: OFF, 1: ON)
* @param    blue is the status of the blue led (0: OFF, 1: ON)
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if(red)
    {
        rgb_r.set(&rgb_r);
    }
    else
    {
        rgb_r.clear(&rgb_r);
    }
    if(green)
    {
        rgb_r.set(&rgb_g);
    }
    else
    {
        rgb_r.clear(&rgb_g);
    }
    if(blue)
    {
        rgb_r.set(&rgb_b);
    }
    else
    {
        rgb_r.clear(&rgb_b);
    }
}

/******************************************************************************/
/**
* @biref    Interrupt handler for the push1 button. Change the rgb light status
*           when the push1 button is pushed.
*
* @param    None.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
static inline void interrupt_push_button(void)
{
    /* local declaration */
    static uint8_t led_status = BIT0;
    /* program statement */
    rgb(led_status&BIT0, led_status&BIT1, led_status&BIT2);
    led_status <<= 1;
    if(led_status&BIT3)
    {
        led_status = BIT0;
    }
}

/*******************************      MAIN      *******************************/
int main(void)
{
    /* local declaration */
    gpio_t push1;
    /* Initialization */
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; /* stop watchdog timer */
    gpio_initialize_even(&rgb_r, P2, PIN0); /* associate rgb_r to P2.0 */
    gpio_initialize_even(&rgb_g, P2, PIN1); /* associate rgb_g to P2.1 */
    gpio_initialize_even(&rgb_b, P2, PIN2); /* associate rgb_b to P2.2 */
    gpio_initialize_odd(&push1, P1, PIN1); /* associate push1 to P1.1 */
    rgb_r.output(&rgb_r); /* configure rgb_r pin as output */
    rgb_g.output(&rgb_g); /* configure rgb_g pin as output */
    rgb_b.output(&rgb_b); /* configure rgb_b pin as output */
    rgb(1,0,0); /* turn RGB light to red */
    push1.input_pullup(&push1); /* config push1 as input with pullup resistor */
    push1.enable_interrupt_falling_edge_odd(&push1, interrupt_push_button, 35);/*
    enable interrupt on push1 button for falling edge and associate to
    interrupt_push_button function with a priority of 35 */
    __enable_interrupts(); /* enable interrupts globally */
    /* program statement */
    while(!0) /* infinite loop */
    {
        ; /* NOP */
    }
}
