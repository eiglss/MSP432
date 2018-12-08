/******************************************************************************/
/**
*      @file : main.c
*   @version : 0
*      @date : December 8, 2018
*    @author : Enzo IGLESIS
* @copyright : Copyright (c) 2018 Enzo IGLESIS, MIT License (MIT)
*******************************************************************************/

/*******************************   LIBRARIES    *******************************/
#include "msp432p401r.h"

/*******************************     MACROS     *******************************/

/*******************************   VARIABLES    *******************************/

/*******************************   FUNCTIONS    *******************************/

/*******************************      MAIN      *******************************/
int main(void)
{
    /* local declaration */
    /* Initialization */
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; /* stop watchdog timer */
    /* program statement */
    while(!0) /* infinite loop */
    {
        ; /* NOP */
    }
}
