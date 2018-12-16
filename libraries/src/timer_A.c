/******************************************************************************/
/**
*      @file : timer_A.c
*   @version : 0
*      @date : December 9, 2018
*    @author : Enzo IGLESIS
* @copyright : Copyright (c) 2018 Enzo IGLESIS, MIT License (MIT)
*******************************************************************************/

/*******************************    LIBRARIES   *******************************/
#include "timer_A.h"
#include "gpio.h"

/*******************************   VARIABLES    *******************************/

/*******************************     MACROS     *******************************/
#define map(x, in_min, in_max, out_min, out_max) (((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

/*******************************   CONSTANTS    *******************************/
/**** gpio ****/
const uint32_t timer_A0_gpio[5][2] = {{(uint32_t)PD, PIN3},   /* 7.3 */
                                      {(uint32_t)PA, PINC},   /* 2.4 */
                                      {(uint32_t)PA, PIND},   /* 2.5 */
                                      {(uint32_t)PA, PINE},   /* 2.6 */
                                      {(uint32_t)PA, PINF}};  /* 2.7 */

const uint32_t timer_A1_gpio[5][2] = {{(uint32_t)PD, PIN8},   /* 8.0 */
                                      {(uint32_t)PD, PIN7},   /* 7.7 */
                                      {(uint32_t)PD, PIN6},   /* 7.6 */
                                      {(uint32_t)PD, PIN5},   /* 7.5 */
                                      {(uint32_t)PD, PIN4}};  /* 7.4 */

const uint32_t timer_A2_gpio[5][2] = {{(uint32_t)PD, PIN9},   /* 8.1 */
                                      {(uint32_t)PC, PIN6},   /* 5.6 */
                                      {(uint32_t)PC, PIN7},   /* 5.7 */
                                      {(uint32_t)PC, PINE},   /* 6.6 */
                                      {(uint32_t)PC, PINF}};  /* 6.7 */

const uint32_t timer_A3_gpio[5][2] = {{(uint32_t)PE, PINC},   /* 10.4 */
                                      {(uint32_t)PE, PIND},   /* 10.5 */
                                      {(uint32_t)PD, PINA},   /* 8.2 */
                                      {(uint32_t)PE, PIN2},   /* 9.2 */
                                      {(uint32_t)PE, PIN3}};  /* 9.3 */

/*******************************   FUNCTIONS    *******************************/
/**** interrupt function ****/ /* see INTERRUPT section */
static void default_handler(void){while(!0);}

/**** Timer A ****/
    /**** Timer ****/
static void (* interrupt_handler_TA0_0)(void) = default_handler;
static void (* interrupt_handler_TA1_0)(void) = default_handler;
static void (* interrupt_handler_TA2_0)(void) = default_handler;
static void (* interrupt_handler_TA3_0)(void) = default_handler;
    /**** Capture/Compare ****/
static void (*interrupt_handler_TA0_N[5])(void) = {default_handler,\
                                                   default_handler,\
                                                   default_handler,\
                                                   default_handler,\
                                                   default_handler};

static void (*interrupt_handler_TA1_N[5])(void) = {default_handler,\
                                                   default_handler,\
                                                   default_handler,\
                                                   default_handler,\
                                                   default_handler};

static void (*interrupt_handler_TA2_N[5])(void) = {default_handler,\
                                                   default_handler,\
                                                   default_handler,\
                                                   default_handler,\
                                                   default_handler};

static void (*interrupt_handler_TA3_N[5])(void) = {default_handler,\
                                                   default_handler,\
                                                   default_handler,\
                                                   default_handler,\
                                                   default_handler};

/**** Timer A ****/
    /**** initialization ****/
/******************************************************************************/
/**
* @brief    Initializing function pointer of the structure timer_A_t.
*
* @param    this is a pointer to the instance timer_A_t that you want to
*           initialize.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
static void timer_A_initialize_struct(timer_A_t * this)
{
    this->up_mode = timer_A_up_mode;
    this->continuous_mode = timer_A_continuous_mode;
    this->updown_mode = timer_A_updown_mode;
    this->capture_mode = timer_A_capture_mode;
    this->compare_mode = timer_A_compare_mode;
    this->set_output_mode = timer_A_set_output_mode;
    this->set_channel_event_us = timer_A_set_channel_event_us;
    this->set_channel_event_pc = timer_A_set_channel_event_pc;
    this->out_bit_set = timer_A_out_bit_set;
    this->out_bit_clear = timer_A_out_bit_clear;
    this->out_bit_toggle = timer_A_out_bit_toggle;
    this->out_bit = timer_A_out_bit;
    this->pause = timer_A_pause;
    this->start = timer_A_start;
    this->raw_value = timer_A_raw_value;
    this->value_us = timer_A_value_us;
    this->value_pc = timer_A_value_pc;
    this->set_period_us = timer_A_set_period_us;
    this->enable_interrupt = timer_A_enable_interrupt;
    this->enable_capture_compare_interrupt = timer_A_enable_capture_compare_interrupt;
    this->disable_interrupt = timer_A_disable_interrupt;
    this->disable_capture_compare_interrupt = timer_A_disable_capture_compare_interrupt;
    this->pwm_duty_cycle = timer_A_pwm_duty_cycle;
    this->pwm_time_on = timer_A_pwm_time_on;
    this->pwm = timer_A_pwm;
}

/******************************************************************************/
/**
* @brief    Initializing function pointer of the structure timer_A_t and assign
*           the specified timer to the instance.
*
* @param    timer is a pointer to the instance timer_A_t that you want to
*           initialize.
* @param    timer_A is the address of the timer_A peripheral that will be
*           associate to the instance timer_A_t.
*
* @return   None.
*
* @note     The use of this function will reset the timer.
*
*******************************************************************************/
void timer_A_initialize(timer_A_t * timer, Timer_A_Type * timer_A)
{
    timer->timer_A = timer_A;
    timer->period_us = 0;
    timer->mode_control = TIMER_A_CTL_MC__STOP;
    timer_A_initialize_struct(timer);
    timer->timer_A->CTL |= TIMER_A_CTL_CLR; /* Reset the timer */
}

    /**** Main mode ****/
/******************************************************************************/
/**
* @brief    Set the prescale and load register to configure time period of the
*           specified timer.
*
* @param    this is a pointer to the instance timer_A_t.
* @param    period_us is the period in microsecond that will be set for the timer.
*           Possible value depend of the clock speed.
*
* @return   -1 if the period is too high else 0.
*
* @note     period to small with interrupt activated could stuck the program in
*           the interrupt function.
*
*******************************************************************************/
static int timer_A_compute_period_us(timer_A_t * this, uint32_t period_us)
{
    /* local declaration */
    uint32_t divider;
    uint64_t timer;
    /* program statement */
    timer = (uint64_t)SystemCoreClock*period_us; /* compute the counter with divider include */
    timer /= 1000000;
    divider = timer>>19;    /* compute input divider */

    if(divider == 0)
    {
        this->timer_A->CTL &= ~TIMER_A_CTL_ID_MASK; /* reset input divider */
    }
    else if(divider < 2)
    {
        this->timer_A->CTL &= ~TIMER_A_CTL_ID_MASK; /* reset input divider */
        this->timer_A->CTL |= TIMER_A_CTL_ID__2; /* prescale by 2 */
        timer = (uint64_t)SystemCoreClock/2*period_us;  /* compute counter with the prescale */
        timer /= 1000000;
    }
    else if(divider < 4)
    {
        this->timer_A->CTL &= ~TIMER_A_CTL_ID_MASK; /* reset input divider */
        this->timer_A->CTL |= TIMER_A_CTL_ID__4; /* prescale by 4 */
        timer = (uint64_t)SystemCoreClock/4*period_us;  /* compute counter with the prescale */
        timer /= 1000000;
    }
    else if(divider < 8)
    {
        this->timer_A->CTL &= ~TIMER_A_CTL_ID_MASK; /* reset input divider */
        this->timer_A->CTL |= TIMER_A_CTL_ID__8; /* prescale by 8 */
        timer = (uint64_t)SystemCoreClock/8*period_us;  /* compute counter with the prescale */
        timer /= 1000000;
    }
    else
    {
        return -1;  /* period_us is too high */
    }
    this->timer_A->EX0 &= ~TIMER_A_EX0_IDEX_MASK;
    this->timer_A->EX0 |= (timer>>16)&TIMER_A_EX0_IDEX_MASK;
    this->timer_A->CCR[0] = (timer/(this->timer_A->EX0+1)); /* set the new timer period */
    this->timer_A->R = 0; /* reset the current counter value */
    this->period_us = period_us;
    return 0;
}

/******************************************************************************/
/**
* @brief    Initializing timer in Up mode: The timer repeatedly counts from zero
*           to the specified value.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    period_us is the period in microsecond that will be set for the timer.
*           Possible value depend of the clock speed.
*
* @return   -1 if the period is too hight.
*
* @note     You can use frequency_to_period_us() to get the result of the period
*           in micro-second of a frequency.
*
*******************************************************************************/
int timer_A_up_mode(timer_A_t * this, uint32_t period_us)
{
    this->timer_A->CTL &= ~TIMER_A_CTL_IE; /* disable interrupt */
    if(timer_A_compute_period_us(this, period_us) != 0) return -1;
    this->timer_A->CTL &= ~TIMER_A_CTL_SSEL_MASK; /* clear clock source bit field */
    this->timer_A->CTL |= TIMER_A_CTL_SSEL__SMCLK; /* select SMCLK as clock source */
    this->timer_A->CTL &= ~TIMER_A_CTL_MC_MASK; /* clear the counter mode bit field */
    this->timer_A->CTL |= TIMER_A_CTL_MC__UP; /* up mode */
    this->mode_control = TIMER_A_CTL_MC__UP;
    return 0;
}

/******************************************************************************/
/**
* @brief    this function set the prescaler of the specified timer.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    divider is the value of the prescaler of the timer. possible value
*           are:    - TIMER_A_DIVIDER_1 = clk/1 = clk
*                   - TIMER_A_DIVIDER_2 = clk/2
*                   - TIMER_A_DIVIDER_4 = clk/4
*                   - TIMER_A_DIVIDER_8 = clk/8
*
* @param    divider_extended is the second stage prescaler. Possible value are:
*               - TIMER_A_DIVIDER_EXTENDED_1 = clk/1 = clk
*               - TIMER_A_DIVIDER_EXTENDED_2 = clk/2
*               - TIMER_A_DIVIDER_EXTENDED_3 = clk/3
*               - TIMER_A_DIVIDER_EXTENDED_4 = clk/4
*               - TIMER_A_DIVIDER_EXTENDED_5 = clk/5
*               - TIMER_A_DIVIDER_EXTENDED_6 = clk/6
*               - TIMER_A_DIVIDER_EXTENDED_7 = clk/7
*               - TIMER_A_DIVIDER_EXTENDED_8 = clk/8
*
* @return   -1 the value of the divider or the divider_extended are not
*           supported else 0.
*
* @note     None.
*
*******************************************************************************/
static int timer_A_compute_divider(timer_A_t * this, uint8_t divider, uint8_t divider_extended)
{
    if((divider&TIMER_A_CTL_ID_MASK) != divider) /* check if the desired divider is legal */
    {
        return -1; /* desired divider is not supported */
    }
    if((divider&TIMER_A_CTL_ID_MASK) != divider) /* check if the desired divider expansion is legal */
    {
        return -1; /* desired divider expansion is not supported */
    }
    this->timer_A->CTL &= ~TIMER_A_CTL_ID_MASK; /* clear input divider */
    this->timer_A->CTL |= divider&TIMER_A_CTL_ID_MASK; /* set the selected input divider */
    this->timer_A->EX0 &= ~TIMER_A_EX0_IDEX_MASK; /* clear the Input divider expansion */
    this->timer_A->EX0 |= divider_extended&TIMER_A_EX0_IDEX_MASK; /* set the selected divider expansion */
        /* compute period_us */
    divider >>= TIMER_A_CTL_ID_OFS; /* computer divider into the real divider value */
    divider = 1<<divider;
    divider_extended += 1; /* compute divider_extended into the real divider value */
    this->period_us = (uint32_t)(divider*divider_extended)/(SystemCoreClock/(uint32_t)1000000);
    return 0; /* all divider configured */
}

/******************************************************************************/
/**
* @brief    Initializing timer in Continuous mode: the timer repeatedly counts
*           up to 0FFFFh and restarts from zero.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    divider is the first stage prescaler. Possible value are:
*               - TIMER_A_DIVIDER_1 = clk/1 = clk
*               - TIMER_A_DIVIDER_2 = clk/2
*               - TIMER_A_DIVIDER_4 = clk/4
*               - TIMER_A_DIVIDER_8 = clk/8
*           Other value are not supported by the timer and the function will
*           return -1
* @param    divider_extended is the second stage prescaler. Possible value are:
*               - TIMER_A_DIVIDER_EXTENDED_1 = clk/1 = clk
*               - TIMER_A_DIVIDER_EXTENDED_2 = clk/2
*               - TIMER_A_DIVIDER_EXTENDED_3 = clk/3
*               - TIMER_A_DIVIDER_EXTENDED_4 = clk/4
*               - TIMER_A_DIVIDER_EXTENDED_5 = clk/5
*               - TIMER_A_DIVIDER_EXTENDED_6 = clk/6
*               - TIMER_A_DIVIDER_EXTENDED_7 = clk/7
*               - TIMER_A_DIVIDER_EXTENDED_8 = clk/8
*
* @return   -1 the value of the divider or the divider_extended are not
*           supported else 0.
*
* @note     None.
*
*******************************************************************************/
int timer_A_continuous_mode(timer_A_t * this, uint8_t divider, uint8_t divider_extended)
{
    this->timer_A->CTL &= ~TIMER_A_CTL_IE; /* disable interrupt */
    if(timer_A_compute_divider(this, divider, divider_extended)) return -1;
    this->timer_A->CTL &= ~TIMER_A_CTL_SSEL_MASK; /* clear clock source bit field */
    this->timer_A->CTL |= TIMER_A_CTL_SSEL__SMCLK; /* select SMCLK as clock source */
    this->timer_A->CTL &= ~TIMER_A_CTL_MC_MASK; /* clear the counter mode bit field */
    this->timer_A->R = 0; /* reset the courent counter value */
    this->timer_A->CTL |= TIMER_A_CTL_MC__CONTINUOUS; /* continuous mode */
    this->mode_control = TIMER_A_CTL_MC__CONTINUOUS;
    return 0;
}

/******************************************************************************/
/**
* @brief    Initializing timer in up/down mode: The timer repeatedly counts from
*           zero to the specified value and countdown from the specified value
*           to 0
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    period_us is the period in microsecond that will be set for the timer.
*           Possible value depend of the clock speed.
*
* @return   -1 if the period is too hight.
*
* @note     None.
*
*******************************************************************************/
int timer_A_updown_mode(timer_A_t * this, uint32_t period_us)
{
    this->timer_A->CTL &= ~TIMER_A_CTL_IE; /* disable interrupt */
    if(timer_A_compute_period_us(this, period_us/2) != 0) return -1;
    this->period_us *= 2;
    this->timer_A->CTL &= ~TIMER_A_CTL_SSEL_MASK; /* clear clock source bit field */
    this->timer_A->CTL |= TIMER_A_CTL_SSEL__SMCLK; /* select SMCLK as clock source */
    this->timer_A->CTL &= ~TIMER_A_CTL_MC_MASK; /* clear the counter mode bit field */
    this->timer_A->CTL |= TIMER_A_CTL_MC__UPDOWN; /* up mode */
    this->mode_control = TIMER_A_CTL_MC__UPDOWN;
    return 0;
}

    /**** Secondary mode ****/
/******************************************************************************/
/**
* @brief    initialize gpio for the timer A.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    channel is the channel number. Possible value are 0 to 4.
* @param    gpio is a pointer to the instance gpio_t that will be initialize.
*
* @return   -1 if the timer_A is unknown, else 0.
*
* @note     None.
*
*******************************************************************************/
static int timer_A_gpio(timer_A_t * this, uint8_t channel, gpio_t * gpio)
{
    /* local declaration */
    DIO_PORT_Interruptable_Type * port;
    uint32_t pin;
    /* program statement */
    if(this->timer_A == TIMER_A0)
    {
        port = (DIO_PORT_Interruptable_Type *)timer_A0_gpio[channel][0];
        pin = timer_A0_gpio[channel][1];
    }
    else if(this->timer_A == TIMER_A1)
    {
        port = (DIO_PORT_Interruptable_Type *)timer_A1_gpio[channel][0];
        pin = timer_A1_gpio[channel][1];
    }
    else if(this->timer_A == TIMER_A2)
    {
        port = (DIO_PORT_Interruptable_Type *)timer_A2_gpio[channel][0];
        pin = timer_A2_gpio[channel][1];
    }
    else if(this->timer_A == TIMER_A3)
    {
        port = (DIO_PORT_Interruptable_Type *)timer_A3_gpio[channel][0];
        pin = timer_A3_gpio[channel][1];
    }
    else
    {
        return -1;
    }
    gpio_initialize(gpio, port, pin);
    return 0;
}

/******************************************************************************/
/**
* @breif    Configure the gpio for the timer capture mode.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    channel is the channel number. Possible value are 0 to 4.
*
* @return   -1 if the timer_A is unknown, else 0.
*
* @note     None.
*
*******************************************************************************/
static int timer_A_gpio_pin_capture(timer_A_t * this, uint8_t channel)
{
    /* local declaration */
    gpio_t gpio;
    /* program statement */
    if(timer_A_gpio(this, channel, &gpio) != 0)
    {
        return -1;
    }
    gpio.input(&gpio);
    if(gpio.port == PD  && (gpio.pin >= PIN8 || gpio.pin <= PIN9))
    {
        gpio.secondary_module(&gpio);
    }
    else
    {
        gpio.primary_module(&gpio);
    }
    return 0;
}

/******************************************************************************/
/**
* @brief    Configure timer in capture mode: record time events. It can be used
*           for speed computations or time measurements.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    channel is the channel number where the capture mode will be
*           enabled. Possible value are 0 to 4.
* @param    edge is capture mode. Possible value are:
*               - TIMER_A_CAP_RISING_EDGE: capture on rising edge
*               - TIMER_A_CAP_FALLING_EDGE: capture on falling edge
*               - TIMER_A_CAP_BOTH_EDGES: capture on both rising and falling
*               edges
*
* @param    input_signal is the input signal for the capture. Possible value
*           are:
*               - TIMER_A_INPUT_CCIA
*               - TIMER_A_INPUT_CCIB
*               - TIMER_A_INPUT_GND
*               - TIMER_A_INPUT_VCC
* @param    sync is for asynchronous capture or synchronous capture. Possible
*           value are:
*               - TIMER_A_ASYNC_CAP: Asynchronous capture.
*               - TIMER_A_SYNC_CAP: synchronous capture.
*
* @return   -1 the channel number is not supported, -2 if timer is unknown else
*            0.
*
* @note     None.
*
*******************************************************************************/
int timer_A_capture_mode(timer_A_t * this, uint8_t channel, uint8_t edge, uint8_t input_signal, uint8_t sync)
{
    if(channel >= 5)
    {
        return -1;
    }
    if(timer_A_gpio_pin_capture(this, channel) != 0)
    {
        return -2;
    }
    this->timer_A->CCTL[channel] &= ~TIMER_A_CCTLN_CM_MASK; /* clear the capture mode */
    this->timer_A->CCTL[channel] |= edge&TIMER_A_CCTLN_CM_MASK; /* set the capture mode */
    this->timer_A->CCTL[channel] &= ~TIMER_A_CCTLN_CCIS_MASK; /* clear the input signal */
    this->timer_A->CCTL[channel] |= input_signal&TIMER_A_CCTLN_CCIS_MASK; /* select the input signal */
    this->timer_A->CCTL[channel] &= ~TIMER_A_CCTLN_SCCI; /* clear the synchronize capture source */
    this->timer_A->CCTL[channel] |= sync&TIMER_A_CCTLN_SCCI; /* select the synchronize mode */
    this->timer_A->CCTL[channel] |= TIMER_A_CCTLN_CAP; /* capture mode */
    this->timer_A->CCTL[channel] &= ~TIMER_A_CCTLN_CCIE; /* Interrupt disabled */
    return 0;
}

/******************************************************************************/
/**
* @brief    Configure timer in compare mode: he compare mode is used to generate
*           PWM output signals or interrupts at specific time intervals.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    channel is the channel number where the compare mode will be
*           enabled. Possible value are 0 to 4.
*
* @return   -1 the channel number is not supported.
*
* @note     None.
*
*******************************************************************************/
int timer_A_compare_mode(timer_A_t * this, uint8_t channel)
{
    if(channel >= 5)
    {
        return -1;
    }
    this->timer_A->CCTL[channel] &= ~TIMER_A_CCTLN_CM_MASK; /* no capture */
    this->timer_A->CCTL[channel] &= ~TIMER_A_CCTLN_CCIS_MASK;
    this->timer_A->CCTL[channel] &= ~TIMER_A_CCTLN_CAP; /* compare mode */
    this->timer_A->CCTL[channel] &= ~TIMER_A_CCTLN_CCIE; /* Interrupt disabled */
    return 0;
}

    /**** Output ****/
/******************************************************************************/
/**
* @brief    Configure the gpio for the timer output mode.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    channel is the channel number where the compare mode will be
*           enabled. Possible value are 0 to 4.
*
* @return   -1 if the timer_A is unknown, else 0.
*
* @note     None.
*
*******************************************************************************/
static int timer_A_gpio_pin_output(timer_A_t * this, uint8_t channel)
{
    /* local declaration */
    gpio_t gpio;
    /* program statement */
    if(timer_A_gpio(this, channel, &gpio) != 0)
    {
        return -1;
    }
    gpio.output(&gpio);
    if(gpio.port == PD  && (gpio.pin >= PIN8 || gpio.pin <= PIN9))
    {
        gpio.secondary_module(&gpio);
    }
    else
    {
        gpio.primary_module(&gpio);
    }
    return 0;
}

/******************************************************************************/
/**
* @brief    Configure the output mode of a channel.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    channel is the channel number where the output mode mode will be
*           enabled. Possible value are 0 to 4.
* @param    output_mode is the output mode that must be set. Possible value are:
*               - TIMER_A_OUT_BIT:          OUT bit value
*               - TIMER_A_OUT_SET:          Set
*               - TIMER_A_OUT_TOGGLE_RESET: Toggle/reset
*               - TIMER_A_OUT_SET_RESET:    Set/reset
*               - TIMER_A_OUT_TOGGLE:       Toggle
*               - TIMER_A_OUT_RESET:        Reset
*               - TIMER_A_OUT_TOGGLE_SET:   Toggle/set
*               - TIMER_A_OUT_RESET_SET:    Reset/set
*
* @return   -1 the channel number is not supported, -2 if timer_A is unknown
*           else 0.
*
* @note     None.
*
*******************************************************************************/
int timer_A_set_output_mode(timer_A_t * this, uint8_t channel, uint8_t output_mode)
{
    if(channel >= 5)
    {
        return -1;
    }
    if(timer_A_gpio_pin_output(this, channel) != 0)
    {
        return -2;
    }
    this->timer_A->CCTL[channel] &= ~TIMER_A_CCTLN_OUTMOD_MASK;
    this->timer_A->CCTL[channel] |= output_mode&TIMER_A_CCTLN_OUTMOD_MASK;
    return 0;
}

/******************************************************************************/
/**
* @brief    Configure the time in micro-second after the beginning of the
*           period, for the interrupt event of the specified channel.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    channel is the channel number where to set the event time. Possible
*           value are 1 to 4. Note that the channel 0 is only available for the
*           continuous mode. The channel 0 for others modes define the period
*           and can be change with timer_A_set_period_us.
* @param    event_us is the time in microsecond after the beginning of the period
*           when an event for the channel will be occurred. If the value is
*           bigger than the period it will be set as the same as the period.
*           (except for up/down mode, see note section below).
*
* @return   -1 the channel number is not supported else 0.
*
* @note     For the up/down mode the maximum possible value is the half of
*           the period.
*
*******************************************************************************/
int timer_A_set_channel_event_us(timer_A_t * this, uint8_t channel, uint32_t event_us)
{
    /* local declaration */
    uint32_t max_in = (this->mode_control == TIMER_A_CTL_MC__UPDOWN)? this->period_us/2 : this->period_us;
    /* program statement */
    if(channel >= 5)
    {
        return -1;
    }
    if(this->mode_control == TIMER_A_CTL_MC__CONTINUOUS)
    {
        if(event_us >= max_in)
        {
            this->timer_A->CCR[channel] = 0xFFFF;
        }
        else
        {
            this->timer_A->CCR[channel] = (uint16_t)map((uint64_t)event_us, (uint64_t)0, (uint64_t)max_in, (uint64_t)0, (uint64_t)0xFFFF);
        }
    }
    else
    {
        if(channel == 0)
        {
            return -1;
        }
        if(event_us >= max_in)
        {
            this->timer_A->CCR[channel] = this->timer_A->CCR[0];
        }
        else
        {
            this->timer_A->CCR[channel] = (uint16_t)map((uint64_t)event_us, (uint64_t)0, max_in, (uint64_t)0, (uint64_t)(this->timer_A->CCR[0]));
        }
    }
    return 0;
}

/******************************************************************************/
/**
* @brief    Configure in percent of the period time after the beginning of the
*           period of interrupt event of the specified channel.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    channel is the channel number where to set the event time. Possible
*           value are 1 to 4. Note that the channel 0 is only available for the
*           continuous mode. The channel 0 for others modes define the period
*           and can be change with timer_A_set_period_us.
* @param    pc_of_period is at how many pc the event must be
*           configured. If the value is bigger than 100% it will be reset at
*           100%. (except for up/down mode, see note section below).
*
* @return   -1 the channel number is not supported else 0.
*
* @note     For the up/down mode the maximum possible value is the half of
*           the period.
*
*******************************************************************************/
int timer_A_set_channel_event_pc(timer_A_t * this, uint8_t channel, float pc_of_period)
{
    return timer_A_set_channel_event_us(this, channel, (uint32_t)((pc_of_period/100.)*this->period_us));
}

/******************************************************************************/
/**
* @brief    Set the out bit of the timer A when the output mode is
*           TIMER_A_OUT_BIT
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    channel is the channel number where to set the event time. Possible
*           value are 0 to 4.
*
* @return   -1 the channel number is not supported else 0.
*
* @note     For the up/down mode the maximum possible value is the half of
*           the period.
*
*******************************************************************************/
int timer_A_out_bit_set(timer_A_t * this, uint8_t channel)
{
    if(channel >= 5)
    {
        return -1;
    }
    this->timer_A->CCTL[channel] |= TIMER_A_CCTLN_OUT;
    return 0;
}

/******************************************************************************/
/**
* @brief    Clear the out bit of the timer A when the output mode is
*           TIMER_A_OUT_BIT
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    channel is the channel number where to set the event time. Possible
*           value are 0 to 4.
*
* @return   -1 the channel number is not supported else 0.
*
* @note     For the up/down mode the maximum possible value is the half of
*           the period.
*
*******************************************************************************/
int timer_A_out_bit_clear(timer_A_t * this, uint8_t channel)
{
    if(channel >= 5)
    {
        return -1;
    }
    this->timer_A->CCTL[channel] &= ~TIMER_A_CCTLN_OUT;
    return 0;
}

/******************************************************************************/
/**
* @brief    Toggle the out bit of the timer A when the output mode is
*           TIMER_A_OUT_BIT
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    channel is the channel number where to set the event time. Possible
*           value are 0 to 4.
*
* @return   -1 the channel number is not supported else 1 if it toggle to one or
*           0 if it toggle to 0.
*
* @note     For the up/down mode the maximum possible value is the half of
*           the period.
*
*******************************************************************************/
int timer_A_out_bit_toggle(timer_A_t * this, uint8_t channel)
{
    if(channel >= 5)
    {
        return -1;
    }
    return (this->timer_A->CCTL[channel] ^= TIMER_A_CCTLN_OUT)? 1 : 0;
}

/******************************************************************************/
/**
* @brief    Drive the out bit of the timer A when the output mode is
*           TIMER_A_OUT_BIT
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    channel is the channel number where to set the event time. Possible
*           value are 0 to 4.
* @param    state is the state of the out bit
*
* @return   -1 the channel number is not supported else 0.
*
* @note     For the up/down mode the maximum possible value is the half of
*           the period.
*
*******************************************************************************/
int timer_A_out_bit(timer_A_t * this, uint8_t channel, uint8_t state)
{
    if(channel >= 5)
    {
        return -1;
    }
    if(state)
    {
        timer_A_out_bit_set(this, channel);
    }
    else
    {
        timer_A_out_bit_clear(this, channel);
    }
    return 0;
}

/******************************************************************************/
/**
* @brief    Pause the timer. The counter register is not reseted.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
*
* @return   None.
*
* @note     Could be restarted without reseting the counter with timer_A_start()
*
*******************************************************************************/
void timer_A_pause(timer_A_t * this)
{
    this->timer_A->CTL &= ~TIMER_A_CTL_MC_MASK;
}

/******************************************************************************/
/**
* @brief    Start the timer. The counter register is not reseted.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
*
* @return   None.
*
* @note     Could be pause without reseting the counter with timer_A_pause()
*           this function is useless if the timer_A_pause() as not be called
*           before, because the counter is automatically run after configuring a
*           main mode (UP, CONTINUOUS or UP/DOWN)
*
*******************************************************************************/
void timer_A_start(timer_A_t * this)
{
    this->timer_A->CTL |= this->mode_control&TIMER_A_CTL_MC_MASK;
}

/******************************************************************************/
/**
* @brief    Return the raw value of the counter of the timer.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
*
* @return   the raw value of the counter of the timer.
*
* @note     None.
*
*******************************************************************************/
uint16_t timer_A_raw_value(timer_A_t * this)
{
    return this->timer_A->R;
}

/******************************************************************************/
/**
* @brief    Return the value in micro-second of the counter of the timer.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
*
* @return   the value in micro-second of the counter of the timer.
*
* @note     Not available for up/down mode.
*
*******************************************************************************/
uint32_t timer_A_value_us(timer_A_t * this)
{
    if(this->mode_control == TIMER_A_CTL_MC__CONTINUOUS)
    {
        return (uint32_t)map((uint64_t)this->timer_A->R, (uint64_t)0, (uint64_t)0xFFFF, (uint64_t)0, (uint64_t)this->period_us);
    }
    return (uint32_t)map((uint64_t)this->timer_A->R, (uint64_t)0, (uint64_t)this->timer_A->CCR[0], (uint64_t)0, (uint64_t)this->period_us);
}

/******************************************************************************/
/**
* @brief    Return the value in percent of the period of the counter of the
*           timer.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
*
* @return   the value in percent of the period of the counter of the timer.
*
* @note     Not available for up/down mode.
*
*******************************************************************************/
float timer_A_value_pc(timer_A_t * this)
{
    if(this->mode_control == TIMER_A_CTL_MC__CONTINUOUS)
    {
        return map((float)this->timer_A->R, 0., (float)0xFFFF, 0., 100.);
    }
    return map((float)this->timer_A->R, 0., (float)this->timer_A->CCR[0], 0., 100.);
}

/******************************************************************************/
/**
* @brief    Configure a new period for the specified timer.
*
* @param    this is a pointer to the instance timer_A_t.
* @param    period_us is the period in microsecond that will be set for the
*           timer. Possible value depend of the clock speed.
*
* @return   -1 if the period is too high or if the mode of the timer is not
*           supported for a period setting else 0.
*
* @note     The period in CONTINUOUS mode can't be set.
*
*******************************************************************************/
int timer_A_set_period_us(timer_A_t * this, uint32_t period_us)
{
    /* local declaration */
    int ret_val;
    /* program statement */
    if(this->mode_control == TIMER_A_CTL_MC__CONTINUOUS || this->mode_control == TIMER_A_CTL_MC__STOP)
    {
        return -1;
    }
    if(this->mode_control == TIMER_A_CTL_MC__UPDOWN)
    {
        period_us /= 2;
    }
    timer_A_pause(this);
    ret_val = timer_A_compute_period_us(this, period_us);
    timer_A_start(this);
    if(this->mode_control == TIMER_A_CTL_MC__UPDOWN)
    {
        this->period_us *= 2;
    }
    return ret_val;
}

/****  Interrupt ****/
/******************************************************************************/
/**
* @brief    Configure the interrupt function when timer reach the end of the
*           period
*
* @param    this is a pointer to the instance timer_A_t.
* @param    interrupt_handler is a pointer to the interrupt function that must
*           be execute when the interrupt flag is raised.
* @param    priority is the priority of the interrupt.
*
* @return   -1 when the timer is unknown else 0.
*
* @note     interrupt_handler will be faster and not take more place if you make
*           an inline function.
*           Don't forget to enable global interrupts with __enable_interrupts();
*           this function must be executed after a initialize of a main mode.
*
*           /!\The use of this interrupts will overpass the capture/compare
*           interrupts
*
*******************************************************************************/
int timer_A_enable_interrupt(timer_A_t * this, void (*interrupt_handler)(void), uint32_t priority)
{
    this->timer_A->CTL &= ~TIMER_A_CTL_IE; /* disable interrupt */
    this->timer_A->CCTL[0] &= ~TIMER_A_CCTLN_CCIE;
    if(this->timer_A == TIMER_A0)
    {
        __NVIC_EnableIRQ(TA0_0_IRQn); /* Enable IRQ of the TIMER32_1 */
        __NVIC_SetPriority(TA0_0_IRQn, priority); /* set the priority of the IRQ */
        interrupt_handler_TA0_0 = interrupt_handler; /* set the interrupt function */
    }
    else if(this->timer_A == TIMER_A1)
    {
        __NVIC_EnableIRQ(TA1_0_IRQn); /* Enable IRQ of the TIMER32_1 */
        __NVIC_SetPriority(TA1_0_IRQn, priority); /* set the priority of the IRQ */
        interrupt_handler_TA1_0 = interrupt_handler; /* set the interrupt function */
    }
    else if(this->timer_A == TIMER_A2)
    {
        __NVIC_EnableIRQ(TA2_0_IRQn); /* Enable IRQ of the TIMER32_1 */
        __NVIC_SetPriority(TA2_0_IRQn, priority); /* set the priority of the IRQ */
        interrupt_handler_TA2_0 = interrupt_handler; /* set the interrupt function */
    }
    else if(this->timer_A == TIMER_A3)
    {
        __NVIC_EnableIRQ(TA3_0_IRQn); /* Enable IRQ of the TIMER32_1 */
        __NVIC_SetPriority(TA3_0_IRQn, priority); /* set the priority of the IRQ */
        interrupt_handler_TA3_0 = interrupt_handler; /* set the interrupt function */
    }
    else
    {
        return -1;
    }
    this->timer_A->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    this->timer_A->CCTL[0] |= TIMER_A_CCTLN_CCIE;
    this->timer_A->CTL &= ~TIMER_A_CTL_IFG; /* clear interrupt */
    this->timer_A->CTL |= TIMER_A_CTL_IE; /* enable interrupt */
    return 0;
}

/******************************************************************************/
/**
* @brief    Configure the interrupt function when a channel reach it's
*           configured value.
*
* @param    this is a pointer to the instance timer_A_t.
* @param    channel is the channel number on which the interrupt must be
*           enabled.
* @param    interrupt_handler is a pointer to the interrupt function that must
*           be execute when the interrupt flag is raised.
* @param    priority is the priority of the interrupt.
*
* @return   -1 if the timer is unknown or if the channel number isn't Supported
*           else 0.
*
* @note     interrupt_handler will be faster and not take more place if you make
*           an inline function.
*           Don't forget to enable global interrupts with __enable_interrupts();
*           this function must be executed after a initialize of a main mode.
*
*           /!\ The interrupt function when timer reach the end of the period
*           is automatically disable else this interrupt will never be catch.
*
*******************************************************************************/
int timer_A_enable_capture_compare_interrupt(timer_A_t * this, uint8_t channel, void (*interrupt_handler)(void), uint32_t priority)
{
    if(channel >= 5)
    {
        return -1;
    }
    this->timer_A->CCTL[channel] &= ~TIMER_A_CCTLN_CCIE; /* Interrupt disabled */
    if(this->timer_A == TIMER_A0)
    {
        __NVIC_EnableIRQ(TA0_N_IRQn); /* Enable IRQ of the TIMER32_1 */
        __NVIC_SetPriority(TA0_N_IRQn, priority); /* set the priority of the IRQ */
        interrupt_handler_TA0_N[channel] = interrupt_handler; /* set the interrupt function */
    }
    else if(this->timer_A == TIMER_A1)
    {
        __NVIC_EnableIRQ(TA1_N_IRQn); /* Enable IRQ of the TIMER32_1 */
        __NVIC_SetPriority(TA1_N_IRQn, priority); /* set the priority of the IRQ */
        interrupt_handler_TA1_N[channel] = interrupt_handler; /* set the interrupt function */
    }
    else if(this->timer_A == TIMER_A2)
    {
        __NVIC_EnableIRQ(TA2_N_IRQn); /* Enable IRQ of the TIMER32_1 */
        __NVIC_SetPriority(TA2_N_IRQn, priority); /* set the priority of the IRQ */
        interrupt_handler_TA2_N[channel] = interrupt_handler; /* set the interrupt function */
    }
    else if(this->timer_A == TIMER_A3)
    {
        __NVIC_EnableIRQ(TA3_N_IRQn); /* Enable IRQ of the TIMER32_1 */
        __NVIC_SetPriority(TA3_N_IRQn, priority); /* set the priority of the IRQ */
        interrupt_handler_TA3_N[channel] = interrupt_handler; /* set the interrupt function */
    }
    else
    {
        return -1;
    }
    timer_A_disable_interrupt(this);
    this->timer_A->CCTL[channel] &= ~TIMER_A_CCTLN_CCIFG; /* Interrupt disabled */
    this->timer_A->CCTL[channel] |= TIMER_A_CCTLN_CCIE; /* Interrupt disabled */
    return 0;
}

/******************************************************************************/
/**
* @brief    Disable the interrupt function for the timer.
*
* @param    this is a pointer to the instance timer_A_t.
*
* @return   -1 if the timer is unknown else 0.
*
* @note     None.
*
*******************************************************************************/
int timer_A_disable_interrupt(timer_A_t * this)
{
    this->timer_A->CTL &= ~TIMER_A_CTL_IE; /* disable interrupt */
    if(this->timer_A == TIMER_A0)
    {
        __NVIC_DisableIRQ(TA0_0_IRQn); /* Disable IRQ of the TIMER32_1 */
        interrupt_handler_TA0_0 = default_handler; /* set the interrupt function */
    }
    else if(this->timer_A == TIMER_A1)
    {
        __NVIC_DisableIRQ(TA1_0_IRQn); /* Disable IRQ of the TIMER32_1 */
        interrupt_handler_TA1_0 = default_handler; /* set the interrupt function */
    }
    else if(this->timer_A == TIMER_A2)
    {
        __NVIC_DisableIRQ(TA2_0_IRQn); /* Disable IRQ of the TIMER32_1 */
        interrupt_handler_TA2_0 = default_handler; /* set the interrupt function */
    }
    else if(this->timer_A == TIMER_A3)
    {
        __NVIC_DisableIRQ(TA3_0_IRQn); /* Disable IRQ of the TIMER32_1 */
        interrupt_handler_TA3_0 = default_handler; /* set the interrupt function */
    }
    else
    {
        return -1;
    }
    this->timer_A->CTL &= ~TIMER_A_CTL_IFG; /* clear interrupt */
    return 0;
}

/******************************************************************************/
/**
* @brief    disable the interrupt function for the channel.
*
* @param    this is a pointer to the instance timer_A_t.
* @param    channel is the channel number on which the interrupt must be
*           enabled.
*
* @return   -1 if the timer is unknown or if the channel number isn't Supported
*           else 0.
*
* @note     None.
*
*******************************************************************************/
int timer_A_disable_capture_compare_interrupt(timer_A_t * this, uint8_t channel)
{
    if(channel >= 5)
    {
        return -1;
    }
    this->timer_A->CCTL[channel] &= ~TIMER_A_CCTLN_CCIE; /* Interrupt disabled */
    if(this->timer_A == TIMER_A0)
    {
        __NVIC_DisableIRQ(TA0_N_IRQn); /* Disable IRQ of the TIMER32_1 */
        interrupt_handler_TA0_N[channel] = default_handler; /* set the interrupt function */
    }
    else if(this->timer_A == TIMER_A1)
    {
        __NVIC_DisableIRQ(TA1_N_IRQn); /* Disable IRQ of the TIMER32_1 */
        interrupt_handler_TA1_N[channel] = default_handler; /* set the interrupt function */
    }
    else if(this->timer_A == TIMER_A2)
    {
        __NVIC_DisableIRQ(TA2_N_IRQn); /* Disable IRQ of the TIMER32_1 */
        interrupt_handler_TA2_N[channel] = default_handler; /* set the interrupt function */
    }
    if(this->timer_A == TIMER_A3)
    {
        __NVIC_DisableIRQ(TA3_N_IRQn); /* Disable IRQ of the TIMER32_1 */
        interrupt_handler_TA3_N[channel] = default_handler; /* set the interrupt function */
    }
    else
    {
        return -1;
    }
    this->timer_A->CCTL[channel] &= ~TIMER_A_CCTLN_CCIFG; /* clear interrupt */
    return 0;
}

/**** PWM ****/

/******************************************************************************/
/**
* @brief    Configure the timer A PWM, on the specified channel, with a new
*           time on in percent.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    duty_cycle is the percent of time that the PWM will be HIGH.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void timer_A_pwm_duty_cycle(timer_A_t * this, float duty_cycle)
{
    this->timer_A->CCR[this->pwm_channel] = (uint16_t)map(duty_cycle, 0., 100., 0., (float)this->timer_A->CCR[0]);
}

/******************************************************************************/
/**
* @brief    Configure the timer A PWM, on the specified channel, with a new
*           time on in microsecond.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    time_on_us is the time that the PWM will be HIGH.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void timer_A_pwm_time_on(timer_A_t * this, uint32_t time_on_us)
{
    this->timer_A->CCR[this->pwm_channel] = (uint16_t)map((uint64_t)time_on_us, (uint64_t)0, this->period_us, (uint64_t)0, (uint64_t)(this->timer_A->CCR[0]));
}

/******************************************************************************/
/**
* @brief    Configure the timer A as a PWM, on the specified channel, with the
*           specified period and the specified time on.
*
* @param    this is a pointer to the instance timer_A_t that you want to use.
* @param    channel is the channel number where the compare mode will be
*           enabled. Possible value are 1 to 4.
* @param    period_us is the period in microsecond that will be set for the timer.
*           Possible value depend of the clock speed.
* @param    time_on_us is the time that the PWM will be HIGH in microsecond.
*
* @return   -1 the channel number is not supported, -2 if timer_A is unknown,
*           -3 if the period is too high, else 0.
*
* @note     None.
*
*******************************************************************************/
int timer_A_pwm(timer_A_t * this, uint8_t channel, uint32_t period_us, uint32_t time_on_us)
{

    if(channel == 0 || channel >= 5)
    {
        return -1;
    }
    this->pwm_channel = channel;
    if(timer_A_up_mode(this, period_us) == -1)
    {
        return -3;
    }
    timer_A_compare_mode(this, channel);
    timer_A_pwm_time_on(this, time_on_us);
    return timer_A_set_output_mode(this, channel, TIMER_A_OUT_RESET_SET);
}

/*******************************   INTERRUPT    *******************************/
void TA0_0_IRQHandler(void)
{
    interrupt_handler_TA0_0();
    TIMER_A0->CTL &= ~TIMER_A_CTL_IFG;
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
}

void TA0_N_IRQHandler(void)
{
    /* local declaration */
    volatile uint8_t i;
    /* program statement */
    for(i = 0; i < 5; i++)
    {
        if(TIMER_A0->CCTL[i]&TIMER_A_CCTLN_CCIE && TIMER_A0->CCTL[i]&TIMER_A_CCTLN_CCIFG)
        {
            interrupt_handler_TA0_N[i]();
            TIMER_A0->CCTL[i] &= ~TIMER_A_CCTLN_CCIFG;
        }
    }
}

void TA1_0_IRQHandler(void)
{
    interrupt_handler_TA1_0();
    TIMER_A1->CTL &= ~TIMER_A_CTL_IFG;
    TIMER_A1->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
}

void TA1_N_IRQHandler(void)
{
    /* local declaration */
    volatile uint8_t i;
    /* program statement */
    for(i = 0; i < 5; i++)
    {
        if(TIMER_A1->CCTL[i]&TIMER_A_CCTLN_CCIE && TIMER_A1->CCTL[i]&TIMER_A_CCTLN_CCIFG)
        {
            interrupt_handler_TA1_N[i]();
            TIMER_A1->CCTL[i] &= ~TIMER_A_CCTLN_CCIFG;
        }
    }
}
void TA2_0_IRQHandler(void)
{
    interrupt_handler_TA2_0();
    TIMER_A2->CTL &= ~TIMER_A_CTL_IFG;
    TIMER_A2->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
}

void TA2_N_IRQHandler(void)
{
    /* local declaration */
    volatile uint8_t i;
    /* program statement */
    for(i = 0; i < 5; i++)
    {
        if(TIMER_A2->CCTL[i]&TIMER_A_CCTLN_CCIE && TIMER_A2->CCTL[i]&TIMER_A_CCTLN_CCIFG)
        {
            interrupt_handler_TA2_N[i]();
            TIMER_A2->CCTL[i] &= ~TIMER_A_CCTLN_CCIFG;
        }
    }
}

void TA3_0_IRQHandler(void)
{
    interrupt_handler_TA3_0();
    TIMER_A3->CTL &= ~TIMER_A_CTL_IFG;
    TIMER_A3->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
}

void TA3_N_IRQHandler(void)
{
    /* local declaration */
    volatile uint8_t i;
    /* program statement */
    for(i = 0; i < 5; i++)
    {
        if(TIMER_A3->CCTL[i]&TIMER_A_CCTLN_CCIE && TIMER_A3->CCTL[i]&TIMER_A_CCTLN_CCIFG)
        {
            interrupt_handler_TA3_N[i]();
            TIMER_A3->CCTL[i] &= ~TIMER_A_CCTLN_CCIFG;
        }
    }
}
