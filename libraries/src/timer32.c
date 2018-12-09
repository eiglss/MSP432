/*******************************************************************************/
/**
*      @file : timer32.c
*   @version : 0
*      @date : December 9, 2018
*    @author : Enzo IGLESIS
* @copyright : Copyright (c) 2018 Enzo IGLESIS, MIT License (MIT)
*******************************************************************************/

/*******************************    LIBRARIES   *******************************/
#include "timer32.h"

/*******************************   VARIABLES    *******************************/

/*******************************   FUNCTIONS    *******************************/
/**** interrupt function ****/ /* see INTERRUPT section */
static void default_handler(void){while(!0);}

/**** Timer 32 ****/
static void (* interrupt_handler_T32_INT1)(void) = default_handler;
static void (* interrupt_handler_T32_INT2)(void) = default_handler;

    /**** initialization ****/
/******************************************************************************/
/**
* @biref    Initializing function pointer of the structure timer32_t.
*
* @param    this is a pointer to the instance timer32_t that you want to initialize.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
static void timer32_initialize_struct(timer32_t * this)
{
    this->free_running = timer32_free_running;
    this->periodic_timer = timer32_periodic_timer;
    this->one_shot_timer = timer32_one_shot_timer;
    this->start = timer32_start;
    this->pause = timer32_pause;
    this->value = timer32_value;
    this->load = timer32_load;
    this->enable_interrupt = timer32_enable_interrupt;
    this->disable_interrupt = timer32_disable_interrupt;
    this->period_us = timer32_period_us;
}

/******************************************************************************/
/**
* @biref    Initializing function pointer of the structure timer32_t and assign
*           the specified timer to the instance.
*
* @param    timer is a pointer to the instance timer32_t that you want to
*           initialize.
* @param    timer32 is the address of the timer32 peripheral that will be
*           associate to the instance timer32_t.
*
* @return   None.
*
* @note     The use of this function will stop the timer.
*
*******************************************************************************/
void timer32_initialize(timer32_t * timer, Timer32_Type * timer32)
{
    timer->timer32 = timer32;
    timer32_initialize_struct(timer);
    timer->timer32->CONTROL &= ~TIMER32_CONTROL_ENABLE; /* Stop the timer */
    timer->timer32->CONTROL &= ~TIMER32_CONTROL_IE; /* disable interrupt */
}

    /**** configuration ****/
/******************************************************************************/
/**
* @biref    Initializing timer in Free-running mode: The counter wraps after
*           reaching its zero value, and continues to count down from the
*           maximum value.
*
* @param    this is a pointer to the instance timer32_t that you want to use.
* @param    size is the size of the counter. Possible value are:
*               - 16 (or 0): 16 bit counter
*               - 32 (or TIMER32_CONTROL_SIZE): 32 bit counter
* @param    prescale is the prescaler of the input clock. Possible value are:
*               - 1 (or TIMER32_CONTROL_PRESCALE_0): input clock/1;
*               - 16 (or TIMER32_CONTROL_PRESCALE_1): input clock/16
*               - 256 (or TIMER32_CONTROL_PRESCALE_2) input clock/256
*
* @return   -1 if the size or prescale are no known value, else 0.
*
* @note     this function stop the timer before setup the timer mode, and
*           disable the interrupt, you have to start the timer with
*           timer32_start function and reconfigure interrupt with
*           timer32_set_interrupt function.
*
*******************************************************************************/
int timer32_free_running(timer32_t * this, uint8_t size, uint16_t prescale)
{
    this->timer32->CONTROL &= ~TIMER32_CONTROL_ENABLE; /* Stop the timer */
    this->timer32->CONTROL &= ~TIMER32_CONTROL_MODE;    /* free-running mode */
    this->timer32->CONTROL &= ~TIMER32_CONTROL_IE;      /* interrupt disabled */
    this->timer32->CONTROL &= ~TIMER32_CONTROL_ONESHOT; /* wrapping mode */
    switch(size)
    {
        case 0:
        case 16:
        {
            this->timer32->CONTROL &= ~TIMER32_CONTROL_SIZE; /* 16-bit counter */
        }
        break;
        case TIMER32_CONTROL_SIZE:
        case 32:
        {
            this->timer32->CONTROL |= TIMER32_CONTROL_SIZE; /* 32-bit counter */
        }
        break;
        default:
        {
            return -1;  /* invalide size */
        }
    }
    this->timer32->CONTROL &= ~TIMER32_CONTROL_PRESCALE_MASK; /* reset prescaler */
    switch(prescale)
    {
        case TIMER32_CONTROL_PRESCALE_0:
        case 1:
        {
            this->timer32->CONTROL |= TIMER32_CONTROL_PRESCALE_0; /* input clock/1  */
        }
        break;
        case TIMER32_CONTROL_PRESCALE_1:
        case 16:
        {
            this->timer32->CONTROL |= TIMER32_CONTROL_PRESCALE_1; /* input clock/16  */
        }
        break;
        case TIMER32_CONTROL_PRESCALE_2:
        case 256:
        {
            this->timer32->CONTROL |= TIMER32_CONTROL_PRESCALE_2; /* input clock/256  */
        }
        break;
        default:
        {
            return -1; /* invalid prescale */
        }
    }
    return 0;
}

/******************************************************************************/
/**
* @biref    Set the prescale and load register to configure time period of the specified
* timer.
*
* @param    this is a pointer to the instance timer32_t.
* @param    period_us is the period in microsecond that will be set for the
*           timer. Possible value depend of the clock speed, but with default
*           clock speed at 3 000 000 Hz it's from 0 to 366503875925 us which in
*           HMS correspond to 4d 5h 48m 23.875925s
*
* @return   -1 if the period is too high else 0.
*
* @note     period to small with interrupt activated could stuck the program in
*           the interrupt function.
*
*******************************************************************************/
static int timer32_set_period(timer32_t * this, uint64_t period_us)
{
    /* local declaration */
    uint32_t prescaler;
    uint64_t counter;
    /* program statement */
    counter = (uint64_t)SystemCoreClock*period_us; /* compute the counter with prescaler include */
    counter /= 1000000;
    prescaler = counter>>32;    /* compute prescaler */
    this->timer32->CONTROL &= ~TIMER32_CONTROL_PRESCALE_MASK; /* reset prescaler */
    if(prescaler == 0)
    {
        ;   /* do nothing, prescaler already configured with 1 and counter get the value */
    }
    else if(prescaler < 16)
    {
        this->timer32->CONTROL |= TIMER32_CONTROL_PRESCALE_1; /* prescale by 16 */
        counter = SystemCoreClock/16*period_us;  /* compute counter with the prescale */
        counter /= 1000000;
    }
    else if(prescaler < 256)
    {
        this->timer32->CONTROL |= TIMER32_CONTROL_PRESCALE_2; /* prescale by 256 */
        counter = SystemCoreClock/256*period_us; /* compute counter with the prescale */
        counter /= 1000000;
    }
    else
    {
        return -1;  /* period_us is too high */
    }
    if(counter <= 0x000000000000FFFF) /* is the counter value under or equal 16-bit */
    {
        this->timer32->CONTROL &= ~TIMER32_CONTROL_SIZE; /* configure a 16-bit counter */
    }
    else
    {
        this->timer32->CONTROL |= TIMER32_CONTROL_SIZE; /* configure a 32-bit counter */
    }
    this->timer32->LOAD = (uint32_t)counter; /* load the counter value */
    return 0;
}

/******************************************************************************/
/**
* @biref    Initializing timer in Periodic timer mode: The counter generates an interrupt
* at a constant interval, reloading the original value after wrapping past zero.
*
* @param    this is a pointer to the instance timer32_t that you want to
*           initialize.
* @param    period_us is the period in microsecond that will be set for the timer.
*           (maximum period is 733007751851 us (8d 11h 36m 45s 751851 us) when
*           the system clock is configure on 1.5 MHz
*
* @return   -1 if the period is too high else 0.
*
* @note     this function stop the timer before setup the timer mode, and
*           disable the interrupt, you have to start the timer with
*           timer32_start function and reconfigure interrupt with
*           timer32_set_interrupt function. period to small with interrupt
*           activated could stuck the program in the interrupt function.
*
*******************************************************************************/
int timer32_periodic_timer(timer32_t * this, uint64_t period_us)
{
    this->timer32->CONTROL &= ~TIMER32_CONTROL_ENABLE; /* Stop the timer */
    this->timer32->CONTROL |= TIMER32_CONTROL_MODE; /* periodic mode */
    this->timer32->CONTROL &= ~TIMER32_CONTROL_IE;  /* disable interrupt */
    this->timer32->CONTROL &= ~TIMER32_CONTROL_ONESHOT; /* wrapping mode */
    return timer32_set_period(this, period_us);
}

/******************************************************************************/
/**
* @biref    Initializing timer in One-shot timer mode: The counter generates an interrupt
* once. When the counter reaches zero, it halts until reprogrammed by the user.
*
* @param    this is a pointer to the instance timer32_t that you want to
*           initialize.
* @param    period_us is the period in microsecond that will be set for the timer.
*           (maximum period is 733007751851 us (8d 11h 36m 45s 751851 us) when
*           the system clock is configure on 1.5 MHz
*
* @return   -1 if the period is too high else 0.
*
* @note     this function stop the timer before setup the timer mode, and
*           disable the interrupt, you have to start the timer with
*           timer32_start function and reconfigure interrupt with
*           timer32_set_interrupt function. Period to small with interrupt
*           activated could stuck the program in the interrupt function.
*
*******************************************************************************/
int timer32_one_shot_timer(timer32_t * this, uint64_t period_us)
{
    this->timer32->CONTROL &= ~TIMER32_CONTROL_ENABLE; /* Stop the timer */
    this->timer32->CONTROL |= TIMER32_CONTROL_MODE; /* periodic mode (to set the start value) */
    this->timer32->CONTROL &= ~TIMER32_CONTROL_IE; /* disable interrupt */
    this->timer32->CONTROL |= TIMER32_CONTROL_ONESHOT; /* one-shot mode */
    return timer32_set_period(this, period_us);
}

/******************************************************************************/
/**
* @biref    Start the timer.
*
* @param    this is a pointer to the instance timer32_t.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void timer32_start(timer32_t * this)
{
    this->timer32->CONTROL |= TIMER32_CONTROL_ENABLE; /* start the timer */
}

/******************************************************************************/
/**
* @biref    Pause the timer.
*
* @param    this is a pointer to the instance timer32_t.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void timer32_pause(timer32_t * this)
{
    this->timer32->CONTROL &= ~TIMER32_CONTROL_ENABLE; /* stop the timer */
}

/******************************************************************************/
/**
* @biref    Return the value of the timer.
*
* @param    this is a pointer to the instance timer32_t.
*
* @return   The raw value of the timer.
*
* @note     None.
*
*******************************************************************************/
uint32_t timer32_value(timer32_t * this)
{
    return this->timer32->VALUE; /* raw value of the timer */
}

/******************************************************************************/
/**
* @biref    Set a load value for the timer
*
* @param    this is a pointer to the instance timer32_t.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void timer32_load(timer32_t * this, uint32_t value)
{
    this->timer32->LOAD = this->timer32->BGLOAD = value; /* load value and reload value */
}

/******************************************************************************/
/**
* @biref    Set a new period in microseconds for the timer
*
* @param    this is a pointer to the instance timer32_t.
*
* @return   -1 if the period is too high else 0.
*
* @note     Stop the timer must be start after use of this function the
*           timer32_start function.
*
*******************************************************************************/
int timer32_period_us(timer32_t * this, uint64_t period_us)
{
    this->timer32->CONTROL &= ~TIMER32_CONTROL_ENABLE; /* stop the timer */
    return timer32_set_period(this, period_us);
}

/******************************************************************************/
/**
* @biref    Configure the interrupt function when timer reach 0;
*
* @param    this is a pointer to the instance timer32_t.
* @param    interrupt_handler is a pointer to the interrupt function that must
*           be use.
*
* @return   -1 when the timer isn't configured else 0.
*
* @note     interrupt_handler will be faster and not take more place if you make
*           an inline function.
*           Don't forget to enable global interrupts with __enable_interrupts();
*
*******************************************************************************/
int timer32_enable_interrupt(timer32_t * this, void (*interrupt_handler)(void), uint32_t priority)
{
    this->timer32->CONTROL &= ~TIMER32_CONTROL_IE; /* disable interrupt */
    if(this->timer32 == TIMER32_1)
    {
        __NVIC_EnableIRQ(T32_INT1_IRQn); /* Enable IRQ of the TIMER32_1 */
        __NVIC_SetPriority(T32_INT1_IRQn, priority); /* set the priority of the IRQ */
        interrupt_handler_T32_INT1 = interrupt_handler; /* set the interrupt function */
    }
    else if(this->timer32 == TIMER32_2)
    {
        __NVIC_EnableIRQ(T32_INT2_IRQn); /* Enable IRQ of the TIMER32_2 */
        __NVIC_SetPriority(T32_INT2_IRQn, priority); /* set the priority of the IRQ */
        interrupt_handler_T32_INT2 = interrupt_handler; /* set the interrupt function */
    }
    else
    {
        return -1;
    }
    this->timer32->CONTROL |= TIMER32_CONTROL_IE; /* enable interrupt */
    this->timer32->INTCLR = 0xFFFFFFFF; /* clear interrupt */
    return 0;
}

/******************************************************************************/
/**
* @brief    Disable the interrupt function of the timer
*
* @param    this is a pointer to the instance timer32_t.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
int timer32_disable_interrupt(timer32_t * this)
{
    this->timer32->CONTROL &= ~TIMER32_CONTROL_IE; /* disable interrupt */
    if(this->timer32 == TIMER32_1)
    {
        __NVIC_DisableIRQ(T32_INT1_IRQn); /* Enable IRQ of the TIMER32_1 */
        interrupt_handler_T32_INT1 = default_handler; /* set the interrupt function */
    }
    else if(this->timer32 == TIMER32_2)
    {
        __NVIC_DisableIRQ(T32_INT2_IRQn); /* Enable IRQ of the TIMER32_2 */
        interrupt_handler_T32_INT2 = default_handler; /* set the interrupt function */
    }
    else
    {
        return -1;
    }
    this->timer32->INTCLR = 0xFFFFFFFF; /* clear interrupt */
    return 0;
}

/*******************************   INTERRUPT    *******************************/
void T32_INT1_IRQHandler(void)
{
    interrupt_handler_T32_INT1();   /* call interrupt function of the user */
    TIMER32_1->INTCLR = 0xFFFFFFFF; /* clear interrupt */
}

void T32_INT2_IRQHandler(void)
{
    interrupt_handler_T32_INT2();   /* call interrupt function of the user */
    TIMER32_2->INTCLR = 0xFFFFFFFF; /* clear interrupt */
}
