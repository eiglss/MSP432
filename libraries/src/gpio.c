/*******************************************************************************/
/**
*      @file : gpio.c
*   @version : 0
*      @date : December 2, 2017
*    @author : Enzo IGLESIS
* @copyright : Copyright (c) 2018 Enzo IGLESIS, MIT License (MIT)
*******************************************************************************/

/*******************************    LIBRARIES   *******************************/
#include "gpio.h"

/*******************************   FUNCTIONS    *******************************/
/**** interrupt function ****/ /* see INTERRUPT section */
static void default_handler(void){while(!0);}

/**** GPIO ****/
static void (* interrupt_handler_PORT1)(void) = default_handler;
static void (* interrupt_handler_PORT2)(void) = default_handler;
static void (* interrupt_handler_PORT3)(void) = default_handler;
static void (* interrupt_handler_PORT4)(void) = default_handler;
static void (* interrupt_handler_PORT5)(void) = default_handler;
static void (* interrupt_handler_PORT6)(void) = default_handler;

/**** configuration ****/
/******************************************************************************/
/**
* @biref    Initializing function pointer of the structure gpio_t.
*
* @param    this is a pointer to the instant gpio_t that you want to initialize.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
static void gpio_initialize_struct(gpio_t * this)
{
    /* input */
    this->input = gpio_input;
    this->input_pulldown = gpio_input_pulldown;
    this->input_pullup = gpio_input_pullup;
    this->output = gpio_output;
    this->primary_module = gpio_primary_module;
    this->secondary_module = gpio_secondary_module;
    this->tertiary_module = gpio_tertiary_module;
    this->set = gpio_set;
    this->clear = gpio_clear;
    this->toggle = gpio_toggle;
    this->read = gpio_read;
    this->read_odd = gpio_read_odd;
    this->read_even = gpio_read_even;
    this->enable_interrupt_rising_edge_odd = gpio_enable_interrupt_rising_edge_odd;
    this->enable_interrupt_falling_edge_odd = gpio_enable_interrupt_falling_edge_odd;
    this->enable_interrupt_rising_edge_even = gpio_enable_interrupt_rising_edge_even;
    this->enable_interrupt_falling_edge_even = gpio_enable_interrupt_falling_edge_even;
}

/******************************************************************************/
/**
* @biref  Initializing function pointer of the structure gpio_t and assign the
*         specified port to the and pin to the instance.
*
* @param    this is a pointer to the instant gpio_t that you want to initialize.
* @param    port is a pointer to the port to assign to the instance.
* @param    pin is the pin to assign to the instance.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void gpio_initialize(gpio_t * this, DIO_PORT_Interruptable_Type * port, uint16_t pin)
{
    gpio_initialize_struct(this);
    this->port = port;
    this->pin = pin;
}

/******************************************************************************/
/**
* @biref    Initializing function pointer of the structure gpio_t and assign the
*           specified port to the and pin to the instance.
*
* @param    this is a pointer to the instant gpio_t that you want to initialize.
* @param    port is a pointer to the port to assign to the instance.
* @param    pin is the pin to assign to the instance.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void gpio_initialize_odd(gpio_t * this, DIO_PORT_Odd_Interruptable_Type * port, uint8_t pin)
{
    gpio_initialize_struct(this);
    this->port_odd = port;
    this->pin_odd = pin;
    this->pin_even = 0x00;
}

/******************************************************************************/
/**
* @biref    Initializing function pointer of the structure gpio_t and assign the
*           specified port to the and pin to the instance.
*
* @param    this is a pointer to the instant gpio_t that you want to initialize.
* @param    port is a pointer to the port to assign to the instance.
* @param    pin is the pin to assign to the instance.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void gpio_initialize_even(gpio_t * this, DIO_PORT_Even_Interruptable_Type * port, uint8_t pin)
{
    gpio_initialize_struct(this);
    this->port_even = port;
    this->pin_odd = 0x00;
    this->pin_even = pin;
}

/******************************************************************************/
/**
* @biref    Setup gpio instance as input.
*
* @param    this is a pointer to the instant gpio_t that you want to set as
*           input
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void gpio_input(gpio_t * this)
{
    this->port->SEL1 &= ~this->pin;
    this->port->SEL0 &= ~this->pin;
    this->port->DIR &= ~this->pin;
    this->port->REN &= ~this->pin;
}

/******************************************************************************/
/**
* @biref    Setup gpio instance as input pull-down.
*
* @param    this is a pointer to the instant gpio_t that you want to set as
*           input pull-down.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void gpio_input_pulldown(gpio_t * this)
{
    this->port->SEL1 &= ~this->pin;
    this->port->SEL0 &= ~this->pin;
    this->port->DIR &= ~this->pin;
    this->port->REN |= this->pin;
    this->port->OUT &= ~this->pin;
}

/******************************************************************************/
/**
* @biref    Setup gpio instance as input pull-up.
*
* @param    this is a pointer to the instant gpio_t that you want to set as
*           input pull-up.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void gpio_input_pullup(gpio_t * this)
{
    this->port->SEL1 &= ~this->pin;
    this->port->SEL0 &= ~this->pin;
    this->port->DIR &= ~this->pin;
    this->port->REN |= this->pin;
    this->port->OUT |= this->pin;
}


/******************************************************************************/
/**
* @biref    Setup gpio instance as OUPUT.
*
* @param    this is a pointer to the instant gpio_t that you want to set as
*           output.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void gpio_output(gpio_t * this)
{
    this->port->SEL1 &= ~this->pin;
    this->port->SEL0 &= ~this->pin;
    this->port->DIR |= this->pin;
}

/******************************************************************************/
/**
* @biref    Setup gpio instance for peripheral function of the primary module.
*
* @param    this is a pointer to the instant gpio_t that you want to set for
*           peripheral function of the primary module.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void gpio_primary_module(gpio_t * this)
{
    this->port->SEL1 &= ~this->pin;
    this->port->SEL0 |= this->pin;
}

/******************************************************************************/
/**
* @biref    Setup gpio instance for peripheral function of the secondary module.
*
* @param    this is a pointer to the instant gpio_t that you want to set for
*           peripheral function of the secondary module.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void gpio_secondary_module(gpio_t * this)
{
    this->port->SEL1 |= this->pin;
    this->port->SEL0 &= ~this->pin;
}

/******************************************************************************/
/**
* @biref    Setup gpio instance for peripheral function of the tertiary module.
*
* @param    this is a pointer to the instant gpio_t that you want to set for
*           peripheral function of the tertiary module.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void gpio_tertiary_module(gpio_t * this)
{
    this->port->SEL1 |= this->pin;
    this->port->SEL0 |= this->pin;
}

/**** drive ****/
    /**** write ****/
/******************************************************************************/
/**
* @biref    Set gpio to HIGHT
*
* @param    this is a pointer to the instant gpio_t that you want to set.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void gpio_set(gpio_t * this)
{
    this->port->OUT |= this->pin;
}

/******************************************************************************/
/**
* @biref    Set gpio to LOW
*
* @param    this is a pointer to the instant gpio_t that you want to clear.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void gpio_clear(gpio_t * this)
{
    this->port->OUT &= ~this->pin;
}

/******************************************************************************/
/**
* @biref    Set gpio to the opposite state of the current state.
*
* @param    this is a pointer to the instant gpio_t that you want to toggle.
*
* @return   None.
*
* @note     None.
*
*******************************************************************************/
void gpio_toggle(gpio_t * this)
{
    this->port->OUT ^= this->pin;
}

    /**** read ****/
/******************************************************************************/
/**
* @brief    Read the state of the gpio instance
*
* @param    this is a pointer to the instant gpio_t that you want to read.
*
* @return   The current state of the pin. PIN0 is on bit 0, PIN1 on bit 1 ...
*           PINF on bit f.
*
* @note     Return is on 16 bits, the port must be considerer as
*           DIO_PORT_Interruptable_Type.
*
*******************************************************************************/
uint16_t gpio_read(gpio_t * this)
{
    return (this->port->IN)&(this->pin);
}

/******************************************************************************/
/**
* @biref    Read the state of the gpio instance
*
* @param    this is a pointer to the instant gpio_t that you want to read.
*
* @return   The current state of the pin. PIN0 is on bit 0, PIN1 on bit 1 ...
*           PIN7 on bit 7.
*
* @note     Return is on 8 bits, the port must be considerer as
*           DIO_PORT_Odd_Interruptable_Type (P1, P3, ...)
*
*******************************************************************************/
uint8_t gpio_read_odd(gpio_t * this)
{
    return (this->port_odd->IN)&(this->pin_odd);
}

/******************************************************************************/
/**
* @biref    Read the state of the gpio instance
*
* @param    this is a pointer to the instant gpio_t that you want to read.
*
* @return   The current state of the pin. PIN0 is on bit 0, PIN1 on bit 1 ...
*           PIN7 on bit 7.
*
* @note     Return is on 8 bits, the port must be considerer as
*           DIO_PORT_Even_Interruptable_Type (P2, P4, ...)
*
*******************************************************************************/
uint8_t gpio_read_even(gpio_t * this)
{
    return (this->port_even->IN)&(this->pin_even);
}

/**** interrupt ****/
/******************************************************************************/
/**
* @biref    Enable interrupt function on odd port.
*
* @param    this is a pointer to the instance gpio_t.
* @param    interrupt_handler is a pointer to the interrupt function that will
*           be execute when the interrupt flag is raised.
* @param    priority is the priority of the interrupt.
*
* @return   -1 when the port is not supported for the interrupt else 0.
*
*******************************************************************************/
static int gpio_enable_interrupt_odd(gpio_t * this, void (*interrupt_handler)(void), uint32_t priority)
{
    this->port_odd->IE &= ~(this->pin_odd); /* Interrupt disabled */
    if(this->port_odd == P1)
    {
        __NVIC_EnableIRQ(PORT1_IRQn); /* Enable IRQ of the P1 */
        __NVIC_SetPriority(PORT1_IRQn, priority); /* set the priority of the IRQ */
        interrupt_handler_PORT1 = interrupt_handler; /* set the interrupt function */
    }
    else if(this->port_odd == P3)
    {
        __NVIC_EnableIRQ(PORT3_IRQn); /* Enable IRQ of the P1 */
        __NVIC_SetPriority(PORT3_IRQn, priority); /* set the priority of the IRQ */
        interrupt_handler_PORT3 = interrupt_handler; /* set the interrupt function */
    }
    else if(this->port_odd == P5)
    {
        __NVIC_EnableIRQ(PORT5_IRQn); /* Enable IRQ of the P1 */
        __NVIC_SetPriority(PORT5_IRQn, priority); /* set the priority of the IRQ */
        interrupt_handler_PORT5 = interrupt_handler; /* set the interrupt function */
    }
    else
    {
        return -1;
    }
    this->port_odd->IE |= this->pin_odd; /* Interrupt enable */
    return 0;
}

/******************************************************************************/
/**
* @biref    Enable interrupt function for rising edge on odd port.
*
* @param    this is a pointer to the instance gpio_t.
* @param    interrupt_handler is a pointer to the interrupt function that will
*           be execute when the interrupt flag is raised.
* @param    priority is the priority of the interrupt.
*
* @return   -1 when the port is not supported for the interrupt else 0.
*
* @note     interrupt_handler will be faster and not take more place if you make
*           an inline function.
*           Don't forget to enable global interrupts with
*           `__enable_interrupts();` this function must be executed after
*           configuring all interrupt function.
*
*******************************************************************************/
int gpio_enable_interrupt_rising_edge_odd(gpio_t * this, void (*interrupt_handler)(void), uint32_t priority)
{
    this->port_odd->IES &= ~(this->pin_odd);
    return gpio_enable_interrupt_odd(this, interrupt_handler, priority);
}

/******************************************************************************/
/**
* @biref    Enable interrupt function for falling edge on odd port.
*
* @param    this is a pointer to the instance gpio_t.
* @param    interrupt_handler is a pointer to the interrupt function that will
*           be execute when the interrupt flag is raised.
* @param    priority is the priority of the interrupt.
*
* @return   -1 when the port is not supported for the interrupt else 0.
*
* @note     interrupt_handler will be faster and not take more place if you make
*           an inline function.
*           Don't forget to enable global interrupts with
*           `__enable_interrupts();` this function must be executed after
*           configuring all interrupt function.
*
*******************************************************************************/
int gpio_enable_interrupt_falling_edge_odd(gpio_t * this, void (*interrupt_handler)(void), uint32_t priority)
{
    this->port_odd->IES |= this->pin_odd;
    return gpio_enable_interrupt_odd(this, interrupt_handler, priority);
}

/******************************************************************************/
/**
* @biref    Enable interrupt function on even port.
*
* @param    this is a pointer to the instance gpio_t.
* @param    interrupt_handler is a pointer to the interrupt function that will
*           be execute when the interrupt flag is raised.
* @param    priority is the priority of the interrupt.
*
* @return   -1 when the port is not supported for the interrupt else 0.
*
*******************************************************************************/
static int gpio_enable_interrupt_even(gpio_t * this, void (*interrupt_handler)(void), uint32_t priority)
{
    this->port_even->IE &= ~(this->pin_even); /* Interrupt disabled */
    if(this->port_even == P2)
    {
        __NVIC_EnableIRQ(PORT2_IRQn); /* Enable IRQ of the P1 */
        __NVIC_SetPriority(PORT2_IRQn, priority); /* set the priority of the IRQ */
        interrupt_handler_PORT2 = interrupt_handler; /* set the interrupt function */
    }
    else if(this->port_even == P4)
    {
        __NVIC_EnableIRQ(PORT4_IRQn); /* Enable IRQ of the P1 */
        __NVIC_SetPriority(PORT4_IRQn, priority); /* set the priority of the IRQ */
        interrupt_handler_PORT4 = interrupt_handler; /* set the interrupt function */
    }
    else if(this->port_even == P6)
    {
        __NVIC_EnableIRQ(PORT6_IRQn); /* Enable IRQ of the P1 */
        __NVIC_SetPriority(PORT6_IRQn, priority); /* set the priority of the IRQ */
        interrupt_handler_PORT6 = interrupt_handler; /* set the interrupt function */
    }
    else
    {
        return -1;
    }
    this->port_even->IE |= this->pin_even; /* Interrupt enable */
    return 0;
}

/******************************************************************************/
/**
* @biref    Enable interrupt function for rising edge on even port.
*
* @param    this is a pointer to the instance gpio_t.
* @param    interrupt_handler is a pointer to the interrupt function that will
*           be execute when the interrupt flag is raised.
* @param    priority is the priority of the interrupt.
*
* @return   -1 when the port is not supported for the interrupt else 0.
*
* @note     interrupt_handler will be faster and not take more place if you make
*           an inline function.
*           Don't forget to enable global interrupts with
*           `__enable_interrupts();` this function must be executed after
*           configuring all interrupt function.
*
*******************************************************************************/
int gpio_enable_interrupt_rising_edge_even(gpio_t * this, void (*interrupt_handler)(void), uint32_t priority)
{
    this->port_even->IES &= ~(this->pin_even);
    return gpio_enable_interrupt_even(this, interrupt_handler, priority);
}

/******************************************************************************/
/**
* @biref    Enable interrupt function for falling edge on even port.
*
* @param    this is a pointer to the instance gpio_t.
* @param    interrupt_handler is a pointer to the interrupt function that will
*           be execute when the interrupt flag is raised.
* @param    priority is the priority of the interrupt.
*
* @return   -1 when the port is not supported for the interrupt else 0.
*
* @note     interrupt_handler will be faster and not take more place if you make
*           an inline function.
*           Don't forget to enable global interrupts with
*           `__enable_interrupts();` this function must be executed after
*           configuring all interrupt function.
*
*******************************************************************************/
int gpio_enable_interrupt_falling_edge_even(gpio_t * this, void (*interrupt_handler)(void), uint32_t priority)
{
    this->port_even->IES |= this->pin_even;
    return gpio_enable_interrupt_even(this, interrupt_handler, priority);
}

/*******************************   INTERRUPT    *******************************/
void PORT1_IRQHandler(void)
{
    interrupt_handler_PORT1(); /* call interrupt handler */
    P1->IFG &= ~P1IFG_M; /* reset interrupt flag */
}

void PORT2_IRQHandler(void)
{
    interrupt_handler_PORT2(); /* call interrupt handler */
    P2->IFG &= ~P2IFG_M; /* reset interrupt flag */
}

void PORT3_IRQHandler(void)
{
    interrupt_handler_PORT3(); /* call interrupt handler */
    P3->IFG &= ~P3IFG_M; /* reset interrupt flag */
}

void PORT4_IRQHandler(void)
{
    interrupt_handler_PORT4(); /* call interrupt handler */
    P4->IFG &= ~P4IFG_M; /* reset interrupt flag */
}

void PORT5_IRQHandler(void)
{
    interrupt_handler_PORT5(); /* call interrupt handler */
    P5->IFG &= ~P5IFG_M; /* reset interrupt flag */
}

void PORT6_IRQHandler(void)
{
    interrupt_handler_PORT6(); /* call interrupt handler */
    P6->IFG &= ~P6IFG_M; /* reset interrupt flag */
}
