/*******************************************************************************/
/**
*      @file : gpio.h
*   @version : 0
*      @date : December 8, 2018
*    @author : Enzo IGLESIS
* @copyright : Copyright (c) 2018 Enzo IGLESIS, MIT License (MIT)
*******************************************************************************/

#ifndef GPIO_H_
#   define GPIO_H_

/*******************************    LIBRARYS    *******************************/
#include "msp432p401r.h"

/*******************************     MACROS     *******************************/
/**** GPIO ****/
/**
PA: port A  (P2_H & P1_L)   (DIO_PORT_Interruptable_Type*)
PB: port B  (P4_H & P3_L)   (DIO_PORT_Interruptable_Type*)
PC: port C  (P6_H & P5_L)   (DIO_PORT_Interruptable_Type*)
PD: port D  (P8_H & P7_L)   (DIO_PORT_Interruptable_Type*)
PE: port E  (P10_H & P9_L)  (DIO_PORT_Interruptable_Type*)

PJ: port J  (DIO_PORT_Not_Interruptable_Type*)

P1: port 1  (DIO_PORT_Odd_Interruptable_Type*)
P2: port 2  (DIO_PORT_Even_Interruptable_Type*)
P3: port 3  (DIO_PORT_Odd_Interruptable_Type*)
P4: port 4  (DIO_PORT_Even_Interruptable_Type*)
P5: port 5  (DIO_PORT_Odd_Interruptable_Type*)
P6: port 6  (DIO_PORT_Even_Interruptable_Type*)
P7: port 7  (DIO_PORT_Odd_Interruptable_Type*)
P8: port 8  (DIO_PORT_Even_Interruptable_Type*)
P9: port 9  (DIO_PORT_Odd_Interruptable_Type*)
P10:port 10 (DIO_PORT_Even_Interruptable_Type*)
**/
/**** PIN ****/
#define PIN0    BIT0
#define PIN1    BIT1
#define PIN2    BIT2
#define PIN3    BIT3
#define PIN4    BIT4
#define PIN5    BIT5
#define PIN6    BIT6
#define PIN7    BIT7
#define PIN8    BIT8    /* _H 0 */
#define PIN9    BIT9    /* _H 1 */
#define PINA    BITA    /* _H 2 */
#define PINB    BITB    /* _H 3 */
#define PINC    BITC    /* _H 4 */
#define PIND    BITD    /* _H 5 */
#define PINE    BITE    /* _H 6 */
#define PINF    BITF    /* _H 7 */

/*******************************     TYPES      *******************************/
typedef struct gpio_t
{
    /**** attributes ****/
    union
    {
        DIO_PORT_Interruptable_Type * port;
        DIO_PORT_Odd_Interruptable_Type * port_odd;
        DIO_PORT_Even_Interruptable_Type * port_even;
    };

    union
    {
        uint16_t pin;
        struct
        {
            uint8_t pin_odd;
            uint8_t pin_even;
        };
    };
    /**** functions ****/
        /**** configuration ****/
    void (*input) (struct gpio_t * this);
    void (*input_pulldown) (struct gpio_t * this);
    void (*input_pullup) (struct gpio_t * this);
    void (*output) (struct gpio_t * this);
    void (*primary_module) (struct gpio_t * this);
    void (*secondary_module) (struct gpio_t * this);
    void (*tertiary_module) (struct gpio_t * this);
    /**** drive ****/
        /**** write ****/
    void (*set) (struct gpio_t * this);
    void (*clear) (struct gpio_t * this);
    void (*toggle) (struct gpio_t * this);
        /**** read ****/
    uint16_t (*read) (struct gpio_t * this);
    uint8_t (*read_odd) (struct gpio_t * this);
    uint8_t (*read_even) (struct gpio_t * this);
    /**** Interrupt ****/
    int (*enable_interrupt_rising_edge_odd) (struct gpio_t * this, void (*interrupt_handler)(void), uint32_t priority);
    int (*enable_interrupt_falling_edge_odd) (struct gpio_t * this, void (*interrupt_handler)(void), uint32_t priority);
    int (*enable_interrupt_rising_edge_even) (struct gpio_t * this, void (*interrupt_handler)(void), uint32_t priority);
    int (*enable_interrupt_falling_edge_even) (struct gpio_t * this, void (*interrupt_handler)(void), uint32_t priority);

}gpio_t;

/*******************************   FUNCTIONS    *******************************/
/**** configuration ****/
void gpio_initialize(gpio_t * gpio, DIO_PORT_Interruptable_Type * port, uint16_t pin);
void gpio_initialize_odd(gpio_t * gpio, DIO_PORT_Odd_Interruptable_Type * port, uint8_t pin);
void gpio_initialize_even(gpio_t * gpio, DIO_PORT_Even_Interruptable_Type * port, uint8_t pin);
void gpio_input(gpio_t * gpio);
void gpio_input_pulldown(gpio_t * gpio);
void gpio_input_pullup(gpio_t * gpio);
void gpio_output(gpio_t * gpio);
void gpio_primary_module(gpio_t * gpio);
void gpio_secondary_module(gpio_t * gpio);
void gpio_tertiary_module(gpio_t * gpio);

/**** drive ****/
    /**** write ****/
void gpio_set(gpio_t * gpio);
void gpio_clear(gpio_t * gpio);
void gpio_toggle(gpio_t * gpio);
    /**** read ****/
uint16_t gpio_read(gpio_t * gpio);
uint8_t gpio_read_odd(gpio_t * gpio);
uint8_t gpio_read_even(gpio_t * gpio);

/**** Interrupt ****/
int gpio_enable_interrupt_rising_edge_odd(gpio_t * this, void (*interrupt_handler)(void), uint32_t priority);
int gpio_enable_interrupt_falling_edge_odd(gpio_t * this, void (*interrupt_handler)(void), uint32_t priority);
int gpio_enable_interrupt_rising_edge_even(gpio_t * this, void (*interrupt_handler)(void), uint32_t priority);
int gpio_enable_interrupt_falling_edge_even(gpio_t * this, void (*interrupt_handler)(void), uint32_t priority);

#endif /* GPIO_H_ */
