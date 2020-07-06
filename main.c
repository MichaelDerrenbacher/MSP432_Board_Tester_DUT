#include "msp.h"


/**
 * main.c
 */

#define PORT_1_PINS    (BIT5 | BIT6 | BIT7)
#define PORT_2_PINS    (BIT3 | BIT4 | BIT5 | BIT6 | BIT7)
#define PORT_3_PINS    (BIT0 | BIT2 | BIT3 | BIT5 | BIT6 | BIT7)
#define PORT_4_PINS    (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7)
#define PORT_5_PINS    (BIT0 | BIT1 | BIT2 | BIT4 | BIT5 | BIT6 | BIT7)
#define PORT_6_PINS    (BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7)



void init_test_pins(void);
void init_irq(void);
void init_uart(void);


void output_low(void);
void output_high(void);

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    init_test_pins();

    init_irq();

    while(1);
}

void output_low(void)
{
    P1->OUT &= ~PORT_1_PINS;
    P2->OUT &= ~PORT_2_PINS;
    P3->OUT &= ~PORT_3_PINS;
    P4->OUT &= ~PORT_4_PINS;
    P5->OUT &= ~PORT_5_PINS;
    P6->OUT &= ~PORT_6_PINS;

}


void output_high(void)
{
    P1->OUT |=  PORT_1_PINS;
    P2->OUT |=  PORT_2_PINS;
    P3->OUT |=  PORT_3_PINS;
    P4->OUT |=  PORT_4_PINS;
    P5->OUT |=  PORT_5_PINS;
    P6->OUT |=  PORT_6_PINS;

}

/*
 *   Set all exposed Lanchpad pins to outputs
 *   with default value high
 *
 */
void init_test_pins(void)
{

    P1->DIR |=  PORT_1_PINS;  // Port is all outputs
    P1->OUT |=  PORT_1_PINS;  // output defaults high

    P2->DIR |=  PORT_2_PINS;
    P2->OUT |=  PORT_2_PINS;

    P3->DIR |=  PORT_3_PINS;
    P3->OUT |=  PORT_3_PINS;

    P4->DIR |=  PORT_4_PINS;
    P4->OUT |=  PORT_4_PINS;

    P5->DIR |=  PORT_5_PINS;
    P5->OUT |=  PORT_5_PINS;

    P6->DIR |=  PORT_6_PINS;
    P6->OUT |=  PORT_6_PINS;

}

/*
 *  Enable interrupts for Lanchpad button press
 */
void init_irq()
{

    P1->DIR &= ~(BIT1 | BIT4);         // Lanchpad buttons are inputs
    P1->REN |=  (BIT1 | BIT4);         // enable internal resistor
    P1->OUT |=  (BIT1 | BIT4);         // resistor is pullup


    P1->IES |=  (BIT1 | BIT4);         // interrupts trigger on falling edge for button press
    P1->IFG &= ~(BIT1 | BIT4);         // clear flags
    P1->IE  |=  (BIT1 | BIT4);         // enable interrupts for Lanchpad buttons

    NVIC->ISER[1] = 1 << (PORT1_IRQn & 31);      // NVIC interrupt enable for P1

    __enable_irq();                            // global interrupt enable

}



/*
 *  Lanchpad external buttons trigger output
 */
void PORT1_IRQHandler(void)
{
    if(P1->IFG & BIT1) // check for output low
    {
        P1->IFG &= ~(BIT1);  // clear interrupt flag

        output_low();   // testing that outputs are low

    }
    else if(P1->IFG & BIT4) // check for output high
    {
        P1->IFG &= ~(BIT4);  // clear interrupt flag

        output_high();  // testing that outputs are high

    }
    else
    {
        while(1);

        // some crazy error occurred, infinite loop for debugging purposes
        // how did you mess up this bad??? Think about whatever you've done

    }
}




