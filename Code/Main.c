/*************************************************************************************
 *  Main.c
 *
 *  Description: The main code of the second mini project in the embedded diploma
 *  Created on: Sep 26, 2018
 *  Author: Ahmad Ganzouri
 *************************************************************************************/

#include "lcd.h"
#include "adc.h"
#define TIMER0_MAX 255
#define ADC_MAX 1023

/* ISR of the Interrupt 2 to reverse the direction of the motor at the falling edge */

ISR(INT2_vect)
{
	PORTB ^= (1<<PB1);
	PORTB ^= (1<<PB0);
}

/* functions prototypes*/

void INT2_Init(void);
void Timer0_PWM_Init(unsigned char set_duty_cycle);

/* global variables definitions */
uint16 duty_cycle;
uint16 pot_value;



int main ()
{
	LCD_init();
	ADC_init();
	Timer0_PWM_Init(0);     /* Init of TIMER0 PWM with 0 duty cycle */
	INT2_Init();            /* Init the Interrupt and SREG */
	DDRB |= (1<<PB0) | (1 << PB1); /* Setting PB0 , PB1 as output pins for the motor */
	PORTB|= (1<<PB0);       /* setting the direction of the motor at the first PB0 =1 , PB1 = 0 */
	LCD_clearScreen();
	LCD_displayString("MINI PROJECT 2");
	LCD_goToRowColumn(1,0);
	LCD_displayString("SPEED");
	while(1)
	{
	/*Reading the value of the pot and convert it by ADC Channel 0*/
     pot_value = ADC_readChannel(0);
     /* Converting the value to the range of 0~256 to fit the TIMER0 range */
     duty_cycle = ((float)pot_value/ADC_MAX)*TIMER0_MAX;
     /* write the duty cycle value into Register OCR0 to change the speed */
     OCR0 = duty_cycle;

     /* to show the percentage of the motor speed at LCD */

     LCD_displayString("   ");
     LCD_goToRowColumn(1,6);
     LCD_intgerToString((duty_cycle*100)/TIMER0_MAX);
     LCD_displayCharacter('%');
	}
}


/* External INT2 enable and configuration function */
void INT2_Init(void)
{
	SREG   &= ~(1<<7);      /* Disable interrupts by clearing I-bit*/
	DDRB   &= (~(1<<PB2));   /* Configure INT2/PB2 as input pin */
	PORTB  |= (1<<PB2);      /* enable internal pull up resistor */
	GICR   |= (1<<INT2);	/* Enable external interrupt pin INT2*/
	MCUCSR &= ~(1<<ISC2);    /* Trigger INT2 with the falling edge*/
	SREG   |= (1<<7);       /* Enable interrupts by setting I-bit*/
}

void Timer0_PWM_Init(unsigned char set_duty_cycle)
{

	TCNT0 = 0; // Timer initial value

	OCR0  = set_duty_cycle; //compare value

	DDRB  = DDRB | (1<<PB3); //set OC0 as output pin --> pin where the PWM signal is generated from MC

	/* Configure timer control register
	 * 1. Fast PWM mode FOC0=0
	 * 2. Fast PWM Mode WGM01=1 & WGM00=1
	 * 3. Clear OC0 when match occurs (non inverted mode) COM00=0 & COM01=1
	 * 4. clock = F_CPU/8 CS00=0 CS01=1 CS02=0
	 */
	TCCR0 = (1<<WGM00) | (1<<WGM01) | (1<<COM01) | (1<<CS01);
}
