
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */


void delay_ms(unsigned short delay_t)
{
	SIM->SCGC6 |= (1 << 24);
	SIM->SOPT2 |= (0x2 << 24);
	TPM0->CONF |= (0x1 << 17);
	TPM0->SC = (1 << 7) | (0x7);
	TPM0->MOD = delay_t*62 + delay_t/2 -1;
	TPM0->SC |= 0x01 << 3;
	while(!(TPM0->SC & 0x80)){}
	return;

}

void servoCenter()
{
    TPM1->CONTROLS[0].CnV = 1400;
}

void servoLeft()
{
    TPM1->CONTROLS[0].CnV = 2400;
}

void servoRight()
{
    TPM1->CONTROLS[0].CnV = 400;
}


void setup_GreenLED() {
	SIM->SCGC5 |= (1<<12); // Setup port for GPIO
	PORTD->PCR[5] &= ~0x700; // Clear First
	PORTD->PCR[5] |= 0x700 & (1 << 8); // Set MUX bits
	GPIOD->PDDR |= (1 << 5); // Setup Pin 5 as output
}

void setup_RedLED()
{
    SIM->SCGC5 |= 1<<13;
    PORTE->PCR[29] &= ~(0x700 << 8);
    PORTE->PCR[29] |= 0x700 & (1 << 8);
    GPIOE->PDDR |= (1 << 29);
}

void blinkGreen()
{

    	if(GPIOD->PDOR & (1 <<5))  //Blinks LEDS Together
    	{	GPIOD->PDOR &= ~(1 << 5);
    		printf("red blink\n");
    	}
    	else
    	{
    		GPIOD->PDOR |= (1 << 5);
    	}

}

void blinkRed()
{

    	if(GPIOE->PDOR & (1 << 29))
    	{
    		    GPIOE->PDOR &= ~(1 << 29);
    			printf("green blink\n");
    	}
    	else
    	{
    			GPIOE->PDOR |= (1 << 29);
    	}

}

void setPortD()
{
	SIM->SCGC5 |= (1<<12);
}

void setPortA()
{
	SIM->SCGC5 |= (1<<12);
}
/**
 * set PD2 as GPIO output
 */
void setPortD_2()
{
	// Setup port for GPIO
	PORTD->PCR[2] &= ~(0x700 << 8);
	PORTD->PCR[2] |= 0x700 & (1 << 8);
	GPIOD->PDDR |= (1 << 2); // Setup Pin 2 as output
}

/**
 * set PA13 as GPIO Input
 */
void setPortA_13()
{
	PORTA->PCR[13] &= ~(0x700 << 8); //clear mux bit
	PORTA->PCR[13] |= 0x700 & (1 << 8); // sit bit as GPIO
	GPIOA->PDDR &= ~(1 << 13); // Setup Pin 13 as input
}

void setTPM_1()
{
    SIM->SCGC6 |= (1 << 25);
    SIM->SOPT2 |= (0x2 << 24);
	PORTA->PCR[12] &= ~(0x700);
	PORTA->PCR[12] |= 0x300;
	TPM1->SC = (1 << 7) | (0x3);
	TPM1->CONTROLS[0].CnSC |= (0x2 << 2) | (0x2 << 4);
	TPM1->CONF |= 0x3 << 6;
	TPM1->MOD = 10000;
	TPM1->SC |= 0x01 << 3;
}

void setPortD2High()
{
		GPIOD->PDOR |= (1 <<2); //set port D high
}

void wait10_us()
{
		for (int i = 0; i < 480; i++)
		{
			__asm volatile ("nop");
		}
}

void setPortD2Low()
{
		GPIOD->PDOR &= ~(1 << 2); //set port d low
}
  void getDistance()
  {
  		unsigned int distance = 0;
  		unsigned int tStart = 0;
  		//unsigned int tStop;
  		unsigned int elapsed = 0;
  		//int divisor = 58;
  		unsigned int overflow = 0;
  		///GPIOD->PDOR &= ~(1 << 2); //set port d low
  		//delay_ms(1000);
  		setPortD2High();
  		printf("getting distance \n");
  		wait10_us();
  		setPortD2Low();


  	/*	while(!(GPIOA->PDIR & (1 << 13))) //wait for echo to go high
  		{
  			printf("wait for high\n");
   		}
   		*/

  		if (GPIOA->PDIR & (1<<13))
  		{
  			printf("high\n");
  			tStart = TPM1->CNT;

  			if (TPM1->SC & (1 << 7)) // if timer overflows
  			{
  				printf("overflowed\n");
  				overflow++; //count how many times it has over flowed
  				printf("Overflow = %u",overflow);
  				TPM1->SC = (1 << 7); //clear timer overflow flag
  			}


  		}
  		else
  		{
  			printf("low\n");
  			elapsed = tStart -TPM1->CNT;
  	 		unsigned int ovfMod = overflow * 10000;
  	  		unsigned int totalTime = elapsed +ovfMod;
  	  		printf("totaltime = %u",totalTime);
  	  		distance = totalTime/58;

  	  		printf("Distance = %u\n",distance);

  		}
  	/*	while(GPIOA->PDIR & ~(1 << 13)) //wait for echo to go high
  		{
  			printf("wait for high\n");
  		}

  			printf("HIGH");
  		tStart = TPM1->CNT; //save tpm1->cnt when echo goes high
  		TPM1->SC = (1 << 7); //clear timer overflow flag

  		while(GPIOA->PDIR & (1 << 13)) //wait for echo to go low
  		{
  			printf("waiting for low\n");
	  			if (TPM1->SC & (1 << 7)) // if timer overflows
	  			{
	  				printf("overflowed\n");
	  				overflow++; //count how many times it has over flowed
	  				printf("Overflow = %u",overflow);
	  				TPM1->SC = (1 << 7); //clear timer overflow flag
	  			}

  		}


  		elapsed = tStart - TPM1->CNT;
  		unsigned int ovfMod = overflow * 10000;
  		unsigned int totalTime = elapsed +ovfMod;
  		printf("totaltime = %u",totalTime);
  		distance = totalTime/58;

  		printf("Distance = %u\n",distance);*/
  		//printf("Distance = %u\n",distance);
  }


int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    PRINTF("Hello World\n");

	//setup_GreenLED();
	//setup_RedLED();
	setPortA();
	setPortD();
	setTPM_1();
	setPortA_13();
	setPortD_2();


    /**
     * Use PWM to test controlling the Servo motor
     * Set up PWM on TPM1 CH0 on PTA12 Pin
     * have different methods to control Servo Positions
     * Pulse width 1ms = 0 degrees
     * Pulse width 1.5ms = 90 degrees
     * Pulse Width 2md = 180 degrees
     */

    /**
     * Implementing Sonar
     * PTD2 GPIO output
     * To measure distance in cm uS/58 = cm
     * send 10us trigger pulse every 50 ms
     * Wait for Echo pulse to go high time how long
     * it stays high in uS and divide by 58 to get distance
     * Set up with TPM1 Prescaler 8, MOD doesnt matter
     * getDistance() function
     * setPTD2 high, wait for 10uS, count nop's, 480 asm volatile nops = 10uS
     * or wait for TPM1->CNT to in increase by 10
     *
     */

    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;



    }
    return 0 ;
}
