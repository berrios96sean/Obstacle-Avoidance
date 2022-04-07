
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
	  volatile unsigned int distance;

	  volatile unsigned int elapsed;

	  volatile unsigned int overflow;
	  volatile unsigned int ovfMod;
	  volatile unsigned int totalTime;

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
	TPM1->SC = (1 << 6) | (0x3);
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

  unsigned int getDistance()
  {


		GPIOD->PDOR |= (1 <<2); //set port D high
  		//printf("getting distance \n");
		for (int i = 0; i < 480; i++)
		{
			__asm volatile ("nop");
		}
		GPIOD->PDOR &= ~(1 << 2); //set port d low




  	/*	if (GPIOA->PDIR & (1<<13))
  		{
  			printf("high\n");
  		}
  		else
  		{
  			printf("low\n");


  		}*/

  		unsigned int tStart = 0;


  			//printf("Low\n");
  		while(!(GPIOA->PDIR & (1 << 13))) //wait for echo to go high
  		{
			__asm volatile ("nop");
  		}
  		//printf("HIGH\n");


  		//printf("tStart %u = ",tStart);
  		tStart = TPM1->CNT; //save tpm1->cnt when echo goes high
  		//printf("tStart %u = ",tStart);
  		TPM1->SC = (1 << 7); //clear timer overflow flag
  		overflow = 0;

  		while(GPIOA->PDIR & (1 << 13)) //wait for echo to go low
  		{

  			__asm volatile ("nop");
  		}

		//printf("overflow = %u",overflow);
  		elapsed = tStart - TPM1->CNT;
  		ovfMod = overflow * 10000;
  		totalTime = elapsed +ovfMod;
  		printf("overflow = %u",overflow);
  		printf("totaltime = %u\n",totalTime);
  		unsigned int div = 58;
  		distance = totalTime/div;
  		//printf("Distance = %d\n",distance);
  		return distance;


  }

  void TPM1_IRQHandler(void)
  {

	  if (TPM1->SC & (1 << 7))
	  {
		  overflow++;
		  TPM1->SC |= (1 << 7);
	  }
	  TPM1->SC |= (1 << 7);
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


    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;

        distance = getDistance();

        printf("Distance = %u",distance);

    }
    return 0 ;
}
