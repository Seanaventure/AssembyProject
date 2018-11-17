#include <MKL46Z4.h>
#include "PinDriver.h"
#include "PitTimer.h"
#include "Exercise11_c.h"
#define  LEDTravelTime 200
#define  NumLED        11
#define  Medium				 300
#define	 Hard					 200
#define	 Easy					 400
#define	 Extreme			 100
int test = 0;
int boolean = 0;
uint8_t time = 0;
void initPins(int leds[], char port){
		for (int i = 0; i < NumLED; i++){
			enablePin(1, port, leds[i]);
	}
}
int main(){
  __ASM ("CPSID I");  /* Mask all KL46 IRQs */
	Init_UART0_IRQ ();
	initPIT(time);
  __ASM ("CPSIE I");  /* Unmask all KL46 IRQs */
	//This array contains all the ports using the LEDs. All are port E
	int ledPorts[] = {31,19,18,17,16,6,3,2,20,21,22};
	initPins(ledPorts, 'E');
	for(;;){
		turnOn('E', ledPorts[test]);
		wait(time, LEDTravelTime);
		turnOff('E', ledPorts[test]);
		if(test == 4){
			boolean = 1;
		}
		if(test == 0){
			boolean = 0;
		}
		if(boolean == 1){
			test = test - 1;
		}
		else{
			test = test + 1;
		}
	}	
}

