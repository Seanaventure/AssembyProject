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
#define  Button				 23
#define  targetLED		 6
int ledIndex = 0;
int reverse = 0;
uint8_t time = 0;
int waittime = 0;
uint8_t lives = 3;
int score = 0;
uint8_t stageActivated = 1;
void initPins(int leds[], char port, int button){
	for (int i = 0; i < NumLED; i++){
			enablePin(1, port, leds[i]);
	}
	enablePin(0, port, button);
}
void GuessInitiated(int ledGuessed){
		if (ledGuessed == targetLED){
				stageActivated = 0;
		}
}
int main(){
  __ASM ("CPSID I");  /* Mask all KL46 IRQs */
	Init_UART0_IRQ ();
	initPIT(time);
  __ASM ("CPSIE I");  /* Unmask all KL46 IRQs */
	//This array contains all the ports using the LEDs. All are port E
	int ledPorts[] = {31,19,18,17,16,6,3,2,20,21,22};
	initPins(ledPorts, 'E', Button);
	turnOn('E', targetLED);
	for (int i = 0; i < 4; i++){
		stageActivated = 1;
		switch(i){
			case 0:
				waittime = Easy;
				break;
			case 1:
				waittime = Medium;
				break;
			case 2:
				waittime = Hard;
			case 3:
				waittime = Extreme;
				break;
		}
		while(stageActivated){
			turnOn('E', ledPorts[ledIndex]);
			setTimer(time, waittime);
			//While waiting for the button input check if the user pressed the button.
			while(!timerExpired(0)){
				if (readPin('E', Button)){
					GuessInitiated(ledPorts[ledIndex]);
				}				
			}
			//If the current LED isnt the target keep it on
			if (!(ledPorts[ledIndex] == targetLED))
				turnOff('E', ledPorts[ledIndex]);
			//TODO: refactor this?
			//I think I did it
			if(ledIndex == NumLED){
				reverse = 1;
			}
			if(ledIndex == 0){
				reverse = 0;
			}
			if(reverse == 1){
				ledIndex -= 1;
			}
			else{
				ledIndex -= 1;
			}
		}	
	}
}

