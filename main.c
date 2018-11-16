#include "PinDriver.h"
#include "PitTimer.h"
#define  LEDTravelTime 500
#define  NumLED        11
void initPins(int leds[], char port){
		for (int i = 0; i < NumLED; i++){
			enablePin(1, port, leds[i]);
	}
}
int main(){
	//This array contains all the ports using the LEDs. All are port E
	int ledPorts[] = {31,19,18,17,16,6,3,2,20,21,22};
	initPins(ledPorts, 'E');
	turnOn('E', ledPorts[0]);
	//wait(LEDTravelTime);
	turnOn('E', ledPorts[3]);
}

