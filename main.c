#include "PinDriver.h"


/**
Initializes a GPIO pin in port D
*/
void initPort(int ledNum, int inout){
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	PORTD->PCR[ledNum] = PORT_PCR_MUX(1u);
	PTD->PDDR |= (inout << ledNum);
}

/**
Will toggle a pin on portd
*/
void togglePin(int pinNum){
		PTD->PTOR |= (1u << pinNum);
}

/*
Will read the digital value from a port on portD
Return:
Returns 1 if the pin is high, 0 if pin is low
*/
int main(){
	enablePin(1, 'D', 7);
	initPort(6, 0);
	PTD->PSOR |= (1<<7);
}

