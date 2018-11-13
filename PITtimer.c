#include "PitTimer.h"

//This is basially the boolean variable. When this is a 1 the wait() function should do an infinite loop until
//its a 1
uint8_t timer1expired = 0;
uint8_t timer2expired = 0;

void initPIT(uint8_t timer){
/*
	First we need to enable disable timer incase its already enabled, then we need to do NVIC
	Specifally, we need to set Priority, clear any current (ICPR), and pending interuppts (ISPR)
*/
		SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;                 //Enable the clock
		PIT->CHANNEL[timer].TCTRL &= ~PIT_ENABLE_TIMER;
		NVIC->IP[PIT_IPR_REGISTER] &= NVIC_IPR_PIT_MASK;  //Set the priority
		NVIC->ICPR[0] = NVIC_ICPR_PIT_MASK;						    //Clear pending interrupts
		NVIC->ISER[0] = NVIC_ISER_PIT_MASK;						    //Enable PIT interrupt
		PIT->MCR=PIT_MCR_FRZ;
		PIT->CHANNEL[timer].TCTRL = PIT_INTERUPT_ENABLE; //Not going to enable the actual timer at this time. Just the interupt
}

void wait(uint8_t timer, int ms){
 /*
This method is similar to delay() in arduino. It is a blocking call that will stay in an infinite loop until a 
certain amount of time has passed
*/
	
}

void setTimer(uint8_t timer, int ms){
/*
	This will simply set the timer to a certain value. This method is useful for when the user wants to start a timer
	and do other stuff while they wait for it to go off. They can check if it went off by looking at timerExpired.
*/
	
}

void PIT_IRQHandler(){
/*
	This is the ISR. All it should really do is set the timer expired value for the correct timer.
*/
	
}

void timerExpired(uint8_t timer){
/*
	This method will return a 1 of the given timer has expired. This is useful mainly for setTimer. I made this a method
	instead of putting timerxexpired so that they are essentialy private variables and this is the getter.
*/
	
}
