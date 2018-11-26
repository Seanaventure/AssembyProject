#include "PitTimer.h"

//This is basially the boolean variable. When this is a 0 the wait() function should do an infinite loop until
//its a 1
uint8_t timer1expired = 0;
uint8_t timer2expired = 0;

void initPIT(uint8_t timer){
/*
	First we need to enable disable timer incase its already enabled, then we need to do NVIC
	Specifally, we need to set Priority, clear any current (ICPR), and pending interuppts (ISPR)
*/
		
  SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;			// Enable PIT module clock 
  PIT->CHANNEL[timer].TCTRL &= ~PIT_TCTRL_TEN_MASK;	  // Disable PIT Timer 0 
  NVIC->IP[PIT_IPR_REGISTER] &= NVIC_IPR_PIT_MASK;		  // Set PIT interrupt priority to 0 (highest) 
  NVIC->ICPR[timer] = NVIC_ICPR_PIT_MASK;						  // Clear any pending PIT interrupts 
  NVIC->ISER[timer] = NVIC_ISER_PIT_MASK;							  // Unmask UART0 interrupts 
  PIT->MCR = PIT_MCR_EN_FRZ;											  // Enable PIT timer module and set to stop in debug mode 

}

void wait(uint8_t timer, int ms){
	setTimer(timer, ms);	
	while(timer1expired != 0){
	}
  PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK;
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
	ms = ms/100;
	ms = ms * 2400000;
	ms = ms - 1;
  PIT->CHANNEL[timer].LDVAL = ms;
	PIT->CHANNEL[timer].TCTRL = PIT_TCTRL_CH_IE;

	//PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TEN_MASK;
	timer1expired = 1;

}

void PIT_IRQHandler(){
/*
	This is the ISR. All it should really do is set the timer expired value for the correct timer.
*/
__asm("CPSID   I");  /* mask interrupts */
	timer1expired = 0;
  /* clear PIT timer 0 interrupt flag */
  PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
  __asm("CPSIE   I");  /* unmask interrupts */

	
}

int timerExpired(uint8_t timer){
/*
	This method will return a 1 of the given timer has expired. This is useful mainly for setTimer. I made this a method
	instead of putting timerxexpired so that they are essentialy private variables and this is the getter.
*/
	return (timer = 1) ? timer1expired:timer2expired;
	
}
