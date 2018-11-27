#include "main.h"
/**
This is the main program for the Game. All the animations and game logic is programmed in here
Authors: Sean Bonaventure, Yoon Kim
*/

uint8_t lives = 3;
uint8_t score = 0;
boolean stageActivated = True;
boolean guessed = False;
int ledPorts[] = {31,19,18,17,16,6,3,2,20,21,22};
boolean buttonInput = True;
int currentStage = 0;

void introFlash(int delay){
	for (int j = 0; j < 3; j++){
		for (int i = 0; i < NumLED; i++){
				turnOn('E', ledPorts[i]);
		}
		wait(0, delay);
		for (int i = 0; i < NumLED; i++){
				turnOff('E', ledPorts[i]);
		}
		wait(0, delay);
	}
}
void cascadeAnim(int delay, boolean left){
		if (left){
			for (int i = 0; i < NumLED; i++){
				turnOn('E', ledPorts[i]);
				wait(0, delay);
				turnOff('E', ledPorts[i]);
			}
		}else{
			for (int i = (NumLED - 1) ; i > 0; i--){
				turnOn('E', ledPorts[i]);
				wait(0, delay);
				turnOff('E', ledPorts[i]);
			}
		}
}
void initPins(int leds[], char port, int button){
	for (int i = 0; i < NumLED; i++){
			enablePin(1, port, leds[i]);
	}
	enablePin(0, port, button);
}
void GuessInitiated(int ledGuessed){
	  guessed = True;
	  PutStringSB("You guessed LED number: ", MaxSize);
		PutNumUB(ledGuessed);
		PutStringSB("\r\n", MaxSize);
		if (ledPorts[ledGuessed] == targetLED){
				PutStringSB("That is correct! Advancing to next stage\r\n", MaxSize);
				introFlash(introFlashLen);
				stageActivated = False;
				currentStage++;
				score++;
		}else{
				PutStringSB("Incorrect :(, you have lost a life\r\n", MaxSize);
				PutStringSB("Current Lives: ", MaxSize);
				lives--;
				PutNumUB(lives);
				PutStringSB("\r\n", MaxSize);
		}
}
void initGame(){
	__ASM ("CPSID I");  /* Mask all KL46 IRQs */
	Init_UART0_IRQ ();
	//Initialize timer 0
	initPIT(0);
  __ASM ("CPSIE I");  /* Unmask all KL46 IRQs */
	//This array contains all the ports using the LEDs. All are port E
	PutStringSB("Welcome to the LED game! \r\n", MaxSize);
	PutStringSB("Would you like to use the button or terminal as your input? Type b/t ", MaxSize);
	char inputType;
	while((inputType = GetChar()) != 'b' && inputType != 't');
	PutChar(inputType);
	PutStringSB("\r\n", MaxSize);
	if (inputType == 't'){
		buttonInput = False;
		PutStringSB("Press the 's' key once the LED's match \r\n", MaxSize);
	}else{
		PutStringSB("Press the button once the LED's match \r\n", MaxSize);
	}
	initPins(ledPorts, 'E', Button);
	cascadeAnim(50, True);
	cascadeAnim(50, False);
	wait(0, 200);
	introFlash(introFlashLen);
	turnOn('E', targetLED);

}

int main(){
  initGame();
	int ledIndex = 0;
	boolean reverse = False;
	int waittime = 0;
	while (currentStage <= 4 && lives != 0){
		turnOn('E', targetLED);
		PutStringSB("Current stage: ", MaxSize);
		stageActivated = True;
		switch(currentStage){
			case 0:
				waittime = Easy;
				PutStringSB("Easy\r\n", MaxSize);
				break;
			case 1:
				waittime = Medium;
				PutStringSB("Medium\r\n", MaxSize);
				break;
			case 2:
				waittime = Hard;
				PutStringSB("Hard\r\n", MaxSize);
				break;
			case 3:
				waittime = Extreme;
				PutStringSB("EXTREME\r\n", MaxSize);
				break;
			case 4:
				waittime = ULTRA_EXTREME;
				PutStringSB("YOU HAVE ENTERED ULTRA EXTREME MODE\r\n", MaxSize);
		}
		PutStringSB("Your current score is: ", MaxSize);
		PutNumUB(score);
		PutStringSB("\r\n", MaxSize);
		PutStringSB("Current Lives: ", MaxSize);
		PutNumUB(lives);
		PutStringSB("\r\n", MaxSize);
		while(stageActivated && lives != 0){
			turnOn('E', ledPorts[ledIndex]);
			setTimer(0, waittime);
			//While waiting for the button input check if the user pressed the button.
			while(!timerExpired(0)){
				//Only read button if its the proper input type
				if (!guessed && readPin('E', Button) && buttonInput)
					GuessInitiated(ledIndex);		
			}
			char guessInput;
			if (!buttonInput){
				if ((guessInput = GetCharNoBlock()) == 's'){
					PutChar(guessInput);
					PutStringSB("\r\n", MaxSize);
					GuessInitiated(ledIndex);		
				}
			}else{
					while(readPin('E', Button) != 0);
			}
			guessed = 0;
			//If the current LED isnt the target keep it on
			if (!(ledPorts[ledIndex] == targetLED))
				turnOff('E', ledPorts[ledIndex]);
			//TODO: refactor this?
			//I think I did it
			if(ledIndex == NumLED){
				reverse = True;
			}
			if(ledIndex == 0){
				reverse = 0;
			}
			if(reverse){
				ledIndex -= 1;
			}
			else{
				ledIndex += 1;
			}
		}	
	}
	PutStringSB("DDu Have Finished!\r\n", MaxSize);
	wait(0, 100);
	return 1;
}

