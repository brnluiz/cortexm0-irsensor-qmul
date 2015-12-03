/*----------------------------------------------------------------------------
	Code for Lab 6

	In this project
		   - the red light flashes on a periodically 
			 - pressing the button toggles the green light
		There are two tasks
	   t_button: receives an event from the button ISR, and toggles green
	   t_led: switches the red on and off, using a delay

	REQUIRED CHANGES
		Change the program so that 
		   1. The leds flash: red/green/blue with each on for 3sec
		   2. Pressing the button stops the fashing; the next press restarts it
			 The response to the button has no noticable delay
			 The flashing continues with the same colour
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <MKL25Z4.H>
#include "GpioDefs.h"
#include "AdcDefs.h"
#include "Settings.h"
#include "Leds.h"
#include "AdcDefs.h"

OS_TID t_evt_mngr;
OS_TID t_tasks[TOTAL_TASKS]; /*  task ids */

void initOutputTone() {
	// Enable Clock
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	// Make the pin as GPIO
	PORTA->PCR[TONE_POS] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[TONE_POS] |= PORT_PCR_MUX(1);
	PORTA->PCR[IR_TX_POS] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[IR_TX_POS] |= PORT_PCR_MUX(1);
	PORTB->PCR[TONE_TOOGLE_POS] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[TONE_TOOGLE_POS] |= PORT_PCR_MUX(1);
	
	// Set ports to outputs
	PTA->PDDR |= MASK(TONE_POS) ;
	PTA->PDDR |= MASK(IR_TX_POS) ;
	PTB->PDDR |= MASK(TONE_TOOGLE_POS) ;
	
	// Turn off
	PTA->PCOR |= MASK(TONE_POS) ;
	PTA->PCOR |= MASK(IR_TX_POS) ;
	PTB->PCOR |= MASK(TONE_TOOGLE_POS) ;
}

void initInputButton(void) {
	SIM->SCGC5 |=  SIM_SCGC5_PORTD_MASK; /* enable clock for port D */

	/* Select GPIO and enable pull-up resistors and interrupts 
		on falling edges for pins connected to switches */
	PORTD->PCR[BUTTON_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0a);
		
	/* Set port D switch bit to inputs */
	PTD->PDDR &= ~MASK(BUTTON_POS);

	/* Enable Interrupts */
	NVIC_SetPriority(PORTD_IRQn, 128); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(PORTD_IRQn);  // clear any pending interrupts
	NVIC_EnableIRQ(PORTD_IRQn);
}

void PORTD_IRQHandler(void) {  
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	if ((PORTD->ISFR & MASK(BUTTON_POS))) {
		isr_evt_set (EVT_BTN_PRESSED, t_evt_mngr);
	}
	// Clear status flags 
	PORTD->ISFR = 0xffffffff; 
	// Ok to clear all since this handler is for all of Port D
}

RearBoxStates getGearboxState(int buttonState, RearBoxStates gearState) {
	if (buttonState == OS_R_EVT && gearState == REAR_BOX_DISENGAGED) {
		gearState = REAR_BOX_ENGAGED;
	}
	else if (buttonState == OS_R_EVT && gearState == REAR_BOX_ENGAGED) {
		gearState = REAR_BOX_DISENGAGED;
	}
	
	return gearState;
}

int encodeSensorVoltage(float measuredVoltage) {
	int voltageState;
	int commonDivisor = 64;
	
	if ( measuredVoltage < (VREF/commonDivisor) ) {
		voltageState = 30;
	}
	else if ( measuredVoltage < (2*VREF/commonDivisor) ) {
		voltageState = 26;
	}
	else if ( measuredVoltage < (3*VREF/commonDivisor) ) {
		voltageState = 22;
	}
	else if ( measuredVoltage < (4*VREF/commonDivisor) ) {
		voltageState = 18;
	}
	else if ( measuredVoltage < (5*VREF/commonDivisor) ) {
		voltageState = 14;
	}
	else if ( measuredVoltage < (6*VREF/commonDivisor) ) {
		voltageState = 10;
	}
	else if ( measuredVoltage < (7*VREF/commonDivisor) ) {
		voltageState = 6;
	}
	else {
		voltageState = 2;
	}

	return voltageState;
}

// Task to feedback user click with LEDs
__task void ledFeedbackTask(void) {
	int state = COLOR_RED;
	greenLEDOnOff(LED_OFF);
	
	while(1) {
		// Wait for the user click
		os_evt_wait_and (EVT_BTN_PRESSED, 0xFFFF);
		switch (state) {
			case COLOR_RED:
				redLEDOnOff   (LED_ON);
				blueLEDOnOff  (LED_OFF);
				
				state = COLOR_BLUE;
				break;
			case COLOR_BLUE:
				redLEDOnOff  (LED_OFF);
				blueLEDOnOff (LED_ON);
				
				state = COLOR_RED;
				break;
		}
		
		// Discard pending notifications
		os_evt_clr (EVT_BTN_PRESSED, t_tasks[T_LEDS]);
	}
}

// Button Event Manager Task
RearBoxStates gearBoxState;
__task void btnEventManagerTask(void) {
	int i = 0;
	int buttonPressed;

	while(1) {
		// Wait until button is pressed
		buttonPressed = os_evt_wait_and (EVT_BTN_PRESSED, 0xffff);
		
		// Propagate the event through all listeners
		for(i=0; i<TOTAL_TASKS;i++) {
			// TODO: Check if it is 0... If is not, then send the event
			os_evt_set (EVT_BTN_PRESSED, t_tasks[i]);
		}
		
		gearBoxState = getGearboxState(buttonPressed, gearBoxState);
		
		// Wait some time to debounce the buton
		os_dly_wait(DEBOUNCE_TIMEOUT);
		
		// Discard pending notifications
		os_evt_clr (EVT_BTN_PRESSED, t_evt_mngr);
	}
}

// Generate a square wave
__task void toneGeneratorTask(void) {
	
	while(1) {
		os_evt_wait_and (EVT_BTN_PRESSED, 1); 

		switch (gearBoxState) {
			case REAR_BOX_DISENGAGED:
				// Turn the square wave generator off
				PTA->PSOR = MASK(TONE_POS);
				break;
			case REAR_BOX_ENGAGED:
				// Toogle the port to generate the square wave
				PTA->PTOR |= MASK(TONE_POS);
				break;
		}
		
		os_evt_clr (EVT_BTN_PRESSED, t_tasks[T_TONE]);
	}
}

// Toogle on/off the generate square wave
int voltageState;
__task void toogleToneTask(void) {
	int timeout = 1000;
	int state = 0;
	
	while(1) {
		os_evt_wait_and (EVT_BTN_PRESSED, timeout);

		switch (gearBoxState) {
			case REAR_BOX_DISENGAGED:
				// Turn the toogler off
				PTB->PSOR = MASK(TONE_TOOGLE_POS);
				break;
			case REAR_BOX_ENGAGED:
				timeout = 100 * voltageState;
				PTB->PTOR |= MASK(TONE_TOOGLE_POS);
				break;
		}
		
		os_evt_clr (EVT_BTN_PRESSED, t_tasks[T_TOOGLER]);
	}
}

void IrTxOnOff (unsigned short onOff) {
	if (onOff == 1) {
		PTA->PCOR = MASK(IR_TX_POS) ;
	} else {
		PTA->PSOR = MASK(IR_TX_POS) ;
	}
}

__task void parkingSensorAcquireTask(void) {
	
	while(1) {
		float measuredVoltage = 0;  // scaled value
		unsigned measureOn = 0, measureOff = 0, measureTotal = 0;
		int i = 0;
		
		os_evt_wait_and (EVT_BTN_PRESSED, 20);

		switch(gearBoxState) {
			case REAR_BOX_DISENGAGED:
				break;
			case REAR_BOX_ENGAGED:
			
				for (i = 0; i < 5; i++) {
					IrTxOnOff(OFF);
					os_dly_wait(3);
					// measure the voltage
					measureOn = measureVoltage();
					
					IrTxOnOff(ON);
					os_dly_wait(3);
					measureOff = measureVoltage();
					measureTotal = measureTotal + (measureOn - measureOff);
				}

				// Scale to an actual voltage, assuming VREF accurate
				measuredVoltage = (VREF * measureTotal) / (ADCRANGE * 5);

				// Encode the measured voltage to a state
				voltageState = encodeSensorVoltage(measuredVoltage);

				break;
		}
	}
}

__task void boot (void) {
  t_evt_mngr = os_tsk_create (btnEventManagerTask, 0);   // start button task
	
	t_tasks[T_LEDS]    = os_tsk_create (ledFeedbackTask, 0);          // start led task (only user feedback)
	t_tasks[T_TONE]    = os_tsk_create (toneGeneratorTask, 0);        // start tone task (generate the tone)
	t_tasks[T_TOOGLER] = os_tsk_create (toogleToneTask, 0);           // start toogler task (toogle the tone)
	t_tasks[T_SENSOR]  = os_tsk_create (parkingSensorAcquireTask, 0); // start sensor task (data acquisition)
	
	os_tsk_delete_self ();
}

int main (void) {
	// Initialize input and outputs
	initOutputLeds();
	initOutputTone();
	initInputButton();
	initAdc();
	
	// Initialize RTX and start init
	os_sys_init(boot);
}
