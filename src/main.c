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
#include "gpio_defs.h"
#include "settings.h"
#include "Pwm.h"

OS_TID t_evt_mngr;
OS_TID t_tasks[TOTAL_TASKS]; /*  task ids */
int oscState = OSC_OFF;      // Oscilloscope State

// Toogle an output for osciloscope measures
void oscToogle(void) {
	if (oscState == OSC_ON)
	{
		PTB->PSOR |= MASK(OSCIL_POS) ;
	}
	else
	{		
		PTB->PCOR |= MASK(OSCIL_POS) ;
	}
	oscState = !oscState;
}

/* ----------------------------------------
	 Configure GPIO output for on-board LEDs 
	   1. Enable clock to GPIO ports
	   2. Enable GPIO ports
	   3. Set GPIO direction to output
	   4. Ensure LEDs are off
 * ---------------------------------------- */
void initOutputLeds() {

	// Enable clock to ports B and D
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;
	
	// Make 3 pins GPIO
	PORTB->PCR[RED_LED_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTB->PCR[RED_LED_POS] |= PORT_PCR_MUX(1);          
	PORTB->PCR[GREEN_LED_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTB->PCR[GREEN_LED_POS] |= PORT_PCR_MUX(1);          
	PORTD->PCR[BLUE_LED_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTD->PCR[BLUE_LED_POS] |= PORT_PCR_MUX(1);    	
	
	// Set ports to outputs
	PTB->PDDR |= MASK(RED_LED_POS) | MASK(GREEN_LED_POS);
	PTD->PDDR |= MASK(BLUE_LED_POS);
	PTB->PDDR |= MASK(OSCIL_POS) ;	

	// Turn off LEDs
	PTB->PSOR = MASK(RED_LED_POS) | MASK(GREEN_LED_POS);
}

void initOutputTone() {
	// Use the Port A2
	
	// Enable Clock
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	
	// Make the pin as GPIO
	PORTA->PCR[TONE_POS] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[TONE_POS] |= PORT_PCR_MUX(1);
	
	// Set ports to outputs
	PTA->PDDR |= MASK(TONE_POS) ;
	
	// Turn off the LED
	PTA->PCOR |= MASK(TONE_POS) ;
}

/*----------------------------------------------------------------------------
  GPIO Input Configuration

  Initialse a Port D pin as an input, with
    - a pullup resistor 
		- an interrupt on the falling edge
  Bit number given by BUTTON_POS - bit 6, corresponding to J2, pin 14
 *----------------------------------------------------------------------------*/
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

/*----------------------------------------------------------------------------
 * Interrupt Handler
 *    - Clear the pending request
 *    - Test the bit for pin 6 to see if it generated the interrupt 
  ---------------------------------------------------------------------------- */
void PORTD_IRQHandler(void) {  
	NVIC_ClearPendingIRQ(PORTD_IRQn);
	if ((PORTD->ISFR & MASK(BUTTON_POS))) {
		isr_evt_set (EVT_BTN_PRESSED, t_evt_mngr);
	}
	// Clear status flags 
	PORTD->ISFR = 0xffffffff; 
	    // Ok to clear all since this handler is for all of Port D
}


/*----------------------------------------------------------------------------
  Function that turns Red LED on or off
 *----------------------------------------------------------------------------*/
void redLEDOnOff (int onOff) {
	if (onOff == LED_ON) {
		PTB->PCOR = MASK(RED_LED_POS) ;
	} else {
		PTB->PSOR = MASK(RED_LED_POS) ;
	}
}

/*----------------------------------------------------------------------------
  Function that turns Green LED on or off
 *----------------------------------------------------------------------------*/
void greenLEDOnOff (int onOff) {
	if (onOff == LED_ON) {
		PTB->PCOR = MASK(GREEN_LED_POS) ;
	} else {
		PTB->PSOR = MASK(GREEN_LED_POS) ;
	}
}

/*----------------------------------------------------------------------------
  Function that turns Blue LED on or off
 *----------------------------------------------------------------------------*/
void blueLEDOnOff (int onOff) {	
	if (onOff == LED_ON) {
		PTD->PCOR = MASK(BLUE_LED_POS) ;
	} else {
		PTD->PSOR = MASK(BLUE_LED_POS) ;
	}
}


/*--------------------------------------------------------------
 *     Task t_led - flash Red LED using a delay
 *--------------------------------------------------------------*/
__task void ledTask (void) {
  while (1) {
    redLEDOnOff(LED_ON);
    os_dly_wait (50);        // delay 50 clock ticks - 0.5sec
    redLEDOnOff(LED_OFF);
    os_dly_wait (50);        // delay 50 clock ticks - 0.5 sec
  }
}


/*------------------------------------------------------------
 *     Task t_button - toggle green LED when ISR sends signal
 *------------------------------------------------------------*/
__task void buttonTask (void) {
	int ledState = LED_OFF ;
  greenLEDOnOff(LED_OFF);
	
	while (1) {
    os_evt_wait_and (EVT_BTN_PRESSED, 0xFFFF);  // wait for an event flag 0x0001
		if (ledState == LED_ON) {
			greenLEDOnOff(LED_OFF);
			ledState = LED_OFF ;
		} else {
			greenLEDOnOff(LED_ON);
			ledState = LED_ON ;
		}
		// guard against button bounces
    os_dly_wait (20);              // delay 200ms
		os_evt_clr (EVT_BTN_PRESSED, t_evt_mngr); // discard pending notifications
  }
}

// Generate the next LED color
int nextLedColor(int ledActualColor) {
	ledActualColor++;
	
	if (ledActualColor > COLOR_BLUE) {
		ledActualColor = 0;
	}
	
	return ledActualColor;
}


// Generate the previous LED color
int previousLedColor(int ledActualColor) {
	ledActualColor--;
	
	if (ledActualColor < 0) {
		ledActualColor = COLOR_BLUE;
	}
	
	return ledActualColor;
}


// Turns off all LEDs
void turnOffAllLeds() {
	redLEDOnOff   (LED_OFF);
	greenLEDOnOff (LED_OFF);
	blueLEDOnOff  (LED_OFF);
}




// Task to cycle the LEDs
__task void ledCycleTask(void) {
	int ledColor = COLOR_RED;
	int buttonPressed = 0;
	int ledCycleActive = 1;
	
	while(1) {
		turnOffAllLeds();
		
		// If the led cyle is meant to be active, then turn the right LEDs
		if (ledCycleActive) {
			switch (ledColor) {
				case COLOR_RED:
					redLEDOnOff   (LED_ON);
					break;
				case COLOR_GREEN:
					greenLEDOnOff (LED_ON);
					break;
				case COLOR_BLUE:
					blueLEDOnOff  (LED_ON);
					break;
			}

			ledColor = nextLedColor(ledColor);
		}
		
		// delay 3sec and wait for the button press
		buttonPressed = os_evt_wait_and (EVT_BTN_PRESSED, LED_TIMEOUT);  // wait for an event flag 0x0001

		// If the button was pressed, then change the active state
		if (buttonPressed == OS_R_EVT) {
			ledCycleActive = !ledCycleActive;
			
			// If the pressed button reactivate the process, get the color previous to the stop click
			if(ledCycleActive) {
				ledColor = previousLedColor(ledColor);
			}
		}
		
		// Discard pending notifications
		os_evt_clr (EVT_BTN_PRESSED, t_tasks[T_LEDS]);
	}
}




// Button Event Manager Task
__task void btnEventManagerTask(void) {
	int i = 0;

	while(1) {
		// Wait until button is pressed
		os_evt_wait_and (EVT_BTN_PRESSED, 0xffff);
		
		// osc toogle
		oscToogle();
		
		// Propagate the event through all listeners
		for(i=0; i<TOTAL_TASKS;i++) {
			os_evt_set (EVT_BTN_PRESSED, t_tasks[i]);
		}
		// Wait some time to debounce the buton
		os_dly_wait(DEBOUNCE_TIMEOUT);
		
		// Discard pending notifications
		os_evt_clr (EVT_BTN_PRESSED, t_evt_mngr);
	}
}

__task void beepTask(void) {
	unsigned short volume = 0;
	
	while (1) {
		os_evt_wait_and (EVT_BTN_PRESSED, 0xFFFF);
		
		if (volume >= 128) {
			volume = 0 ;
		} else {
			volume = volume + 8 ;
		}
		
		setPWMDuty(volume);
		os_evt_clr (EVT_BTN_PRESSED, t_tasks[T_BEEP]);
	}
}

__task void boot (void) {
  t_evt_mngr = os_tsk_create (btnEventManagerTask, 0);  // start button task
	t_tasks[T_BEEP]  = os_tsk_create (beepTask, 0);        // start led task
	t_tasks[T_LEDS]  = os_tsk_create (ledCycleTask, 0);        // start led task
  os_tsk_delete_self ();
}

int main (void) {
	// Initialize input and outputs
  initOutputLeds();
	initOutputTone();
  initInputButton();
	
	// Initialize TMP0 as PWM
	initTPM0PWM();
	
  os_sys_init(boot);          // Initialize RTX and start init
}
