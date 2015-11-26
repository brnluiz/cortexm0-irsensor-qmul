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
#include "Settings.h"
#include "Pwm.h"
#include "Leds.h"

OS_TID t_evt_mngr;
OS_TID t_tasks[TOTAL_TASKS]; /*  task ids */

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


// Task to feedback user click with LEDs
__task void ledFeedbackTask(void) {
	int ledColor = COLOR_RED;
	greenLEDOnOff(LED_OFF);
	
	while(1) {
		// Wait for the user click
		os_evt_wait_and (EVT_BTN_PRESSED, 0xFFFF);
		switch (ledColor) {
			case COLOR_RED:
				redLEDOnOff   (LED_ON);
				blueLEDOnOff  (LED_OFF);
				
				ledColor = COLOR_BLUE;
				break;
			case COLOR_BLUE:
				redLEDOnOff  (LED_OFF);
				blueLEDOnOff (LED_ON);
				
				ledColor = COLOR_RED;
				break;
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

__task void pwmTask(void) {
	unsigned short volume = 0;
	
	while (1) {
		os_evt_wait_and (EVT_BTN_PRESSED, 0xFFFF);
		
		if (volume >= 128) {
			volume = 0 ;
		} else {
			volume = volume + 8 ;
		}
		
		setPWMDuty(volume);
		os_evt_clr (EVT_BTN_PRESSED, t_tasks[T_PWM]);
	}
}

__task void timerTask(void) {
	while(1) {
		PTA->PTOR |= MASK(TONE_POS);
		os_dly_wait(TIMER_TIMEOUT);
	}
}

__task void boot (void) {
  t_evt_mngr = os_tsk_create (btnEventManagerTask, 0);   // start button task
	
	t_tasks[T_LEDS]  = os_tsk_create (ledFeedbackTask, 0); // start led task (only user feedback)
  t_tasks[T_PWM]   = os_tsk_create (pwmTask, 0);         // start pwm task (control the volume)
	t_tasks[T_TIMER] = os_tsk_create (timerTask, 0);       // start timer task (generate the tone)
	
	os_tsk_delete_self ();
}

int main (void) {
	// Initialize input and outputs
  initOutputLeds();
	initOutputTone();
  initInputButton();
	
	// Initialize TMP0 as PWM
	initTPM0PWM();
	
	// Initialize RTX and start init
  os_sys_init(boot);
}
