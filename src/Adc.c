#include <MKL25Z4.H>
#include "AdcDefs.h"


/* ------------------------------------
   Initialise ADC 
	 
	 Note: there is no initialisation of the PCR as
	  the ADC function are the default. 
		The pin is determined by the ADC_CHANNEL macro.
		The clock to the port MUST BE ENABLED: this is not done here.
   ------------------------------------- */
void initAdc(void) {
	
  // Enable clock to ADC
	SIM->SCGC6 |= (1UL << SIM_SCGC6_ADC0_SHIFT) ;

	// Set the ADC0_CFG1 to 0x9C, which is 1001 1100
	//   1 --> low power conversion
	//   00 --> ADIV is 1, no divide
	//   1 --> ADLSMP is long sample time
	//   11 --> MODE is 16 bit conversion
	//   01 --> ADIClK is bus clock / 2
	ADC0->CFG1 = 0x9C ;
	
	// Set the ADC0_SC2 register to 0
	//   0 --> DATRG - s/w trigger
	//   0 --> ACFE - compare disable
	//   0 --> ACFGT - n/a when compare disabled
	//   0 --> ACREN - n/a when compare disabled
	//   0 --> DMAEN - DMA is disabled
	//   00 -> REFSEL - defaults V_REFH and V_REFL selected
	ADC0->SC2 = 0 ;
	
}

/* --------------------------------------------
    Variable to hold result
      Declare volatile to ensure changes seen in debugger
   -------------------------------------------- */
volatile unsigned res ;            // raw value 

/* ---------------------------------------
     Table one measurement of the ADC input
   --------------------------------------- */
unsigned measureVoltage(void) {

   unsigned res = 0 ;
	 
	 // Write to ADC0_SC1A 
	 //   0 --> AIEN Conversion interrupt diabled
	 //   0 --> DIFF single end conversion 
	 //   01000 --> ADCH, selecting AD8 
   ADC0->SC1[0] = ADC_CHANNEL ; // writing to this clears the COCO flag 
	 
	 // test the conversion complete flag, which is 1 when completed
   while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK))
       ; // empty loop
	
   // Read results from ADC0_RA as an unsigned integer	
   res = ADC0->R[0] ; // reading this clears the COCO flag  
	 
   return res ;
}

