#ifndef ADC_DEFS_H
#define ADC_DEFS_H


// Freedom KL25Z ADC Channel
#define ADC_CHANNEL (8)	// on port B
#define VREF (3.3) // reference voltage
#define ADCRANGE (0xffff) // maximum for a 16 bit conversion

// External variables
extern volatile unsigned res ;            // raw value 
 
// Function prototypes 
unsigned measureVoltage(void) ;
void initAdc(void) ;

#endif
