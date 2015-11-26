#ifndef PWM_H
#define PWM_H
#include <MKL25Z4.H>

#define TPM_CHAN (1)
#define PWM_PORT (PORTA)
#define PWM_PIN (4)
#define ALT_TPM (3)
#define PWM_DUTY_MAX (128)

// prototypes
void initTPM0PWM(void) ;
void setPWMDuty(unsigned int duty) ;

#endif
