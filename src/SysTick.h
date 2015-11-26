#ifndef SYSTICK_H
#define SYSTICK_H

#include <MKL25Z4.H>
// Function prototypes for cycle timing using SysTick

void initSysTick(uint32_t ticksPerSec) ;
void waitSysTickCounter(int ticks) ;

#endif