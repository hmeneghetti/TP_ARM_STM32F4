#ifndef __TIMER2_H
#define __TIMER2_H

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "utils.h"

void PeriodicCaller_Init(void);
void PeriodicCaller_Start(void);
void PeriodicCaller_Reset(void);
void PeriodicCaller_Stop(void);
void PeriodicCaller_Set(uint32_t value);

#endif /* __TIMER2_H */
