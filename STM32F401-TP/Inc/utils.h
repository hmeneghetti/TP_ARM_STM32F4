#ifndef __UTILS_H
#define __UTILS_H

/* Includes ------------------------------------------------------------------*/

#include "arm_math.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */

void TickTock_Init(void);
void TickTock_Start(void);
void TickTock_Stop(void);
unsigned int TickVal(void);
uint32_t Get_Timer_Value(void);

void arm_rms_q15_m4(q15_t * pSrc, uint32_t blockSize, q15_t * pResult);

#endif /* __UTILS_H */
