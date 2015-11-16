#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "utils.h"

TIM_HandleTypeDef   TIM_Handle;

arm_status arm_sqrt_q15(q15_t in, q15_t * pOut);

void TickTock_Init(void)
{
  __TIM5_CLK_ENABLE();
  TIM_Handle.Instance = TIM5;
  TIM_Handle.Init.Period = 0xFFFFFFFF;
  TIM_Handle.Init.Prescaler = 83;
  TIM_Handle.Init.ClockDivision = 0;
  TIM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&TIM_Handle);
  HAL_TIM_Base_Start(&TIM_Handle);
}

void TickTock_Start(void)
{
  __HAL_TIM_SetCounter(&TIM_Handle,0);
}

void TickTock_Stop(void){
	
  uint32_t  TockValue;
  TockValue=__HAL_TIM_GetCounter(&TIM_Handle);
  printf("Tiempo transcurrido: %u uS\n",TockValue);
	
}

uint32_t Get_Timer_Value(void){
	uint32_t value;
	
	value = __HAL_TIM_GetCounter(&TIM_Handle);
	
	return value;
}

unsigned int TickVal(void) {
	return __HAL_TIM_GetCounter(&TIM_Handle);
}

///////////////////////////////////-------------------------------///////////////////////////////
//	Funcion RMS	//
/**    
 * @brief  Q15 square root function.    
 * @param[in]   in     input value.  The range of the input value is [0 +1) or 0x0000 to 0x7FFF.    
 * @param[out]  *pOut  square root of input value.    
 * @return The function returns ARM_MATH_SUCCESS if the input value is positive
 * and ARM_MATH_ARGUMENT_ERROR if the input is negative.  For
 * negative inputs, the function returns *pOut = 0.
 */

arm_status arm_sqrt_q15(q15_t in, q15_t * pOut){
	
  q15_t number, temp1, var1, signBits1, half;
  q31_t bits_val1;
  float32_t temp_float1;
	
  union
  {
    q31_t fracval;
    float32_t floatval;
  } tempconv;

  number = in;

  /* If the input is a positive number then compute the signBits. */
  if(number > 0)
  {
    signBits1 = __CLZ(number) - 17;

    /* Shift by the number of signBits1 */
    if((signBits1 % 2) == 0)
    {
      number = number << signBits1;
    }
    else
    {
      number = number << (signBits1 - 1);
    }

    /* Calculate half value of the number */
    half = number >> 1;
    /* Store the number for later use */
    temp1 = number;

    /*Convert to float */
    temp_float1 = number * 3.051757812500000e-005f;
    /*Store as integer */
    tempconv.floatval = temp_float1;
    bits_val1 = tempconv.fracval;
    /* Subtract the shifted value from the magic number to give intial guess */
    bits_val1 = 0x5f3759df - (bits_val1 >> 1);  // gives initial guess  
    /* Store as float */
    tempconv.fracval = bits_val1;
    temp_float1 = tempconv.floatval;
    /* Convert to integer format */
    var1 = (q31_t) (temp_float1 * 16384);

    /* 1st iteration */
    var1 = ((q15_t) ((q31_t) var1 * (0x3000 -
                                     ((q15_t)
                                      ((((q15_t)
                                         (((q31_t) var1 * var1) >> 15)) *
                                        (q31_t) half) >> 15))) >> 15)) << 2;
    /* 2nd iteration */
    var1 = ((q15_t) ((q31_t) var1 * (0x3000 -
                                     ((q15_t)
                                      ((((q15_t)
                                         (((q31_t) var1 * var1) >> 15)) *
                                        (q31_t) half) >> 15))) >> 15)) << 2;
    /* 3rd iteration */
    var1 = ((q15_t) ((q31_t) var1 * (0x3000 -
                                     ((q15_t)
                                      ((((q15_t)
                                         (((q31_t) var1 * var1) >> 15)) *
                                        (q31_t) half) >> 15))) >> 15)) << 2;

    /* Multiply the inverse square root with the original value */
    var1 = ((q15_t) (((q31_t) temp1 * var1) >> 15)) << 1;

    /* Shift the output down accordingly */
    if((signBits1 % 2) == 0)
    {
      var1 = var1 >> (signBits1 / 2);
    }
    else
    {
      var1 = var1 >> ((signBits1 - 1) / 2);
    }
    *pOut = var1;

    return (ARM_MATH_SUCCESS);
  }
  /* If the number is a negative number then store zero as its square root value */
  else
  {
    *pOut = 0;
    return (ARM_MATH_ARGUMENT_ERROR);
  }
}

//	Funcion para el calculo de la raiz cuadrada	//
/**    
 * @brief Root Mean Square of the elements of a Q15 vector.    
 * @param[in]       *pSrc points to the input vector    
 * @param[in]       blockSize length of the input vector    
 * @param[out]      *pResult rms value returned here    
 * @return none.    
 *    
 * @details    
 * <b>Scaling and Overflow Behavior:</b>    
 *    
 * \par    
 * The function is implemented using a 64-bit internal accumulator.    
 * The input is represented in 1.15 format.    
 * Intermediate multiplication yields a 2.30 format, and this    
 * result is added without saturation to a 64-bit accumulator in 34.30 format.    
 * With 33 guard bits in the accumulator, there is no risk of overflow, and the    
 * full precision of the intermediate multiplication is preserved.    
 * Finally, the 34.30 result is truncated to 34.15 format by discarding the lower     
 * 15 bits, and then saturated to yield a result in 1.15 format.    
 *    
 */

void arm_rms_q15_m4(
  q15_t * pSrc,
  uint32_t blockSize,
  q15_t * pResult)
{
  q63_t sum = 0;                                 /* accumulator */

  /* Run the below code for Cortex-M4 and Cortex-M3 */

  q31_t in;                                      /* temporary variable to store the input value */
  q15_t in1;                                     /* temporary variable to store the input value */
  uint32_t blkCnt;                               /* loop counter */

  /* loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = (A[0] * A[0] + A[1] * A[1] + ... + A[blockSize-1] * A[blockSize-1]) */
    /* Compute sum of the squares and then store the results in a temporary variable, sum */
    in = *__SIMD32(pSrc)++;
    sum = __SMLALD(in, in, sum);
    in = *__SIMD32(pSrc)++;
    sum = __SMLALD(in, in, sum);

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.    
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4u;

  while(blkCnt > 0u)
  {
    /* C = (A[0] * A[0] + A[1] * A[1] + ... + A[blockSize-1] * A[blockSize-1]) */
    /* Compute sum of the squares and then store the results in a temporary variable, sum */
    in1 = *pSrc++;
    sum = __SMLALD(in1, in1, sum);

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* Truncating and saturating the accumulator to 1.15 format */
  /* Store the result in the destination */
  arm_sqrt_q15(__SSAT((sum / (q63_t)blockSize) >> 15, 16), pResult);

  /* Truncating and saturating the accumulator to 1.15 format */
  /* Store the result in the destination */
  arm_sqrt_q15(__SSAT((sum / (q63_t)blockSize) >> 15, 16), pResult);

}

/**    
 * @} end of RMS group    
 */
