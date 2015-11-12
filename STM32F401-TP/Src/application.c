/**
  ******************************************************************************
  * @file    application.c 
  * @author  Gustavo Muro
  * @version V0.0.1
  * @date    30/05/2015
  * @brief   Archivo de aplicaci�n.
  ******************************************************************************
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  *
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  *
  * 3. Neither the name of the copyright holder nor the names of its
  *    contributors may be used to endorse or promote products derived from this
  *    software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  * POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "application.h"
#include "ff.h"
#include "waveplayer.h"
#include "waverecorder.h"
#include "ff.h"    
#include "ff_gen_drv.h"
#include "usbh_diskio.h"
#include "main.h"
#include "utils.h"
#include "audioFilter.h"
#include "arm_math.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  APPSTATE_IDLE = 0,
  APPSTATE_GEN_SINE,
  APPSTATE_MOUNT_FS,
  APPSTATE_UMOUNT_FS,
  APPSTATE_WRITE,
  APPSTATE_PLAY,
}appState_enum;

/* Private define ------------------------------------------------------------*/

#define SINE_GEN_AUDIO_SAMPLE_RATE    8000

#define SINE_GEN_DURATION             10

#define SINE_GEN_1KHZ_LENGTH          (SINE_GEN_AUDIO_SAMPLE_RATE/1000)

#define SINE_GEN_500HZ_LENGTH         (SINE_GEN_AUDIO_SAMPLE_RATE/500)

/* Private variables ---------------------------------------------------------*/
static FATFS USBDISKFatFs;           /* File system object for USB disk logical drive */
static char USBDISKPath[4];          /* USB Host logical drive path */
static appState_enum appState = APPSTATE_IDLE;
//static audioFilter_filterSel_enum filterSel = AUDIO_FILTER_FILTER_SEL_LOW_PASS;
static uint8_t usbConnected = 0;

/* Variable used by FatFs*/
static FIL FileRead;
static FIL FileWrite;
/*
static const int16_t sine_1khz_FS8khz[SINE_GEN_1KHZ_LENGTH] =
{
  0, 23169, 32767, 23169, 0, -23169, 32767, -23169
};
*/
static const int16_t sine_500hz_FS8khz[SINE_GEN_500HZ_LENGTH] =
{
  0,12539,23169,30272,32767,30272,23169,12539,0,-12539,-23169,-30272,-32767,-30272,-23169,-12539
};



/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

// Estados posibles de la Maquina de estado que detecta pulsos
typedef enum {
	ST_IDLE = 0,
	ST_PULSE,
} StPulse;

#define TRUE		1
#define FALSE		0

#define WAIT_MAX	1500	// Tiempo a esperar por un maximo [us]
#define UMBRAL		2000	// Valor a partir del cual se considera presencia de un pulso

StPulse PState	= ST_IDLE;	// Estado SM pulsos
int16_t max		= 0;		// Maximo valor dentro de una muestra
UINT	cont	= 0;		// Contador de tonos

/*
	Consulta si hay algun valor mayor al umbral en el buffer
*/
int16_t MayorUmbral(int16_t *pBuff, int32_t length) {
	UINT i;
	
	for(i = 0; i < length; i++) {
		if(pBuff[i] >= UMBRAL)
			return TRUE;
	}
	
	return FALSE;
}

/*
	Maquina de estados detectora de pulsos
*/
void PulseSM(int16_t *pBuff, int32_t length) {
	switch(PState) {
		// Esperando Pulso
		case ST_IDLE:
			if(MayorUmbral(pBuff, length) == TRUE) {
				if(++cont < 6)
					BSP_LED_On(LED6);
				else
					BSP_LED_On(LED3);
				
				if(cont == 6)
					cont = 0;
				
				TickTock_Start();
				PState = ST_PULSE;
			}
			break;
		
		// Procesando Pulso
		case ST_PULSE:
			if(MayorUmbral(pBuff, length) == TRUE) {
				TickTock_Stop();
				TickTock_Start();
			}

			if(TickVal() > WAIT_MAX) {
				BSP_LED_Off(LED6);
				BSP_LED_Off(LED3);
				
				PState = ST_IDLE;
			}
			break;
	}
}

//void SM_Pulse_Detect(int16_t *pBuff)

  /**    
   * @brief  Q15 square root function.    
   * @param[in]   in     input value.  The range of the input value is [0 +1) or 0x0000 to 0x7FFF.    
   * @param[out]  *pOut  square root of input value.    
   * @return The function returns ARM_MATH_SUCCESS if the input value is positive
   * and ARM_MATH_ARGUMENT_ERROR if the input is negative.  For
   * negative inputs, the function returns *pOut = 0.
   */

arm_status arm_sqrt_q15(
  q15_t in,
  q15_t * pOut)
{
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

int32_t getDataCB(int16_t *pBuff, int32_t length)
{
  UINT bytesread = 0;
	int16_t *pBuff_RMS;
	int16_t Audio_BufferRMS[length];	//	Buffer para almacenar las muestras filtradas
	
	pBuff_RMS = &Audio_BufferRMS[0];
	
  f_read(&FileRead, pBuff, length*sizeof(int16_t), (void *)&bytesread); 
  
  audioFilter_filter(pBuff, pBuff, length);
  
	arm_rms_q15_m4(pBuff, length, pBuff_RMS);
	
	printf("Valores:  %u RMS\n", *pBuff_RMS);
	
  PulseSM(pBuff_RMS, length);
  
  return bytesread;
}

int32_t getDataSineCB(int16_t *pBuff, int32_t length)
{
  static int8_t count = 0;
  int32_t ret = length * 2;
  
  TickTock_Start();
  
  while (length)
  {
    *pBuff = sine_500hz_FS8khz[count];
    count++;
    if (SINE_GEN_500HZ_LENGTH <= count)
    {
      count = 0;
    }
    pBuff++;
    length--;
  }
  
  TickTock_Stop();
  
  return ret;
}


/* Exported functions ------------------------------------------------------- */

extern void application_init(void)
{
  /*##-1- Link the USB Host disk I/O driver ##################################*/
  if(FATFS_LinkDriver(&USBH_Driver, USBDISKPath) != 0)
  {
    Error_Handler();
  }
  
  TickTock_Init();
  
  audioFilter_init();
}

extern void application_task(void)
{
  UINT bytesread = 0;
  WAVE_FormatTypeDef waveformat;
  
  switch (appState)
  {
    case APPSTATE_IDLE:
      if (usbConnected)
      {
        appState = APPSTATE_MOUNT_FS;
      }
      break;
    
    case APPSTATE_GEN_SINE:
      waveformat.SampleRate = SINE_GEN_AUDIO_SAMPLE_RATE;
      waveformat.FileSize = SINE_GEN_AUDIO_SAMPLE_RATE * SINE_GEN_DURATION * \
                            sizeof(int16_t) + sizeof(WAVE_FormatTypeDef);
      waveformat.NbrChannels = CHANNEL_MONO;
      WavePlayerStart(waveformat, getDataSineCB, 70);
      break;
    
    case APPSTATE_MOUNT_FS:
      if (f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0 ) != FR_OK ) 
      {
        /* FatFs initialization fails */
        Error_Handler();
      }
      else
      {
        appState = APPSTATE_PLAY;
      }
      break;
    
    case APPSTATE_UMOUNT_FS:
      f_mount(NULL, (TCHAR const*)"", 1);
      appState = APPSTATE_IDLE;
      break;
    
    case APPSTATE_WRITE:
		
		//pasa alto
			audioFilter_filterSel(AUDIO_FILTER_FILTER_SEL_HIGH_PASS);
		
      if (f_open(&FileWrite, WAVE_REC_1000_NAME_COMPLETO, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
      {
        Error_Handler();
      }
      else
      {
				if (f_open(&FileRead, WAVE_NAME_COMPLETO, FA_READ) != FR_OK)
				{
					Error_Handler();
				}
				else
				{
					f_read (&FileRead, &waveformat, sizeof(waveformat), &bytesread);
					
					WaveRecord(&FileWrite, waveformat, getDataCB);
					f_close(&FileWrite);
					appState = APPSTATE_PLAY;
				}
				f_close(&FileRead);
			}
	
			//pasa bajo
			
			audioFilter_filterSel(AUDIO_FILTER_FILTER_SEL_LOW_PASS);
		
      if (f_open(&FileWrite, WAVE_REC_50_NAME_COMPLETO, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
      {
        Error_Handler();
      }
      else
      {
				if (f_open(&FileRead, WAVE_NAME_COMPLETO, FA_READ) != FR_OK)
				{
					Error_Handler();
				}
				else
				{
					f_read (&FileRead, &waveformat, sizeof(waveformat), &bytesread);
					WaveRecord(&FileWrite, waveformat, getDataCB);
					f_close(&FileWrite);
					appState = APPSTATE_PLAY;
				}
				f_close(&FileRead);
			}
			
			appState = APPSTATE_PLAY;
      break;

			
    case APPSTATE_PLAY:
			if (f_open(&FileRead, WAVE_NAME_COMPLETO, FA_READ) != FR_OK){
				
        Error_Handler();
				
      }else{
      
				/* Read sizeof(WaveFormat) from the selected file */
        f_read (&FileRead, &waveformat, sizeof(waveformat), &bytesread);
        WavePlayerStart(waveformat, getDataCB, 80);
        f_close(&FileRead);
      }
      break;
			
			
    default:
      appState = APPSTATE_IDLE;
      break;
  }
}

extern void application_conect(void)
{
  usbConnected = 1;
}
extern void application_disconect(void)
{
  usbConnected = 0;
}
/* End of file ---------------------------------------------------------------*/

