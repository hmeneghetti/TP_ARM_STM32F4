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
#include "timer2.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  APPSTATE_IDLE = 0,
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
static uint8_t usbConnected = 0;

/* Variable used by FatFs*/
static FIL FileRead;
static FIL FileWrite;

uint16_t sync_tick = 0;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

//	Estados de la SM para deteccion de pulsos.
#define St_Idle							0
#define St_Pulse_det				1

//	Valor de RMS para deteccion de tono.
#define RMS_VALUE_DETECT		1000

//	Maquina de estados para la deteccion y encendido de leds.	//
void SM_Pulse_Detect(int16_t rms_value){
	
	static UINT sm_status = St_Idle;
	static UINT count = 0;
	
	switch(sm_status){
		
		// Espero el primer pulso.
		case St_Idle:
			if(rms_value > RMS_VALUE_DETECT){
				count++;					
				if(count < 6)
					BSP_LED_On(LED6);
				else{
					BSP_LED_On(LED3);
					count = 0;
					//	Deshabilito interrupcion para atomicidad.
					HAL_NVIC_DisableIRQ(TIM2_IRQn);
					//	Reset del contador de sinc.
					sync_tick = 60;
					PeriodicCaller_Set(999999);
					//	Habilito interrupcion.
					HAL_NVIC_EnableIRQ(TIM2_IRQn);
				}
				sm_status = St_Pulse_det;
			}
			break;
		
		// Apago leds una vez que cesa el tono.
		case St_Pulse_det:
			if(rms_value < RMS_VALUE_DETECT){
				BSP_LED_Off(LED6);
				BSP_LED_Off(LED3);
				
				sm_status = St_Idle;
			}
			break;
	}
	return;
}

int32_t getDataCB(int16_t *pBuff, int32_t length){
	
  UINT bytesread = 0;
	int16_t Audio_RMS_value;	//	Valor RMS de 512 muestras
	static int16_t count=0;
	
  f_read(&FileRead, pBuff, length*sizeof(int16_t), (void *)&bytesread);
  
  audioFilter_filter(pBuff, pBuff, length);
  
	arm_rms_q15_m4(pBuff, length, &Audio_RMS_value);
	
	count++;
	printf("Valores: %d, %d RMS\n", count, Audio_RMS_value);

  SM_Pulse_Detect(Audio_RMS_value);
	
  return bytesread;
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
				appState = APPSTATE_IDLE;
//        Error_Handler();
				
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

