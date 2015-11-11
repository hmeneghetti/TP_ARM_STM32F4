/**
  ******************************************************************************
  * @file    waverecorder.c 
  * @author  Gustavo Muro
  * @version V0.0.1
  * @date    30/05/2015
  * @brief   Filtrado de audio.
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WAVERECORDER_H
#define __WAVERECORDER_H

/* Includes ------------------------------------------------------------------*/ 
#include "waveFormat.h"
#include "ff.h"

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  call back getData
  * @param  pBuff: Puntero a donde guardar las muestras obtenidas
  * @param  length: Cantidad de muestras que se desean obtener
  * @retval bytes leidos, si hubo error devolver -1
  */
typedef int32_t (*WaveRecord_getDataCB_type)(int16_t *pBuff, int32_t length);

/* Exported constants --------------------------------------------------------*/                                                                                    
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/**
  * @brief  Iniciar almacenamiento de audio
  * @param  FileWrite: puntero al handler del archivo donde guardar los datos
  * @param  waveformat: Estructura con información de formato de audio
  * @param  getDataCB: Puntero a funcion para obtener muestras de audio
  * @retval none
  */
void WaveRecord(FIL *FileWrite, WAVE_FormatTypeDef waveformat, 
  WaveRecord_getDataCB_type getDataCB);

#endif /* __WAVERECORDER_H */

/* End of file ---------------------------------------------------------------*/
