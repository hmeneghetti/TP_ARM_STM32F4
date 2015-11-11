/**
  ******************************************************************************
  * @file    audioFilter.h 
  * @author  Gustavo Muro
  * @version V0.0.1
  * @date    30/05/2015
  * @brief   header Filtrado de audio.
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
#ifndef AUDIO_FILTER_H
#define AUDIO_FILTER_H

/* Includes ------------------------------------------------------------------*/ 
#include "arm_math.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  AUDIO_FILTER_FILTER_SEL_LOW_PASS = 0,
  AUDIO_FILTER_FILTER_SEL_HIGH_PASS,
	AUDIO_FILTER_FILTER_SEL_BAND_PASS,
  AUDIO_FILTER_TOTAL_FILTERS,
}audioFilter_filterSel_enum;

/* Exported constants --------------------------------------------------------*/                                                                                    
/* Exported macro ------------------------------------------------------------*/

#define AUDIO_FILTER_LP_44100_9600_12000_16_STEREO

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  inicializa filtrado de audio con el filtro seleccionado y el formato
						de audio indicado.
  * @retval None
  */
extern void audioFilter_init(void);

/**
  * @brief  Filtra muestras de audio.
  * @param  src: puntero al las muestas de audio a filtrar
  * @param  dest: puntero a donde dejar las muestras filtradas
  * @param  length: cantidad de muestras
  * @retval None
  */
extern void audioFilter_filter(q15_t *src, q15_t *dest, uint32_t length);

extern void audioFilter_filterSel(audioFilter_filterSel_enum sel);

#endif /* AUDIO_FILTER_H */

/* End of file ---------------------------------------------------------------*/
