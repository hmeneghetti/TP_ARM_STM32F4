/**
  ******************************************************************************
  * @file    Audio/Audio_playback_and_record/Inc/waveplayer.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    25-May-2015
  * @brief   Header for waveplayer.c module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */   
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WAVEPLAYER_H
#define __WAVEPLAYER_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "waveFormat.h"

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  call back getData
  * @param  pBuff: Puntero a donde guardar las muestras obtenidas
  * @param  length: Cantidad de muestras que se desean obtener
  * @retval bytes leidos, si hubo error devolver -1
  */
typedef int32_t (*WavePlayer_getDataCB_type)(int16_t *pBuff, int32_t length);

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define CHANNEL_MONO    1
#define CHANNEL_STEREO  2

/* Exported functions ------------------------------------------------------- */
/**
  * @brief  Iniciar reproducción de Audio
  * @param  waveformat: Estructura con información de formato de audio
  * @param  getDataCB: Puntero a funcion para obtener muestras de audio
  * @param  volume: volumen de reproducción (0 a 100)
  * @retval none
  */
void WavePlayerStart(WAVE_FormatTypeDef waveformat, 
  WavePlayer_getDataCB_type getDataCB,
  uint8_t volume);

#endif /* __WAVEPLAYER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
