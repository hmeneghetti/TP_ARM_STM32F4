/**
  ******************************************************************************
  * @file    waverecord.c 
  * @author  Gustavo Muro
  * @version V0.0.1
  * @date    30/05/2015
  * @brief   Guardado de audio en archivo wav.
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
#include "waverecorder.h"
#include "application.h"
#include "ff.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Variable used by FatFs*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void WaveRecord(FIL *FileWrite, WAVE_FormatTypeDef waveformat, 
  WaveRecord_getDataCB_type getDataCB)
{
  int32_t bytesread = 0;
  uint32_t AudioRemSize = 0;
  UINT byteswritten;
  int16_t sample;
  
  waveformat.ChunkID=__REV(CHUNK_ID);
  waveformat.FileFormat=__REV(FILE_FORMAT);
  waveformat.SubChunk1ID=__REV(FORMAT_ID);
  waveformat.SubChunk1Size=__REV(WAVE_FORMAT_DATA_LENGTH);
  waveformat.AudioFormat = __REV16(WAVE_FORMAT_PCM);
  waveformat.SubChunk2ID=__REV(DATA_ID);
  
  f_write(FileWrite, &waveformat, sizeof(waveformat), &byteswritten);
  
  AudioRemSize = waveformat.FileSize - sizeof(WAVE_FormatTypeDef);
  
  while (AudioRemSize > 0)
  {
    bytesread = getDataCB(&sample, 1);
    
    if (bytesread <= 0)
    {
      AudioRemSize = 0;
    }
    else
    {
      f_write(FileWrite, &sample, sizeof(sample), &byteswritten); 
      
      if (byteswritten != sizeof(sample))
      {
        Error_Handler();
      }
      
      AudioRemSize -= bytesread;
      if (AudioRemSize < sizeof(sample))
      {
        AudioRemSize = 0;
      }
    }
  }
}
/* End of file ---------------------------------------------------------------*/

