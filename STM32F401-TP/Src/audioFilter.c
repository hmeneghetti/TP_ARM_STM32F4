/**
  ******************************************************************************
  * @file    audioFilter.c 
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

/* Includes ------------------------------------------------------------------*/
#include "audioFilter.h"
#include "arm_math.h"
#include "filters.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  const q15_t *pCoeff;
  int32_t length;
}dataFilter_type;

/* Private define ------------------------------------------------------------*/
#define BLOCK_SIZE            1

/* Private variables ---------------------------------------------------------*/
static q15_t firStateI16[BLOCK_SIZE + FILTER_MAX_LENGHT];
static arm_fir_instance_q15 firInstance;
static const dataFilter_type dataFilters[AUDIO_FILTER_TOTAL_FILTERS] =
{
  {
    lp16000_50_1000,
    LP_FS16000_50_1000_LENGTH,
  },
  {
    hp16000_1000_50,
    HP_FS16000_1000_50_LENGTH,
  },
  {
    bp16000_1000,
    BP_FS16000_1000_LENGTH,
  }
};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
extern void audioFilter_init(void)
{
	audioFilter_filterSel(AUDIO_FILTER_FILTER_SEL_BAND_PASS);
}

extern void audioFilter_filter(q15_t *src, q15_t *dest, uint32_t length)
{
	uint32_t i;
	
	for (i = 0 ; i < length ; i++)
	{
    arm_fir_q15(&firInstance, &src[i], &dest[i], BLOCK_SIZE);
	}
}

extern void audioFilter_filterSel(audioFilter_filterSel_enum sel)
{
	/* inicializa la estructura del filtro. */
  arm_fir_init_q15(&firInstance, 
		dataFilters[sel].length,
		(q15_t*)dataFilters[sel].pCoeff,
		(q15_t*)&firStateI16, 
		BLOCK_SIZE);
}



/* End of file ---------------------------------------------------------------*/
