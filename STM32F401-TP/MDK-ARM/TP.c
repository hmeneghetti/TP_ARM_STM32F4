#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "utils.h"
/*
static TIM_HandleTypeDef               TIM_Handle5;
static TIM_HandleTypeDef               TIM_Handle2;
static PeriodicCalledFunction_type     PeriodicCalledFcn;
static uint8_t                         started=0;

void TickTock_Init(void)
{
        __TIM5_CLK_ENABLE();
        TIM_Handle5.Instance = TIM5;
        TIM_Handle5.Init.Period = 0xFFFFFFFF;
        TIM_Handle5.Init.Prescaler = 83;
        TIM_Handle5.Init.ClockDivision = 0;
        TIM_Handle5.Init.CounterMode = TIM_COUNTERMODE_UP;
        HAL_TIM_Base_Init(&TIM_Handle5);
        HAL_TIM_Base_Start(&TIM_Handle5);
}

void TickTock_Start(void)
{
        __HAL_TIM_SetCounter(&TIM_Handle5,0);
}

void TickTock_Start_OneShot(void)
{
        if (started==0)
        {
                __HAL_TIM_SetCounter(&TIM_Handle5,0);
                started=1;
        }
}

void TickTock_Stop(void)
{
        uint32_t        TockValue;
        TockValue=__HAL_TIM_GetCounter(&TIM_Handle5);
        started=0;
        printf("Tiempo transcurrido: %u uS\n",TockValue);
}

void printfPCM(int16_t *pBuff, int32_t length)
{
        for ( int32_t x = 0; x < length; x++ ) {
                printf("PCM: %d \n", pBuff[x]);
        }
}

void PeriodicCaller_Init(void)
{
        __TIM2_CLK_ENABLE();

        TIM_Handle2.Instance = TIM2;
        TIM_Handle2.Init.Period = 999999;
        TIM_Handle2.Init.Prescaler = 83;
        TIM_Handle2.Init.ClockDivision = 0;
        TIM_Handle2.Init.CounterMode = TIM_COUNTERMODE_UP;
        HAL_TIM_Base_Init(&TIM_Handle2);

        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void PeriodicCaller_Start(PeriodicCalledFunction_type PeriodicCalledFunction)
{
        TickTock_Init();
        PeriodicCalledFcn=PeriodicCalledFunction;
        HAL_TIM_Base_Start(&TIM_Handle2);
        HAL_TIM_Base_Start_IT(&TIM_Handle2);
        __HAL_TIM_SetCounter(&TIM_Handle2,0);
}

void PeriodicCaller_Reset(void)
{
        __HAL_TIM_SetCounter(&TIM_Handle2,0);
}

void PeriodicCaller_Stop(void)
{
        HAL_TIM_Base_Stop(&TIM_Handle2);
}

void TIM2_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&TIM_Handle2, TIM_FLAG_UPDATE) != RESET)      //Incase other interrupts are also running
    {
        if (__HAL_TIM_GET_ITSTATUS(&TIM_Handle2, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_FLAG(&TIM_Handle2, TIM_FLAG_UPDATE);
                                                PeriodicCalledFcn();
        }
    }

}

uint8_t RisingEdgeDetector(uint8_t signal)
{
        static uint8_t lastSignal=0;

        if (signal!=lastSignal && signal==1){
                lastSignal=signal;
                return 1;
        }else{
                lastSignal=signal;
                return 0;
        }
}
*/
