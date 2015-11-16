#include "timer2.h"
#include "stm32f401_discovery.h"
#include "application.h"

static TIM_HandleTypeDef TIM_Handle5;
static TIM_HandleTypeDef TIM_Handle2;

extern uint16_t sync_tick;

void Sync_Pulse_Int(void);

void PeriodicCaller_Init(void){
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

void PeriodicCaller_Start(void){
	
	HAL_TIM_Base_Start(&TIM_Handle2);
	HAL_TIM_Base_Start_IT(&TIM_Handle2);
	__HAL_TIM_SetCounter(&TIM_Handle2,0);
	
}


void PeriodicCaller_Reset(void){
	
	__HAL_TIM_SetCounter(&TIM_Handle2,0);
	
}

void PeriodicCaller_Stop(void){
	
	HAL_TIM_Base_Stop(&TIM_Handle2);
	
}

void PeriodicCaller_Set(uint32_t value){
  __HAL_TIM_SetCounter(&TIM_Handle2, value);
}


void TIM2_IRQHandler(void){
	
	if(__HAL_TIM_GET_FLAG(&TIM_Handle2, TIM_FLAG_UPDATE) != RESET){ //In case other interrupts are also running
		if(__HAL_TIM_GET_ITSTATUS(&TIM_Handle2, TIM_IT_UPDATE) != RESET){
			__HAL_TIM_CLEAR_FLAG(&TIM_Handle2, TIM_FLAG_UPDATE);
			//	Llamada a la funcion de sincronizacion.
			Sync_Pulse_Int();
		}
	}
	return;
}



