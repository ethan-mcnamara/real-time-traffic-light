/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"
#include "../inc/stm32f4xx_rcc.h"


// GPIO Pins used in the program
#define RED_LED GPIO_Pin_0
#define AMBER_LED GPIO_Pin_1
#define GREEN_LED GPIO_Pin_2
#define POT GPIO_Pin_3
#define SHIFT_REGISTER_DATA GPIO_Pin_6
#define SHIFT_REGISTER_CLK GPIO_Pin_7
#define SHIFT_REGISTER_RST GPIO_Pin_8

// Initialize the pin as a GPIO output pin
void initializePin_Out(int GPIO_pin)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// Initialize the pin as a GPIO input pin
void initializePin_In(int GPIO_pin)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void initializeADC1()
{
    ADC_InitTypeDef ADC_InitStruct;
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConv = DISABLE;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_NbrOfConversion = 1;
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADC1, &ADC_InitStruct);
	ADC_Cmd(ADC1, ENABLE);
}

static uint16_t ADC_Start_Conversion()
{
	uint16_t converted_data;
	// Start ADC conversion
	ADC_SoftwareStartConv(ADC1);
	// Wait until conversion is finish
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	// Get the value
	converted_data = ADC_GetConversionValue(ADC1);
	return converted_data;
}


int main(void)
{
    // Enable clock for GPIO port C
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// Enable clock for ADC1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

//	 Initialize GPIOC Pins
//	 initializePin_Out(RED_LED);
//	 initializePin_Out(AMBER_LED);
//	 initializePin_Out(GREEN_LED);
//	 initializePin_Out(SHIFT_REGISTER_DATA);
//   initializePin_Out(SHIFT_REGISTER_CLK);
//   initializePin_Out(SHIFT_REGISTER_RST);
	initializePin_In(POT);
	initializeADC1();

	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_84Cycles);

	uint16_t pot_output = 0;
	while(1)
	{
		pot_output = ADC_Start_Conversion();
		printf("Value is: %d\n", pot_output);
	}

    return 0;

}

