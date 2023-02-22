/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
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

#define RED_LIGHT_DEFAULT_DELAY 4
#define GREEN_LIGHT_DEFAULT_DELAY 8
#define AMBER_LIGHT_DEFAULT_DELAY 2
#define CAR_SPEED_DEFAULT 1

#define POT_MIN_VALUE 25
#define POT_MAX_VALUE 2386
#define POT_VALUE_RANGE 2361

#define mainQUEUE_LENGTH 100

#define green  	0
#define amber  	1
#define red  	2

 #define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

 #define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

xQueueHandle xQueue_handle = 0;
float pot_value = 0;
uint8_t car_array [19] = { 0 };
uint8_t cars_moving = 0;

static void Manager_Task( void *pvParameters );
static void Potentiometer_Task( void * pvParameters );
static void CarLights_Task( void *pvParameters );

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

void writeCarLeds(uint8_t *array)
{
	GPIO_ResetBits(GPIOC, SHIFT_REGISTER_RST);

	for (int i = 18; i >= 0; --i)
	{
		GPIO_SetBits(GPIOC, SHIFT_REGISTER_RST);
		if (array[i])
		{
			GPIO_SetBits(GPIOC, SHIFT_REGISTER_DATA);
		}
		else
		{
			GPIO_ResetBits(GPIOC, SHIFT_REGISTER_DATA);
		}
		GPIO_SetBits(GPIOC, SHIFT_REGISTER_CLK);
		GPIO_ResetBits(GPIOC, SHIFT_REGISTER_CLK);
	}
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

static void prvSetupHardware( void );


int main(void)
{
    // Enable clock for GPIO port C
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// Enable clock for ADC1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

//	 Initialize GPIOC Pins
	 initializePin_Out(RED_LED);
	 initializePin_Out(AMBER_LED);
	 initializePin_Out(GREEN_LED);
	 initializePin_Out(SHIFT_REGISTER_DATA);
	 initializePin_Out(SHIFT_REGISTER_CLK);
	 initializePin_Out(SHIFT_REGISTER_RST);
	 initializePin_In(POT);
	 initializeADC1();

//	 GPIO_ResetBits(GPIOC, SHIFT_REGISTER_RST);

	 prvSetupHardware();

	 // Initialize ADC
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_84Cycles);

	/* Create the queue used by the queue send and queue receive tasks.
	http://www.freertos.org/a00116.html */
	xQueue_handle = xQueueCreate( 	mainQUEUE_LENGTH,		/* The number of items the queue can hold. */
							sizeof( uint16_t ) );	/* The size of each item the queue holds. */

	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( xQueue_handle, "MainQueue" );

	xTaskCreate( Manager_Task, "Manager", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate( Potentiometer_Task, "Potentiometer", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate( CarLights_Task, "CarLights", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

    return 0;

}

static void Manager_Task( void *pvParameters )
{
	uint16_t tx_data = amber;
	TickType_t delay_ticks;
	while(1)
	{
		delay_ticks = 1000 / portTICK_PERIOD_MS;

		GPIO_ResetBits(GPIOC, RED_LED);
		GPIO_ResetBits(GPIOC, AMBER_LED);
		GPIO_ResetBits(GPIOC, GREEN_LED);
		if(tx_data == amber)
		{
			GPIO_SetBits(GPIOC, AMBER_LED);
			delay_ticks *= AMBER_LIGHT_DEFAULT_DELAY;
			cars_moving = 0;
		}
		if(tx_data == green)
		{
			GPIO_SetBits(GPIOC, GREEN_LED);
			delay_ticks *= GREEN_LIGHT_DEFAULT_DELAY * (0.25 + pot_value);
			cars_moving = 1;
		}
		if(tx_data == red)
		{
			GPIO_SetBits(GPIOC, RED_LED);
			delay_ticks *= RED_LIGHT_DEFAULT_DELAY * (1.25 - pot_value);
			cars_moving = 0;
		}
		if( xQueueSend(xQueue_handle,&tx_data,1000))
		{
			if(++tx_data == 3)
				tx_data = 0;
			vTaskDelay(delay_ticks);
		}
		else
		{
			printf("Manager Failed!\n");
		}
	}
}

static void Potentiometer_Task( void *pvParameters )
{
	float unmodified_pot_value;
	while (1)
	{
		unmodified_pot_value = ADC_Start_Conversion();
		unmodified_pot_value -= POT_MIN_VALUE;
		unmodified_pot_value /= POT_VALUE_RANGE;
		unmodified_pot_value = max(unmodified_pot_value, 0);
		unmodified_pot_value = min(unmodified_pot_value, 1);
		pot_value = unmodified_pot_value;
	}
}

static void CarLights_Task( void *pvParameters )
{
	uint8_t copy_car_array [18] = { 0 };
	TickType_t delay_ticks;

	while (1)
	{
		delay_ticks = 1000 / portTICK_PERIOD_MS;

		for (int i = 0; i < 18; ++i)
		{
			copy_car_array[i] = car_array[i];
		}

		for (int i = 17; i >= 0; --i)
		{
			if (i > 7)
			{
				copy_car_array[i + 1] = copy_car_array[i];
				copy_car_array[i] = 0;
			}
			else
			{
				if (cars_moving)
				{
					copy_car_array[i + 1] = copy_car_array[i];
					copy_car_array[i] = 0;
				}
				else
				{
					if (!copy_car_array[i + 1] && i != 7)
					{
						copy_car_array[i + 1] = copy_car_array[i];
						copy_car_array[i] = 0;
					}
				}
			}
		}

		if (rand() % 100 < pot_value * 100)
		{
			copy_car_array[0] = 1;
		}
		else if (!copy_car_array[0])
		{
			copy_car_array[0] = 0;
		}

		writeCarLeds(copy_car_array);

		for (int i = 0; i < 19; ++i)
		{
			car_array[i] = copy_car_array[i];
		}

		vTaskDelay(delay_ticks * pot_value * CAR_SPEED_DEFAULT);
	}

}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}

