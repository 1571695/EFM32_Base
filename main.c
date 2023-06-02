/***************************************************************************//**
 * @file
 * @brief FreeRTOS Blink Demo for Energy Micro EFM32GG_STK3700 Starter Kit
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include "librFreeRTOS.h"

#define STACK_SIZE_FOR_TASK    (configMINIMAL_STACK_SIZE + 10)
#define TASK_PRIORITY          (tskIDLE_PRIORITY + 1)


QueueHandle_t RGBab,RGB;

typedef struct Message{
	uint8_t Ra;
	uint8_t Rb;
	uint8_t Ga;
	uint8_t Gb;
	uint8_t Ba;
	uint8_t Bb;
} Message;

typedef struct Color{
	uint16_t R;
	uint16_t G;
	uint16_t B;
} Color;

int _write(int file, const char *ptr, int len) {
    int x;
    for (x = 0; x < len; x++) {
       ITM_SendChar (*ptr++);
    }
    return (len);
}



/* Structure with parameters for LedBlink */
typedef struct {
  /* Delay between blink of led */
  portTickType delay;
  /* Number of led */
  int          ledNo;
} TaskParams_t;

/***************************************************************************//**
 * @brief Simple task which is blinking led
 * @param *pParameters pointer to parameters passed to the function
 ******************************************************************************/
static void LedBlink(void *pParameters)
{
  TaskParams_t     * pData = (TaskParams_t*) pParameters;
  const portTickType delay = pData->delay;

  for (;; ) {
    BSP_LedToggle(pData->ledNo);
    vTaskDelay(delay);
  }
}

static void display(void *pParameters){
	for(;;){

		Color colores;
		if(xQueueReceive(RGB, &colores, 10) == pdPASS){
			printf("RGB %d,%d,%d \n", colores.R, colores.G, colores.B);
		}
	}
}

static void traetement(void *pParameters){
	for(;;){

		Color colores;

		Message recMes;

		xQueueReceive(RGBab, &recMes, 10);

		//Concatenate values to get 16bit color range.

		colores.R = (((uint16_t) recMes.Rb << 8 | (uint16_t) recMes.Ra)*255)/65535;
		colores.G = (((uint16_t) recMes.Gb << 8 | (uint16_t) recMes.Ga)*255)/65535;
		colores.B = (((uint16_t) recMes.Bb << 8 | (uint16_t) recMes.Ba)*255)/65535;

		if(xQueueSendToBack(RGB, &colores, 10) != pdPASS){
			int queueTestOk = 12;
		}
	}
}

static void semaphFunct(void *pParameters){
	for(;;){
		//Using semaphore for controlling the reading of the sensor.

				//Tratamiento de la información recibida por el sensor.
				//Reconocer componente i2c y demás
				uint8_t i2cAddr = 0x88;
				BSP_I2C_Init(i2cAddr);	//Reconocer dirección de módulo i2c --> device_addr
				bool trueFalse = I2C_Test();
				if(trueFalse == true){
					//Set RGB Operating modes --> we've to set mode in B2-B0 to GREEN/RED/BlUE 101
					//Set RGB Data Sensing range to B3 1 to get 10k colors. This implies using the 2 complete registers of each color.
					//Set 16 bit to B4 in ADC Resolution
					uint8_t regAux = 0x01;
					uint8_t dataAux = 0x05;
					I2C_WriteRegister(regAux, dataAux);

					//Read Status Flag register and check if conversion has been done B1

					regAux = 0x08;
					Message mensaje1;


					//Read RGB colors
					regAux = 0x09;
					I2C_ReadRegister(regAux, &mensaje1.Ra);
					regAux = 0x0A;
					I2C_ReadRegister(regAux, &mensaje1.Rb);
					regAux = 0x0B;
					I2C_ReadRegister(regAux, &mensaje1.Ga);
					regAux = 0x0C;
					I2C_ReadRegister(regAux, &mensaje1.Gb);
					regAux = 0x0D;
					I2C_ReadRegister(regAux, &mensaje1.Ba);
					regAux = 0x0E;
					I2C_ReadRegister(regAux, &mensaje1.Bb);

					regAux = 0x0E;

					if(xQueueSendToBack(RGBab, &mensaje1, 10) != pdPASS){
						int queueSendOk = 1;
					}

					vTaskDelay(pdMS_TO_TICKS(500));


				} else {
					printf("Test failed\n");
					vTaskDelay(500);
				}
			}

}

/***************************************************************************//**
 * @brief  Main function
 ******************************************************************************/
int main(void)
{
	/* Chip errata */
  CHIP_Init();

  /* If first word of user data page is non-zero, enable Energy Profiler trace */
  BSP_TraceProfilerSetup();

  /* Initialize LED driver */
  BSP_LedsInit();
  /* Setting state of leds*/
  BSP_LedSet(0);
  BSP_LedSet(1);

  /* Initialize SLEEP driver, no callbacks are used */
  SLEEP_Init(NULL, NULL);
	#if (configSLEEP_MODE < 3)
	  /* do not let to sleep deeper than define */
	  SLEEP_SleepBlockBegin((SLEEP_EnergyMode_t)(configSLEEP_MODE + 1));
	#endif

  /* Parameters value for tasks*/
  static TaskParams_t parametersToTask1 = { pdMS_TO_TICKS(1000), 0 };
  static TaskParams_t parametersToTask2 = { pdMS_TO_TICKS(500), 1 };

  //Creamos las colas
  RGBab = xQueueCreate(1,sizeof(Message));
  RGB = xQueueCreate(1,sizeof(Color));
  int cosa = sizeof(Color);
  cosa = sizeof(Message);


  /*Create two task for blinking leds*/
  xTaskCreate(LedBlink, (const char *) "LedBlink1", STACK_SIZE_FOR_TASK, &parametersToTask1, TASK_PRIORITY, NULL);
  xTaskCreate(LedBlink, (const char *) "LedBlink2", STACK_SIZE_FOR_TASK, &parametersToTask2, TASK_PRIORITY, NULL);
  xTaskCreate(semaphFunct, (const char *) "SemaphFunct", STACK_SIZE_FOR_TASK, NULL, TASK_PRIORITY, NULL);

  xTaskCreate(traetement, (const char *) "traetement", STACK_SIZE_FOR_TASK, NULL, TASK_PRIORITY, NULL);
  xTaskCreate(display, (const char *) "display", STACK_SIZE_FOR_TASK, NULL, TASK_PRIORITY, NULL);

  /*Start FreeRTOS Scheduler*/
  vTaskStartScheduler();



  return 0;
}
