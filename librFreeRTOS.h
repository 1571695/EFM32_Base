#define librFreeRTOS.h

#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "croutine.h"

#include "em_chip.h"
#include "bsp.h"
#include "bsp_trace.h"

#include "sleep.h"

#include <stdio.h>
#include <stdbool.h>
#include "em_i2c.h"
#include "em_gpio.h"
#include "em_cmu.h"

void BSP_I2C_Init(uint8_t addr);
bool I2C_WriteRegister(uint8_t reg, uint8_t data);
bool I2C_ReadRegister(uint8_t reg, uint8_t *val);
bool I2C_Test();