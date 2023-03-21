#ifndef LED_COMPONENT_H
#define LED_COMPONENT_H

#include "pin_mux.h"
#include "fsl_port.h"
#include "fsl_ftm.h"
#include "fsl_debug_console.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Motor_Control_Component.h"
#include "Terminal_Component.h"

#include <stdarg.h>

// Added imports
#include "fsl_common.h"
#include "clock_config.h"
#include "board.h"
#include "timers.h"

// Channels taken from https://github.com/Ashpan/COE-4DS4-Labs/blob/main/Lab0/Lab0_Q5/lab0_q5_hello_world/source/hello_world.c

#define FTM_LED 				FTM3
#define FTM_RED_CHANNEL			kFTM_Chnl_1 //Define red LED channel
#define FTM_GREEN_CHANNEL		kFTM_Chnl_5 //Define green LED channel
#define FTM_BLUE_CHANNEL		kFTM_Chnl_4 //Define blue LED channel

extern QueueHandle_t led_queue;

void setupLEDBootClocks();
void setupLEDComponent();
void setupLEDs();

void ledTask(void* pvParameters);
void ledReceiveTask(void* pvParameters);

#endif /* TERMINAL_COMPONENT_H */
