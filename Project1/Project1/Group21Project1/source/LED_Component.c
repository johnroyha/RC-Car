#include "LED_Component.h"

QueueHandle_t led_queue;

// defines the RGB LEDs to use
#define BOARD_RED_LED_GPIO_PIN   (1u)
#define BOARD_BLUE_LED_GPIO_PIN  (8u)
#define BOARD_GREEN_LED_GPIO_PIN (9u)

#define TRUE (1u)
#define FALSE (0u)
#define MODULE_NAME ("LED Component")

// Define the color
#define RED (0xff0000)
#define YELLOW (0xffff00)
#define GREEN (0x00ff00)

void setupLEDComponent()
{
	printf("\n"); // Remove after fixing printing format
	setupLEDs();

    /*************** LED Task ***************/
	// Create LED Queue
	led_queue = xQueueCreate(1, sizeof(uint16_t));

	// Error checking for LED Queue
	if (led_queue == NULL) {
		PRINTF("Queue creation failed!.\r\n");
		while(1);
	}


	//Create LED Task
	BaseType_t status;

	// Create ledTask
	status = xTaskCreate(ledTask, "ledTask", 200, (void *)led_queue, 3, NULL);

	// Error checking for creating LED Task
	if (status != pdPASS) {
		PRINTF("%s: Task creation failed!\r\n", MODULE_NAME);
		while(1);
	}

}

void setupLEDs()
{
	// Initialize PWM for the LEDs
	// Copied from https://github.com/Ashpan/COE-4DS4-Labs/blob/main/Lab0/Lab0_Q5/lab0_q5_hello_world/source/hello_world.c

	ftm_config_t ftmInfo;
	ftm_chnl_pwm_signal_param_t ftmParamRed;
	ftm_chnl_pwm_signal_param_t ftmParamGreen;
	ftm_chnl_pwm_signal_param_t ftmParamBlue;

	// RED LED FTM
	ftmParamRed.chnlNumber = kFTM_Chnl_1;
	ftmParamRed.level = kFTM_HighTrue;
	ftmParamRed.dutyCyclePercent = 0;
	ftmParamRed.firstEdgeDelayPercent = 0U;
	ftmParamRed.enableComplementary = false;
	ftmParamRed.enableDeadtime = false;

	// BLUE LED FTM
	ftmParamGreen.chnlNumber = kFTM_Chnl_5;
	ftmParamGreen.level = kFTM_HighTrue;
	ftmParamGreen.dutyCyclePercent = 0;
	ftmParamGreen.firstEdgeDelayPercent = 0U;
	ftmParamGreen.enableComplementary = false;
	ftmParamGreen.enableDeadtime = false;

	// GREEN LED FTM
	ftmParamBlue.chnlNumber = kFTM_Chnl_4;
	ftmParamBlue.level = kFTM_HighTrue;
	ftmParamBlue.dutyCyclePercent = 0;
	ftmParamBlue.firstEdgeDelayPercent = 0U;
	ftmParamBlue.enableComplementary = false;
	ftmParamBlue.enableDeadtime = false;

	FTM_GetDefaultConfig(&ftmInfo);

	FTM_Init(FTM3, &ftmInfo);
	FTM_SetupPwm(FTM3, &ftmParamRed, 1U, kFTM_EdgeAlignedPwm, 5000U, CLOCK_GetFreq(
	kCLOCK_BusClk));
	FTM_SetupPwm(FTM3, &ftmParamGreen, 1U, kFTM_EdgeAlignedPwm, 5000U, CLOCK_GetFreq(
	kCLOCK_BusClk));
	FTM_SetupPwm(FTM3, &ftmParamBlue, 1U, kFTM_EdgeAlignedPwm, 5000U, CLOCK_GetFreq(
	kCLOCK_BusClk));
	FTM_StartTimer(FTM3, kFTM_SystemClock);

}


void ledTask(void *pvParameters) {
	unsigned long color;
	uint16_t receivedInput;
	QueueHandle_t queue1 = (QueueHandle_t)pvParameters;
	BaseType_t status;

	while (1) {
		status = xQueueReceive(queue1, (void *)&receivedInput, portMAX_DELAY);
//		printf("%s: Received %d from RC Task\n", MODULE_NAME, receivedInput);

		if (status != pdPASS) {
			printf("Queue Receive failed!.\r\n");
		}

		// Range is from 1000 to 2000, so map into thirds for speed.
		if (receivedInput == 1000){ color = GREEN; }
		if (receivedInput == 1500){ color = YELLOW; }
		if (receivedInput == 2000){ color = RED; }

		// set colors of LED
		FTM_UpdatePwmDutycycle(FTM_LED, FTM_RED_CHANNEL, kFTM_EdgeAlignedPwm, (((color >> 16) & (0xFF))/255)*100);
		FTM_UpdatePwmDutycycle(FTM_LED, FTM_GREEN_CHANNEL, kFTM_EdgeAlignedPwm, (((color >> 8) & (0xFF))/255)*100);
		FTM_UpdatePwmDutycycle(FTM_LED, FTM_BLUE_CHANNEL, kFTM_EdgeAlignedPwm, (((color) & (0xFF))/255)*100);
		FTM_SetSoftwareTrigger(FTM_LED, true);

//		printf("%s: Set LED Color = %d\r\n", MODULE_NAME, receivedInput);

		// Add delay so it doesn't update so often
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}
