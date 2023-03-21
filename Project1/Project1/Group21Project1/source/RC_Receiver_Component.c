#include "RC_Receiver_Component.h"
#include "fsl_uart.h"

SemaphoreHandle_t rc_hold_semaphore;
TaskHandle_t rc_task_handle;

typedef struct {
	uint16_t header;
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint16_t ch4;
	uint16_t ch5;
	uint16_t ch6;
	uint16_t ch7;
	uint16_t ch8;
} RC_Values;

uart_config_t config;
RC_Values rc_values;
uint8_t* ptr = (uint8_t*) &rc_values;

void setupRCReceiverComponent()
{
	setupRCPins();
	setupUART_RC();

    /*************** RC Task ***************/
	//Create RC Semaphore

	//Create RC Task
	BaseType_t status;
	status = xTaskCreate(rcTask, "rcTask", 200, NULL, 2, NULL);
	if (status != pdPASS)
	{
	    PRINTF("Task creation failed!.\r\n");
	    while (1);
	}
}

void setupRCPins()
{
	//Configure RC pins
	BOARD_InitBootPins();
	UART_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200;
	config.enableTx = false;
	config.enableRx = true;
}

void setupUART_RC()
{
	//setup UART for RC receiver
    UART_Init(UART1, &config, CLOCK_GetFreq(kCLOCK_CoreSysClk));
}


void rcTask(void* pvParameters)
{
	TsMotorData send_to_motor;
	while (1)
		{
            // Check if the task should be paused
			UART_ReadBlocking(UART1, ptr, 1);
			if(*ptr != 0x20)
				continue;
			UART_ReadBlocking(UART1, &ptr[1], sizeof(rc_values) - 1);

			if(rc_values.header == 0x4020)
			{
//				// Angle
//				printf("Channel 1 = %d\t", rc_values.ch1);
//
//				// LED and Motor values
//				printf("Channel 3 = %d\t", rc_values.ch3);
//
//				// Forwards or backwards
//				printf("Channel 6 = %d\t", rc_values.ch6);
//
//				// Forwards or backwards
//				printf("Channel 8 = %d\t\n", rc_values.ch8);
			}

			float speed = 0.0;
			float angle = 0.0;
			speed = (((float) rc_values.ch3) / 10.0) - 100.0;
			angle = (((float) rc_values.ch1) / 5.0) - 300.0;

			 switch (rc_values.ch8) {
			 	case (1500):	// med speed
			 		speed = speed * (2.0/3.0);
			 		break;
			 	case (1000):
			 		speed = speed * (1.0/3.0);
			 		break;
			 	case (2000):
			 		break;
			 	default:
//			 		printf("Unknown speed mode detected!\n");
			 		break;
			 }

			// Check for reverse switch enabled/disabled
			if (rc_values.ch6 > 1500) {
				speed *= -1;
			}
			
			BaseType_t status;

			 status = xQueueSendToBack(angle_queue, (void *)&angle, portMAX_DELAY);
			 if (status != pdPASS) {
				PRINTF("Angle queue Send failed!.\r\n");
				while(1);
			 }

			 status = xQueueSendToBack(motor_queue, (void *)&speed, portMAX_DELAY);
			 if (status != pdPASS) {
			 	PRINTF("Motor queue Send failed!.\r\n");
			 	while(1);
			 }


			// Send LED
            xQueueSendToBack(led_queue, (void *)&rc_values.ch8, portMAX_DELAY);

            // Wait for next data from radio receiver
            vTaskDelay(1 / portTICK_PERIOD_MS); // 10 ms delay
		}



}

