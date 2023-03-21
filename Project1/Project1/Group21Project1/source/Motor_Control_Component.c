#include "Motor_Control_Component.h"

#define POS_MOTOR_OFFSET (0.06775)

QueueHandle_t motor_queue;
QueueHandle_t angle_queue;

void setupMotorComponent()
{
	setupMotors();

    /*************** Motor Task ***************/
	//Create Motor Queue
	motor_queue = xQueueCreate(1, sizeof(float));
	if (motor_queue == NULL) {
		PRINTF("Motor queue creation failed!.\r\n");
		while(1);
	}
	BaseType_t status;
	//Create Motor Task
	status = xTaskCreate(motorTask, "motorTask", 200, (void *)motor_queue, 3, NULL);
	if (status != pdPASS)
	{
		PRINTF("motorTask creation failed!.\r\n");
		while (1);
	}

    /*************** Position Task ***************/
	//	Create Angle Queue
	angle_queue = xQueueCreate(1, sizeof(float));
	if (angle_queue == NULL) {
		PRINTF("Angle queue creation failed!.\r\n");
		while(1);
	}
	
	//Create Position Task
	status = xTaskCreate(positionTask, "positionTask", 200, (void *)angle_queue, 3, NULL);
	if (status != pdPASS)
	{
		PRINTF("positionTask creation failed!.\r\n");
		while (1);
	}
}

//Initialize PWM for DC motor
void setupMotors() {
	ftm_config_t ftmInfo;
	ftm_chnl_pwm_signal_param_t ftmParam_DC_Motor, ftmParam_Servo_Motor;
	ftm_pwm_level_select_t pwmLevel = kFTM_HighTrue;

	ftmParam_DC_Motor.chnlNumber = FTM_CHANNEL_DC_MOTOR;
	ftmParam_DC_Motor.level = pwmLevel;
	ftmParam_DC_Motor.dutyCyclePercent = 7;
	ftmParam_DC_Motor.firstEdgeDelayPercent = 0U;
	ftmParam_DC_Motor.enableComplementary   = false;
	ftmParam_DC_Motor.enableDeadtime        = false;

	ftmParam_Servo_Motor.chnlNumber = FTM_CHANNEL_SERVO_MOTOR;
	ftmParam_Servo_Motor.level = pwmLevel;
	ftmParam_Servo_Motor.dutyCyclePercent = 7;
	ftmParam_Servo_Motor.firstEdgeDelayPercent = 0U;
	ftmParam_Servo_Motor.enableComplementary   = false;
	ftmParam_Servo_Motor.enableDeadtime        = false;

	FTM_GetDefaultConfig(&ftmInfo);
	ftmInfo.prescale = kFTM_Prescale_Divide_128;
	FTM_Init(FTM_MOTORS, &ftmInfo);
	FTM_SetupPwm(FTM_MOTORS, &ftmParam_DC_Motor, 1U, kFTM_EdgeAlignedPwm, 50U, CLOCK_GetFreq(kCLOCK_BusClk));
	FTM_SetupPwm(FTM_MOTORS, &ftmParam_Servo_Motor, 1U, kFTM_EdgeAlignedPwm, 50U, CLOCK_GetFreq(kCLOCK_BusClk));
	FTM_StartTimer(FTM_MOTORS, kFTM_SystemClock);

	updatePWM_dutyCycle(FTM_CHANNEL_DC_MOTOR, POS_MOTOR_OFFSET);

	FTM_SetSoftwareTrigger(FTM_MOTORS, true);
}

void updatePWM_dutyCycle(ftm_chnl_t channel, float dutyCycle)
{
	uint32_t cnv, cnvFirstEdge = 0, mod;

	/* The CHANNEL_COUNT macro returns -1 if it cannot match the FTM instance */
	assert(-1 != FSL_FEATURE_FTM_CHANNEL_COUNTn(FTM_MOTORS));

	mod = FTM_MOTORS->MOD;
	if(dutyCycle == 0U)
	{
		/* Signal stays low */
		cnv = 0;
	}
	else
	{
		cnv = mod * dutyCycle;
		/* For 100% duty cycle */
		if (cnv >= mod)
		{
			cnv = mod + 1U;
		}
	}

	FTM_MOTORS->CONTROLS[channel].CnV = cnv;
}

void motorTask(void *pvParameters)
{
	while(1) {
		float speed = 0;
		float DCMotorDutyCycle = 0.0;
		QueueHandle_t queue1 = (QueueHandle_t)pvParameters;

		 BaseType_t status = xQueueReceive(queue1, (void *)&speed, portMAX_DELAY);
		 if (status != pdPASS) {
		 	PRINTF("Motor queue receive failed!.\r\n");
		 	while(1);
		 }

		DCMotorDutyCycle = speed * 0.025f/100.0f + POS_MOTOR_OFFSET;

		updatePWM_dutyCycle(FTM_CHANNEL_DC_MOTOR, DCMotorDutyCycle);

		FTM_SetSoftwareTrigger(FTM_MOTORS, true);

//		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

void positionTask(void* pvParameters)
{
    float servoInputStart = -100.0;
    float servoInputEnd = 100.0;
    float servoOutputStart = 0.05;
    float servoOutputEnd = 0.1;

	while(1) {
		float ServoMotorAngle = 0;
		QueueHandle_t queue1 = (QueueHandle_t)pvParameters;
		float ServoMotorDutyCycle = 0.0;

		 BaseType_t status = xQueueReceive(queue1, (void *)&ServoMotorAngle, portMAX_DELAY);
		 if (status != pdPASS) {
			while(1);
		 }

		ServoMotorDutyCycle = servoOutputStart + ((servoOutputEnd - servoOutputStart) / (servoInputEnd - servoInputStart)) * (ServoMotorAngle - servoInputStart);
		updatePWM_dutyCycle(FTM_CHANNEL_SERVO_MOTOR, ServoMotorDutyCycle);
		FTM_SetSoftwareTrigger(FTM_MOTORS, true);

//		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}
