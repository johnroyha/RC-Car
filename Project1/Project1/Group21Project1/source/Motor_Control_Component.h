#ifndef MOTOR_CONTROL_COMPONENT_H
#define MOTOR_CONTROL_COMPONENT_H

#include "pin_mux.h"
#include "fsl_port.h"
#include "fsl_ftm.h"
#include "fsl_debug_console.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Terminal_Component.h"

#define FTM_MOTORS				    FTM0
#define FTM_CHANNEL_DC_MOTOR	    kFTM_Chnl_0
#define FTM_CHANNEL_SERVO_MOTOR		kFTM_Chnl_3

typedef enum {
    DATA_SOURCE_RC = 0,
    DATA_SOURCE_TERM,
    DATA_SOURCE_ACCEL 
} TeDataSource;

typedef struct {
    TeDataSource source;
    uint8_t speed;
} TsMotorData;

typedef struct {
    TeDataSource source;
    uint8_t speed;
} TsAngleData;

extern QueueHandle_t motor_queue, angle_queue;

void setupMotorComponent();
void setupMotors();

void updatePWM_dutyCycle(ftm_chnl_t channel, float dutyCycle);

void motorTask();
void positionTask(void* pvParameters);


#endif /* MOTOR_CONTROL_COMPONENT_H */
