#include "FreeRTOS.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_uart.h"

#include "Motor_Control_Component.h"
#include "RC_Receiver_Component.h"
#include "Terminal_Component.h"
#include "LED_Component.h"
#include "Accelerometer_Component.h"

int main(void)
{
    /* Init board hardware. */
    BOARD_InitBootClocks();
    BOARD_InitBootPins();
    setupMotorComponent();
    setupLEDComponent();
    setupRCReceiverComponent();

    vTaskStartScheduler();

    while(1)
    {}
}
