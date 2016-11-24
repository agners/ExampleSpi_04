/*
 ============================================================================
 Name        : Exercicio_05_M7.c
 Author      : Raul Munoz
 Version     :
 Copyright   : Toradex
 Description : Read BTN3 and send status through RPMsg to A7.
 	 	 	   Receive BTN1 status through RPmsg and change LED3 status according to this.
============================================================================
 */

#include "rpmsg/rpmsg_rtos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "string.h"
#include "board.h"
#include "mu_imx.h"
#include "debug_console_imx.h"
#include <stdio.h>
#include "gpio_pins.h"
#include "gpio_imx.h"
#include "gpio_ctrl.h"
#include "rdc_semaphore.h"


/**
 * msleep
 *
 * @param num_msec - delay time in ms.
 *
 * This is not an accurate delay, it ensures at least num_msec passed when return.
 */
void msleep(int num_msec)
{
    uint32_t loop;

    /* Recalculate the CPU frequency */
    SystemCoreClockUpdate();

    /* Calculate the CPU loops to delay, each loop has 3 cycles */
    loop = SystemCoreClock / 3 / 1672 * num_msec;

    /* There's some difference among toolchains, 3 or 4 cycles each loop */
    while (loop)
    {
        __NOP();
        loop--;
    }
}

SemaphoreHandle_t xSemaphore;


typedef struct
{
	uint8_t data[24];
}AnalogData_t;

typedef struct
{
	AnalogData_t * const analogData;
    int head;
    int tail;
    const int maxLen;
}circBuf_t;

#define CIRCBUF_DEF(x,y) AnalogData_t x##_space[y]; circBuf_t x = { x##_space,0,0,y}
CIRCBUF_DEF(cb, 100);

//AnalogData_t dataBuffer[32];
//circBuf_t cp = {&dataBuffer, 0,0,32};

int circBufPush(circBuf_t *c, AnalogData_t data)
{
	int next = c->head + 1;
	if (next >= c->maxLen)
		next = 0;

	// Cicular buffer is full
	if (next == c->tail)
		return -1;  // quit with an error

	c->analogData[c->head] = data;
	c->head = next;
	return 0;
}

int circBufPop(circBuf_t *c, AnalogData_t *data, int * index)
{
	// if the head isn't ahead of the tail, we don't have any characters
	if (c->head == c->tail)
	{
		return -1;  // quit with an error
	}

	*data = c->analogData[c->tail];
	memset(c->analogData[c->tail].data, 0, sizeof(AnalogData_t));

	int next = c->tail + 1;
	if(next >= c->maxLen)
		next = 0;

	c->tail = next;
	*index = next;
	return 0;
}


/*
 * APP decided interrupt priority
 */
#define APP_MU_IRQ_PRIORITY 3

/******************************************************************************
*
* Function Name: ToggleTask
* Comments: this task is used to turn toggle on/off LED.
*
******************************************************************************/
void ToggleTask(void *pvParameters)
{
	AnalogData_t localAnalogData;
    PRINTF("\n\rToggleTask\n\r");
    RDC_SEMAPHORE_Lock(BOARD_GPIO_KEY_RDC_PDAP);
	NVIC_DisableIRQ(BOARD_GPIO_KEY_IRQ_NUM);
	GPIO_ClearStatusFlag(BOARD_GPIO_KEY_CONFIG->base, BOARD_GPIO_KEY_CONFIG->pin);
	GPIO_SetPinIntMode(BOARD_GPIO_KEY_CONFIG->base, BOARD_GPIO_KEY_CONFIG->pin, true);
	NVIC_EnableIRQ(BOARD_GPIO_KEY_IRQ_NUM);
    RDC_SEMAPHORE_Unlock(BOARD_GPIO_KEY_RDC_PDAP);
    do
	{
	    if (pdTRUE == xSemaphoreTake(xSemaphore, portMAX_DELAY))
	    {
	    	GPIO_Ctrl_ToggleLed0();
	        RDC_SEMAPHORE_Lock(BOARD_GPIO_KEY_RDC_PDAP);
	    	GPIO_ClearStatusFlag(BOARD_GPIO_KEY_CONFIG->base, BOARD_GPIO_KEY_CONFIG->pin);
	    	GPIO_SetPinIntMode(BOARD_GPIO_KEY_CONFIG->base, BOARD_GPIO_KEY_CONFIG->pin, true);
	        GPIO_ClearStatusFlag(BOARD_GPIO_KEY_CONFIG->base, BOARD_GPIO_KEY_CONFIG->pin);
	    	NVIC_EnableIRQ(BOARD_GPIO_KEY_IRQ_NUM);
	        RDC_SEMAPHORE_Unlock(BOARD_GPIO_KEY_RDC_PDAP);
	        memset(&localAnalogData, 0xAA, sizeof(AnalogData_t));
	        circBufPush(&cb, localAnalogData);
	        GPIO_Ctrl_ToggleLed0();
	    }
	} while (1);
}

/******************************************************************************
*
* Function Name: SwitchTask
* Comments: this task is used to change blinking frequency.
*
******************************************************************************/
void SwitchTask(void *pvParameters)
{
    int result;
    struct remote_device *rdev = NULL;
    struct rpmsg_channel *app_chnl = NULL;
    void *rx_buf;
    int len;
    void *tx_buf;
    unsigned long size;
	AnalogData_t localAnalogDataRead;
	char buff[200];
    /* Print the initial banner */
    PRINTF("\r\nRPMSG String Echo FreeRTOS RTOS API Demo...\r\n");

    /* RPMSG Init as REMOTE */
    PRINTF("RPMSG Init as Remote\r\n");
    result = rpmsg_rtos_init(0 /*REMOTE_CPU_ID*/, &rdev, RPMSG_MASTER, &app_chnl);
    assert(result == 0);

    PRINTF("Name service handshake is done, M4 has setup a rpmsg channel [%d ---> %d]\r\n", app_chnl->src, app_chnl->dst);


    while (true)
    {

    	memset(buff, 0, sizeof(buff));
    	int index = 0;
    	GPIO_Ctrl_ToggleLed1();
    	char ret = circBufPop(&cb, &localAnalogDataRead, &index);
    	if(ret == 0)
    	{

    		sprintf(buff, "%d", index);
    		for(int i=0; i < 24; i++)
    		{
    			sprintf(buff, "%s;%hhu", buff, localAnalogDataRead.data[i]);
    		}
    		sprintf(buff, "%sEND\n", buff);
    		/* Get tx buffer from RPMsg */
    		tx_buf = rpmsg_rtos_alloc_tx_buffer(app_chnl->rp_ept, &size);
    		assert(tx_buf);

    		len = strlen(buff);
    		/* Copy string to RPMsg tx buffer */
    		memcpy(tx_buf, buff, len);
    		/* Echo back received message with nocopy send */
    		result = rpmsg_rtos_send_nocopy(app_chnl->rp_ept, tx_buf, len, app_chnl->dst);
    		assert(result == 0);

    		vTaskDelay(0);
    		GPIO_Ctrl_ToggleLed1();
    	}
    	else
    	{
    		vTaskDelay(configTICK_RATE_HZ);
    		GPIO_Ctrl_ToggleLed1();
    	}
    }
}

/*
 * MU Interrrupt ISR
 */
void BOARD_MU_HANDLER(void)
{
    /*
     * calls into rpmsg_handler provided by middleware
     */
    rpmsg_handler();
}


/******************************************************************************
*
* Function Name: main
* Comments: main function, toggle LED and switch the blinking frequency by key.
*
******************************************************************************/
int main(void)
{
    /* hardware initialiize, include RDC, IOMUX, Uart debug initialize */
    hardware_init();
    PRINTF("\n\r====================== GPIO Example ========================\n\r");

    /* GPIO module initialize, configure "LED" as output and button as interrupt mode. */
    GPIO_Ctrl_Init();

    /* Data acquire sync semi4 init. */
    xSemaphore = xSemaphoreCreateBinary();

    /*
     * Prepare for the MU Interrupt
     *  MU must be initialized before rpmsg init is called
     */
    MU_Init(BOARD_MU_BASE_ADDR);
    NVIC_SetPriority(BOARD_MU_IRQ_NUM, APP_MU_IRQ_PRIORITY);
    NVIC_EnableIRQ(BOARD_MU_IRQ_NUM);


    /* Create a the APP main task. */
    xTaskCreate(ToggleTask, "Toggle Task", configMINIMAL_STACK_SIZE + 100,
                NULL, tskIDLE_PRIORITY+4, NULL);
    xTaskCreate(SwitchTask, "Switch Task", configMINIMAL_STACK_SIZE + 100,
                NULL, tskIDLE_PRIORITY+1, NULL);

    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();

    /* should never reach this point. */
    while (true);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
