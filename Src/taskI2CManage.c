/*
 * taskI2CManage.c
 *
 *  Created on: 12 nov. 2019
 *      Author: Gerardo
 */
#include "FreeRTOS.h"
#include "task.h"
#include "i2c.h"
#include "stm32l4xx_ll_i2c.h"

/*private prototypes*/
void taskI2CManage_task(void * pvParameters);
void taskI2CManageslave_task(void * pvParameters);

void taskI2CManage_init(void){

	//Configure_I2C_Master();

	/* Handle I2C3 events (Master) */
	//Handle_I2C_Master();

	xTaskCreate(
			taskI2CManage_task,       /* Function that implements the task. */
			"I2C management task",          /* Text name for the task. */
			configMINIMAL_STACK_SIZE,      /* Stack size in words, not bytes. */
			( void * ) 1,    /* Parameter passed into the task. */
			tskIDLE_PRIORITY,/* Priority at which the task is created. */
			NULL);      /* Used to pass out the created task's handle. */

	xTaskCreate(
				taskI2CManageslave_task,       /* Function that implements the task. */
				"I2C management task",          /* Text name for the task. */
				configMINIMAL_STACK_SIZE,      /* Stack size in words, not bytes. */
				( void * ) 1,    /* Parameter passed into the task. */
				tskIDLE_PRIORITY,/* Priority at which the task is created. */
				NULL);      /* Used to pass out the created task's handle. */
}

void config_master(void){

	/* (1) Enables GPIO clock and configures the I2C3 pins **********************/
	/*    (SCL on PC.0, SDA on PC.1)                     **********************/

	/* Enable the peripheral clock of GPIOC */
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

	/* Configure SCL Pin as : Alternate function, High Speed, Open drain, Pull up */
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOC, LL_GPIO_PIN_0, LL_GPIO_AF_4);
	LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_0, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_0, LL_GPIO_OUTPUT_OPENDRAIN);
	LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_0, LL_GPIO_PULL_UP);

	/* Configure SDA Pin as : Alternate function, High Speed, Open drain, Pull up */
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOC, LL_GPIO_PIN_1, LL_GPIO_AF_4);
	LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_1, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_1, LL_GPIO_OUTPUT_OPENDRAIN);
	LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_1, LL_GPIO_PULL_UP);

	/* (2) Enable the I2C3 peripheral clock and I2C3 clock source ****************/

	/* Enable the peripheral clock for I2C3 */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C3);

	/* Set I2C3 clock source as SYSCLK */
	LL_RCC_SetI2CClockSource(LL_RCC_I2C3_CLKSOURCE_SYSCLK);

	/* (3) Configure I2C3 functional parameters ********************************/

	/* Disable I2C3 prior modifying configuration registers */
	LL_I2C_Disable(I2C3);

	/* Configure the SDA setup, hold time and the SCL high, low period */
	/* (uint32_t)0x00F02B86 = I2C_TIMING*/
	LL_I2C_SetTiming(I2C3, (uint32_t)0x00F02B86);

	/* Configure the Own Address1                   */
	/* Reset Values of :
	 *     - OwnAddress1 is 0x00
	 *     - OwnAddrSize is LL_I2C_OWNADDRESS1_7BIT
	 *     - Own Address1 is disabled
	 */
	//LL_I2C_SetOwnAddress1(I2C3, 0x00, LL_I2C_OWNADDRESS1_7BIT);
	//LL_I2C_DisableOwnAddress1(I2C3);

	/* Enable Clock stretching */
	/* Reset Value is Clock stretching enabled */
	//LL_I2C_EnableClockStretching(I2C3);

	/* Configure Digital Noise Filter */
	/* Reset Value is 0x00            */
	//LL_I2C_SetDigitalFilter(I2C3, 0x00);

	/* Enable Analog Noise Filter           */
	/* Reset Value is Analog Filter enabled */
	//LL_I2C_EnableAnalogFilter(I2C3);

	/* Enable General Call                  */
	/* Reset Value is General Call disabled */
	//LL_I2C_EnableGeneralCall(I2C3);

	/* Configure the 7bits Own Address2               */
	/* Reset Values of :
	 *     - OwnAddress2 is 0x00
	 *     - OwnAddrMask is LL_I2C_OWNADDRESS2_NOMASK
	 *     - Own Address2 is disabled
	 */
	//LL_I2C_SetOwnAddress2(I2C3, 0x00, LL_I2C_OWNADDRESS2_NOMASK);
	//LL_I2C_DisableOwnAddress2(I2C3);

	/* Configure the Master to operate in 7-bit or 10-bit addressing mode */
	/* Reset Value is LL_I2C_ADDRESSING_MODE_7BIT                         */
	//LL_I2C_SetMasterAddressingMode(I2C3, LL_I2C_ADDRESSING_MODE_7BIT);

	/* (4) Enable I2C3 **********************************************************/
	LL_I2C_Enable(I2C3);
}

void config_slave(void){
	uint32_t timing = 0;

	  /* (1) Enables GPIO clock and configures the I2C3 pins **********************/
	  /*    (SCL on PC.0, SDA on PC.1)                     **********************/

	  /* Enable the peripheral clock of GPIOC */
	  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

	  /* Configure SCL Pin as : Alternate function, High Speed, Open drain, Pull up */
	  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
	  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_6, LL_GPIO_AF_4);
	  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_OPENDRAIN);
	  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);

	  /* Configure SDA Pin as : Alternate function, High Speed, Open drain, Pull up */
	  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
	  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_7, LL_GPIO_AF_4);
	  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_OPENDRAIN);
	  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_UP);

	  /* (2) Enable the I2C3 peripheral clock and I2C3 clock source ****************/

	  /* Enable the peripheral clock for I2C3 */
	  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

	  /* Set I2C3 clock source as SYSCLK */
	  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_SYSCLK);

	  /* (3) Configure I2C3 functional parameters ********************************/

	  /* Disable I2C3 prior modifying configuration registers */
	  LL_I2C_Disable(I2C1);

	  /* Configure the SDA setup, hold time and the SCL high, low period */
	  /* Timing register value is computed with the STM32CubeMX Tool,
	    * fast Mode @400kHz with I2CCLK = 80MHz, rise time = 100ns,
	    * fall time = 10ns
	    * Timing Value = (uint32_t)0x00F02B86
	    */
	  timing = __LL_I2C_CONVERT_TIMINGS(0x0, 0xF, 0x0, 0x2B, 0x86);
	  LL_I2C_SetTiming(I2C1, timing);

	  /* Configure the Own Address1 :
	   *  - OwnAddress1 is SLAVE_OWN_ADDRESS
	   *  - OwnAddrSize is LL_I2C_OWNADDRESS1_7BIT
	   *  - Own Address1 is enabled
	   */
	  LL_I2C_SetOwnAddress1(I2C1, 0x64, LL_I2C_OWNADDRESS1_7BIT);
	  LL_I2C_EnableOwnAddress1(I2C1);

	  /* Enable Clock stretching */
	  /* Reset Value is Clock stretching enabled */
	  //LL_I2C_EnableClockStretching(I2C3);

	  /* Configure Digital Noise Filter */
	  /* Reset Value is 0x00            */
	  //LL_I2C_SetDigitalFilter(I2C3, 0x00);

	  /* Enable Analog Noise Filter           */
	  /* Reset Value is Analog Filter enabled */
	  //LL_I2C_EnableAnalogFilter(I2C3);

	  /* Enable General Call                  */
	  /* Reset Value is General Call disabled */
	  //LL_I2C_EnableGeneralCall(I2C3);

	  /* Configure the 7bits Own Address2               */
	  /* Reset Values of :
	   *     - OwnAddress2 is 0x00
	   *     - OwnAddrMask is LL_I2C_OWNADDRESS2_NOMASK
	   *     - Own Address2 is disabled
	   */
	  //LL_I2C_SetOwnAddress2(I2C3, 0x00, LL_I2C_OWNADDRESS2_NOMASK);
	  //LL_I2C_DisableOwnAddress2(I2C3);

	  /* (4) Enable I2C3 **********************************************************/
	  LL_I2C_Enable(I2C1);
}
void taskI2CManage_task(void * pvParameters){


	volatile uint8_t data = 0 ;
	volatile uint8_t u8_recData[10];
	volatile int rec_index = 0;

	while(1){

		if(!LL_GPIO_IsInputPinSet( GPIOC, LL_GPIO_PIN_13 )){
			/* (1) Initiate a Start condition to the Slave device ***********************/

			/* Master Generate Start condition for a write request :              */
			/*    - to the Slave with a 7-Bit SLAVE_OWN_ADDRESS                   */
			/*    - with a auto stop condition generation when transmit all bytes */
			LL_I2C_HandleTransfer(I2C3, 0x64, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

			/* (2) Loop until end of transfer received (STOP flag raised) ***************/

			/* Loop until STOP flag is raised  */
			while(!LL_I2C_IsActiveFlag_STOP(I2C3))
			{
				/* (2.1) Transmit data (TXIS flag raised) *********************************/

				/* Check TXIS flag value in ISR register */
				if(LL_I2C_IsActiveFlag_TXIS(I2C3))
				{
					/* Write data in Transmit Data register.
			      TXIS flag is cleared by writing data in TXDR register */
					data++;
					LL_I2C_TransmitData8(I2C3, 0x05);
				}
			}

			/* (3) Clear pending flags, Data consistency are checking into Slave process */

			/* End of I2C_SlaveReceiver_MasterTransmitter Process */
			LL_I2C_ClearFlag_STOP(I2C3);

		}
		vTaskDelay(100);
	}
}

void taskI2CManageslave_task(void * pvParameters){

	uint8_t      aReceiveBuffer[0xF] = {0};
	__IO uint8_t ubReceiveIndex      = 0;
	while(1){

		while(!LL_I2C_IsActiveFlag_ADDR(I2C1))
		{
		}

		/* Verify the Address Match with the OWN Slave address */
		if(LL_I2C_GetAddressMatchCode(I2C1) == 0x64)
		{
			/* Verify the transfer direction, a write direction, Slave enters receiver mode */
			if(LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_WRITE)
			{
				/* Clear ADDR flag value in ISR register */
				LL_I2C_ClearFlag_ADDR(I2C1);
			}
			else
			{
				/* Clear ADDR flag value in ISR register */
				LL_I2C_ClearFlag_ADDR(I2C1);
			}
		}
		else
		{
			/* Clear ADDR flag value in ISR register */
			LL_I2C_ClearFlag_ADDR(I2C1);

			/* Call Error function */
		}

		/* (2) Loop until end of transfer received (STOP flag raised) ***************/

		/* Loop until STOP flag is raised  */
		while(!LL_I2C_IsActiveFlag_STOP(I2C1))
		{
			/* (2.1) Receive data (RXNE flag raised) **********************************/

			/* Check RXNE flag value in ISR register */
			if(LL_I2C_IsActiveFlag_RXNE(I2C1))
			{
				/* Read character in Receive Data register.
		      RXNE flag is cleared by reading data in RXDR register */
				aReceiveBuffer[ubReceiveIndex++] = LL_I2C_ReceiveData8(I2C1);
			}
		}

		/* (3) Clear pending flags, Check Data consistency **************************/

		/* End of I2C_SlaveReceiver_MasterTransmitter_DMA Process */
		LL_I2C_ClearFlag_STOP(I2C3);

		vTaskDelay(100);
	}
}
