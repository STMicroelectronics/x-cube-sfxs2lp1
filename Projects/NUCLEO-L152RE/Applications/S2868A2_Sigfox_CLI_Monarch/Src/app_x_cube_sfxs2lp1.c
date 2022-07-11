/**
  ******************************************************************************
  * File Name          :  stmicroelectronics_x-cube-sfxs2lp1_4_0_0.c
  * Description        : This file provides code for the configuration
  *                      of the STMicroelectronics.X-CUBE-SFXS2LP1.4.0.0 instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "main.h"

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include "assert.h"
#include "stddef.h"
#include "app_x_cube_sfxs2lp1.h"
#include "s2868a2_conf.h"
#include "s2868a2.h"
#include  "retriever_api.h"
#include "s2lp_management.h"

/* Private Macros -----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef USE_FLASH
#ifdef __ICCARM__
#pragma data_alignment=FLASH_PAGE_SIZE
#endif
#endif

/* Private variables ---------------------------------------------------------*/
uint8_t ret_err, use_public_key = 0;
uint32_t cust_counter=0;
uint8_t customer_data[12]={0};
uint8_t customer_resp[8];

/* Private Functions ---------------------------------------------------------*/
/*static void Fatal_Error(void); */

/**
* @brief  Initialize the Sigfox CLI Example
* @retval None
*/
void MX_X_CUBE_SFXS2LP1_Init(void)
{
  /*Local Variables*/
  NVM_BoardDataType sfxConfiguration;
  ST_SFX_ERR stSfxRetErr;

  /* S2LP ON */
  S2868A2_RADIO_EnterShutdown();
  HAL_Delay(10);
  S2868A2_RADIO_ExitShutdown();

  S2868A2_RADIO_Init();

  S2868A2_SPI_DeInit();
  S2868A2_SPI_Init();
  HAL_SPI_RegisterCallback(&hspi, HAL_SPI_TX_RX_COMPLETE_CB_ID, SFX_SPI_TxRxCpltCallback);
  S2868A2_RADIO_SPI_NSS_PIN_HIGH();
  HAL_Delay(10);

  /* uC IRQ enable */
  RadioIRQEnable(1,0);

  RadioTimersTimConfig(&COM_TIM_HANDLE,64-1,1000-1);
  RadioTimersState(&COM_TIM_HANDLE, ENABLE);

  BSP_COM_Init(COM1);
  HAL_UART_RegisterCallback(&hcom_uart[COM1], HAL_UART_TX_COMPLETE_CB_ID, SFX_COM_TxCpltCallback);
  HAL_UART_Receive_DMA(&hcom_uart[COM1],rxQ,400);

  /* Set the EEPROM availability */
  RadioSetHasEeprom(EEPROM_PRESENT);

  /* Auto detect settings, if EEPROM is available */
  if (RadioGetHasEeprom())
  {
    /* Identify the S2-LP RF board reading some production data */
    S2LPManagementIdentificationRFBoard();
  }
  else
  {
    /* Set XTAL frequency with offset */
    S2LP_RADIO_SetXtalFrequency(XTAL_FREQUENCY+XTAL_FREQUENCY_OFFSET);

    /* Set the frequency base */
    S2LP_ManagementSetBand(BOARD_FREQUENCY_BAND);

    /* Configure PA availability */
    S2LP_ManagementSetRangeExtender(DetetctPA());
  }

  /* TCXO Initialization */
  TCXO_Init();

  /* Reset S2LP */
  S2868A2_RADIO_EnterShutdown();
  HAL_Delay(10);
  S2868A2_RADIO_ExitShutdown();

  /* Init the Sigfox Library and the device for Sigfox communication*/
  stSfxRetErr = ST_Sigfox_Init(&sfxConfiguration, 0);

  if(stSfxRetErr != ST_SFX_ERR_NONE)
  {
    /* If an error occured reading Sigfox credentials (for example the board has never been registered)
    * automatically set the test mode credentials. */
    if(stSfxRetErr == ST_SFX_ERR_CREDENTIALS)
    {
      sfxConfiguration.id = 0;
      memset(sfxConfiguration.pac, 0x00, 8);
      sfxConfiguration.rcz = 0;

      set_testMode(1);
    }
    else
	{
      /* Fatal_Error(); */
	}
  }

  /* Set Sigfox configuration for the commands layer */
  set_id(sfxConfiguration.id);
  set_pac(sfxConfiguration.pac);
  set_rcz(sfxConfiguration.rcz);

  /* Calibrate RTC in case of STM32*/
  /* The low level driver uses the internal RTC as a timer while the STM32 is in low power.
  This function calibrates the RTC using an auxiliary general purpose timer in order to
  increase its precision. */
  ST_MCU_API_TimerCalibration(500);

  /* User Application */
  if(get_testMode())
    printf("Sigfox CLI demo - TEST MODE (board not registered)\r\n");
  else
    printf("Sigfox CLI demo\r\n");

  printf("ID: %.8X - PAC: ", (unsigned int)sfxConfiguration.id);
  for(uint16_t i = 0; i < sizeof(sfxConfiguration.pac); i++)
  {
    printf("%.2X", sfxConfiguration.pac[i]);
  }
  printf("\r\n");

}

/**
* @brief  X-CUBE SFXS2LP1 Process
* @retval None
*/
void  MX_X_CUBE_SFXS2LP1_Process(void)
{
      /* CLI parser loop */
    if(processCmdInput(interactive))
    {
	if (interactive) {
	  printf(">");
	}
    }
}

#ifdef __cplusplus
}
#endif

