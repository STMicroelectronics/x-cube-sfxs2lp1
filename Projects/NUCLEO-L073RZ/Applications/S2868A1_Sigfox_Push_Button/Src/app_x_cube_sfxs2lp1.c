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
#include "s2868a1_conf.h"
#include "stm32l0xx_nucleo.h"
#include "s2lp_management.h"
#include "retriever_api.h"

/* Private Macros -----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef USE_FLASH
#ifdef __ICCARM__
#pragma data_alignment=FLASH_PAGE_SIZE
#endif
#endif

/* Exported variables --------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Some variables to store the application data to transmit */
uint8_t ret_err, use_public_key = 0;
uint32_t cust_counter=0;
uint8_t customer_data[12]={0};
uint8_t customer_resp[8];

/**
* @brief  Initialize the Sigfox CLI Example
* @retval None
*/
void MX_X_CUBE_SFXS2LP1_Init(void)
{
   /*Local Variables*/
  NVM_BoardDataType sfxConfiguration;
  ST_SFX_ERR stSfxRetErr;

  /* Configure USER Key Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* S2LP ON */
  S2868A1_RADIO_EnterShutdown();
  HAL_Delay(10);
  S2868A1_RADIO_ExitShutdown();

  S2868A1_RADIO_Init();

  S2868A1_SPI_DeInit();
  S2868A1_SPI_Init();
  HAL_SPI_RegisterCallback(&hspi, HAL_SPI_TX_RX_COMPLETE_CB_ID, SFX_SPI_TxRxCpltCallback);
  S2868A1_RADIO_SPI_NSS_PIN_HIGH();
  HAL_Delay(10);

  /* uC IRQ enable */
   RadioIRQEnable(1,0);

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
  S2868A1_RADIO_EnterShutdown();
  HAL_Delay(10);
  S2868A1_RADIO_ExitShutdown();

    /* Calibrate RTC in case of STM32*/
  /* The low level driver uses the internal RTC as a timer while the STM32 is in low power.
  This function calibrates the RTC using an auxiliary general purpose timer in order to
  increase its precision. */
  ST_MCU_API_TimerCalibration(500);

  /* FEM Initialization */
  FEM_Operation(FEM_SHUTDOWN);

  /* Init the Sigfox Library and the device for Sigfox communication*/
  stSfxRetErr = ST_Sigfox_Init(&sfxConfiguration, 1);

  if(stSfxRetErr != ST_SFX_ERR_NONE)
    /*Fatal_Error(); */

  if(use_public_key)
    enc_utils_set_public_key(1);

  BSP_LED_Init(LED2);

}

/**
* @brief  X-CUBE SFXS2LP1 Process
* @retval None
*/
void  MX_X_CUBE_SFXS2LP1_Process(void)
{
  /* Go in low power with the STM32 waiting for an external interrupt */
  setGpioLowPower();
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
  ST_MCU_API_SetSysClock();
  setGpioRestore();

  if(but_pressed)
  {
   LedBlink(6);

    /* If the interrupt is raised, prepare the buffer to send with a 4-bytes counter */
    cust_counter++;

    for(uint8_t i=0;i<4;i++)
      customer_data[i]=(uint8_t)(cust_counter>>((3-i)*8));

    /* Call the send_frame function */
    SIGFOX_API_send_frame(customer_data,4,customer_resp,2,0);

    LedBlink(6);
    but_pressed=0;
  }
}

#ifdef __cplusplus
}
#endif

