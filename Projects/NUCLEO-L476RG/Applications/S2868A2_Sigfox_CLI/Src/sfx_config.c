/**
  * @file    sfx_config.c
  * @author  STM32ODE Team, Noida
  * @brief   This file defines the manufacturer's MCU functions to be implemented
  * for library usage.External API dependencies to link with this library.
  * Error codes of the MCU API functions are described below.
  * The Manufacturer can add more error code taking care of the limits defined.
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
#include <string.h>
#include "assert.h"
#include "stddef.h"
#include "sfx_config.h"
#include "bsp_ip_conf.h"
#include "retriever_api.h"
#include "serial_utils.h"
#include "s2868a2.h"
#include "s2868a2_conf.h"
#include "stm32l4xx_nucleo_bus.h"
#include "stm32l4xx.h"
#include "stm32l4xx_hal_gpio.h"

/** @defgroup Sigfox configuration file
  * @{
  */

#define DELAY_CS_SCLK			0x70	/* Delay between CSn falling edge & start of SCLK */

/* Private variables ---------------------------------------------------------*/
static volatile RangeExtType s_RfRangeExtender = RANGE_EXT_NONE;
uint32_t  PAGEError = 0;
static volatile uint8_t spi_in_use=0;
static uint8_t rx_buff[128];
static uint8_t waiting_irq=0;
volatile static uint8_t SpiTxCpltFlag = 0;

GpioIrqHandler *GpioIrq[] = {

#if (USE_S2868A2_RADIO_GPIO_0 == 1)
  HAL_EXTI_SFX_Callback,
#endif
#if (USE_S2868A2_RADIO_GPIO_1 == 1)
  HAL_EXTI_SFX_Callback,
#endif
#if (USE_S2868A2_RADIO_GPIO_2 == 1)
  HAL_EXTI_SFX_Callback,
#endif
#if (USE_S2868A2_RADIO_GPIO_3 == 1)
  HAL_EXTI_SFX_Callback,
#endif
};

static volatile uint32_t s_XtalFrequency=50000000;
volatile static uint32_t s_lXtalFrequency=50000000;
static volatile uint32_t s_RfXtalFrequency = 50000000;
static uint8_t s_eeprom  = 0;
static uint32_t timer_cnt=0;
uint8_t rxQ[RECEIVE_QUEUE_SIZE];
uint16_t rxHead = 0;
uint16_t rxTail = 0;
volatile uint16_t rxUsed = 0;
DMA_HandleTypeDef dma_handle_rx,dma_handle_tx;
uint8_t txQ[TRANSMIT_QUEUE_SIZE];
uint16_t txHead = 0;
uint16_t txTail = 0;
volatile uint16_t txUsed = 0,txLastRequest=0;
uint8_t dmaTransmitting = 0;
/*Flags declarations*/
volatile int MasterFlag = 0;
__IO uint32_t KEYStatusData = 0x00;
uint16_t exitCounter = 0;
uint16_t txCounter = 0;
uint16_t wakeupCounter = 0;
uint16_t dataSendCounter = 0x00;
static volatile FlagStatus s_xTIMChCompareModeRaised = RESET;
volatile uint8_t interactive = 1;
volatile uint8_t recv_cmd;
volatile CommandEntry* cf=NULL;
CommandEntry CommandTable[] = {
  { "help", helpAction, "", "List commands"},
  { "interactive", interactiveAction, "u", "Set interactive mode"},
  { "fw_version", fwVersionAction, "", "Get fw version"},
  { "reboot", rebootAction, "", "reboots the uC"},

  COMMAND_TABLE,
#ifndef MON_REF_DES
  EEPROM_COMMANDS_TABLE,
#endif
  { NULL, NULL, NULL, NULL } /* NULL action makes this a terminator  */
};
/* Private functions -------------------------------------------------------*/
static void SetSpiTxCompleteFlag(void);
static void ResetSpiTxCompleteFlag(void);
static uint8_t GetSpiTxCompleteFlag(void);
static void prepareDmaTx(void);

/**
 * @brief  Enable the MCU Pin as Interrupt for Radio S2-LP.
 * @param  state : Enable(1) or Disable(0).
 * @param  edge_direction : rising(1) or Falling(0).
 * @retval None.
 */
void RadioIRQEnable(uint8_t state, uint8_t edge_direction)
{
  /*Select Edge*/
  if(edge_direction == 0)
  {
    S2868A2_RADIO_GPIO_Init_Update(S2LP_RADIO_GPIO, RADIO_MODE_EXTI_IN,FALLING);
  }
  else if(edge_direction == 1)
  {
    S2868A2_RADIO_GPIO_Init_Update(S2LP_RADIO_GPIO, RADIO_MODE_EXTI_IN,RISING);
  }
  else
  {
    S2868A2_RADIO_GPIO_Init_Update(S2LP_RADIO_GPIO, RADIO_MODE_EXTI_IN,FALLING);
  }

  /* Handle Interrupt state */
  if(state == 1)
  {
    /* uC IRQ enable */
    S2868A2_RADIO_IoIrqEnable(GpioIrq);
  }
  else if (state == 0)
  {
    /* uC IRQ Disable */
    S2868A2_RADIO_IoIrqDisable(GpioIrq);
  }
}

/**
* @brief  Computes two integer value prescaler and period such that
*         Cycles = prescaler * period.
* @param  lCycles the specified cycles for the desired timer value.
* @param  pnPrescaler prescaler factor.
* @param  pnCounter period factor.
* @retval None.
*/
void RadioTimersFindFactors(uint32_t lCycles,
                            uint16_t *pnPrescaler,
                            uint16_t *pnCounter)
{
  uint16_t b0;
  uint16_t a0;
  long err, err_min=lCycles;

  *pnPrescaler = a0 = ((lCycles-1)/0xffff) + 1;
  *pnCounter = b0 = lCycles / *pnPrescaler;

  for (; *pnPrescaler < 0xffff-1; (*pnPrescaler)++) {
    *pnCounter = lCycles / *pnPrescaler;
    err = (long)*pnPrescaler * (long)*pnCounter - (long)lCycles;
    if (ABS(err) > (*pnPrescaler / 2)) {
      (*pnCounter)++;
      err = (long)*pnPrescaler * (long)*pnCounter - (long)lCycles;
    }
    if (ABS(err) < ABS(err_min)) {
      err_min = err;
      a0 = *pnPrescaler;
      b0 = *pnCounter;
      if (err == 0) break;
    }
  }

  *pnPrescaler = a0;
  *pnCounter = b0;
}

/**
* @brief  Configures the GPIO into Low Power
* @param  None
* @retval None.
*/

void RadioSetGpioLowPwr(void)
{
	/*SAMPLE CODE*/
  /* For STM32, gpio power consumption is reduced when GPIOs are configured as
  no pull - analog, during this configuration we have to sure thath wake-up pins
  still remain as digital inputs*/

  /* Below is the Sample code for Putting the GPIOs in LOW power to
     further reduce the consumption.
  // Build the first part of the init structre for all GPIOs and ports
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Mode       = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull       = GPIO_NOPULL;
  GPIO_InitStructure.Speed      = GPIO_SPEED_HIGH;

  // -------------------------------- PORT A -------------------------------------
  // SDN , CS S2-LP, CS E2PROM, UART RX/TX, BUTTON1
  GPIO_InitStructure.Pin = GPIO_PIN_All & (~GPIO_PIN_8) & (~GPIO_PIN_1) & (~GPIO_PIN_9)\
    & (~GPIO_PIN_2) & (~GPIO_PIN_3) & (~GPIO_PIN_4) & (~GPIO_PIN_0)& (~GPIO_PIN_6) & (~GPIO_PIN_7) & (~GPIO_PIN_13) & (~GPIO_PIN_14);
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  // -------------------------------- PORT B -------------------------------------
  GPIO_InitStructure.Pin = GPIO_PIN_All & (~GPIO_PIN_4) & (~GPIO_PIN_0)& (~GPIO_PIN_3);
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  // -------------------------------- PORT C -------------------------------------
  // IRQ Pin + TCXO Enable
  GPIO_InitStructure.Pin = GPIO_PIN_All & (~GPIO_PIN_0)& (~GPIO_PIN_7);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  // -------------------------------- PORT D -------------------------------------
  GPIO_InitStructure.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

  HAL_SuspendTick();      */
}

/**
* @brief  Configures the GPIO into Low Power
* @param  None
* @retval None.
*/

void RadioRestoreGpio(void)
{
	/*SAMPLE CODE*/
  /* For STM32, restore every gpio, previosly set as analog as digital */

  /*Below is the sample code to restore the
      GPIO of the SPI after exit from Low Power
  // Restore all Gpio CLKs
  STM32_GPIO_CLK_ENABLE();

   // SPI_MOSI/MISO
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Mode       = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull       = GPIO_PULLUP;
  GPIO_InitStructure.Speed      = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Alternate  = BUS_SPI1_MISO_GPIO_AF;
  GPIO_InitStructure.Pin        = BUS_SPI1_MISO_GPIO_PIN;
  HAL_GPIO_Init(BUS_SPI1_MISO_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Mode       = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull       = GPIO_PULLUP;
  GPIO_InitStructure.Speed      = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Alternate  = BUS_SPI1_MOSI_GPIO_AF;
  GPIO_InitStructure.Pin        = BUS_SPI1_MOSI_GPIO_PIN;
  HAL_GPIO_Init(BUS_SPI1_MOSI_GPIO_PORT, &GPIO_InitStructure);

  // SPI_CLK
  GPIO_InitStructure.Mode       = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull       = GPIO_PULLUP;
  GPIO_InitStructure.Alternate  = BUS_SPI1_SCK_GPIO_AF;
  GPIO_InitStructure.Pin        = BUS_SPI1_SCK_GPIO_PIN;
  HAL_GPIO_Init(BUS_SPI1_SCK_GPIO_PORT, &GPIO_InitStructure);

  HAL_ResumeTick();  */
}

/**
* @brief  Configures the specified timer to raise an interrupt every time the counter
*         reaches the nPeriod value counting with a prescaler of nPrescaler.
* @note   The specified timer is configured but not enabled.
* @param  TIM_TimeBaseStructure Timer Handler of the timer to be set.
*          This parameter can be a pointer to @ref TIM_HandleTypeDef .
* @param  nPrescaler prescaler factor.
* @param  nPeriod period factor.
* @retval None.
*/
void RadioTimersTimConfig(TIM_HandleTypeDef* TIM_TimeBaseStructure,
                          uint16_t nPrescaler,
                          uint16_t nPeriod)
{

 if(TIM_TimeBaseStructure == (&SFX_TIM_HANDLE))
  {

    BSP_IP_SFX_TIM_Init();

    /* USER CODE END TIM2_Init 1 */
    SFX_TIM_HANDLE.Instance = BSP_SFX_TIM;
    SFX_TIM_HANDLE.Init.Prescaler = nPrescaler;
    SFX_TIM_HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
    SFX_TIM_HANDLE.Init.Period = nPeriod;
    SFX_TIM_HANDLE.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    SFX_TIM_HANDLE.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&SFX_TIM_HANDLE) != HAL_OK)
    {
      Error_Handler();
    }
   HAL_TIM_RegisterCallback(&SFX_TIM_HANDLE, HAL_TIM_PERIOD_ELAPSED_CB_ID, SFX_TIM_PeriodElapsedCallback);
  }
  if(TIM_TimeBaseStructure == (&COM_TIM_HANDLE))
  {
    BSP_IP_COM_TIM_Init();
    /* USER CODE END TIM2_Init 1 */
    COM_TIM_HANDLE.Instance = BSP_COM_TIM;
    COM_TIM_HANDLE.Init.Prescaler = nPrescaler;
    COM_TIM_HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
    COM_TIM_HANDLE.Init.Period = nPeriod;
    COM_TIM_HANDLE.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    COM_TIM_HANDLE.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&COM_TIM_HANDLE) != HAL_OK)
    {
      Error_Handler();
    }
    HAL_TIM_RegisterCallback(&COM_TIM_HANDLE, HAL_TIM_PERIOD_ELAPSED_CB_ID, COM_TIM_PeriodElapsedCallback);
  }
}

/****************************** EEPROM *********************************/
/**
* @brief  Read a page of the EEPROM.
*         A page size is 32 bytes.
*         The pages are 256.
*         Page 0 address: 0x0000
*         Page 1 address: 0x0020
*         ...
*         Page 255 address: 0x1FE0
* @param  None
* @retval None
*/
uint8_t EepromRead(uint16_t nAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t  status;
  status = S2868A2_EEPROM_ReadPage(EEPROM_INSTANCE,nAddress, cNbBytes, pcBuffer);

  return(status);
}

/**
* @brief  Write a page of the EEPROM.
*         A page size is 32 bytes.
*         The pages are 256.
*         Page 0 address: 0x0000
*         Page 1 address: 0x0020
*         ...
*         Page 255 address: 0x1FE0
*         It is allowed to write only a page for each operation. If the bytes
*         exceed the single page location, the other bytes are written at the
*         beginning.
* @param  None
* @retval None
*/
uint8_t EepromWrite(uint16_t nAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t status;
 status = (uint8_t) S2868A2_EEPROM_WritePage(EEPROM_INSTANCE, nAddress, cNbBytes,pcBuffer);

 return(status);
}

/******************************** S2-LP *******************************/
/**
* @brief  This function Indiactes SPI Raw Transfer complete
* @param  None
* @retval None
*/
__weak void RadioSpiRawTC(void)
{

}

/**
* @brief  This function is Tx/Rx complete callback
* @param  hspi: Pointer to the SPI handle
* @retval None
*/
void SFX_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
   ResetSpiTxCompleteFlag();

  if(waiting_irq)
  {
    S2868A2_RADIO_SPI_NSS_PIN_HIGH();
    waiting_irq=0;
    RadioSpiRawTC();
  }
}

/**
* @brief  Perform a raw SPI Write transaction with the passed buffer.
*               To perform read or write transactions:
                  - the 1st byte must be the write bytecode (0x00)
                  - 2nd byte must be the register/FIFO address
                  - from the 3rd bytes on the buffer must contain the data to be written
* @param  cNbBytes: number of bytes to be written into TX FIFO
* @param  pInBuffer: pointer to data to write
* @param  pOutBuffer: pointer to data to read
* @param  can_return_before_tc:  if this flag is 1,
*         it means that the function can be non-blocking returning immediatelly.
*               In this case the SPI uses a combination of DMA+IRQ.
* @retval Device status
*/
void RadioSpiRaw(uint8_t cNbBytes,
                        uint8_t* pInBuffer,
                        uint8_t* pOutBuffer,
                        uint8_t can_return_before_tc)
{

  uint8_t* pOutBuffer_ = pOutBuffer;
  volatile uint32_t ctr;
  uint8_t tmpFlag;

  if(pOutBuffer==NULL)
      pOutBuffer_=rx_buff;

  spi_in_use = 1;

  if(can_return_before_tc)
  {
    waiting_irq=1;
    /*Add code to enable SPI_DMA TX and RX interrupt*/
  }
  else
  {
    waiting_irq = 1;
    SetSpiTxCompleteFlag();
  }

  S2868A2_RADIO_SPI_NSS_PIN_LOW();
    for( ctr=0;ctr<DELAY_CS_SCLK;ctr++);
  S2868A2_SPI_SendRecv(pInBuffer, pOutBuffer_, cNbBytes);

  if(!can_return_before_tc)
  {
      do{
	     tmpFlag  = GetSpiTxCompleteFlag();
	    }while(tmpFlag);
    for( ctr=0;ctr<DELAY_CS_SCLK;ctr++);
    S2868A2_RADIO_SPI_NSS_PIN_HIGH();
  }
  spi_in_use = 0;

}

/**
 * @brief  Sets the Flag to denote the SPI transmission complete
 * @param  None
 * @retval None
 */
static void SetSpiTxCompleteFlag(void)
{
  SpiTxCpltFlag = 1;
}

/**
 * @brief  Resets the Flag after SPI transmission complete
 * @param  None
 * @retval None
 */
static void ResetSpiTxCompleteFlag(void)
{
  SpiTxCpltFlag = 0;

}

/**
 * @brief  Return the SPI transmission complete flag
 * @param  None
 * @retval Flag status
 */
static uint8_t GetSpiTxCompleteFlag(void)
{
  return (SpiTxCpltFlag);
}

/**
 * @brief  Front End Module Operation function.
 * This function configures the PA according to the desired status.
 * This function can be redefined for special needs.
 * @param  operation Specifies the operation to perform.
 *         This parameter can be one of following parameters:
 *         @arg FEM_SHUTDOWN: Shutdown PA
 *         @arg FEM_TX_BYPASS: Bypass the PA in TX
 *         @arg FEM_TX: TX mode
 *         @arg FEM_RX: RX mode
 * @retval None
 */
void FEM_Operation(FEM_OperationType operation)
{
}

/**
* @brief  Set SPI in use Flag.
* @param  None
* @retval None
*/
void S2LPSetSpiInUse(uint8_t state)
{
  spi_in_use = state;
}

/**
* @brief  Get whether SPI in use.
* @param  None
* @retval None
*/
uint8_t S2LPGetSpiInUse(void)
{
  return spi_in_use;
}

/*
* @brief  Configure RTC clock.
* @param  None
* @retval None
*/
void Config_RTC_Clk(void)
{
  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

}

/**
* @brief  Detect Power Amplifier
* @retval Range Extender Type
*/
RangeExtType DetetctPA(void)
{
#if S2LP_FEM_PRESENT == S2LP_FEM_NO
  	return RANGE_EXT_NONE;
#else
#ifdef MON_REF_DES
	return RANGE_EXT_SKYWORKS_SKY66420;
#else
	return RANGE_EXT_CUSTOM;
#endif
#endif
}

/**
 * @brief  Opens Sigfox Library according to the zone.
 * @param rcz The Radio Zone
 * @retval Returns 0 if ok.
 */
ST_SFX_ERR St_Sigfox_Open_RCZ(uint8_t rcz)
{
  ST_SFX_ERR open_err = ST_SFX_ERR_NONE;

  switch(rcz)
  {
    case 1:
      {
        /* Turn PA off in RC1/3/5/6/7 */
        ST_RF_API_set_pa(0);
        /* RC1 - open the Sigfox library */
        if(SIGFOX_API_open(&(sfx_rc_t)RC1)!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }
        break;
      }
    case 2:
      {
        /* Turn PA off in RC2 and RC4 */
        ST_RF_API_set_pa(1);
        /* RC2 - open the Sigfox library */
        if(SIGFOX_API_open(&(sfx_rc_t)RC2)!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }

        /* In FCC we can choose the macro channel to use by a 86 bits bitmask
        In this case we use the first 9 macro channels */
        sfx_u32 config_words[3]=RC2_SM_CONFIG;

        /* Set the standard configuration with default channel to 1 */
        if(SIGFOX_API_set_std_config(config_words,0)!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }
        break;
      }
    case 3:
      {
        volatile uint8_t ret;
        /* Turn PA off in RC1/3/5/6/7 */
        ST_RF_API_set_pa(0);
        ret=SIGFOX_API_open(&(sfx_rc_t)RC3C);
        /* RC3 - open the Sigfox library */
        if(ret!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }

        /* In FCC we can choose the macro channel to use by a 86 bits bitmask
        In this case we use 9 consecutive macro channels starting from 63 (920.8MHz) */
        sfx_u32 config_words[3]=RC3C_CONFIG;

        /* Set the standard configuration with default channel to 63 */
        if(SIGFOX_API_set_std_config(config_words,0)!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }
        break;
      }
    case 4:
      {
        volatile uint8_t ret;
        /* Turn PA off in RC2 and RC4 */
        ST_RF_API_set_pa(1);

        ret=SIGFOX_API_open(&(sfx_rc_t)RC4);
        /* RC4 - open the Sigfox library */
        if(ret!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }

        /* In FCC we can choose the macro channel to use by a 86 bits bitmask
        In this case we use 9 consecutive macro channels starting from 63 (920.8MHz) */
        sfx_u32 config_words[3]=RC4_SM_CONFIG;

        /* Set the standard configuration with default channel to 63 */
        if(SIGFOX_API_set_std_config(config_words,1)!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }
        break;
      }
    case 5:
      {
        volatile uint8_t ret;
        /* Turn PA off in RC1/3/5/6/7 */
        ST_RF_API_set_pa(0);
        ret=SIGFOX_API_open(&(sfx_rc_t)RC5);
        /* RC5 - open the Sigfox library */
        if(ret!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }

        /* In FCC we can choose the macro channel to use by a 86 bits bitmask
        In this case we use 9 consecutive macro channels starting from 63 (920.8MHz) */
        sfx_u32 config_words[3]=RC5_CONFIG;

        /* Set the standard configuration with default channel to 63 */
        if(SIGFOX_API_set_std_config(config_words,0)!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }
        break;
      }
    case 6:
      {
        /* Turn PA off in RC1/3/5/6/7 */
        ST_RF_API_set_pa(0);
        /* RC6 - open the Sigfox library */
        if(SIGFOX_API_open(&(sfx_rc_t)RC6)!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }
        break;
      }
    case 7:
      {
        /* Turn PA off in RC1/3/5/6/7 */
        ST_RF_API_set_pa(0);
        /* RC7 - open the Sigfox library */
        if(SIGFOX_API_open(&(sfx_rc_t)RC7)!=0)
        {
          /* Stuck in case of error */
          open_err = ST_SFX_ERR_OPEN;
        }
        break;
      }
    default:
      {
        /* Stuck the application for a out of range number */
        open_err = ST_SFX_ERR_RC_UNKNOWN;
        break;
      }
    }
  return open_err;
}
/**
* @brief  System main function.
* @param  *sfxConfig: Pointer to the xFxConfig
* @param  openAfterInit: Open after init flag
* @retval ST_SFX_ERR : Sigfox Error status
*/
ST_SFX_ERR ST_Sigfox_Init(NVM_BoardDataType *sfxConfig, uint8_t openAfterInit)
{
  ST_SFX_ERR ret_err = ST_SFX_ERR_NONE;

  /* Configure XTAL frequency and offset for the RF Library */
  ST_RF_API_set_xtal_freq(S2LPManagementGetXtalFrequency());

  /* Macro that defines and initializes the nvmconfig structure */
  INIT_NVM_CONFIG(nvmConfig);

  /* Sigfox Credentials Management */

  /* Configure the NVM_API */
  SetNVMInitial(&nvmConfig);

  ST_MCU_API_Shutdown(1);
  HAL_Delay(1);

  /* Set EEPROM CS */
  S2868A2_RADIO_SPI_NSS_PIN_HIGH();

  /* Retrieve Sigfox info from EEPROM */
  if(enc_utils_retrieve_data(&sfxConfig->id, sfxConfig->pac, &sfxConfig->rcz) != 0)
    ret_err = ST_SFX_ERR_CREDENTIALS;
  else
    sfxConfig->freqOffset = S2LPManagementGetXtalFrequency();

  /* If the retriever returns an error (code different from ST_SFX_ERR_NONE) the application will halt */
  /* Otherwise, open the Sigfox Library according to the zone stored in the device */
  if(openAfterInit && ret_err == ST_SFX_ERR_NONE)
    ret_err = St_Sigfox_Open_RCZ(sfxConfig->rcz);

  return ret_err;
}

/**
* @brief  Blink the LED indefinitely stucking the application.
* @param  None
* @retval None
*/
/*static void Fatal_Error(void)
{
  BSP_LED_Init(LED2);
  while(1)
  {
    HAL_Delay(100);
    BSP_LED_Toggle(LED2);
  }
}*/

/**
* @brief  Gets the XTAL frequency.
* @param  None
* @retval The XTAL frequency
*/
uint32_t S2LPManagementGetXtalFrequency(void)
{
  return s_XtalFrequency;
}

/**
* @brief  This function is to query if EEPROM is present or not.
* @param  None
* @retval 1 (yes) or 0 (no).
*/
uint8_t RadioGetHasEeprom(void)
{
#if EEPROM_PRESENT == EEPROM_YES
  return s_eeprom;
#else
  return 0;
#endif
}

/**
* @brief  This function is to set if EEPROM is present or not.
* @param  1 (yes) or 0 (no).
* @retval None
*/
void RadioSetHasEeprom(uint8_t eeprom)
{
#if EEPROM_PRESENT == EEPROM_YES
  s_eeprom = eeprom;
#else
  s_eeprom = 0;
#endif
}

/**
 * @brief  Puts at logic 1 the TCXO pin.
 * @param  None.
 * @retval None.
 */
void RadioTcxoOn(void)
{
  S2868A2_RADIO_EnableTCXO();
}

/**
* @brief  print output result
* @param  status.
* @param  rssi.
* @retval None
*/
void ST_MANUF_report_CB(uint8_t status, int32_t rssi)
{
  if(status)
    printf("{TEST PASSED! RSSI=%d}\n\r",(int)rssi);
  else
    printf("{TEST FAILED!}\n\r");

  RadioComTriggerTx();
  while(txUsed);
}

/**
* @brief  Returns TRUE if a valid command was receveid. FALSE otherwise.
* @param  interactive :interactive mode variable
* @retval TRUE or FALSE
*/
uint8_t processCmdInput (uint8_t interactive)
{
  static uint8_t buff[COMMAND_BUFFER_LENGTH];
  static uint16_t len = 0;
  static uint16_t currIndex = 0;
  if (interactive) {
    /* continue calling emberSerialReadPartialLine() until it returns success, indicating a full line has been read */
    if (!serialReadPartialLine((char *)buff, COMMAND_BUFFER_LENGTH, &currIndex)) {
      return 0;
    }

    len=0;
    /* search foward looking for first CR, LF, or NULL to determine length of the string  */
    while((buff[len]!='\n') && (buff[len] !='\r') && (buff[len]!='\0')) {
      len++;
    }
    buff[len ++] = '\r'; /*set the final char to be CR */
    buff[len ++] = '\n'; /*set the final char to be NL */

    currIndex = 0;
    return processCommandString(buff, len);
  } else {
    return processCommandInput();
  }
}

/**
* @brief  Prints interactive action.
* @param  None
* @retval None
*/
void interactiveAction(void)
{
  interactive = unsignedCommandArgument(0);
  responsePrintf("{&N utility call... &t2x}\r\n",
                 "interactive",
                 "mode",
                 interactive);
}

/**
* @brief  Prints help action.
* @param  None
* @retval None
*/
void helpAction(void)
{
  CommandEntry *cmd;

  for (cmd = CommandTable; cmd->name != NULL; cmd++) {
    if(!CMD_HIDDEN(cmd))
    {
      printf ("%s %s %s\r\n\r\n",
              cmd->name,
              cmd->argumentTypes,
              cmd->description);
    }
  }
}

/**
* @brief  Prints Firmware version.
* @param  None
* @retval None
*/
void fwVersionAction(void)
{
  responsePrintf("{&N API call...&ts}\r\n",
		 "fw_version",
		 "value",FW_VERSION);
}
/**
* @brief  Perfomrs system reboot and prints the action.
* @param  None
* @retval None
*/
void rebootAction(void)
{
  responsePrintf("{&N API call...&ts}\r\n",
		 "reboot");

  HAL_NVIC_SystemReset();
}

/**
* @brief  Triggeres COM port.
* @param  None.
* @retval None.
*/
void RadioComTriggerTx(void)
{
  prepareDmaTx();
}

/**
* @brief  Updates Tx Queue.
* @param  None.
* @retval None.
*/
void updatetxQ(void)
{
  uint16_t  _temp;
  uint16_t dmaResidual =  __HAL_DMA_GET_COUNTER(hcom_uart[COM1].hdmatx);

  UART_ENTER_CRITICAL();

  txTail=(txTail+(txLastRequest-dmaResidual))%TRANSMIT_QUEUE_SIZE;

  _temp = txLastRequest-dmaResidual;
  txUsed-=_temp;

  txLastRequest=dmaResidual;
  UART_EXIT_CRITICAL();
}

/**
* @brief  Put char in Tx Queue.
* @param  Buffer : Pointer to the buffer
* @param  Size :  Size
* @retval None.
*/
void enqueueTxChars(const unsigned char * buffer, uint16_t size)
{

  while ( size > 0 ) {

    while(txUsed>TRANSMIT_QUEUE_SIZE-size)
    {
      updatetxQ();
    }

    UART_ENTER_CRITICAL();
    txQ[txHead] = *buffer++;
    txUsed++;
    txHead = (txHead+1) % TRANSMIT_QUEUE_SIZE;
    size--;
    UART_EXIT_CRITICAL();
  }
}

/**
* @brief  Sets the baud rate.
* @param  baudrate : Baud rate to be set
* @retval None.
*/
void RadioComBaudrate(uint32_t baudrate)
{
  hcom_uart[COM1].Init.BaudRate=baudrate;
  __HAL_UART_DISABLE(&hcom_uart[COM1]);
  HAL_UART_Init(&hcom_uart[COM1]);
  __HAL_UART_ENABLE(&hcom_uart[COM1]);
}

/**
* @brief  Prepare DMA for Tx.
* @param  None.
* @retval None.
*/
static void prepareDmaTx(void)
{
  if(!dmaTransmitting && txUsed!=0)
  {
    UART_ENTER_CRITICAL();
    dmaTransmitting=1;

    if(txTail+txUsed<TRANSMIT_QUEUE_SIZE)
    {
      txLastRequest=txUsed;
    }
    else
    {
      txLastRequest=(TRANSMIT_QUEUE_SIZE-txTail);
    }

    if(HAL_UART_Transmit_DMA(&hcom_uart[COM1],&txQ[txTail],txLastRequest)==HAL_OK)
    {

    }

    UART_EXIT_CRITICAL();
  }

}

#if defined(__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
/**
* @brief  Read.
* @param  File.
* @param  ptr.
* @param  len.
* @retval number of chars.
*/
int _read (int file, char *ptr, int len)
{
  int nChars = 0;

  /* This template only reads from "standard in", for all other file
  * handles it returns failure. */
  if (file != _LLIO_STDIN)
  {
    return (0);
  }

  if(hcom_uart[COM1].hdmarx)
  {
    rxHead=(RECEIVE_QUEUE_SIZE -  __HAL_DMA_GET_COUNTER(hcom_uart[COM1].hdmarx))%RECEIVE_QUEUE_SIZE;
    if(rxHead>=rxTail)
      rxUsed=rxHead-rxTail;
    else
      rxUsed=RECEIVE_QUEUE_SIZE-rxTail+rxHead;

  }

  for(nChars = 0; (rxUsed>0) && (nChars < len); nChars++) {
    *ptr++ = rxQ[rxTail];
    rxTail = (rxTail+1) % RECEIVE_QUEUE_SIZE;
    rxUsed--;
  }

  return nChars;
}

/**
* @brief  Write.
* @param  File.
* @param  ptr.
* @param  len.
* @retval lenght.
*/
int _write(int file, char *ptr, int len)
{
  if (ptr == 0)
  {
    uint8_t c=0;
    enqueueTxChars((unsigned char *)&c,1);
    return 0;
  }

  updatetxQ();
  enqueueTxChars((unsigned char *)ptr,len);

  return len;
}

/**
* @brief  Read.
* @param  File.
* @param  ptr.
* @param  len.
* @retval number of chars.
*/

int sfx_serial_read ( char *ptr, int len)
{
  int nChars = 0;

  if(hcom_uart[COM1].hdmarx)
  {
    rxHead=(RECEIVE_QUEUE_SIZE -  __HAL_DMA_GET_COUNTER(hcom_uart[COM1].hdmarx))%RECEIVE_QUEUE_SIZE;
    if(rxHead>=rxTail)
      rxUsed=rxHead-rxTail;
    else
      rxUsed=RECEIVE_QUEUE_SIZE-rxTail+rxHead;

  }

  for(nChars = 0; (rxUsed>0) && (nChars < len); nChars++) {
    *ptr++ = rxQ[rxTail];
    rxTail = (rxTail+1) % RECEIVE_QUEUE_SIZE;
    rxUsed--;
  }

  return nChars;
}

/**
* @brief  Write.
* @param  File.
* @param  ptr.
* @param  len.
* @retval lenght.
*/
int sfx_serial_write(char *ptr, int len)
{
  if (ptr == 0)
  {
    uint8_t c=0;
    enqueueTxChars((unsigned char *)&c,1);
    return 0;
  }

  updatetxQ();
  enqueueTxChars((unsigned char *)ptr,len);

  return len;
}

/**
* @brief  __io_getcharNonBlocking.
* @param  data: Pointer to the data.
* @retval 0 or 1.
*/
uint8_t __io_getcharNonBlocking(uint8_t *data)
{
	uint8_t charData;
	charData = sfx_serial_read ((char *)data,1);
	return(charData);
}

/**
* @brief  __io_putchar.
* @param  ch: the data.
* @retval char.
*/

/* With GCC, small printf (option LD Linker->Libraries->Small printf*/
int __io_putchar(int ch)
{
  sfx_serial_write((char *)&ch, 1);
  return ch;
}

/**
* @brief  __io_getchar.
* @param  ch: the data.
* @retval char.
*/

int __io_getchar(void)
{
  char c;
  sfx_serial_read(&c, 1);
  return (int)(c);
}

/**
* @brief  __io_flush.
* @param  None.
* @retval None.
*/
void __io_flush( void )
{
  sfx_serial_write(NULL, 0);
}

#elif defined(__ICCARM__)

/* IAR Standard library hook for serial output  */

/**
* @brief  Read.
* @param  File.
* @param  ptr.
* @param  len.
* @retval number of chars.
*/

int sfx_serial_read ( char *ptr, int len)
{
  int nChars = 0;

  if(hcom_uart[COM1].hdmarx)
  {
    rxHead=(RECEIVE_QUEUE_SIZE -  __HAL_DMA_GET_COUNTER(hcom_uart[COM1].hdmarx))%RECEIVE_QUEUE_SIZE;
    if(rxHead>=rxTail)
      rxUsed=rxHead-rxTail;
    else
      rxUsed=RECEIVE_QUEUE_SIZE-rxTail+rxHead;

  }

  for(nChars = 0; (rxUsed>0) && (nChars < len); nChars++) {
    *ptr++ = rxQ[rxTail];
    rxTail = (rxTail+1) % RECEIVE_QUEUE_SIZE;
    rxUsed--;
  }

  return nChars;
}

/**
* @brief  Write.
* @param  File.
* @param  ptr.
* @param  len.
* @retval lenght.
*/
int sfx_serial_write(char *ptr, int len)
{
  if (ptr == 0)
  {
    uint8_t c=0;
    enqueueTxChars((unsigned char *)&c,1);
    return 0;
  }

  updatetxQ();
  enqueueTxChars((unsigned char *)ptr,len);

  return len;
}

/**
* @brief  __io_putchar.
* @param  c: char
* @retval None.
*/
void __io_putchar( char c )
{
  sfx_serial_write((char *)&c, 1);
}

/**
* @brief  __read.
* @param  handle: handle
* @param  buffer: pointer to the buffer
* @param  size: size
* @retval size.
*/
size_t __read(int handle, unsigned char * buffer, size_t size)
{
  int nChars = 0;

  /* This template only reads from "standard in", for all other file
  * handles it returns failure. */
  if (handle != _LLIO_STDIN)
  {
    return _LLIO_ERROR;
  }

  if(hcom_uart[COM1].hdmarx)
  {
     rxHead=(RECEIVE_QUEUE_SIZE - __HAL_DMA_GET_COUNTER(hcom_uart[COM1].hdmarx))%RECEIVE_QUEUE_SIZE;
    if(rxHead>=rxTail)
      rxUsed=rxHead-rxTail;
    else
      rxUsed=RECEIVE_QUEUE_SIZE-rxTail+rxHead;

  }

  for(nChars = 0; (rxUsed>0) && (nChars < size); nChars++) {
    *buffer++ = rxQ[rxTail];
    rxTail = (rxTail+1) % RECEIVE_QUEUE_SIZE;
    rxUsed--;
  }

  return nChars;
}

/**
* @brief  __io_getchar.
* @param  None
* @retval None.
*/
int __io_getchar()
{
  char c;
  sfx_serial_read(&c, 1);
  return (int)(c);
}

/**
* @brief  __io_flush.
* @param  None
* @retval None.
*/
void __io_flush( void )
{
  sfx_serial_write(NULL, 0);
}

/**
* @brief  __io_getcharNonBlocking.
* @param  Data: Pointer to the data
* @retval 0 or 1.
*/
uint8_t __io_getcharNonBlocking(uint8_t *data)
{
	uint8_t charData;
	charData = sfx_serial_read ((char *)data,1);
	return(charData);
}

/**
* @brief  __write.
* @param  handle: handle.
* @param  buffer: pointe rto the buffer .
* @param  size: Size to be printed.
* @retval size.
*/
__weak size_t __write(int handle, const unsigned char * buffer, size_t size)
{

  /* This template only writes to "standard out" and "standard err",
  * for all other file handles it returns failure. */
  if (handle != _LLIO_STDOUT && handle != _LLIO_STDERR) {
    return _LLIO_ERROR;
  }
  if (buffer == 0) {
    uint8_t c=0;
    enqueueTxChars(&c,1);
    return 0;
  }

  updatetxQ();
  enqueueTxChars(buffer,size);

  return size;
}

#else

/**
* @brief  __read.
* @param  handle: handle
* @param  buffer: pointer to the buffer
* @param  size: size
* @retval size.
*/
size_t __read(int handle, unsigned char * buffer, size_t size)
{
  int nChars = 0;

  /* This template only reads from "standard in", for all other file
  * handles it returns failure. */
  if (handle != _LLIO_STDIN)
  {
    return _LLIO_ERROR;
  }

  if(hcom_uart[COM1].hdmarx)
  {
     rxHead=(RECEIVE_QUEUE_SIZE - __HAL_DMA_GET_COUNTER(hcom_uart[COM1].hdmarx))%RECEIVE_QUEUE_SIZE;
    if(rxHead>=rxTail)
      rxUsed=rxHead-rxTail;
    else
      rxUsed=RECEIVE_QUEUE_SIZE-rxTail+rxHead;

  }

  for(nChars = 0; (rxUsed>0) && (nChars < size); nChars++) {
    *buffer++ = rxQ[rxTail];
    rxTail = (rxTail+1) % RECEIVE_QUEUE_SIZE;
    rxUsed--;
  }

  return nChars;
}

/**
* @brief  __io_getchar.
* @param  None
* @retval None.
*/
int __io_getchar()
{
  unsigned char c;
  __read(_LLIO_STDIN, &c, 1);
  return (int)(c);
}

/**
* @brief  __io_flush.
* @param  None
* @retval None.
*/
void __io_flush( void )
{
  __write(_LLIO_STDOUT, NULL, 0);
}

/**
* @brief  __io_getcharNonBlocking.
* @param  Data: Pointer to the data
* @retval 0 or 1.
*/
uint8_t __io_getcharNonBlocking(uint8_t *data)
{
  if (__read(_LLIO_STDIN,data,1))
    return 1;
  else
    return 0;
}

#endif

#ifdef __CC_ARM
/**
* @brief  fflush.
* @param  fp: Pointer to the file
* @retval 0 or 1.
*/
int fflush(FILE *fp)
{
  int status;
  status = sfx_serial_write(NULL, 0);
  return status;
}

#endif

/**
* @brief  USART Tx complete callback
* @param  huart: poiter to the UART handle typedef
* @retval None.
*/
void SFX_COM_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uint16_t  _temp;
  txTail=(txTail+txLastRequest)%TRANSMIT_QUEUE_SIZE;

  _temp = txLastRequest;
  txUsed-=_temp;

  txLastRequest=0;
  dmaTransmitting=0;

  prepareDmaTx();
}

/**
* @brief  Enqueue Rx Buffer.
* @param  buffer : Pointer to the buffer
* @param  size   :Size of the buffer
* @retval None.
*/
void enqueueRxChars(unsigned char * buffer, uint16_t size)
{
  while (( size > 0 ) && (rxUsed < (RECEIVE_QUEUE_SIZE-1))) {
    rxQ[rxHead] = *buffer++;
    rxHead = (rxHead+1) % RECEIVE_QUEUE_SIZE;
    rxUsed++;
    size--;
  }
}

/**
* @brief  Append to the buffer.
* @param  None
* @retval 0 or 1.
*/
uint8_t append_to_buf(void)
{
  uint8_t c;

  HAL_UART_Receive(&hcom_uart[COM1], &c, 1, 100);
  rxQ[rxHead] = c;
  rxHead=(rxHead+1)%RECEIVE_QUEUE_SIZE;
  rxUsed++;

  if(c=='\n' || c=='\r')
    return 1;

  return 0;

}

#ifdef __ICCARM__
/**
 * @brief  TCXO initialization function.
 * This function automatically sets the TCXO according to
 *   the information stored in the device EEPROM.
 * This function can be redefined for special needs.
 * @param  None
 * @retval None
 */
__weak void TCXO_Init()
{
	/* Not implemented: at the moment there isn't any eval kit with TCXO */
}

/**
 * @brief  TCXO Operation function.
 * This function configures the TCXO according to the desired status.
 * This function can be redefined for special needs.
 * @param  operation Specifies the operation to perform.
 *         This parameter can be one of following parameters:
 *         @arg TCXO_ON: Turns on TCXO
 *         @arg TCXO_OFF: Turns off TCXO
 * @retval None
 */
__weak void TCXO_Operation(TCXO_OperationType operation)
{
	/* Not implemented: at the moment there isn't any eval kit with TCXO */
}

#endif

#ifdef __CC_ARM
/**
 * @brief  TCXO initialization function.
 * This function automatically sets the TCXO according to
 *   the information stored in the device EEPROM.
 * This function can be redefined for special needs.
 * @param  None
 * @retval None
 */
void __attribute__((weak)) TCXO_Init()
{
	/* Not implemented: at the moment there isn't any eval kit with TCXO */
}

/**
 * @brief  TCXO Operation function.
 * This function configures the TCXO according to the desired status.
 * This function can be redefined for special needs.
 * @param  operation Specifies the operation to perform.
 *         This parameter can be one of following parameters:
 *         @arg TCXO_ON: Turns on TCXO
 *         @arg TCXO_OFF: Turns off TCXO
 * @retval None
 */
void __attribute__((weak)) TCXO_Operation(TCXO_OperationType operation)
{
	/* Not implemented: at the moment there isn't any eval kit with TCXO */
}

#endif

#ifdef __GNUC__
/**
 * @brief  TCXO initialization function.
 * This function automatically sets the TCXO according to
 *   the information stored in the device EEPROM.
 * This function can be redefined for special needs.
 * @param  None
 * @retval None
 */
void __attribute__((weak)) TCXO_Init()
{
	/* Not implemented: at the moment there isn't any eval kit with TCXO */
}

/**
 * @brief  TCXO Operation function.
 * This function configures the TCXO according to the desired status.
 * This function can be redefined for special needs.
 * @param  operation Specifies the operation to perform.
 *         This parameter can be one of following parameters:
 *         @arg TCXO_ON: Turns on TCXO
 *         @arg TCXO_OFF: Turns off TCXO
 * @retval None
 */
void __attribute__((weak)) TCXO_Operation(TCXO_OperationType operation)
{
	/* Not implemented: at the moment there isn't any eval kit with TCXO */
}

#endif

/**
* @brief  VCOM Timer Time period elasped Callback.
* @param  None
* @retval None
*/
void COM_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == BSP_COM_TIM)
  {
    COM_TIM_Callback();
  }
}
/**
* @brief  Sigfox Time period elasped Callback.
* @param  None
* @retval None
*/
void SFX_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == BSP_SFX_TIM)
  {
    SFX_TIM_Callback();
    RadioTimersState(htim, DISABLE);
  }
}

/**
* @brief  Sigfox TImer Callback.
* @param  None
* @retval None
*/
void COM_TIM_Callback(void)
{
    timer_cnt++;
    RadioComTriggerTx();
}

/**
* @brief  Sigfox TImer Callback.
* @param  None
* @retval None
*/
void SFX_TIM_Callback(void)
{
  ST_RF_API_Timer_Channel_Clear_CB();
}

/**
* @brief  Sigfox Clear Wake up flag.
* @param  None
* @retval None
*/
void SFX_ClearWKUP_Flag(void)
{
  __HAL_RTC_CLEAR_FLAG(RTC_EXTI_LINE_WAKEUPTIMER_EVENT);
}

/**
* @brief  Sigfox Set Wakeup timer.
* @param  None
* @retval None
*/
void SFX_SetWKUP_Timer(uint32_t CtrDiv)
{
  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,CtrDiv,RTC_WAKEUPCLOCK_RTCCLK_DIV16);
}

#ifdef __cplusplus
}
#endif

