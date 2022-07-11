/**
  ******************************************************************************
  * File Name          :  stmicroelectronics_x-cube-sfxs2lp1_4_0_0.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_X_CUBE_SFXS2LP1_H
#define __APP_X_CUBE_SFXS2LP1_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "s2lp.h"
#include "stm32l0xx_hal.h"
#include "stm32l0xx.h"
#include "stm32l0xx_it.h"
#include "stm32l0xx_nucleo.h"
#include "s2868a1.h"
#include "RTE_Components.h"
#include "sfx_config.h"
#include "nvm_api.h"
#include "s2lp_sdkapi_mapping.h"
/* Exported Defines --------------------------------------------------------*/

/* Exported Variables -------------------------------------------------------*/

/* Exported Functions --------------------------------------------------------*/
void MX_X_CUBE_SFXS2LP1_Init(void);
void  MX_X_CUBE_SFXS2LP1_Process(void);

#ifdef __cplusplus
}
#endif
#endif /* __APP_X_CUBE_SFXS2LP1_H*/

