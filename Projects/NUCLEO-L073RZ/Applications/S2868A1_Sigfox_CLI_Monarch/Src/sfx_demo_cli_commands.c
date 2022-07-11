/**
  * @file    sfx_demo_cli_commands.c
  * @author  STM32ODE Team, NOIDA
  * @brief   sigfox CLI commands demo
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

/* Includes ------------------------------------------------------------------*/
#include "assert.h"
#include "stddef.h"
#include <stdio.h>
#include <string.h>
#include "stdarg.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "utils.h"
#include "command-interpreter2.h"
#include "sfx_demo_cli_commands.h"
#include "nvm_api.h"
#include "sigfox_types.h"
#include "sigfox_api.h"
#include "sigfox_monarch_api.h"
#include "monarch_api.h"
#include "retriever_api.h"
#include "mcu_api.h"
#include "st_mcu_api.h"
#include "st_rf_api.h"
#include "addon_sigfox_rf_protocol_api.h"
#include "nvm_api.h"
#include "rf_api.h"
#include "sfx_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint32_t id;
static uint8_t pac[8];
static uint8_t rcz=1;
static uint32_t config_words[3];
static uint32_t timer_enable;
static uint8_t _isPublicKey = 0;
static uint8_t _testModeActive = 0;
/* Exported Function prototypes  ---------------------------------------------*/
extern uint8_t responsePrintf(const char * formatString, ...);

/**
* @brief  Set id.
* @param  id_new :New id to be set
* @retval None
*/
void set_id(uint32_t id_new)
{
  id=id_new;
}

/**
* @brief  Set pac.
* @param  pac_new :pointer to new pac
* @retval None
*/
void set_pac(uint8_t *pac_new)
{
  for(uint8_t i=0;i<8;i++)
  {
    pac[i]=pac_new[i];
  }
}

/**
* @brief  Set rcz.
* @param  rcz_new : new rcz to be set
* @retval None
*/
void set_rcz(uint8_t rcz_new)
{
  rcz=rcz_new;
}

/**
* @brief  Set _testModeActive for non-registered boards.
* @param  enableTestMode : Enable test mode
* @retval None
*/
void set_testMode(uint8_t enableTestMode)
{
  _testModeActive = enableTestMode;

  if(_testModeActive)
  {
    enc_utils_set_test_id(1);
    enc_utils_set_test_key(1);
  }
  else
  {
    enc_utils_set_test_id(0);
    enc_utils_set_test_key(0);
  }
}
/**
* @brief  Get Test mode.
* @param  None
* @retval Get Test mode
*/
uint8_t get_testMode(void)
{
  return _testModeActive;
}

/**
* @brief  Set id action.
* @param  None
* @retval None
*/
void ST_get_id_Action(void)
{
  responsePrintf("{&N API call...&t4x}\r\n",
		 "get_id", "id", id);
}

/**
* @brief  Set pac action.
* @param  None
* @retval None
*/
void ST_get_pac_Action(void)
{
  uint16_t i,index = 0;
  char array[20];
  responsePrintf("{&N API call...\r\n", "get_pac");
  responsePrintf("{%t: ","pac");

  for (i=0; i<8; i++)
  {
    index += sprintf(&array[index], "%X",(unsigned int) pac[i]);
  }
  responsePrintf(array);

  responsePrintf("}\r\n}\r\n");
}

/**
* @brief  Get RCZ action.
* @param  None
* @retval None
*/
void ST_get_rcz_Action(void)
{

  responsePrintf("{&N API call...&tx}\r\n",
		 "get_rcz", "rcz", rcz);
}

/**
* @brief  Set RCZ action.
* @param  None
* @retval None
*/
void ST_set_rcz_Action(void)
{
  uint8_t rcz_new=unsignedCommandArgument(0);

  if((rcz_new==2 || rcz_new==4) && (rcz==2 || rcz==4))
  {
    rcz=rcz_new;
  }

  responsePrintf("{&N API call...&tx}\r\n",
		 "set_rcz", "rcz", rcz);
}

/**
* @brief  Node close action.
* @param  None
* @retval None
*/
void node_close_Action(void)
{
  sfx_error_t err;

  err=SIGFOX_API_close();
  responsePrintf("{&N API call...&tx}\r\n",
		 "node_close", "sfx_error", err );
}

/**
* @brief  Reset node action.
* @param  None
* @retval None
*/
void node_reset_Action(void)
{
  sfx_error_t err=0;

  responsePrintf("{&N API call...&tx}\r\n",
		 "node_reset", "sfx_error", err);
}

/**
* @brief  Open node action.
* @param  None
* @retval None
*/
void node_open_Action(void)
{
  uint32_t err=0x80000001;

  ST_RF_API_set_pa(0);

  if(rcz==1)
  {
    err = (uint32_t)SIGFOX_API_open(&(sfx_rc_t)RC1);
  }
  else if(rcz==2)
  {
    ST_RF_API_set_pa(1);
    err = (uint32_t)SIGFOX_API_open(&(sfx_rc_t)RC2);
  }
  else if(rcz==3)
  {
    err = (uint32_t)SIGFOX_API_open(&(sfx_rc_t)RC3C);
  }
  else if(rcz==4)
  {
    ST_RF_API_set_pa(1);
    err = (uint32_t)SIGFOX_API_open(&(sfx_rc_t)RC4);
  }
  else if(rcz==5)
  {
    err = (uint32_t)SIGFOX_API_open(&(sfx_rc_t)RC5);
  }

  responsePrintf("{&N API call...&t4x}\r\n",
		 "node_open", "sfx_error", err);
}

/**
* @brief  Open node with zone action.
* @param  None
* @retval None
*/
void node_open_with_zone_Action(void)
{
  uint8_t user_rcz = unsignedCommandArgument(0);
  uint8_t rcz_backup = rcz;

  rcz = user_rcz;
  node_open_Action();
  rcz = rcz_backup;
}

/**
* @brief  Node get info action.
* @param  None
* @retval None
*/
void node_get_info_Action(void)
{
  sfx_error_t err;
  uint8_t info;
  err=SIGFOX_API_get_info(&info);
  responsePrintf("{&N API call...&tx&tx}\r\n",
                 "node_get_info",
                 "info",
                 info,
                 "sfx_error",
                 err );

  if(err != 0)
  {
    /*Case of error*/
  }
}

/**
* @brief  Node get version action.
* @param  None
* @retval None
*/
void node_get_version_Action(void)
{
	static sfx_error_t err;
  responsePrintf("{&N API call...\r\n","node_get_version");
  err = _printVersion(VERSION_SIGFOX);
  if(err != 0)
  {
    /*Case of error*/
  }
}

/**
* @brief  Node send frame action.
* @param  None
* @retval None
*/
void node_send_frame_Action(void)
{
  sfx_error_t err;
  uint8_t customer_data[256];
  uint8_t customer_data_len = copyStringArgument(0, customer_data, 255, 0);
  uint8_t tx_repeat=unsignedCommandArgument(1);
  uint8_t initiate_downlink_flag=(uint8_t)unsignedCommandArgument(2);
  uint8_t customer_resp[8]={0};

  err=SIGFOX_API_send_frame(customer_data,customer_data_len,\
    customer_resp,tx_repeat,initiate_downlink_flag);
  responsePrintf("{&N API call...&tx\r\n","node_send_frame", "sfx_error", err);

  if(initiate_downlink_flag && !err)
  {
    responsePrintf("{%t: ","customer_resp");
    for(uint16_t i = 0; i < 7; i++) {
      responsePrintf("0x%x,", customer_resp[i]);
    }
    responsePrintf("0x%x}\n\r", customer_resp[7]);
  }
  responsePrintf("}\n\r");
}

/**
* @brief  Node test mode action.
* @param  None
* @retval None
*/
void node_test_mode_Action(void)
{
  sfx_error_t err=0;
  sfx_rc_enum_t rc=(sfx_rc_enum_t)unsignedCommandArgument(0);
  sfx_test_mode_t test_mode=(sfx_test_mode_t)unsignedCommandArgument(1);
  sfx_u8 nvBuff[SFX_NVMEM_BLOCK_SIZE];
  sfx_u16 rolloverCounter = 0;

  /* PA mangement */
  if (rc==1 || rc==4)
    ST_RF_API_set_pa(1);
  else
  	ST_RF_API_set_pa(0);

  if(MCU_API_get_nv_mem(nvBuff) == SFX_ERR_NONE)
    rolloverCounter = nvBuff[SFX_NVMEM_RL];

  enc_utils_set_test_id(1);

  if(!_isPublicKey)
    enc_utils_set_test_key(1);

  err=ADDON_SIGFOX_RF_PROTOCOL_API_test_mode(rc,test_mode);

  // Revert to pre-test key only for registerd device
  // Non-registered boards will keep the test id and key
  if(!_testModeActive)
  {
    if(!_isPublicKey)
      enc_utils_set_test_key(0);

    enc_utils_set_test_id(0);
  }

  responsePrintf("{&N API call...&td &t2x}\r\n",
		 "node_test_mode", "rollover counter", rolloverCounter, "sfx_error", err );
}

void node_monarch_test_mode_Action(void)
{
  sfx_error_t err=0;

  sfx_rc_enum_t rc=(sfx_rc_enum_t)unsignedCommandArgument(0);
  sfx_test_mode_t test_mode=(sfx_test_mode_t)unsignedCommandArgument(1);
  sfx_u8 rc_capabilities=(sfx_u8)unsignedCommandArgument(2); 	//31 up to RC5,  63 up to RC6

  if (unsignedCommandArgument(0)==1 || unsignedCommandArgument(0)==4)
    ST_RF_API_set_pa(1);
  else
    ST_RF_API_set_pa(0);

#ifdef MONARCH_CLI_TESTS
      if (unsignedCommandArgument(1)>6)
        ST_RF_API_reduce_output_power(80);
      else if (unsignedCommandArgument(1)==4)
        ST_RF_API_reduce_output_power(80);
#endif
  enc_utils_set_test_id(1);

  if(!_isPublicKey)
    enc_utils_set_test_key(1);

  err=ADDON_SIGFOX_RF_PROTOCOL_API_monarch_test_mode(rc,test_mode,rc_capabilities);

  if(!_isPublicKey)
    enc_utils_set_test_key(0);

  enc_utils_set_test_id(0);

  responsePrintf("{&N API call...&tx}\r\n",
                 "ADDON_SIGFOX_RF_PROTOCOL_API_monarch_test_mode", "sfx_error", err );
}

/**
* @brief  Get Library version action.
* @param  None
* @retval None
*/
void get_lib_version_Action(void)
{
  sfx_error_t err;
  sfx_version_type_t libType = (sfx_version_type_t)unsignedCommandArgument(0);

  responsePrintf("{&N API call...\r\n","get_lib_version");
  err = _printVersion(libType);

  if(err != 0)
  {
  /*Case of error*/
  }

}

/**
* @brief  Set Low power action.
* @param  None
* @retval None
*/
void ST_set_low_power_Action(void)
{

  ST_MCU_API_LowPower((uint8_t)unsignedCommandArgument(0));

  responsePrintf("{&N API call...}\r\n",
		 "set_low_power");
}

#ifndef MON_REF_DES
/**
* @brief  Performs EEPROM Page write  action.
* @param  None
* @retval None
*/
void EepromWritePageAction(void)
{
  if(RadioGetHasEeprom()) {

    uint8_t buffer[32];
    uint16_t cPageNum = unsignedCommandArgument(0);
    uint8_t cAddressOffset = unsignedCommandArgument(1);
    uint16_t  cAddressLocation = cPageNum*32 + cAddressOffset;

    uint8_t cNbBytes = copyStringArgument(2, buffer, 255, 0);

    if((cAddressOffset+cNbBytes)>32)
    {
      cNbBytes = 32-cAddressOffset;
    }

    S2868A1_RADIO_EnterShutdown();
    HAL_Delay(10);
    EepromWrite(cAddressLocation, cNbBytes, buffer);
    HAL_Delay(10);
    /* get from EEPROM ID and the AES key decrypted from there */
    enc_utils_retrieve_data(&id,pac,&rcz);
    set_id(id);
    set_pac(pac);
    set_rcz(rcz);

    responsePrintf("{&N API call...}\r\n", "_e2prom_write_page");
    S2868A1_RADIO_ExitShutdown();
  }
}
/**
* @brief  Performs EEPROM page read action.
* @param  None
* @retval None
*/
void EepromReadPageAction(void)
{
  if(RadioGetHasEeprom())
  {
    uint8_t buffer[32];
    uint16_t cPageNum = unsignedCommandArgument(0);
    uint8_t cAddressOffset = unsignedCommandArgument(1);
    uint8_t cNbBytes = unsignedCommandArgument(2);
    uint16_t  cAddressLocation = cPageNum*32 + cAddressOffset;

    if((cAddressOffset+cNbBytes)>32)
    {
      cNbBytes = 32-cAddressOffset;
    }

    /* Put the radio off and read the EEPROM*/
    S2868A1_RADIO_EnterShutdown();
    HAL_Delay(10);
    EepromRead(cAddressLocation, cNbBytes, buffer);
    S2868A1_RADIO_ExitShutdown();

    responsePrintf("{&N API callback...\r\n", "_e2prom_read_page");
    responsePrintf("{%t: ","Data");

    for(uint8_t i = 0; i < cNbBytes-1; i++)
      responsePrintf("0x%x,",buffer[i]);

    responsePrintf("0x%x",buffer[cNbBytes-1]);
    responsePrintf("}}\r\n");
  }
}
#endif

/**
* @brief  Set standard node config action.
* @param  None
* @retval None
*/
void node_set_std_config_Action(void)
{
  sfx_error_t err;

  for(uint32_t i=0;i<3;i++)
    config_words[i]=unsignedCommandArgument(i);

  timer_enable=(uint16_t)unsignedCommandArgument(3);

  err=SIGFOX_API_set_std_config((sfx_u32*)config_words,timer_enable);

  responsePrintf("{&N API call...&tx}\r\n",
		 "node_set_std_config", "sfx_error", err );
}

/**
* @brief  Get standard node config action.
* @param  None
* @retval None
*/
void node_get_std_config_Action(void)
{

  responsePrintf("{&N API call...\r\n","node_get_std_config");

  responsePrintf("{%t: ","config_words");

  for(uint8_t i = 0; i < 2; i++)
  {
    responsePrintf("0x%4x,",config_words[i]);
  }
  responsePrintf("0x%4x",config_words[2]);

  responsePrintf("}\r\n");
  responsePrintf("&t2x","timer_enable", timer_enable);
  responsePrintf("&t4x}\r\n","sfx_error", 0);
}
/**
* @brief  Set public key action.
* @param  None
* @retval None
*/
void node_set_public_key_Action(void)
{
  uint8_t en=unsignedCommandArgument(0);

  if(_testModeActive)
    enc_utils_set_test_key(!en);

  uint8_t err = enc_utils_set_public_key(en);

  if(!err)
    _isPublicKey = en;
  else
    _isPublicKey = 0;

  responsePrintf("{&N API call...&t2x}\r\n", "switch_public_key", "sfx_error", err);
}

/**
* @brief  Set node test credentals action.
* @param  None
* @retval None
*/
void node_set_test_credentials_Action(void)
{
  uint8_t val = unsignedCommandArgument(0);
  sfx_error_t err=0;

  set_testMode(val);

  responsePrintf("{&N API call...&t2x}\r\n", "switch_test_credentials", "sfx_error", err);
}

/*****************************************************************************/
/*		               XTAL FREQUENCY OFFSET 																 */
/*****************************************************************************/
/**
* @brief  Set cystal Frequency offset action.
* @param  None
* @retval None
*/
void set_xtal_frequency_offset_Action(void)
{
  sfx_error_t err;
  sfx_s32 xtalCompValue=(int32_t)signedCommandArgument(0); /* The Xtal drift */
  sfx_u32 xtal_freq;

  /* Retrieve xtal frequancy from rf_api */
  err= ST_RF_API_get_xtal_freq(&xtal_freq);
  /*xtalCompValue is a value measured during manufacturing as follows:
  xtalCompValue=fnominal-fmeasured. To compensate such value it should
  be reported to xtal freq and then subtracted*/
  err = ST_RF_API_set_xtal_freq(xtal_freq + xtalCompValue);  /* Override RF_API Xtal value */

    if(!err)
  	err = NVM_UpdateOffset(NVM_FREQ_OFFSET, xtalCompValue);

  responsePrintf("{&N API call...&tx}\r\n",
		 "set_xtal_frequency_offset","sfx_error", err);
}
/**
* @brief  Get cystal Frequency offset action.
* @param  None
* @retval None
*/
void get_xtal_frequency_Action(void)
{
  sfx_error_t err;
  sfx_u32 xtal_freq=0;

  err=ST_RF_API_get_xtal_freq(&xtal_freq);

  responsePrintf("{&N API call...&t4x &tx}\r\n",
		 "get_xtal_frequency", "xtal_freq", xtal_freq, "sfx_error", err );
}

/*****************************************************************************/
/*			RSSI OFFSET 																	 */
/*****************************************************************************/
/**
* @brief  Performs RSSI offset action.
* @param  None
* @retval None
*/
void VENDOR_set_rssi_offset_Action(void)
{
  sfx_s8 rssi_offset = (sfx_s8)signedCommandArgument(0);
  sfx_error_t err;

  err = ST_RF_API_set_rssi_offset(rssi_offset);

  NVM_UpdateOffset(NVM_RSSI_OFFSET, rssi_offset);

  responsePrintf("{&N API call...&tx}\r\n",
		 "ST_MANUF_API_set_rssi_offset", "sfx_error", err);
}

/**
* @brief  Get RSSI offset action.
* @param  None
* @retval None
*/
void VENDOR_get_rssi_offset_Action(void)
{
  sfx_error_t err;
  sfx_s8 rssi_offset;

  err=ST_RF_API_get_rssi_offset(&rssi_offset);

  responsePrintf("{&N API call...&td &tx}\r\n",
		 "ST_MANUF_API_get_rssi_offset", "rssi_offset", rssi_offset, "sfx_error", err);
}
/*****************************************************************************/
/*                            LBT THRESHOLD OFFSET                           */
/*****************************************************************************/
/**
* @brief  Sets LBT Threshold action.
* @param  None
* @retval None
*/
void set_lbt_thr_offset_Action(void)
{
  sfx_s8 lbt_thr = (sfx_s8)signedCommandArgument(0);
  sfx_s8 err;

  err = ST_RF_API_set_lbt_thr_offset(lbt_thr);

  NVM_UpdateOffset(NVM_LBT_OFFSET, lbt_thr);

  responsePrintf("{&N API call...&tx}\r\n",
		 "set_lbt_thr_offset", "sfx_error", err);
}

/**
* @brief  Gets LBT Threshold action.
* @param  None
* @retval None
*/
void get_lbt_thr_offset_Action(void)
{
  sfx_error_t err;
  sfx_s8 lbt_thr;

  err=ST_RF_API_get_lbt_thr_offset(&lbt_thr);

  responsePrintf("{&N API call...&td &tx}\r\n",
		 "ST_MANUF_API_get_rssi_offset", "lbt_thr", lbt_thr, "sfx_error", err);
}

/**
* @brief  Set payload encryption.
* @param  None
* @retval None
*/
void VENDOR_PayloadEncription(void)
{
  sfx_s8 enable_encryption=(sfx_s8)unsignedCommandArgument(0);
  sfx_error_t err = 0;

  ST_MCU_API_SetEncryptionPayload(enable_encryption);

  responsePrintf("{&N API call...&tx}\r\n",
		 "VENDOR_set_payload_encryption", "sfx_error", err);
}

/**
* @brief  Sets SMPS Voltage action.
* @param  None
* @retval None
*/
void set_smps_voltage_Action(void)
{
  sfx_u8 mode=(uint8_t)unsignedCommandArgument(0);
  ST_RF_API_smps(mode);
  responsePrintf("{&N API call...}\r\n",
                 "set_smps_voltage");
}

/**
* @brief  Sigfox API start continuous transmission.
* @param  None
* @retval None
*/
void SIGFOX_API_start_continuous_transmission_Action(void)
{
  sfx_u32 frequency=(uint32_t)unsignedCommandArgument(0);
  sfx_modulation_type_t mode=(sfx_modulation_type_t)unsignedCommandArgument(1);
  sfx_error_t err;

  err=SIGFOX_API_start_continuous_transmission(frequency, mode);

  responsePrintf("{&N API call...&tx}\r\n",
		 "SIGFOX_API_start_continuous_transmission", "sfx_error", err);
}

/**
* @brief  Sigfox API stop continuous transmission.
* @param  None
* @retval None
*/
void SIGFOX_API_stop_continuous_transmission_Action(void)
{
  sfx_error_t err;

  err=SIGFOX_API_stop_continuous_transmission();

  responsePrintf("{&N API call...&tx}\r\n",
		 "SIGFOX_API_stop_continuous_transmission", "sfx_error", err);
}

/**
* @brief  Reduce output power action.
* @param  None
* @retval None
*/
void reduce_output_power_Action(void)
{
  sfx_error_t err;
  sfx_s16 reduction=(int16_t)unsignedCommandArgument(0);

  err = ST_RF_API_reduce_output_power(reduction);

  responsePrintf("{&N API call...&t2x}\r\n", "reduce_output_power","sfx_error", err);
}

/**
* @brief  Switch power action.
* @param  None
* @retval None
*/
void switch_pa_Action(void)
{

  ST_RF_API_set_pa((int8_t)unsignedCommandArgument(0));

  responsePrintf("{&N API call...}\r\n",
                 "switch_pa");
}
/**
* @brief  Sigfox API send out of band action.
* @param  None
* @retval None
*/
void SIGFOX_API_send_out_of_band_Action(void)
{
  sfx_error_t err;

  err=SIGFOX_API_send_outofband(SFX_OOB_RC_SYNC);

  responsePrintf("{&N API call...&tx}\r\n",
		 "SIGFOX_API_send_out_of_band", "sfx_error", err);
}

/**
* @brief  Sigfox API sets RC Sync Period action.
* @param  None
* @retval None
*/
void SIGFOX_API_set_rc_sync_period_Action(void)
{
  sfx_error_t err;

  sfx_u16 period = (int16_t)unsignedCommandArgument(0);

  err = SIGFOX_API_set_rc_sync_period(period);

  responsePrintf("{&N API call...&tx}\r\n",
		 "SIGFOX_API_set_rc_sync_period", "sfx_error", err);
}

/**
* @brief  print version.
* @param  None
* @retval None
*/
sfx_error_t _printVersion(sfx_version_type_t versionType)
{
  sfx_error_t err;
  uint8_t* version_p;
  uint8_t size;

  err = SIGFOX_API_get_version(&version_p, &size, versionType);

  responsePrintf("{%t: ","version");

  if(!err)
  {
    for(uint8_t i=0;i<size;i++)
      responsePrintf("%c", version_p[i]);

    responsePrintf("}\r\n");
  }
  else
    responsePrintf("--- }\r\n");

  responsePrintf("&t4x\r\n","sfx_error", err);
  responsePrintf("}\r\n");

  return err;
}

/**
* @brief  Sigfox API send bit action.
* @param  None
* @retval None
*/
void SIGFOX_API_send_bit_Action(void)
{
  sfx_error_t err;
  sfx_u8 bit_value=unsignedCommandArgument(0);
  sfx_u8 tx_repeat=unsignedCommandArgument(1);
  sfx_u8 customer_resp[100];

  sfx_bool initiate_downlink_flag=(sfx_bool)unsignedCommandArgument(2);
  err=SIGFOX_API_send_bit(bit_value,customer_resp,tx_repeat,initiate_downlink_flag);
  responsePrintf("{&N API call...&tx}\r\n",
		 "SIGFOX_API_send_bit", "sfx_error", err );
}

/*****************************************************************************/
/*													MONARCH SCAN FUNCTIONS													 */
/*****************************************************************************/
/**
* @brief  Callback called from monarch rc scan.
* @param  rc_bit_mask
* @param  rssi
* @retval status of operation
*/
sfx_u8 callback_for_found(sfx_u8 rc_bit_mask, sfx_s16 rssi )
{
  printf("return rc_bit_mask %d\r\n", rc_bit_mask);
  printf("return rssi %d\r\n", rssi);

  switch (rc_bit_mask)
  {
  case 0x01:  //RC1
    {
      printf("Detected RC1!!!:\r\n");
    }
    break;
  case 0x02: //RC2
    {
      printf("Detected RC2!!!:\r\n");
    }
    break;
  case 0x04:  //RC3a
    {
      printf("Detected RC3!!!:\r\n");
    }
    break;
  case 0x08:  //RC4
    {
      printf("Detected RC4!!!:\r\n");
    }
    break;
  case 0x10: //RC5
    {
      printf("Detected RC5!!!:\r\n");
    }
    break;
  case 0x20:  //RC6
    {
      printf("Detected RC6!!!:\r\n");
    }
    break;
  case 0x40:  //RC7
    {
	printf("Detected RC7!!!:\r\n");
    }
    break;
  }
  return 1;
}

/**
* @brief  Execute Monarch scan function.
* @param  None
* @retval None
*/
void node_execute_monarch_scan_Action(void)
{
  sfx_error_t err;
  err=SIGFOX_MONARCH_API_execute_rc_scan ((sfx_u8)unsignedCommandArgument(0),
                                          (sfx_u16)unsignedCommandArgument(1),
                                          (sfx_timer_unit_enum_t)unsignedCommandArgument(2),
                                          callback_for_found);
  responsePrintf("{&N API call...}\r\n",
                 "SIGFOX_MONARCH_API_execute_rc_scan","sfx_error", err);
}

/**
* @brief  Stop Monarch scan function.
* @param  None
* @retval None
*/
void node_stop_monarch_scan_Action(void)
{
  sfx_error_t err;
  err=SIGFOX_MONARCH_API_stop_rc_scan();
  responsePrintf("{&N API call...}\r\n",
                 "SIGFOX_MONARCH_API_stop_rc_scan_Action","sfx_error", err);
}

/**
* @brief  Set Public key.
* @param  en
* @retval None
*/
void ST_Sigfox_SetPublicKey(uint8_t en)
{
  uint8_t err;
  err = enc_utils_set_public_key(en);

  if(err) /*Case of Error*/
  {

  }
}

