/**
  * @file    SigfoxDemo_CLI_Commands.h
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

#ifndef __SIGFOX_DEMO_CLI_COMMANDS_H
#define __SIGFOX_DEMO_CLI_COMMANDS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sigfox_types.h"
#include "sigfox_api.h"

/* Private Macros -----------------------------------------------------------*/
#define CMD_HIDDEN(CMD)	(strcmp(CMD->name,"_e2prom_write_page")==0 || \
	strcmp(CMD->name,"_e2prom_read_page")==0|| \
	strcmp(CMD->name,"interactive")==0	|| \
	strcmp(CMD->name,"_set_rcz")==0)

#define COMMAND_TABLE \
{ "node_close",          		node_close_Action,          					"",		"Close the library" }, \
{ "node_reset",          		node_reset_Action,          					"",		"Reset the library" }, \
{ "node_open",           		node_open_Action,           					"",		"Open the library" }, \
{ "node_open_with_zone", 		node_open_with_zone_Action,					"u",		"Open the library in the selected RC Zone. 1=RC1, 2=RC2, 3=RC3c, 4=RC4, 5=RC5" }, \
{ "node_get_info",       		node_get_info_Action,						"",		"Returns info on send frame depending on the mode used" }, \
{ "node_get_version",			node_get_version_Action,					"", 		"Returns the version of the Sigfox library" }, \
{ "node_send_frame",     		node_send_frame_Action,						"buu",	"Send a frame" }, \
{ "node_send_oob",     			SIGFOX_API_send_out_of_band_Action,				"",		"Send OOB frame" }, \
{ "node_set_rc_sync_period",     	SIGFOX_API_set_rc_sync_period_Action,			"v",		"Set the RC SYNC period" }, \
{ "node_send_bit",			SIGFOX_API_send_bit_Action, 					"uu",		"Send a bit n times"}, \
{ "node_execute_monarch_scan",   	node_execute_monarch_scan_Action, 				"uvu",  	"Execute Monarch scan. rc_capability, time, unit(0=ms, 1=s, 2=min, 3=hour)" }, \
{ "node_stop_monarch_scan",      	node_stop_monarch_scan_Action,    				"",		"This function stops any ongoing RC scan" }, \
{ "node_set_std_config", 		node_set_std_config_Action, 					"wwwv", 	"Set the standard channel configuration"}, \
{ "node_get_std_config", 		node_get_std_config_Action, 					"",     	"Returns the standard channel configuration"}, \
{ "set_low_power",       		ST_set_low_power_Action,    					"u",    	"Low power is used only in the radio phases"}, \
{ "get_id",              		ST_get_id_Action,           					"",     	"ID stored in the current node (EEPROM/FLASH)" }, \
{ "get_pac",             		ST_get_pac_Action,          					"",     	"PAC stored in the current node (EEPROM/FLASH)" }, \
{ "get_rcz",             		ST_get_rcz_Action,          					"",     	"RCZ stored in the current node (EEPROM/FLASH)" }, \
{ "get_lib_version",     		get_lib_version_Action,     					"u",    	"Get version of specified module. 0=Sigfox, 1=MCU_API, 2=RF_API, 5=MONARCH_API, 6=DEVICE_CONFIG_API"},\
{ "_set_rcz",            		ST_set_rcz_Action,          					"",     	"" }, \
{ "switch_public_key",   		node_set_public_key_Action, 					"u",    	"Switch to public key" }, \
{ "switch_test_credentials", 		node_set_test_credentials_Action,   			"u",		"Set test credentials 1=On, 0=Off"}, \
{ "node_test_mode",      		node_test_mode_Action,      					"uu",		"Enter a specified test mode"},\
{ "node_monarch_test_mode", 		node_monarch_test_mode_Action, 				"uuu",	"Enter a specified test mode for monarch. rcz test_mode rc_capabilities"},\
{ "set_payload_encryption",		VENDOR_PayloadEncription, 					"u", 		"Enables payload encryption "}, \
{ "start_continuous_transmission",	SIGFOX_API_start_continuous_transmission_Action,	"wu", 	"Executes a continuous wave or modulation"}, \
{ "stop_continuous_transmission", 	SIGFOX_API_stop_continuous_transmission_Action,		"", 		"Stops a continuous wave or modulation"}, \
{ "switch_pa", 				switch_pa_Action, 						"u", 		"Turn Power Amplifier On/Off (when applicable): 1=On, 0=Off"}, \
{ "reduce_output_power", 		reduce_output_power_Action, 					"v", 		"Reduce the output power: Pout=Pmax-(0.5 * reduce factor) (dBm)"}, \
{ "set_smps_voltage", 			set_smps_voltage_Action, 					"u", 		"Set the smps voltage of the device (see UM for available codes)"}, \
{ "set_xtal_frequency_offset",	set_xtal_frequency_offset_Action, 				"w", 		"Align the crystal frequency adding the compensation value"}, \
{ "get_xtal_frequency",			get_xtal_frequency_Action, 					"", 		"Returns the crystal frequency compensation value"}, \
{ "set_rssi_offset",			VENDOR_set_rssi_offset_Action, 				"w", 		"Set RSSI calibration value"}, \
{ "get_rssi_offset",			VENDOR_get_rssi_offset_Action, 				"",		"Get RSSI calibration value"}, \
{ "set_lbt_thr_offset",			set_lbt_thr_offset_Action, 					"w", 		"Set the LBT threshold value to use during Sigfox certification"}, \
{ "get_lbt_thr_offset",			get_lbt_thr_offset_Action, 					"",		"Get the LBT threshold value to use during Sigfox certification"}

#ifndef MON_REF_DES
#define EEPROM_COMMANDS_TABLE \
{ "_e2prom_write_page",  EepromWritePageAction,      "vub",  ""}, \
{ "_e2prom_read_page",   EepromReadPageAction,       "vuu",  ""}
#endif

/* Private Function prototypes  ---------------------------------------------*/
void set_id(uint32_t id_new);
void set_pac(uint8_t *pac_new);
void set_rcz(uint8_t rcz_new);
void set_testMode(uint8_t enableTestMode);
uint8_t get_testMode(void);
void node_reset_Action(void);
void node_close_Action(void);
void node_open_Action(void);
void node_open_with_zone_Action(void);
void node_get_info_Action(void);
void node_get_version_Action(void);
void node_send_frame_Action(void);
void node_execute_monarch_scan_Action(void);
void node_stop_monarch_scan_Action(void);
void ST_get_id_Action(void);
void ST_get_pac_Action(void);
void ST_set_low_power_Action(void);
void ST_get_rcz_Action(void);
void ST_set_rcz_Action(void);
void node_set_std_config_Action(void);
void node_get_std_config_Action(void);
void node_test_mode_Action(void);
void node_monarch_test_mode_Action(void);
void EepromWritePageAction(void);
void EepromReadPageAction(void);
void node_set_public_key_Action(void);
void node_set_test_credentials_Action(void);
void ST_set_id_Action(void);
void ST_set_pac_Action(void);
void VENDOR_PayloadEncription(void);
void set_smps_voltage_Action(void);
void get_lib_version_Action(void);
void SIGFOX_API_start_continuous_transmission_Action(void);
void SIGFOX_API_stop_continuous_transmission_Action(void);
void switch_pa_Action(void);
void reduce_output_power_Action(void);
void SIGFOX_API_send_out_of_band_Action(void);
void SIGFOX_API_send_bit_Action(void);
sfx_error_t _printVersion(sfx_version_type_t versionType);
/* Offsets compensation */
void set_xtal_frequency_offset_Action(void);
void get_xtal_frequency_Action(void);
void VENDOR_frequency_offset_Action(void);
void get_frequency_offset_Action(void);
void VENDOR_set_rssi_offset_Action(void);
void VENDOR_get_rssi_offset_Action(void);
void set_lbt_thr_offset_Action(void);
void get_lbt_thr_offset_Action(void);
void SIGFOX_API_set_rc_sync_period_Action(void);
void ST_Sigfox_SetPublicKey(uint8_t en);
#endif

