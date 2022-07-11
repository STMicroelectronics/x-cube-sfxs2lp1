## <b>S2915A1\_Sigfox\_CLI for Sigfox on S2LP Description</b>

Sigfox Demo GUI PC application provides an interactive interface to transmit messages to the sigfox network

Example Description:

This is a ST-SigFox demo that runs CLI example. SigFox Demo GUI PC application provides an interactive interface to transmit messages to the sigfox network and program the ST X-NUCLEO-S2868A1/X-NUCLEO-S2868A2/X-NUCLEO-S2915A1 nodes with the Sigfox ID to ready the node for network communication .

* For Sigfox application demonstration One S2-LP Expansion Board + STM32 Nucleo is programmed as node
* Another side,Message can be seen on Sigfox Network with Sigfox data collector unit
* It operates in different Radio configuration zone RCZ1/RCZ2/RCZ3/RCZ4

* X-NUCLEO-S2868A1/X-NUCLEO-S2868A2 (for RCZ1 and RCZ3)
* X-NUCLEO-S2915A1 (for RCZ2 and RCZ4)

* Sigfox firmware configurations

* NUCLEO\_L0\_CLI - Configuration to be used on the NUCLEO-L073RZ
* NUCLEO\_L0\_CLI_MONARCH - Configuration to be used on the NUCLEO-L073RZ
* NUCLEO\_L0\_PUSH_BUTTON - Configuration to be used on the NUCLEO-L073RZ
* NUCLEO\_L1\_CLI - Configuration to be used on the NUCLEO-L152RE
* NUCLEO\_L1\_CLI_MONARCH - Configuration to be used on the NUCLEO-L152RE
* NUCLEO\_L1\_PUSH_BUTTON - Configuration to be used on the NUCLEO-L152RE
* NUCLEO\_L4\_CLI - Configuration to be used on the NUCLEO-L476RG
* NUCLEO\_L4\_CLI_MONARCH - Configuration to be used on the NUCLEO-L476RG
* NUCLEO\_L4\_PUSH_BUTTON - Configuration to be used on the NUCLEO-L476RG

* Connect the Sigfox Data Collector Unit to PC.
* Flash the Node(Nucleo Board Mouneted with X-NUCLEO-S2868A1/X-NUCLEO-S2868A2/X-NUCLEO-S2915A1) with desired configuration as explained above. And connect it with Sigfox GUI. Even any terminal utility can be used to execute the command. In case of Push Button examples, just pressing the user button on NUCLEO ,will send the sigfox packet.

Known limitations:

* When starting the project from Example Selector in STM32CubeMX and regenerating it from ioc file, you may face a build issue because the Contiki-NG middleware is copied in a different folder and all the files are duplicated. To solve it, please delete the project folder of the IDE that you are using (EWARM, MDK-ARM or STM32CubeIDE). Other duplicate files that you should delete, if you started the project for the Nucleo-F401RE board, from Src and Inc folders, are: stm32f4xx\_nucleo.c, stm32f4xx\_nucleo.h and stm32f4xx\_nucleo\_errno.h. The same operations apply if you starts the project for another STM32 Nucleo board (e.g. for Nucleo-L053R8, the files to be removed are stm32l0xx\_nucleo.c, stm32l0xx\_nucleo.c, stm32l0xx\_nucleo.h and stm32l0xx\_nucleo_errno.h).
* case of "USE_FLASH" is not tested in case of NUCLEO-L476RG

### <b>Keywords</b>

Sigfox,CLI, SPI, SUBGHZ, UART, TIM

### <b>Directory contents</b>

* app\_x\_cube_sfxs2lp1.c This file provides interface between the main.c and Sigfox based applications.
    
* main.c Main program body.
    
* s2lp_management.c This file provides code for Management functions for S2-LP radio.
    
* sfx_config.c This file provides code for sigfox configuration
    
* sfx\_demo\_cli_commands.c This file provides code for Sigfox commands for CLI
    
* stm32**xx\_hal\_msp.c This file provides code for the MSP Initialization and de-Initialization.
    
* stm32**xx\_nucleo\_bus.c Source file for the BSP BUS IO driver.
    
* stm32**xx_nucleo.c Source file for the BSP Common driver
    
* system_stm32**xx.c CMSIS Cortex-M4 Device Peripheral Access Layer System Source File.
    
* stm32**xx_it.c Source code for interrupt Service Routines.
    

### <b>Hardware and Software environment</b>

* This example runs on STM32 Nucleo devices with X-NUCLEO-S2868A1 or X-NUCLEO-S2868A2 or X-NUCLEO-S2915A1 STM32 expansion board
* This example has been tested with STMicroelectronics:
    * NUCLEO-L073RZ RevC board
    * NUCLEO-L152RE RevC board
    * NUCLEO-L476RG RevC board and can be easily tailored to any other supported device and development board.

ADDITIONAL\_BOARD : X-NUCLEO-S2915A1 https://www.st.com/content/st\_com/en/products/ecosystems/stm32-open-development-environment/stm32-nucleo-expansion-boards/stm32-ode-connect-hw/x-nucleo-s2915a1.html ADDITIONAL\_COMP : S2-LP https://www.st.com/content/st\_com/en/products/wireless-connectivity/long-range/proprietary-sub-1-ghz-products/s2-lp.html

### <b>How to use it?</b>

* In order to make the program work, you must do the following:
    * WARNING: before opening the project with any toolchain be sure your folder installation path is not too in-depth since the toolchain may report errors after building.
    * The tested tool chain and environment is explained in the Release notes
    * Open the suitable toolchain (STM32CubeIDE, IAR, Keil) and open the project for the required STM32 Nucleo board
    * Rebuild all files and load your image into target memory.
    * Run the example.
    * Alternatively, you can download the pre-built binaries in “Binary” folder included in the distributed package.

### <b>Author</b>

SRA Application Team

### <b>License</b>

Copyright (c) 2021 STMicroelectronics. All rights reserved.

This software is licensed under terms that can be found in the LICENSE file in the root directory of this software component. If no LICENSE file comes with this software, it is provided AS-IS.