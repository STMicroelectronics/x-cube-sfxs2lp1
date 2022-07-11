/**
  * @file    serial_utils.c
  * @author  STM32ODE Team, Noida
  * @brief   Driver for serial_utilities  management
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
#include <stdint.h>
#include "serial_utils.h"
#include "sfx_config.h" 

/* Private Marcos ------------------------------------------------------------*/
#define TRUE 1
#define FALSE 0


/* Private variables ---------------------------------------------------------*/


/* Functions -----------------------------------------------------------------*/
#if defined(__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
extern int __io_putchar(int ch) __attribute__((weak));
extern int __io_getchar(void) __attribute__((weak));

#else
 extern void __io_putchar( char c );
#endif
 

/**
* @brief  write Hex.
* @param  charBuffer : Pointer to the  char buffer
* @param  value : Value
* @param  charCount : Number of bytes
* @retval Converted value
*/
uint8_t *writeHex(uint8_t *charBuffer, uint16_t value, uint8_t charCount)
{
  uint8_t c = charCount;
  charBuffer += charCount;
  for (; c; c--) {
    uint8_t n = value & 0x0F;
    value = value >> 4;
    *(--charBuffer) = n + (n < 10
                           ? '0'
                           : 'A' - 10);
  }
  return charBuffer + charCount;
}/* end writeHex() */

/**
* @brief  Write a string.
* @param  string : Pointer to the  char string
* @retval Status of operation
*/
uint8_t serialWriteString(const char * string)
{
   while (*string != '\0')
   {
     __io_putchar(*string);
     string++;
   }
   return TRUE;
}/* end serialWriteString() */

/**
* @brief  Wait on serial .
* @param  None
* @retval Status of operation
*/
uint8_t serialWaitSend(void)
{
  __io_flush();
  return TRUE;
}/* end serialWaitSend() */

/**
* @brief  Read a serial input line.
* @param  data: Pointer to the data
* @param  max:Max index
* @param  index:current index
* @retval Status of operation
*/
uint8_t serialReadPartialLine(char *data, uint16_t max, uint16_t * index)
{
  uint8_t err;
  uint8_t ch;

  if (((*index) == 0) || ((*index) >= max))
    data[0] = '\0';

  for (;;) {   
    err = serialReadByte(&ch);
    /* no new serial char?, keep looping*/
    if (!err) return err;

    /* handle bogus characters */
    if ( ch > 0x7F ) continue;

    /* handle leading newline - fogBUGZ # 584*/
    if (((*index) == 0) &&
        ((ch == '\n') || (ch == 0))) continue;

    /* Drop the CR, or NULL that is part of EOL sequence.*/
    if ((*index) >= max) {
      *index = 0;
      if ((ch == '\r') || (ch == 0)) continue;
    }

    /* handle backspace */
    if ( ch == 0x8 ) {
      if ( (*index) > 0 ) {
        /* delete the last character from our string */
        (*index)--;
        data[*index] = '\0';
        /* echo backspace */
        serialWriteString("\010 \010");
      }
      /* don't add or process this character */
      continue;
    }

    /* if the string is about to overflow, fake in a CR  */
    if ( (*index) + 2 > max ) {
      ch = '\r';
    }

    serialWriteByte(ch); /* term char echo */

    /* upcase that char */
    if ( ch>='a' && ch<='z') ch = ch - ('a'-'A');

    /* build a string until we press enter  */
    if ( ( ch == '\r' ) || ( ch == '\n' ) ) {
      data[*index] = '\0';

      if (ch == '\r') {
        serialWriteByte('\n'); /* "append" LF  */
        *index = 0;                       /* Reset for next line; \n next */
      } else {
        serialWriteByte('\r'); /* "append" CR  */
        *index = max;          /* Reset for next line; \r,\0 next  */
      }

      return TRUE;
    } 
      
    data[(*index)++] = ch;
  }
}/* end serialReadPartialLine() */

/**
* @brief  Write serial data.
* @param  data: Pointer to the data
* @param  length:Length of the data
* @retval Status of operation
*/
uint8_t serialWriteData(uint8_t *data, uint8_t length)
{
#if defined(__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
  volatile uint8_t test;
  test = sfx_serial_write((char*)data, length);
  return TRUE;
#else
  uint8_t ctr=0;
  for (ctr = 0; ctr < length; ctr++) 
  {
    __io_putchar(data[ctr]);
  }
  return TRUE;
#endif
}/* end serialWriteData() */


/**
* @brief  Write a byte.
* @param  dataByte: Data to be written
* @retval Status of operation
*/
uint8_t serialWriteByte(uint8_t dataByte)
{
  return serialWriteData(&dataByte, 1);
}/* end serialWriteByte() */


/**
* @brief  Non blocking read.
* @param  dataByte: Pointer to the data
* @retval TRUE if there is a data, FALSE otherwise
*/
uint8_t serialReadByte(uint8_t *dataByte)
{
#if defined(__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
		uint8_t test;
	test =  sfx_serial_read((char *)dataByte, 1);
	
	return(test);
#else        
  return __io_getcharNonBlocking(dataByte);
#endif
}




