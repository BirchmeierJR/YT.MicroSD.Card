/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.h
  * @brief   This file contains the common defines and functions prototypes for
  *          the user_diskio driver.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
 /* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_DISKIO_H
#define __USER_DISKIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* USER CODE BEGIN 0 */


/* Includes ------------------------------------------------------------------*/
#include "diskio.h"
#include "ff_gen_drv.h"
#include "ffconf.h"
#include "ff.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/*
 * Defines to support for Micro SD memory card.
 * - Configure if disk write operation required (_USE_WRITE), generally set to 1
 * - Configure chip select (CS) GPIO used to select SD Memory card. The code defines
 * 		the CS to be active low. Specify a port and pin number.
 * - Configure a SPI port with the a number of configuration parameters as
 * 		summarized below.
 * 		Mode:						Full Duplex - Master
 * 		Format: 					Motorola
 * 		Data Size:					8 Bits
 * 		First Bit:					MSB First
 * 		Clock Polarity (CPOL): 		Low
 * 		Clock Phase (CPHA):			1 Edge
 * 		CRC Calculation:			Disabled
 * 		NSS Signal Type:			Software
 *
 *		Also, I configured a pull-up resistor for the MISO pin. I did this as it
 *		is a floating input signal when the SD card is not enabled or driving it.
 *		I'm not sure if this was required, but I think it is. Regardless, it should
 *		not hurt the system's operation.
 * - Configuration SPI clock's Baud Rate Generator (BR). To support the SD card via SPI,
 * 		two clock speeds are required. When initializing the card at boot, or new
 * 		insertion of the card, a slow clock is required (100 KHz - 400 KHz). Post
 * 		initialization, the clock speed can be increases to a more desirable speed.
 * 		To keep the code simple, I don't read clock speed and associated pre-scaler
 * 		configurations to determine the proper BR settings to get slow/fast clock
 * 		speed. Instead I'll let you determine the proper values for your system.
 *
 * 		For example, my system uses a 84MHz system clock, with fpclk = 42MHz.
 * 		From RM0368 Reference Manual section 20.5.1 'SPI Control Register 1', the
 * 		BR settings of 6 & 1 gives appropriate clock speeds.
 *
 * OPTIONS:
 * - _ff_sdCard_isWrEnabled() can be fleshed out if you need to support card write protection
 * 		setting.
 *
 *
 ***** BEGIN: CAUTION *****
 *
 * The function f_mkfs(), used for format a FAT FS onto the SD Card has been verified
 * to format the card in a way that is compatible with Windows 10. The function takes
 * a pointer to a working buffer. This is true, assuming the working buffer is the same
 * size as a sector (i.e. 512 bytes).
 *
 * Conversely, f_mkfs() appears to fail when the working buffer is n*512, where n is an
 * integer greater than 1. A larger working buffer should allow a faster format operation
 * in that CMD25 (Write Multiple Blocks) can be used. However, for unknown reasons, CMD25
 * appears to fail. Perhaps it is a memory card issue (I looked at the SPI activity via
 * a logic analyzer and card enters a busy state for an indefinite time ... over 3 seconds).
 *
 ***** END: CAUTION *******
 *
 * REFERENCE Material:
 * - "SD Specifications: Part 1 Physical Layer Simplified Specification", Version 9.10
 * 		dated December 1, 2023. Not required reading, however if you're interested
 * 		there is a lot of useful, interesting, and informative material in this document.
 * 		It is freely available, and at the time of this writing, accessible at
 * 		'www.sdcard.org'.
 * - For a high level overview of SM Memory cards, I found the information on ChaN's website
 * 		to be useful. NOTE: This is a non-secure site:
 * 		"http://elm-chan.org/docs/mmc/mmc_e.html".
 */

#define _USE_WRITE							1
#define DEF_FF_SD_CARD_CS_PORT           	MicroSD_CS_GPIO_Port
#define DEF_FF_SD_CARD_CS_PIN            	MicroSD_CS_Pin
#define DEF_FF_SPI_PORT               		SPI2
#define DEF_FF_SPI_CLCK_BR_SLOW				(6)	// 328  KHz = (fpclk = 84MHz/2) / 128
#define DEF_FF_SPI_CLCK_BR_FAST				(1)	// 10.5 MHz = (fpclk = 84MHz/2) / 4

// ioctl() debug option: allows limited debug messages via "SWV Data Console"
// Example:
//		uint8_t	fDebugSetting	= 1;				// Alternately, setting fDebugSetting = 1, would set debug to disable (i.e. default setting)
//		disk_ioctl(0, CTRL_DEBUG, &fDebugSetting);	// enables debug
#define CTRL_DEBUG		(0xFF)

// Verify required operating environment exists. Specifically, we're designed to:
// Use FreeRTOS, with re-entrant protection and FAT FS use of mutex to protect
// against multi-threaded operation.
#if defined (_FS_REENTRANT)
	#if (0 == _FS_REENTRANT)
		#error Implementation requires FreeRTOS with FatFS REENTRANT/MUTEX protection
	#else
		#if (0 == _USE_MUTEX)
			#error Implementation requires FreeRTOS with FatFS REENTRANT/MUTEX protection
		#endif
	#endif
#else
	#error Implementation requires FreeRTOS with FatFS REENTRANT/MUTEX protection
#endif

#if (_FATFS == 68300)
	#if (1 != _VOLUMES)
	#error Code updates are required to support something other than 1 disk
	#endif

	#define FF_VOLUMES		_VOLUMES
	#define FF_FS_TIMEOUT 	_FS_TIMEOUT
#endif

#if (_FATFS	== 80286)
	#if defined (FF_FS_REENTRANT)
		#if (0 == FF_FS_REENTRANT)
			#error Implementation requires FreeRTOS with FatFS REENTRANT/MUTEX protection
		#else
		#endif
	#else
		#error Implementation requires FreeRTOS with FatFS REENTRANT/MUTEX protection
	#endif

	#if (1 != FF_VOLUMES)
	#error Code updates are required to support something other than 1 disk
	#endif
#endif

/* Exported functions ------------------------------------------------------- */
extern Diskio_drvTypeDef  USER_Driver;

/* USER CODE END 0 */

#ifdef __cplusplus
}
#endif

#endif /* __USER_DISKIO_H */
