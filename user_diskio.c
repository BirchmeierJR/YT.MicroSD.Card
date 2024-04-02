/* USER CODE BEGIN Header */
/**
******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
  ******************************************************************************
  * Copyright (C) Tilen Majerle, 2014
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
  *
  ******************************************************************************
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  *
  * Copyright (c) 2024, John R Birchmeier
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
  *
  * NOTE: This code is derived from original work by:
  * o Tilen Majerle, and
  * o STMicroelectronics
  * The original was a much more general solution. It was more sophisticated and
  * a bit more complicated as compared to this tailored solution. My apologies
  * to Tilen Majerle for the hack job I did to the code. Much credit for the good
  * in this is theirs, and any bugs are likely mine.
  ******************************************************************************
   */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "main.h"			// port & pin definitions
#include "ff_gen_drv.h"
#include "user_diskio.h"
#include "diskio.h"
#include "cmsis_os.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* MMC/SD command *
 * JRB: See "Physical Layer Simplified Specification Version 9.10", section
 * "4.7.4 Detailed Command Description".
 */
#define CMD0	(0)			/* GO_IDLE_STATE */
#define CMD1	(1)			/* SEND_OP_COND (MMC) */
#define	ACMD41	(0x80+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(8)			/* SEND_IF_COND = Send Interface Condition (Valid when card is in IDLE state)*/
#define CMD9	(9)			/* SEND_CSD */
#define CMD10	(10)		/* SEND_CID */
#define CMD12	(12)		/* STOP_TRANSMISSION */
#define ACMD13	(0x80+13)	/* SD_STATUS (SDC) */
#define CMD16	(16)		/* SET_BLOCKLEN */
#define CMD17	(17)		/* READ_SINGLE_BLOCK */
#define CMD18	(18)		/* READ_MULTIPLE_BLOCK */
#define CMD23	(23)		/* SET_BLOCK_COUNT (MMC) */
#define	ACMD23	(0x80+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(24)		/* WRITE_BLOCK */
#define CMD25	(25)		/* WRITE_MULTIPLE_BLOCK */
#define CMD32	(32)		/* ERASE_ER_BLK_START */
#define CMD33	(33)		/* ERASE_ER_BLK_END */
#define CMD38	(38)		/* ERASE */
#define CMD55	(55)		/* APP_CMD */
#define CMD58	(58)		/* READ_OCR */

#define _TOKEN_STRT_DATA_BLOCK_		(0xFE)	// Data Block Token (CMD17/18/24)
#define _TOKEN_STOP_DATA_			(0xFD)	// @todo collect these into a group
#define _TOKEN_STRT_DATA_BLOCK_25	(0xFC)	// Data Block Token (CMD25)
#define _DEF_TX_DUMMY_VALUE			(0xFF)	// Tx value to generate clock while RX data via MISO line
#define _DEF_MAX_DATA_BLOCK_SZ		(512)	// [cnt] octets

/* MMC card type flags (MMC_GET_TYPE) */
#define CT_MMC		0x01		/* MMC ver 3 */
#define CT_SD1		0x02		/* SD ver 1 */
#define CT_SD2		0x04		/* SD ver 2 */
#define CT_SDC		(CT_SD1|CT_SD2)	/* SD */
#define CT_BLOCK	0x08		/* Block addressing */


// **** MACROs *****


#define M_FF_SPI_IS_BUSY(SPIx) 					(((SPIx)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || ((SPIx)->SR & SPI_SR_BSY))
#define M_FF_SPI_WAIT_IF_BUSY(SPIx)    			while (M_FF_SPI_IS_BUSY(SPIx))
#define M_FF_SPI_CHCK_ENABLED(SPIx)   			if (!((SPIx)->CR1 & SPI_CR1_SPE)) {return;}
#define M_FF_SPI_CHCK_ENABLED_RESP(SPIx, val)	if (!((SPIx)->CR1 & SPI_CR1_SPE)) {return (val);}
#define M_FF_SD_CARD_CS_ACTIVE(pObj)			HAL_GPIO_WritePin(pObj->sdCardCsPort, pObj->sdCardCsPin, 0)
#define M_FF_SD_CARD_CS_INACTIVE(pObj)			HAL_GPIO_WritePin(pObj->sdCardCsPort, pObj->sdCardCsPin, 1)


#define __ff_printfDebug(pObj, ...)				if (pObj->fIsDebugEn) { printf(__VA_ARGS__); }

/* Private variables ---------------------------------------------------------*/
/* Disk status */

typedef struct _FF_VOLUME_OBJ_ST_
{
	DSTATUS 			sdCardStatus;		// Physical drive status
	BYTE 				sdCardType;			// Card type flags
	uint8_t				sdCardIsInit;		// non-zero if Initializer() completed with success
	uint8_t				fIsDebugEn;			// Debug printf() via SWV Debug Console Window
	SPI_TypeDef*		pSPI;				// SPI: pointer to SPI object used to support this SD card
	GPIO_TypeDef*		sdCardCsPort;		// SPI: Chip Select Port
	short unsigned int	sdCardCsPin;		// SPI: Chip Select Pin
	uint8_t				spiClkBrSlow;		// SPI: Clock Baud Rate setting for slow clock (100KHz to 400 KHz)
	uint8_t				spiClkBrFast;		// SPI: Clock Baud Rate setting for fast clock
	osMutexId_t			mutex;
} _FF_VOLUME_OBJ_ST;

static _FF_VOLUME_OBJ_ST	thisObj[FF_VOLUMES];

//static _FF_VOLUME_OBJ_ST*	pThisObjX = &thisObj[0];
/**
 * @brief  Tx & Rx a single byte over SPI
 * @param  *SPIx: Pointer to SPIx peripheral you will use, where x is between 1 to 6
 * @param  data: 8-bit data size to send over SPpdrv >= FF_VOLUMES)etval Received byte from slave device
 */
static uint8_t _ff_spi_txAndRx(_FF_VOLUME_OBJ_ST* pThisObj, uint8_t data) {
	/* Check if SPI is enabled */

	M_FF_SPI_CHCK_ENABLED_RESP(pThisObj->pSPI, 0);	// CAUTION: This function may RETURN! Not a fan of this construct.

	/* Wait for previous transmissions to complete if DMA TX enabled for SPI */
	M_FF_SPI_WAIT_IF_BUSY(pThisObj->pSPI);

	/* TX: Fill output buffer with data */
	pThisObj->pSPI->DR = data;

	/* Wait for transmission to complete */
	M_FF_SPI_WAIT_IF_BUSY(pThisObj->pSPI);

	/* RX: Return data from buffer */
	return pThisObj->pSPI->DR;
}

/* Initialize MMC interface */
static void _ff_spi_init(_FF_VOLUME_OBJ_ST*	pThisObj)
{
	pThisObj->pSPI->CR1 |= SPI_CR1_SPE;
	M_FF_SD_CARD_CS_INACTIVE(pThisObj);	// i.e. inactive CS for Micro SD card
	osDelay(pdMS_TO_TICKS(10));			// 10ms seems long, but not an issue as this is a one-time initialization
}


/* Receive multiple bytes, sending a dummy value (0xFF) to generate corresponding clock signal */
static void _ff_spi_rxMultiple
(
	_FF_VOLUME_OBJ_ST*	pThisObj,
	BYTE *buff,		/* Pointer to data buffer */
	UINT btr		/* Number of bytes to receive (even number) */
)
{
	/* Read multiple bytes (need to send dummy value so SPI clock will tick & we'll get data from card via MISO) */
	uint32_t i;
	if (btr & 1)	{	__ff_printfDebug(pThisObj, "%s.%d: btr (%d) must be even\n", __FUNCTION__, __LINE__, btr);	return; }

	/* Check if SPI is enabled */
	M_FF_SPI_CHCK_ENABLED(pThisObj->pSPI);

	/* Wait for previous transmissions to complete if DMA TX enabled for SPI */
	M_FF_SPI_WAIT_IF_BUSY(pThisObj->pSPI);

	for (i = 0; i < btr; i++)
	{
		/* Fill output buffer with data */
		pThisObj->pSPI->DR = _DEF_TX_DUMMY_VALUE;	// generates clock so card can send data via MISO line.

		/* Wait for SPI to end everything */
		M_FF_SPI_WAIT_IF_BUSY(pThisObj->pSPI);

		/* Save data to buffer */
		buff[i] = pThisObj->pSPI->DR;
	}
	return;
}


#if _USE_WRITE
/* Send multiple bytes to SD card via SPI */
static void _ff_spi_txMultiple
(
	_FF_VOLUME_OBJ_ST*	pThisObj,
	const BYTE *buff,	/* Pointer to the data */
	UINT btx			/* Number of bytes to send (even number) */
)
{
	/* Write multiple bytes */

	uint32_t i;
	if (btx & 1)	{	__ff_printfDebug(pThisObj, "%s.%d: btx (%d) must be even\n", __FUNCTION__, __LINE__, btx);	return; }

	/* Check if SPI is enabled */
	M_FF_SPI_CHCK_ENABLED(pThisObj->pSPI);

	/* Wait for previous transmissions to complete if DMA TX enabled for SPI */
	M_FF_SPI_WAIT_IF_BUSY(pThisObj->pSPI);

	for (i = 0; i < btx; i++)
	{
		/* Fill output buffer with data */
		pThisObj->pSPI->DR = buff[i];

		/* Wait for SPI to end everything */
		M_FF_SPI_WAIT_IF_BUSY(pThisObj->pSPI);

		/* Read data register */
		(void)pThisObj->pSPI->DR;
	}
	return;

}
#endif


/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/

/* Wait for SD Card to be ready (i.e. not busy, where a busy is defined to
 * be the case where MISO signal is held low. When no long low, the rx
 * value from the card will be all ones (i.e. 0xFF)).
 * 1:Ready, 0:Timeout */
static int _ff_wait_sdCardReady
(
	_FF_VOLUME_OBJ_ST*	pThisObj,
	UINT wt			/* Timeout [ms] */
)
{
	#define _DEF_MISO_IS_READY_	(0xFF)
	BYTE d;
	uint16_t	nCntNoDel	= 0;
	// We'll try several attempts without delay, waiting for the SD card to respond with 0xFF
	// Failing that, then we delay in 1ms increments for up to a total of 'wt' ms.
	do
	{
		d = _ff_spi_txAndRx(pThisObj, _DEF_TX_DUMMY_VALUE);
		nCntNoDel++;
	} while ((d != _DEF_MISO_IS_READY_) && (nCntNoDel < 5));	/* Wait for card goes ready or timeout */

	if (d != _DEF_MISO_IS_READY_)
	{
		uint16_t	nTimeMs		= 0;
		do {

			d = _ff_spi_txAndRx(pThisObj, _DEF_TX_DUMMY_VALUE);
			if (_DEF_MISO_IS_READY_ != d)
			{	// 2nd interrogation.
				// JRB NOTE: I noted with the Lenovo card, the card appears to report busy condition
				// (i.e. MISO line held low) until we interrogate the card by sending 0xFF. Then if the
				// card is no longer busy, it sets MISO high at the end of 0xFF. As such, I added a
				// 2nd interrogation, to see if busy condition just cleared. If not cleared, then
				// we'll delay again.
				d = _ff_spi_txAndRx(pThisObj, _DEF_TX_DUMMY_VALUE);
			}
		} while ((d != _DEF_MISO_IS_READY_) && (nTimeMs++ < wt) && (osOK == osDelay(pdMS_TO_TICKS(1))));	/* Wait for card goes ready or timeout */
	}

	if (d != _DEF_MISO_IS_READY_)
	{
		__ff_printfDebug(pThisObj, "%s.%d: timeout\n", __FUNCTION__, __LINE__);
	}
	return (d == _DEF_MISO_IS_READY_) ? 1 : 0;
}

/*-----------------------------------------------------------------------*/
/* Deselect card and release SPI                                         */
/*-----------------------------------------------------------------------*/

static void _ff_sdCardCS_setInactive (_FF_VOLUME_OBJ_ST* pThisObj)
{
	M_FF_SD_CARD_CS_INACTIVE(pThisObj);						/* CS (i.e. CS is inactive) */
	_ff_spi_txAndRx(pThisObj, _DEF_TX_DUMMY_VALUE);	/* Dummy clock (force DO hi-z for multiple slave SPI) */
}

/*-----------------------------------------------------------------------*/
/* Select card and wait for ready                                        */
/*-----------------------------------------------------------------------*/

static int _ff_sdCardCS_setActive (_FF_VOLUME_OBJ_ST* pThisObj)	/* 1:OK, 0:Timeout */
{
	M_FF_SD_CARD_CS_ACTIVE(pThisObj);
	_ff_spi_txAndRx(pThisObj, _DEF_TX_DUMMY_VALUE);	/* Dummy clock (force DO enabled) */

	if (_ff_wait_sdCardReady(pThisObj, 500))
	{
		return 1;	/* OK */
	}
	_ff_sdCardCS_setInactive(pThisObj);
	__ff_printfDebug(pThisObj, "%s.%d: timeout\n", __FUNCTION__, __LINE__);
	return 0;	/* Timeout */
}


/*-----------------------------------------------------------------------*/
/* Receive a data packet from the Micro SD Card, waiting up to 200 ms
 * for the block to start, else timeout.
 *-----------------------------------------------------------------------*/
static int _ff_spi_rxDataBlk (	/* 1:OK, 0:Error */
	_FF_VOLUME_OBJ_ST*	pThisObj,
	BYTE *buff,			/* Data buffer */
	UINT btr			/* Data block length (byte) */
)
{

	#define _TIM_MS_WAIT_			(200)	// [ms]
	BYTE 		token;
	uint16_t	tMsCnt	= 0;

	// Wait up to '_TIM_MS_WAIT_' ms for rx data block
	do
	{	// Wait for DataStart token in timeout of _TIM_MS_WAIT_ [ms]
		token = _ff_spi_txAndRx(pThisObj, _DEF_TX_DUMMY_VALUE);
	} while ((token == 0xFF) && (tMsCnt++ < _TIM_MS_WAIT_) && (osOK == osDelay(pdMS_TO_TICKS(1))));

	if (token != _TOKEN_STRT_DATA_BLOCK_)
	{
		__ff_printfDebug(pThisObj, "%s.%d: timeout - token=%0x02X\n", __FUNCTION__, __LINE__, token);
		return 0;		// Function fails if invalid DataStart token or timeout
	}

	_ff_spi_rxMultiple(pThisObj, buff, btr);		// Store trailing data to the buffer
	_ff_spi_txAndRx(pThisObj, _DEF_TX_DUMMY_VALUE); _ff_spi_txAndRx(pThisObj, _DEF_TX_DUMMY_VALUE);			// Discard CRC
	return 1;						// Function succeeded

}

/*-----------------------------------------------------------------------*/
/* Send a data packet to the Micro SD Card, waiting up to 500 ms to
 * begin transmission, waiting for the card to be ready to accept the data
 *-----------------------------------------------------------------------*/

#if _USE_WRITE
static int _ff_spi_txDataBlk (	/* 1:OK, 0:Failed */
	_FF_VOLUME_OBJ_ST*	pThisObj,
	const BYTE *buff,	/* Pointer to 512 byte data to be sent */
	BYTE token			/* Token */
)
{

	BYTE resp;

	if (!_ff_wait_sdCardReady(pThisObj, 500))
	{
		__ff_printfDebug(pThisObj, "%s.%d: Not Ready\n", __FUNCTION__, __LINE__);
		return 0;		/* Wait for card ready */
	}

	_ff_spi_txAndRx(pThisObj, token);					/* Send token */
	if (token != _TOKEN_STOP_DATA_)
	{				/* Send data if token is other than StopTran */
		_ff_spi_txMultiple(pThisObj, buff, _DEF_MAX_DATA_BLOCK_SZ);		/* Data */
		_ff_spi_txAndRx(pThisObj, _DEF_TX_DUMMY_VALUE); _ff_spi_txAndRx(pThisObj, _DEF_TX_DUMMY_VALUE);	/* Dummy CRC */

		resp = _ff_spi_txAndRx(pThisObj, _DEF_TX_DUMMY_VALUE);				/* Receive data resp */
		//@todo JRB - update 0x1F mask and 0x05 response
		if ((resp & 0x1F) != 0x05)		/* Function fails if the data packet was not accepted */
		{
			__ff_printfDebug(pThisObj, "%s.%d: packet not accepted\n", __FUNCTION__, __LINE__);
			return 0;
		}
	}
	return 1;
}
#endif


/*-----------------------------------------------------------------------*/
/* Send a command packet to the MMC                                      */
/*-----------------------------------------------------------------------*/

static BYTE _ff_spi_txCommand (		/* Return value: R1 resp (bit7==1:Failed to send) */
	_FF_VOLUME_OBJ_ST*	pThisObj,
	BYTE cmd,		/* Command index */
	DWORD arg		/* Argument */
)
{
	BYTE n, res;

	if (cmd & 0x80) {	/* Send a CMD55 prior to ACMD<n> */
		cmd &= 0x7F;
		res = _ff_spi_txCommand(pThisObj, CMD55, 0);
		if (res > 1){
			__ff_printfDebug(pThisObj, "%s.%d: Failure\n", __FUNCTION__, __LINE__);
			return res;
		}
	}

	/* Select the card and wait for ready except to stop multiple block read */
	if (cmd != CMD12)
	{
		_ff_sdCardCS_setInactive(pThisObj);
		if (!_ff_sdCardCS_setActive(pThisObj))
		{
			__ff_printfDebug(pThisObj, "%s.%d: Failure\n", __FUNCTION__, __LINE__);
			return 0xFF;
		}
	}

	/* Send command packet */
	_ff_spi_txAndRx(pThisObj, 0x40 | cmd);				/* Start + command index */
	_ff_spi_txAndRx(pThisObj, (BYTE)(arg >> 24));		/* Argument[31..24] */
	_ff_spi_txAndRx(pThisObj, (BYTE)(arg >> 16));		/* Argument[23..16] */
	_ff_spi_txAndRx(pThisObj, (BYTE)(arg >> 8));			/* Argument[15..8] */
	_ff_spi_txAndRx(pThisObj, (BYTE)arg);				/* Argument[7..0] */
	n = 0x01;										/* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;						/* Valid CRC for CMD0(0) */
	if (cmd == CMD8) n = 0x87;						/* Valid CRC for CMD8(0x1AA) */
	_ff_spi_txAndRx(pThisObj, n);

	/* Receive command resp */
	if (cmd == CMD12) {
		_ff_spi_txAndRx(pThisObj, _DEF_TX_DUMMY_VALUE);					/* Discard following one byte when CMD12 */
	}

	n = 10;								/* Wait for response (10 bytes max) */
	do {
		res = _ff_spi_txAndRx(pThisObj, _DEF_TX_DUMMY_VALUE);		// JRB: Send 0xFF (i.e. want clock to rx byte as return value from _ff_spi_txAndRx())
	} while ((res & 0x80) && --n);
	if (0 == n)
	{
		__ff_printfDebug(pThisObj, "%s.%d: Failure..timeout\n", __FUNCTION__, __LINE__);
	}
	return res;							/* Return received response */
}

static inline uint8_t _ff_sdCard_isWrEnabled(_FF_VOLUME_OBJ_ST*	pThisObj)
{
	return 1;
}

/*
 * Return:
 * 1: Success
 * 0: Failure
 */
int ff_mutex_create (int vol)
{
	static const osMutexAttr_t	osMutexAttr =
	{
		.name		= "FatFs",
		.attr_bits	= osMutexPrioInherit,
		.cb_mem		= NULL,
		.cb_size	= 0,
	};
	/* Create a sync object */
	_FF_VOLUME_OBJ_ST*	pThisObj = &thisObj[vol];
	if (vol >= FF_VOLUMES)		{ return 0; }
	if (pThisObj->mutex)	{ return 0; }
	pThisObj->mutex	= osMutexNew(&osMutexAttr);
	return (pThisObj->mutex) ? 1 : 0;
}
void ff_mutex_delete (int vol)
{
	/* Delete a sync object */
	if (vol >= FF_VOLUMES) 				{ return; }
	_FF_VOLUME_OBJ_ST*	pThisObj = &thisObj[vol];
	if (pThisObj->mutex)
	{
		osMutexDelete(pThisObj->mutex);
		pThisObj->mutex = 0;
	}
	return;
}
/*
 * Return:
 * 1: Success
 * 0: Failure/Timeout
 */
int ff_mutex_take (int vol)
{
	osStatus_t	osStatus;
	if (vol >= FF_VOLUMES) 				{ return 0; }
	_FF_VOLUME_OBJ_ST*	pThisObj = &thisObj[vol];
	if (NULL == pThisObj->mutex)	{ return 0; }
	/* Lock sync object */
	osStatus = osMutexAcquire (pThisObj->mutex, FF_FS_TIMEOUT);	// FF_FS_TIMEOUT defined in os tick time (as opposed to ms ... hence no pdMS_TO_TICKS())
	return (osOK == osStatus) ? 1 : 0;
}

void ff_mutex_give (int vol)
{
	/* Unlock sync object */
	if (vol >= FF_VOLUMES) 				{ return; }
	_FF_VOLUME_OBJ_ST*	pThisObj = &thisObj[vol];
	if (pThisObj->mutex)
	{
		osMutexRelease(pThisObj->mutex);
	}
}


/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */

	if (pdrv >= FF_VOLUMES) return RES_PARERR;
	_FF_VOLUME_OBJ_ST*	pThisObj	= &thisObj[pdrv];

	BYTE n, cmd, ty, ocr[4];
	uint8_t	rVal;

	if (pThisObj->sdCardIsInit) { return pThisObj->sdCardStatus; }

	// BEGIN: ioctl() NOTE: If you need to support more than one volume, you could update ioctl() to set the SPI port,
	// associated CS(port, pin), and BR settings. I did not do this, as it seems a bit easier to use and I expect most
	// won't have more than one Micro SD card. That said, I believe the code is largely set up to easily add support
	// for additional SD cards.
	pThisObj->pSPI			= DEF_FF_SPI_PORT;
	pThisObj->sdCardCsPort	= DEF_FF_SD_CARD_CS_PORT;
	pThisObj->sdCardCsPin	= DEF_FF_SD_CARD_CS_PIN;
	pThisObj->spiClkBrSlow	= DEF_FF_SPI_CLCK_BR_SLOW;
	pThisObj->spiClkBrFast	= DEF_FF_SPI_CLCK_BR_FAST;
	// END: ioctl() NOTE

	//Initialize CS pin
	{
		//@todo JRB pre-initialization, set SPI clock to low frequency
		// See SD Card Association document "Part 1 Physical Layer Simplified", section
		// 4.4 Clock Control. At SD card initialization, the clock shall be in the
		// range of 100KHz to 400KHz. Post initialization, the clock can be set to a
		// higher value (@todo not sure how to determine this number on the fly just yet).
		// Regardless, post ACMD41, the clock speed can be increased.
		uint16_t	regCR2 	= pThisObj->pSPI->CR2;
		pThisObj->pSPI->CR2	= 0;
		pThisObj->pSPI->CR1	&= 	~SPI_CR1_BR_Msk;		// clear previous Baud Rate Control settings
		pThisObj->pSPI->CR1	|= pThisObj->spiClkBrSlow << SPI_CR1_BR_Pos;			// BR=6	=> 42 MHz / 128 = 0.33KHz
		pThisObj->pSPI->CR2	= regCR2;
		osDelay(10);
	}

//	TM_FATFS_InitPins();
	_ff_spi_init(pThisObj);		// JRB: Leaves CS High (i.e. not enabled)


	// JRB: See NEC Applicatin Note "SM Memory Card Interface Using SPI", section 2.2.3.1
	// SD/MMC Initialization. Which in part states "...deselecting card and sending 10 pad characters..."
	for (n = 10; n; n--) {
		_ff_spi_txAndRx(pThisObj, _DEF_TX_DUMMY_VALUE);
	}

	ty = 0;
	// JRB: Send CMD0:	0x40 0x00000000 0x95		where <0x40 | 0x00> is command with 32-bit 0 value & CRC (0x95); SD card responds with 0x01
	// The response is of type R1, where R1 is single octet
	// bit0		in idle state
	// bit1		Erase Reset
	// bit2		Illegal Command
	// bit3		Command CRC Error
	// bit4		Erase Sequence Error
	// bit5		Address Error
	// bit6		Parameter Error
	// bit7		0
	//
	// CMD0 puts card into idle/reset state. Attempt CMD0 2x times.
	// If we fail to reach idle state, then return with error.
#define _DEF_R1_SUCCESS			(0x00)		// Success; non-Idle State
#define _DEF_R1_SUCCESS_IDLE	(0x01)		// Success: Idle State
#define _DEF_R1_MSK_TIMEOUT		(0x80)		// No response/timeout
#define _DEF_R1_MSK_ERROR		(0x7E)		// one-or-more error conditions
	rVal = _ff_spi_txCommand(pThisObj, CMD0, 0);
	if (_DEF_R1_SUCCESS == rVal)
	{	// Success, but not IDLE STATE; retry giving the card a bit more time.
		// JRB NOTE: I tried two different brands of Micro SD cards, and they both
		// failed to obtain IDLE state on first attempt. Subsequently they did
		// report IDLE state on second attempt, even without the 1ms delay, which
		// I still included as a safety margin.
		osDelay(pdMS_TO_TICKS(1));
		rVal = _ff_spi_txCommand(pThisObj, CMD0, 0);
	}
	if (_DEF_R1_SUCCESS_IDLE == rVal)
	{	/* Put the card SPI/Idle state */
		uint16_t	tDelayCnt	= 0;
		if (_ff_spi_txCommand(pThisObj, CMD8, 0x1AA) == 1)
		{	/* SDv2? */
			for (n = 0; n < 4; n++)
			{
				ocr[n] = _ff_spi_txAndRx(pThisObj, _DEF_TX_DUMMY_VALUE);	/* Get 32 bit return value of R7 resp */
			}
			if (ocr[2] == 0x01 && ocr[3] == 0xAA)
			{
				/* Is the card supports vcc of 2.7-3.6V? */
				while ((tDelayCnt++ < 1000) && (osOK == osDelay(pdMS_TO_TICKS(1))) && _ff_spi_txCommand(pThisObj, ACMD41, 1UL << 30)) ;	/* Wait for end of initialization with ACMD41(HCS) */
				if ((osOK == osDelay(pdMS_TO_TICKS(2))) && (_ff_spi_txCommand(pThisObj, CMD58, 0) == 0))
				{		/* Check CCS bit in the OCR */
					for (n = 0; n < 4; n++)
					{
						ocr[n] = _ff_spi_txAndRx(pThisObj, _DEF_TX_DUMMY_VALUE);
					}
					ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* Card id SDv2 */
				}
			}
		} else
		{	/* Not SDv2 card */
			if (_ff_spi_txCommand(pThisObj, ACMD41, 0) <= 1)
			{	/* SDv1 or MMC? */
				ty = CT_SD1; cmd = ACMD41;	/* SDv1 (ACMD41(0)) */
			} else
			{
				ty = CT_MMC; cmd = CMD1;	/* MMCv3 (CMD1(0)) */
			}
			while ((tDelayCnt++ < 1000) && (osOK == osDelay(pdMS_TO_TICKS(1))) && _ff_spi_txCommand(pThisObj, cmd, 0)) ;	/* Wait for end of initialization with ACMD41(HCS) */
			if ((osOK == osDelay(pdMS_TO_TICKS(2))) || _ff_spi_txCommand(pThisObj, CMD16, _DEF_MAX_DATA_BLOCK_SZ) != 0)
			{	/* Set block length: 512 */
				ty = 0;
			}
		}
	}
	else
	{
		__ff_printfDebug(pThisObj, "%s.%d _ff_spi_txCommand(CMD0,0) R1=0x%02X, expected 0x01\n", __FUNCTION__, __LINE__, rVal);
	}
	pThisObj->sdCardType = ty;	/* Card type */
	_ff_sdCardCS_setInactive(pThisObj);

	if (ty) {			/* OK */
		pThisObj->sdCardStatus &= ~STA_NOINIT;	/* Clear STA_NOINIT flag */
	} else {			/* Failed */
		__ff_printfDebug(pThisObj, "%s.%d Init failed (ty=%d)\n", __FUNCTION__, __LINE__, ty);
		pThisObj->sdCardStatus = STA_NOINIT;
	}

	if (!_ff_sdCard_isWrEnabled(pThisObj)) {
		pThisObj->sdCardStatus |= STA_PROTECT;
	} else {
		pThisObj->sdCardStatus &= ~STA_PROTECT;
	}
	{	// Set SPI clock since card initialization is complete
		// See STMicroelectronic RM0368 Reference Manual, section 20.5.1 SPI Control Register 1 for clock speed
		uint16_t	regCR2 = pThisObj->pSPI->CR2;
		pThisObj->pSPI->CR2	= 0;
		pThisObj->pSPI->CR1 &= 	~SPI_CR1_BR_Msk;		// clear previous Baud Rate Control settings
#if (USE_LESSER_CLKSPD_LOGIC_ANALYZER)
#warning Running at slower clock to better work with my logic analyzer
		pThisObj->pSPI->CR1 	|= 	SPI_CR1_BR_1;			// BR=0x2	=> 42 MHz / 8 = 5.25 MHz
#else
		pThisObj->pSPI->CR1	|= (pThisObj->spiClkBrFast) << SPI_CR1_BR_Pos;			// BR=1	=> 42 MHz / 4 = 10.5 MHz
#endif
		pThisObj->pSPI->CR2	= regCR2;
		osDelay(10);
	}
	if (0 == pThisObj->sdCardStatus)
	{
		pThisObj->sdCardIsInit	= 1;
	}
	return pThisObj->sdCardStatus;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
	if (pdrv >= FF_VOLUMES) return RES_PARERR;
	_FF_VOLUME_OBJ_ST*	pThisObj	= &thisObj[pdrv];

	/* Check if write is enabled */
	if (!_ff_sdCard_isWrEnabled(pThisObj)) {
		pThisObj->sdCardStatus |= STA_PROTECT;
	} else {
		pThisObj->sdCardStatus &= ~STA_PROTECT;
	}

	return pThisObj->sdCardStatus;	/* Return disk status */
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
	if (pdrv >= FF_VOLUMES) return RES_PARERR;
	_FF_VOLUME_OBJ_ST*	pThisObj	= &thisObj[pdrv];
	if (pThisObj->sdCardStatus & STA_NOINIT)
	{
		__ff_printfDebug(pThisObj, "%s.%d: Error No Init\n", __FUNCTION__, __LINE__);
		return RES_NOTRDY;
	}

	if (!(pThisObj->sdCardType & CT_BLOCK)) {
		sector *= _DEF_MAX_DATA_BLOCK_SZ;	/* LBA ot BA conversion (byte addressing cards) */
	}

	if (count == 1) {	/* Single sector read */
		if ((_ff_spi_txCommand(pThisObj, CMD17, sector) == 0)	/* READ_SINGLE_BLOCK */
			&& _ff_spi_rxDataBlk(pThisObj, buff, _DEF_MAX_DATA_BLOCK_SZ))
			count = 0;
	} else {				/* Multiple sector read */
		if (_ff_spi_txCommand(pThisObj, CMD18, sector) == 0) {	/* READ_MULTIPLE_BLOCK */
			do {
				if (!_ff_spi_rxDataBlk(pThisObj, buff, _DEF_MAX_DATA_BLOCK_SZ)) {
					break;
				}
				buff += _DEF_MAX_DATA_BLOCK_SZ;
			} while (--count);
			_ff_spi_txCommand(pThisObj, CMD12, 0);				/* STOP_TRANSMISSION */
		}
	}
	_ff_sdCardCS_setInactive(pThisObj);

	return count ? RES_ERROR : RES_OK;	/* Return result */
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
  /* USER CODE HERE */
	if (pdrv >= FF_VOLUMES) return RES_PARERR;
	_FF_VOLUME_OBJ_ST*	pThisObj	= &thisObj[pdrv];

	if (!_ff_sdCard_isWrEnabled(pThisObj)) {
		__ff_printfDebug(pThisObj, "%s.%d: Write Protected\n", __FUNCTION__, __LINE__);
		return RES_WRPRT;
	}
	if (pThisObj->sdCardStatus & STA_NOINIT) {
		__ff_printfDebug(pThisObj, "%s.%d: No Init\n", __FUNCTION__, __LINE__);
		return RES_NOTRDY;	/* Check drive status */
	}
	if (pThisObj->sdCardStatus & STA_PROTECT) {
		__ff_printfDebug(pThisObj, "%s.%d: Failure\n", __FUNCTION__, __LINE__);
		return RES_WRPRT;	/* Check write protect */
	}

	if (!(pThisObj->sdCardType & CT_BLOCK)) {
		sector *= _DEF_MAX_DATA_BLOCK_SZ;	/* LBA ==> BA conversion (byte addressing cards) */
	}

	if (count == 1) {	/* Single sector write */
		if ((_ff_spi_txCommand(pThisObj, CMD24, sector) == 0)	/* WRITE_BLOCK */
			&& _ff_spi_txDataBlk(pThisObj, buff, 0xFE))
			count = 0;
	} else {				/* Multiple sector write */
		if (pThisObj->sdCardType & CT_SDC) _ff_spi_txCommand(pThisObj, ACMD23, count);	/* Predefine number of sectors */
		if (_ff_spi_txCommand(pThisObj, CMD25, sector) == 0) {	/* WRITE_MULTIPLE_BLOCK */
			do {
				if (!_ff_spi_txDataBlk(pThisObj, buff, 0xFC))
				{
					__ff_printfDebug(pThisObj, "%s.%d: Failure\n", __FUNCTION__, __LINE__);
					break;
				}
				buff += _DEF_MAX_DATA_BLOCK_SZ;
			} while (--count);
			if (!_ff_spi_txDataBlk(pThisObj, 0, 0xFD)) {	/* STOP_TRAN token */
				__ff_printfDebug(pThisObj, "%s.%d: Failure\n", __FUNCTION__, __LINE__);
				count = 1;
			}
		}
	}
	_ff_sdCardCS_setInactive(pThisObj);

	return (count) ? RES_ERROR : RES_OK;	/* Return result */
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
	if (pdrv >= FF_VOLUMES) return RES_PARERR;
	_FF_VOLUME_OBJ_ST*	pThisObj	= &thisObj[pdrv];
#if _USE_IOCTL != 1
#error Requires use of IOCTL to support a generic Micro SD Card
#endif

	DRESULT res;
	BYTE n, csd[16];
//	DWORD *dp, st, ed, csize;
	DWORD csize;

	res = RES_ERROR;

	switch (cmd)
	{
		case CTRL_EJECT:
			extern Disk_drvTypeDef  disk;
			pThisObj->sdCardIsInit		= 0;
			disk.is_initialized[pdrv]	= 0;
			res	= FR_OK;
			break;
		case CTRL_DEBUG:
			if (buff)
			{
				pThisObj->fIsDebugEn	= (*(uint8_t*)buff) ? 1 : 0;
				res = FR_OK;
			}
			break;
		default:
		{
			if (pThisObj->sdCardStatus & STA_NOINIT) {	return RES_NOTRDY;	}
			switch (cmd)
			{
			case CTRL_SYNC :		/* Wait for end of internal write process of the drive */
				if (_ff_sdCardCS_setActive(pThisObj)) res = RES_OK;
				break;
			case GET_SECTOR_SIZE:	/* Get sector size */
				*(DWORD*)buff  = 512;	// Sector size; @todo is there a way to get this from the card?
				res = RES_OK;
				break;
			case GET_SECTOR_COUNT :	/* Get drive capacity in unit of sector (DWORD) */
				if ((_ff_spi_txCommand(pThisObj, CMD9, 0) == 0) && _ff_spi_rxDataBlk(pThisObj, csd, 16)) {
					if ((csd[0] >> 6) == 1) {	/* SDC ver 2.00 */
						csize = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
						*(DWORD*)buff = csize << 10;
					} else {					/* SDC ver 1.XX or MMC ver 3 */
						n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
						csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
						*(DWORD*)buff = csize << (n - 9);
					}
					res = RES_OK;
				}
				break;

			case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
				if (pThisObj->sdCardType & CT_SD2) {	/* SDC ver 2.00 */
					if (_ff_spi_txCommand(pThisObj, ACMD13, 0) == 0) {	/* Read SD status */
						_ff_spi_txAndRx(pThisObj, _DEF_TX_DUMMY_VALUE);
						if (_ff_spi_rxDataBlk(pThisObj, csd, 16)) {				/* Read partial block */
							for (n = 64 - 16; n; n--) _ff_spi_txAndRx(pThisObj,  _DEF_TX_DUMMY_VALUE);	/* Purge trailing data */
							*(DWORD*)buff = 16UL << (csd[10] >> 4);
							res = RES_OK;
						}
					}
				} else {					/* SDC ver 1.XX or MMC */
					if ((_ff_spi_txCommand(pThisObj, CMD9, 0) == 0) && _ff_spi_rxDataBlk(pThisObj, csd, 16)) {	/* Read CSD */
						if (pThisObj->sdCardType & CT_SD1) {	/* SDC ver 1.XX */
							*(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
						} else {					/* MMC */
							*(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
						}
						res = RES_OK;
					}
				}
				break;
			default:
				res = RES_PARERR;
				break;
			}
		}
	}

	_ff_sdCardCS_setInactive(pThisObj);

	return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

