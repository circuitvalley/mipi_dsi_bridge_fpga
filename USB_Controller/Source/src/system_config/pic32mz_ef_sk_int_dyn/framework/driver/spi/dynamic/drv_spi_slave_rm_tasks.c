/*******************************************************************************
  SPI Driver Functions for Dynamic Standard Buffer Driver Tasks Functions

  Company:
    Microchip Technology Inc.

  File Name:
    drv_spi_dynamic_slave_rm_tasks.c

  Summary:
    SPI driver tasks functions

  Description:
    The SPI device driver provides a simple interface to manage the SPI
    modules on Microchip microcontrollers. This file contains implemenation
    for the SPI driver.
    
  Remarks:
  This file is generated from framework/driver/spi/template/drv_spi_dynamic_slave_rm_tasks.c.ftl
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//DOM-IGNORE-END

#include "driver/spi/src/dynamic/drv_spi_internal.h"
#include <stdbool.h>


            
int32_t DRV_SPI_SlaveRMSend8BitISR( struct DRV_SPI_DRIVER_OBJECT * pDrvInstance )
{
    register SPI_MODULE_ID spiId = pDrvInstance->spiId;
    register DRV_SPI_JOB_OBJECT * currentJob = pDrvInstance->currentJob;
    if (currentJob == NULL)
    {
        return 0;
    }
    if (!PLIB_SPI_TransmitBufferIsEmpty(spiId))
    {
        return 0;
    }

    // Add in DMA stuff here

    PLIB_SPI_BufferWrite(spiId, currentJob->txBuffer[currentJob->dataTxed]);
    currentJob->dataTxed++;
    currentJob->dataLeftToTx--;

    SYS_INT_SourceStatusClear(pDrvInstance->txInterruptSource);
    return 0;


}

int32_t DRV_SPI_SlaveRMReceive8BitISR( struct DRV_SPI_DRIVER_OBJECT * pDrvInstance )
{
    register SPI_MODULE_ID spiId = pDrvInstance->spiId;
    register DRV_SPI_JOB_OBJECT * currentJob = pDrvInstance->currentJob;

    if (currentJob->dataLeftToRx + currentJob->dummyLeftToRx == 0)
    {
        return 0;
    }

    if (!PLIB_SPI_ReceiverBufferIsFull(spiId))
    {
        return 0;
    }

    // Put in DMA stuff here
    if (currentJob->dataLeftToRx != 0)
    {
        currentJob->rxBuffer[currentJob->dataRxed] = PLIB_SPI_BufferRead(spiId);
        currentJob->dataRxed++;
        currentJob->dataLeftToRx--;
    }
    else
    {
        PLIB_SPI_BufferRead(spiId);
        currentJob->dummyLeftToRx--;
    }

    if (currentJob->dataLeftToRx + currentJob->dummyLeftToRx == 0)
    {
        SYS_INT_SourceDisable(pDrvInstance->rxInterruptSource);
        SYS_INT_SourceEnable(pDrvInstance->txInterruptSource);
    }
    SYS_INT_SourceStatusClear(pDrvInstance->rxInterruptSource);
    return 0;

}


