/*******************************************************************************
    Copy Right:  Gaurav Singh
    website: www.circuitvalley.com 
    Created on July 12, 2018
    
    This file is part of Circuitvalley USB Display .
    Circuitvalley USB Display is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    Circuitvalley USB USB Display  is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with Circuitvalley USB Display .  If not, see <http://www.gnu.org/licenses/>.
*******************************************************************************/


#ifndef _SPI_DMA_H_    /* Guard against multiple inclusion */
#define _SPI_DMA_H_
#include "app.h"
#include "peripheral/dma/processor/dma_processor.h"
typedef enum {

    SPI_ID_1 = _SPI1_BASE_ADDRESS,
    SPI_ID_2 = _SPI2_BASE_ADDRESS,
    SPI_ID_3 = _SPI3_BASE_ADDRESS,
    SPI_ID_4 = _SPI4_BASE_ADDRESS,
    SPI_NUMBER_OF_MODULES = 4

} SPI_MODULE_ID;

typedef struct __attribute__((packed , aligned(4)))
{
    __SPI2CONbits_t           SPIxCON;
    volatile unsigned int     SPIxCONCLR;
    volatile unsigned int     SPIxCONSET;
    volatile unsigned int     SPIxCONINV;
    __SPI2STATbits_t          SPIxSTAT;
    volatile unsigned int     SPIxSTATCLR;
    volatile unsigned int     SPIxSTATSET;
    volatile unsigned int     SPIxSTATINV;
    volatile unsigned int     SPIxBUF;
    volatile unsigned int     offset1[3];
    volatile unsigned int     SPIxBRG;
#ifdef _SPI2CON2_w_MASK
    volatile unsigned int     offset2[3];
    __SPI2CON2bits_t          SPIxCON2;
    volatile unsigned int     SPIxCON2CLR;
    volatile unsigned int     SPIxCON2SET;
    volatile unsigned int     SPIxCON2INV;
#endif
 
} spi_registers_t;


void init_spi_dma(SPI_MODULE_ID index );

void transfer_spi_dma(display_line_t *display_line);

#endif 

/* *****************************************************************************
 End of File
 */
