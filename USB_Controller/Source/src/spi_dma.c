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

/* This is the register set structure of USART module */

#include <xc.h>
#include <sys/kmem.h>
#include "system/common/sys_common.h"
#include "spi_dma.h"
#include "system/int/sys_int.h"
#include "system_config/pic32mz_ef_sk_int_dyn/framework/peripheral/dma/templates/dma_Registers.h"
#include "app.h"

#define ConvertToPhysicalAddress(a) ((uint32_t)KVA_TO_PA(a))

display_line_t *display_spi_line;
static uint8_t dummy[1024];
void init_spi_dma(SPI_MODULE_ID index) 
{

    spi_registers_t volatile * spi = ((spi_registers_t *)(index));
    spi->SPIxCON.FRMEN = 0;         //disable frame
    spi->SPIxCON.FRMSYNC = 0;   
    spi->SPIxCON.FRMPOL = 0;        //SS pin active low
    spi->SPIxCON.MSSEN = 0;         //SS disable
    spi->SPIxCON.ENHBUF = 0;
    spi->SPIxCON.MCLKSEL = 0;       //1:master clock as source 0: PCLK
    spi->SPIxCON.DISSDO  = 0;       //sdo enable
    spi->SPIxCON.MODE16 = 0;        //8bit mode
    spi->SPIxCON.MODE32 = 0;        //8bit mode
    spi->SPIxCON.SMP = 0;           //input sample at middle of data output
    spi->SPIxCON.CKE = 1;           //output change on falling edge
    spi->SPIxCON.CKP = 0;           //idle state is low
    spi->SPIxCON.MSTEN = 1;         //master mode enable
    spi->SPIxCON.DISSDI = 0;        //SDI en 
    spi->SPIxCON.STXISEL = 1;       //TXIF sets when buffer is empty
    spi->SPIxCON.SRXISEL = 1;       //RXIF sets when buffer is  full
    spi->SPIxCON2.AUDEN = 0;       //disable i2s 
    
    spi->SPIxBRG = 0;               // divide by 4
    spi->SPIxCON.ON = 1;

    IEC4bits.SPI2RXIE = 0; //SPI RX interrupt only need when last byte.

    volatile dma_ch_register_t *dma_regs_rx = (dma_ch_register_t *)(_DMAC_BASE_ADDRESS + sizeof(dma_register_t) + DMA_CHANNEL_3 * sizeof(dma_ch_register_t));

    dma_regs_rx->DCHxCON.CHEN  = 0;
    dma_regs_rx->DCHxCON.CHPRI = 2;   //channel priority
    dma_regs_rx->DCHxCON.CHCHN = 0;
    dma_regs_rx->DCHxCON.CHAEN = 0;     //auto re-enable to do dummy rx 
    
    dma_regs_rx->DCHxECON.CHSIRQ = DMA_TRIGGER_SPI_2_RECEIVE;
    dma_regs_rx->DCHxECON.SIRQEN = 1; //enable interrupt cell tranfser
    
    dma_regs_rx->DCHxINT.CHDDIE = 1; //destination done interrupt enable flag CHSDIF

    dma_regs_rx->DCHxDSA = ConvertToPhysicalAddress((uint32_t)&(dummy));        //destination address
    dma_regs_rx->DCHxSSA = ConvertToPhysicalAddress((uint32_t)&(spi->SPIxBUF)); //source start address
    dma_regs_rx->DCHxCSIZ = 1; //one byte transfer on trigger_source event
    //dma_regs_rx->DCHxDSIZ = 1; //destination size 1 byte            //destination size
    dma_regs_rx->DCHxSSIZ = 1; //destination size 1 byte            //source size

    
    
    volatile dma_ch_register_t *dma_regs_tx = (dma_ch_register_t *)(_DMAC_BASE_ADDRESS + sizeof(dma_register_t) + DMA_CHANNEL_2 * sizeof(dma_ch_register_t));

    dma_regs_tx->DCHxCON.CHEN  = 0;
    dma_regs_tx->DCHxCON.CHPRI = 2;   //channel priority
    dma_regs_tx->DCHxCON.CHCHN = 0;

    dma_regs_tx->DCHxECON.CHSIRQ = DMA_TRIGGER_SPI_2_RECEIVE;
    dma_regs_tx->DCHxECON.SIRQEN = 1; //enable interrupt cell tranfser
    
    dma_regs_tx->DCHxINT.CHSDIE = 1; //source done interrupt enable flag CHSDIF

    dma_regs_tx->DCHxDSA = ConvertToPhysicalAddress((uint32_t)&(spi->SPIxBUF));
    dma_regs_tx->DCHxCSIZ = 1; //one byte transfer on trigger_source event
    dma_regs_tx->DCHxDSIZ = 1; //destination size 1 byte
    
    
    SYS_INT_VectorPrioritySet(_DMA3_VECTOR, INT_PRIORITY_LEVEL4);
    
    SYS_INT_VectorSubprioritySet(_DMA3_VECTOR, INT_SUBPRIORITY_LEVEL0);
    
    SYS_INT_VectorPrioritySet(_DMA2_VECTOR, INT_PRIORITY_LEVEL4);
    
    SYS_INT_VectorSubprioritySet(_DMA2_VECTOR, INT_SUBPRIORITY_LEVEL0);
    
    SYS_INT_VectorPrioritySet(_SPI2_RX_VECTOR, INT_PRIORITY_LEVEL4);    //SPI 2 Rx interrupt is need to clear CS (last byte of transaction.)
    
    SYS_INT_VectorSubprioritySet(_SPI2_RX_VECTOR, INT_SUBPRIORITY_LEVEL0);
    
    DMACONbits.SUSPEND = 0;
    LATDbits.LATD10 = 1;
    DMACONbits.ON = 1;
}

void transfer_spi_dma(display_line_t *display_line)
{
    volatile dma_ch_register_t *dma_regs_tx = (dma_ch_register_t *)(_DMAC_BASE_ADDRESS + sizeof(dma_register_t) + DMA_CHANNEL_2 * sizeof(dma_ch_register_t));
    volatile dma_ch_register_t *dma_regs_rx = (dma_ch_register_t *)(_DMAC_BASE_ADDRESS + sizeof(dma_register_t) + DMA_CHANNEL_3 * sizeof(dma_ch_register_t));


    LATDbits.LATD10 = 0;    //cs active
    
    display_spi_line = display_line;
    
    display_spi_line->spi_busy = true;
    dma_regs_tx->DCHxCON.CHEN = 0;
    dma_regs_tx->DCHxSSA = ConvertToPhysicalAddress((uint32_t)&(display_line->command));
 
    dma_regs_tx->DCHxSSIZ = display_spi_line->available_length + 1;
    dma_regs_tx->DCHxINT.CHSDIF = 0;   //clear source done
    
    //IEC4bits.DMA2IE = 1;    //Dma interrupt will result in call of DmaCh2 ISR
    dma_regs_rx->DCHxCON.CHEN = 0;
    dma_regs_rx->DCHxDSIZ = display_spi_line->available_length + 1; //destination size for rx
    dma_regs_rx->DCHxINT.CHSHIF = 0;   //clear Destination done

    IEC4bits.DMA3IE = 1;    //Dma interrupt will result in call of DmaCh3 ISR
    dma_regs_rx->DCHxCON.CHEN = 1;
    dma_regs_tx->DCHxCON.CHEN = 1;
    dma_regs_tx->DCHxECON.CFORCE = 1;
    
}

void __ISR(_DMA3_VECTOR, ipl4AUTO) _IntHandlerSysDmaCh3(void)
{
    volatile dma_ch_register_t *dma_regs_rx = (dma_ch_register_t *)(_DMAC_BASE_ADDRESS + sizeof(dma_register_t) + DMA_CHANNEL_3 * sizeof(dma_ch_register_t));
    LATDbits.LATD10 = 1;    //cs deactivate
    dma_regs_rx->DCHxINT.CHDDIF = 0; //source done interrupt enable flag CHSDIF
    IFS4bits.DMA3IF = 0;

    display_spi_line->available_length = 0; 
    display_spi_line->spi_busy = false; 
}


/* *****************************************************************************
 End of File
 */