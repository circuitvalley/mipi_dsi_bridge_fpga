/***********************************************************************
External Bus Interface (EBI) Peripheral Library Interface

  Company:
    Microchip Technology Inc.
    
  File Name:
    plib_ebi.h
    
  Summary:
    EBI Peripheral Library Interface Header.
    
  Description:    
    This header file contains the function prototypes and definitions of
    the data types and constants that make up the interface to the EBI
    Peripheral Library.
  ***********************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright 2013-2015 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

#ifndef _PLIB_EBI_H
#define _PLIB_EBI_H

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Includes
// *****************************************************************************
// *****************************************************************************

#include "peripheral/ebi/processor/ebi_processor.h"

// *****************************************************************************
// *****************************************************************************
// Section: EBI Peripheral Library Exists Routines
// *****************************************************************************
// *****************************************************************************


/*******************************************************************************
  Function:
    PLIB_EBI_BaseAddressSet(EBI_MODULE_ID index, int ChipSelectNumber, 
	                        uint32_t BaseAddress)
    
  Summary:
    Sets the base address for physical memory at each Chip Select pin.
	
  Description:
    This function sets the base address for physical memory at each Chip
    Select pin.
	
  Precondition:
    None.
	
  Parameters:
    index -             Identifier for the device instance
    ChipSelectNumber -  Identifies which Chip Select address pin is being
                        assigned an address
    BaseAddress -       A physical address for memory
	
  Returns:
    None.
	
  Remarks:
    None.
	
*/
void PLIB_EBI_BaseAddressSet(EBI_MODULE_ID index, int ChipSelectNumber, 
                             uint32_t BaseAddress);

/*******************************************************************************
  Function:
    PLIB_EBI_BaseAddressGet (EBI_MODULE_ID index, int ChipSelectNumber)
    
  Summary:
    Returns the base address set for each Chip Select.
	
  Description:
    This function returns the base address for each Chip Select pin.
	
  Precondition:
    None.  
  
  Parameters:
    index -             Identifier for the device instance
    ChipSelectNumber -  Identifies which Chip Select address pin address is
                        being read
						
  Returns:
    Returns an unsigned integer that is the physical address of the
    attached device.
	
  Remarks:
    None.                                                                  
*/
uint32_t PLIB_EBI_BaseAddressGet (EBI_MODULE_ID index, int ChipSelectNumber);

/*******************************************************************************
  Function:
    PLIB_EBI_MemoryCharacteristicsSet(EBI_MODULE_ID index, int ChipSelectNumber, 
	                                  EBI_MEMORY_TYPE MemoryType, 
									  EBI_MEMORY_SIZE MemorySize, 
									  EBI_CS_TIMING TimingReg)
    
  Summary:
    Sets the characteristics for memory or attached devices attached to the 
	specified pin.
	
  Description:
    This function sets the characteristics for memory or attached devices attached 
	to the specified pin.
	
  Precondition:
    None.
	
  Parameters:
    index -             Identifier for the device instance
    ChipSelectNumber -  Identifies which Chip Select pin is used 
    MemoryType -        Identifies which memory is used
    MemorySize -        An enum type, which sets the size of the attached memory
                        device
    TimingRegSet -      Identifies the timing register
	
  Returns:
    None.
	
  Remarks:
    None.
	
*/
void PLIB_EBI_MemoryCharacteristicsSet(EBI_MODULE_ID index, int ChipSelectNumber, 
                                       EBI_MEMORY_TYPE MemoryType, 
									   EBI_MEMORY_SIZE MemorySize, 
									   EBI_CS_TIMING TimingReg);

/*******************************************************************************
  Function:
    PLIB_EBI_MemoryTimingConfigSet(EBI_MODULE_ID index, int CS_Timing_Reg, 
	                                                    int PageReadTime, 
														int DataTurnAroundTime, 
														int WritePulseWidth, 
														int AddressHoldTime, 
														int AddressSetupTime, 
														int ReadCycleTime)
    
  Summary:
    Sets the cycle time for page reading.
	
  Description:
    This function sets the cycle time for page reading.
	
  Precondition:
    None.
	
  Parameters:
    index -               Identifier for the device instance
    CS_Timing_Reg -       Identifies which Chip Select Timing register to use EBISMT<0:2>
    PageReadTime -        The clock cycle needed for a Memory Page read
    DataTurnAroundTime -  Time between read-to-write, write-to-read, and
                          read-to-read when Chip Select changes state.
    WritePulseWidth -     The clock cycles needed for a memory write
    AddressHoldTime -     The clock time needed for the address bus to hold
    AddressSetupTime -    The time needed for the address to settle
    ReadCycleTime -       _nt_
	
  Returns:
    None.
	
  Remarks:
    None.
*/
void PLIB_EBI_MemoryTimingConfigSet(EBI_MODULE_ID index, int CS_Timing_Reg, 
                                                         int PageReadTime, 
														 int  DataTurnAroundTime, 
														 int WritePulseWidth, 
														 int AddressHoldTime, 
														 int AddressSetupTime, 
														 int ReadCycleTime);

/*******************************************************************************
  Function:
    PLIB_EBI_ReadyModeSet (EBI_MODULE_ID index, bool ReadyPin0, bool ReadyPin1, 
	                                            bool ReadyPin2)
    
  Summary:
    Sets the use of Ready mode for each pin.
	
  Description:
    This function sets the use of Ready mode for each pin. The attached device will 
	either pull the ready pin high or low.
	
  Precondition:
    None.
  
  Parameters:
    index -        Identifier for the device instance; ReadyPin0, ReadyPin1, ReadyPin2
    ReadyPin(x) -  Identifies the ready pin (1-3) 
	
  Returns:
    None.
	
  Remarks:
    None.
	
*/
void PLIB_EBI_ReadyModeSet (EBI_MODULE_ID index, bool ReadyPin0, bool ReadyPin1, 
                                                 bool ReadyPin2);

/*******************************************************************************
  Function:
    PLIB_EBI_ReadyModeGet (EBI_MODULE_ID index, int ChipSelectNumber)
    
  Summary:
    Returns whether or not Ready mode was set.
	
  Description:
    This function returns the state of the register RDYMODE.
    
  Precondition:
    None.  
  
  Parameters:
    index -             Identifier for the device instance
    ChipSelectNumber -  Identifies the specific Chip Select pin
	
  Returns:
        This function returns a bool.
		- 1 = Ready input is used
		- 0 = Ready input is not used
	
  Remarks:
    None.                                                                 
*/
bool PLIB_EBI_ReadyModeGet (EBI_MODULE_ID index, int ChipSelectNumber);

/*******************************************************************************
  Function:
    PLIB_EBI_MemoryPagingSet (EBI_MODULE_ID index, int ChipSelectNumber, 
	                          bool PageMode, EBI_PAGE_SIZE MemoryPageSize)
    
  Summary:
    Sets the size of the memory page if paging is used.
	
  Description:
    This function sets the size of the memory page if paging is used.
	
  Precondition:
    None.  
  
  Parameters:
    index -             Identifier for the device instance
    ChipSelectNumber -  Sets for that Chip Select pin <3:0>
    PageMode -          Enable or disable Page mode
    MemoryPageSize -    Size of memory pages
	
  Returns:
    - 1 = Device supports Page mode
    - 0 = Device does not support Page mode
	
  Remarks:
    None. 
*/
void PLIB_EBI_MemoryPagingSet (EBI_MODULE_ID index, int ChipSelectNumber, 
                               bool PageMode, EBI_PAGE_SIZE MemoryPageSize);

/*******************************************************************************
  Function:
    PLIB_EBI_PageModeGet (EBI_MODULE_ID index,int ChipSelectNumber)
    
  Summary:
    Returns the Paging mode settings.
	
  Description:
    This function returns the state of the register PAGEMODE.
       
  Precondition:
    None.
	
  Parameters:
    index -             Identifier for the device instance
    ChipSelectNumber -  Identifies the specific Chip Select pin
	
  Returns:
    - 1 = Page Mode is used
	- 0 = Page Mode is not used
	
  Remarks:
    None.                                                                  
*/
bool PLIB_EBI_PageModeGet (EBI_MODULE_ID index,int ChipSelectNumber);

/*******************************************************************************
  Function:
    PLIB_EBI_MemoryPageSizeGet (EBI_MODULE_ID index, int ChipSelectNumber)
    
  Summary:
    Returns the Paging mode settings.
	
  Description:
    This function returns the state of the register PAGESIZE.
    
  Precondition:
    None.  
  
  Parameters:
    index -             Identifier for the device instance
    ChipSelectNumber -  Identifies the specific Chip Select pin
	
  Returns:
    Size of the memory page.
	
  Remarks:
    None.                                                                   
*/
EBI_PAGE_SIZE PLIB_EBI_MemoryPageSizeGet (EBI_MODULE_ID index,int ChipSelectNumber);

/*******************************************************************************
  Function:
    PLIB_EBI_PageReadCycleTimeGet (EBI_MODULE_ID index,int ChipSelectNumber)
    
  Summary:
    Returns the cycle time for a page read.
	
  Description:
    This function returns the cycle time for a page read. Read cycle time = 
	TPRC + 1cycle. The value set and return are the same, the controller performs 
	the read on the next clock, which is why there is a +1.
	
  Precondition:
    None. 
  
  Parameters:
    index -             Identifier for the device instance
    ChipSelectNumber -  Identifies the specific Chip Select pin
	
  Returns:
    Returns an integer, which is the number of clock cycles used to read a 
	memory page.
	
  Remarks:
    None.                                                                         
*/
int PLIB_EBI_PageReadCycleTimeGet (EBI_MODULE_ID index,int ChipSelectNumber);

/*******************************************************************************
  Function:
    PLIB_EBI_DataTurnAroundTimeGet (EBI_MODULE_ID index,int ChipSelectNumber)
    
  Summary:
    Returns the data turn-around time set for the EBI bus.
	
  Description:
    This function returns the data turn-around time set for the EBI Bus.
    Returns the clock cycles (0-7) for static memory between
    read-to-write, write-to-read, and read-to-read when Chip Select
    changes.
	
  Precondition:
    None.
  
  Parameters:
    index -             Identifier for the device instance
    ChipSelectNumber -  Identifies the specific Chip Select pin
	
  Returns:
    Returns an integer, which is the number of clock cycles that is set in the 
	turn-around register.
	
  Remarks:
    None.                                                                          
*/
int PLIB_EBI_DataTurnAroundTimeGet (EBI_MODULE_ID index,int ChipSelectNumber);

/*******************************************************************************
  Function:
    PLIB_EBI_WritePulseWidthGet (EBI_MODULE_ID index,int ChipSelectNumber)
    
  Summary:
    Returns the set hold time in clock cycles.
	
  Description:
    This function returns the set hold time in clock cycles.     
    Write Pulse with = TWP + 1 clock cycle.
	
  Precondition:
    None.  
  
  Parameters:
    index -             Identifier for the device instance
    ChipSelectNumber -  Identifies the specific Chip Select pin
	
  Returns:
    Returns an integer, which is the number of clock cycles to which the write 
	pulse width is set.
	
  Remarks:
    None.                                                                         
*/
int PLIB_EBI_WritePulseWidthGet (EBI_MODULE_ID index,int ChipSelectNumber);

/*******************************************************************************
  Function:
    PLIB_EBI_AddressHoldTimeGet (EBI_MODULE_ID index,int ChipSelectNumber)
    
  Summary:
    Returns the address hold time.
	
  Description:
    This function returns the address and data hold time.
	
  Precondition:
    None. 
  
  Parameters:
    index -             Identifier for the device instance
    ChipSelectNumber -  Identifies the specific Chip Select pin
	
  Returns:
    Returns an integer, which is the Address and Data Hold time in the number 
	of clocks.
	
  Remarks:
    None.                                                                           
*/
int PLIB_EBI_AddressHoldTimeGet (EBI_MODULE_ID index,int ChipSelectNumber);

/*******************************************************************************
  Function:
    PLIB_EBI_AddressSetupTimeGet (EBI_MODULE_ID index,int ChipSelectNumber)
    
  Summary:
    Returns the setup hold time.
	
  Description:
    This function returns the setup hold time. A value of '0' is only valid in 
	the case of SSRAM.
	
  Precondition:
    None.  
  
  Parameters:
    index -             Identifier for the device instance
    ChipSelectNumber -  Identifies the specific Chip Select pin
	
  Returns:
    Returns an integer, which is the address setup time in the number of clocks.
	
  Remarks:
    None.                                                                        
*/
int PLIB_EBI_AddressSetupTimeGet (EBI_MODULE_ID index,int ChipSelectNumber);

/*******************************************************************************
  Function:
    PLIB_EBI_ReadCycleTimeGet (EBI_MODULE_ID index, int ChipSelectNumber)
    
  Summary:
    Returns the read time in the number of clock cycles.
	
  Description:
    This function returns the read time in number of clock cycles.
    Read Cycle time = TRC +1 clock cycle. The controller will always take one
    extra clock (the next clock) for a Read Cycle. The return will be the
    time set by the user.
	
    Setting a Read Cycle time to a '0' will return a '0' when read, but the
    controller will add '1' to that value for the next available clock.
	
  Precondition:
    None.
  
  Parameters:
    index -             Identifier for the device instance
    ChipSelectNumber -  Identifies which Chip Select number to which the device 
						is attached

  Returns:
    Returns an integer, which is the read cycle time in system clocks.
	
  Remarks:
    None.                                                                      
*/
int PLIB_EBI_ReadCycleTimeGet (EBI_MODULE_ID index,int ChipSelectNumber);

/*******************************************************************************
  Function:
    PLIB_EBI_FlashTimingSet (EBI_MODULE_ID index, int FlashTiming)
    
  Summary:
    Sets the timing for hold in reset for external Flash.
	
  Description:
    This function sets the number of clock cycles to hold the external Flash 
	memory in reset.
	
  Precondition:
    NOR Flash must be used with EBI instead of SRAM.
	
  Parameters:
    index -        Identifier for the device instance
    FlashTiming -  An integer, which is the number of clock cycles
	
  Returns:
    None.
	
  Remarks:
    None.                                                                 
*/
void PLIB_EBI_FlashTimingSet (EBI_MODULE_ID index, int FlashTiming);

/*******************************************************************************
  Function:
    PLIB_EBI_FlashTimingGet (EBI_MODULE_ID index)
    
  Summary:
    Returns the set Flash timing for external Flash.
	
  Description:
    This function returns the set number of clock cycles to hold the external
    Flash memory in reset.
	
  Precondition:
    MemoryType must be set to Flash.
	
  Parameters:
    None.  
	
  Returns:
    Returns an integer, which is the number of clock cycles.
	
  Remarks:
    None.                                                              
*/
int PLIB_EBI_FlashTimingGet (EBI_MODULE_ID index);

/*******************************************************************************
  Function:
    PLIB_EBI_StaticMemoryWidthRegisterSet (EBI_MODULE_ID index, int RegisterNumber, 
	                             EBI_STATIC_MEMORY_WIDTH StaticMemoryWidthRegister)
    
  Summary:
    Sets the data width of static memory.
	
  Description:
    This function sets the data width of static memory.    
    
  Precondition:
    None.  
  
  Parameters:
    index						- Identifier for the device instance
    RegisterNumber				- The ID for the register being set
    StaticMemoryWidthRegister	- Identifies a bus width of either 8 bits or 16 bits
	
  Returns:
    None.
	
  Remarks:
    None.
*/
void PLIB_EBI_StaticMemoryWidthRegisterSet (EBI_MODULE_ID index, int RegisterNumber,  
                                 EBI_STATIC_MEMORY_WIDTH StaticMemoryWidthRegister);

/*******************************************************************************
  Function:
	PLIB_EBI_StaticMemoryWidthRegisterGet (EBI_MODULE_ID index, 
	                                       int RegisterNumber)

  Summary:
    Returns the set width of the data bus.

  Description:
    This function returns the set width of the data bus.

  Preconditions:
    None.
	
  Parameters:
    index - Identifier for the device instance

  Returns:
    An enum type, which is the bus mode in use.
	
  Remarks:
	None.
*/  
EBI_STATIC_MEMORY_WIDTH PLIB_EBI_StaticMemoryWidthRegisterGet (EBI_MODULE_ID index, 
                                                               int RegisterNumber);

/**********************************************************************************
  Function:
    PLIB_EBI_FlashPowerDownModeSet (EBI_MODULE_ID index, bool FlashPowerDownMode)
    
  Summary:
    Sets the pin state for Flash devices on Power-down and Reset.
	
  Description:
    This function sets the pin state for Flash devices upon a Power-down and Reset.
	
  Precondition:
    None.  
  
  Parameters:
    index 				-  Identifier for the device instance
    FlashPowerDownMode 	-  A bool, which sets the power-down state for Flash memory
	
  Returns:
    None.
	
  Remarks:
    None.                                                                          
*/
void PLIB_EBI_FlashPowerDownModeSet (EBI_MODULE_ID index, bool FlashPowerDownMode);

//******************************************************************************
/* Function:
	PLIB_EBI_FlashPowerDownModeGet (EBI_MODULE_ID index)

  Summary:
    Returns the set power-down state.

  Description:
    This function returns the set power-down state.

  Preconditions:
    None.
	
  Parameters:
    index - Identifier for the device instance

  Returns:
    A bool, depending on the setting of PLIB_EBI_FlashPowerDownModeSet.
	
  Remarks:
	None.
*/
bool PLIB_EBI_FlashPowerDownModeGet (EBI_MODULE_ID index);

//*******************************************************************************

//*******************************************************************************
//*******************************************************************************
/* CFGEBIA: EBI Address Pin Configuration  */
//*******************************************************************************
//*******************************************************************************

/**************************************************************************
  Function:
    PLIB_EBI_ControlEnableSet (EBI_MODULE_ID index, bool EnableBit)
    
  Summary:
    Sets the EBI bus ON/OFF enable bit.
	
  Description:
    This function sets the EBI bus ON/OFF enable bit.
	
  Precondition:
    None.  
  
  Parameters:
    index -      Identifier for the device instance
    EnableBit -  Identifies the enable bit
	
  Returns:
    None.
	
  Remarks:
    None.                                                                  
*/
void PLIB_EBI_ControlEnableSet (EBI_MODULE_ID index, bool EnableBit);

/*************************************************************************
  Function:
    PLIB_EBI_ControlEnableGet (EBI_MODULE_ID index)
    
  Summary:
    Returns the status of the EBI enable bit.
	
  Description:
    This function returns the status of the EBI enable bit.
	
  Precondition:
    None.  
  
  Parameters:
    index -  Identifier for the device instance
	
  Returns:
    Boolean:
	- 1 = The EBI module is ON
	- 0 = The EBI module is OFF
	
  Remarks:
    None.                                                                 
*/
bool PLIB_EBI_ControlEnableGet (EBI_MODULE_ID index);

/*******************************************************************************
  Function:
    PLIB_EBI_AddressPinEnableBitsSet (EBI_MODULE_ID index, 
	                                  EBI_ADDRESS_PORT AddressPort)
    
  Summary:
    Sets the address pins used for EBI.
	
  Description:
    This function sets the address pins used for EBI.
	
  Precondition:
    None.  
  
  Parameters:
    index -        Identifier for the device instance
    AddressPort -  Identifies how many pins are used for addressing on the EBI
                   bus
				   
  Returns:
    None.
	
  Remarks:
    None.                                                                                   
*/
void PLIB_EBI_AddressPinEnableBitsSet (EBI_MODULE_ID index, 
                                       EBI_ADDRESS_PORT AddressPort);

//*******************************************************************************
//*******************************************************************************
/* CFGEBIC: EBI Control Pin Configuration  */
//*******************************************************************************
//*******************************************************************************

/********************************************************************************
  Function:
    PLIB_EBI_ReadyPin3ConfigSet (EBI_MODULE_ID index, bool ReadyPin3Enable, 
	                                                  bool ReadyPin3Invert)
    
  Summary:
    Sets the control use of ReadyPin3 and inverts the status.
	
  Description:
    This function sets the control use of ReadyPin3, and then inverts the status.
	
  Precondition:
    None.  
  
  Parameters:
    index -            Identifier for the device instance
    ReadyPin3Invert -  Identifies the ON/OFF for invert
    ReadyPin3Enable -  Identifies if this pin is used by the control part
	
  Returns:
    None.
	
  Remarks:
    None.                                                                                            
*/
void PLIB_EBI_ReadyPin3ConfigSet (EBI_MODULE_ID index, bool ReadyPin3Enable, 
                                                       bool ReadyPin3Invert);

/*******************************************************************************
  Function:
    PLIB_EBI_ReadyPin2ConfigSet (EBI_MODULE_ID index, bool ReadyPin2Enable, 
	                                                  bool ReadyPin2Invert)
    
  Summary:
    Sets the control use of ReadyPin2 and inverts the status.
	
  Description:
    This function sets the control use of ReadyPin2 and inverts the status.
	
  Precondition:
    None.  
  
  Parameters:
    index -            Identifier for the device instance
    ReadyPin2Invert -  Identifies the ON/OFF for invert
    ReadyPin2Enable -  Identifies if this pin is used by the control part
	
  Returns:
    None.
	
  Remarks:
    None.                                                                                            
*/
void PLIB_EBI_ReadyPin2ConfigSet (EBI_MODULE_ID index, bool ReadyPin2Enable, 
                                                       bool ReadyPin2Invert);

/*******************************************************************************
  Function:
    PLIB_EBI_ReadyPin1ConfigSet (EBI_MODULE_ID index, bool ReadyPin1Enable, 
	                                                  bool ReadyPin1Invert)
    
  Summary:
    Sets the control use of ReadyPin1 and inverts the status.
	
  Description:
    This function sets the control use of ReadyPin1, and then inverts the status.
	
  Precondition:
    None.  
  
  Parameters:
    index -            Identifier for the device instance
    ReadyPin1Invert -  Identifies the ON/OFF for invert
    ReadyPin1Enable -  Identifies if this pin is used by the control part
	
  Returns:
    None.
	
  Remarks:
    None.                                                                                            
*/
void PLIB_EBI_ReadyPin1ConfigSet (EBI_MODULE_ID index, bool ReadyPin1Enable, 
                                                       bool ReadyPin1Invert);

/*******************************************************************************
  Function:
    PLIB_EBI_ReadyPinSensSet (EBI_MODULE_ID index, bool SensitivityControl)
    
  Summary:
    Sets the sensitivity of the Ready pin.
	
  Description:
    This function sets the sensitivity of the Ready pin.
	
  Precondition:
    None.  
  
  Parameters:
    index				-  Identifier for the device instance
    SensitivityControl	-  Selection edge or level detection
	
  Returns:
    None.
	
  Remarks:
    None.                                                                                            
*/
void PLIB_EBI_ReadyPinSensSet (EBI_MODULE_ID index, bool SensitivityControl);

/*******************************************************************************
  Function:
    PLIB_EBI_FlashResetPinSet (EBI_MODULE_ID index,  bool FlashReadPin)
    
  Summary:
    Sets the control use of the Flash Reset pin.
	
  Description:
    This function sets the control use of the Flash Reset pin.

  Precondition:
    None.  
  
  Parameters:
    index			- Identifier for the device instance
    FlashReadPin	- Flash Reset pin is used

  Returns:
    None.
	
  Remarks:
    None.                                                                                            
*/
void PLIB_EBI_FlashResetPinSet (EBI_MODULE_ID index, bool FlashResetPin);

/*******************************************************************************
  Function:
    PLIB_EBI_FlashResetPinGet (EBI_MODULE_ID index)
    
  Summary:
    Sets the control use of Flash Reset pin.
	
  Description:
    This function set the control use of the Flash Reset pin.

  Precondition:
    None.  
  
  Parameters:
    index -               Identifier for the device instance

  Returns:
    Returns a Boolean.
	
  Remarks:
    None.                                                                                            
*/
bool PLIB_EBI_FlashResetPinGet (EBI_MODULE_ID index);

/*******************************************************************************
  Function:
    PLIB_EBI_WriteOutputControlSet (EBI_MODULE_ID index, bool WriteEnable, 
	                                bool OutputEnable)
    
  Summary:
    Sets the Write Enable and Output Enable control pins.
	
  Description:
    This function sets the Write Enable and Output Enable control pins.
	
  Precondition:
    None.  
  
  Parameters:
    index			- Identifier for the device instance
    WriteEnable		- Used for enabling Write Enable
    OutputEnable	- Used for enabling Output Enable
	
  Returns:
    None.
	
  Remarks:
    None.                                                                                              
*/
void PLIB_EBI_WriteOutputControlSet (EBI_MODULE_ID index, bool WriteEnable, 
                                                          bool OutputEnable);

/*******************************************************************************
  Function:
    PLIB_EBI_ByteSelectPinSet (EBI_MODULE_ID index, bool ByteSelect0, bool ByteSelect1)
    
  Summary:
    Sets the data Byte Select High <15:8> and Low <7:0> enable pins for use.
	
  Description:
    This function sets the data Byte Select High <15:8> and Low <7:0>
    enable pins for use.
	If the system uses Byte Select High/Low pins for the data, the pins
    must be enabled for use in the EBI controller.
	
  Precondition:
    The EBIDEN0 and EBIDEN1 registers must first be enabled, which is done
    using the function PLIB_EBI_DataEnableSet.
	
  Parameters:
    index -        Identifier for the device instance
    ByteSelect0 -  Identifies the Lower Byte Select Pin for enabling
    ByteSelect1 -  Identifies the Upper Byte Select Pin for enabling
	
  Returns:
    None.
	
  Remarks:
    None.                                                                                        
*/
void PLIB_EBI_ByteSelectPinSet (EBI_MODULE_ID index, bool ByteSelect0, bool ByteSelect1);

/*******************************************************************************
  Function:
    PLIB_EBI_ChipSelectEnableSet (EBI_MODULE_ID index, bool ChipSelect0, 
	               bool ChipSelect1, bool ChipSelect2, bool ChipSelect3)
    
  Summary:
    Sets the Chip Select pins for use with the EBI or GPIO.
	
  Description:
    This functions sets which Chip Select pins to use with the EBI or GPIO.
	
  Precondition:
    None.  
  
  Parameters:
    index -        Identifier for the device instance
    ChipSelect0 -  Identifies control of Chip Select 0 for enabling
    ChipSelect1 -  Identifies control of Chip Select 1 for enabling
    ChipSelect2 -  Identifies control of Chip Select 2 for enabling
    ChipSelect3 -  Identifies control of Chip Select 3 for enabling
	
  Returns:
    None.
	
  Remarks:
    None.
*/
void PLIB_EBI_ChipSelectEnableSet (EBI_MODULE_ID index, bool ChipSelect0, 
                    bool ChipSelect1, bool ChipSelect2, bool ChipSelect3);

/*******************************************************************************
  Function:
    PLIB_EBI_DataEnableSet (EBI_MODULE_ID index, bool DataUpperByte, bool DataLowerByte)
    
  Summary:
    Sets the use of Data Byte Select Pins, High and Low, for control with EBI or GPIO.
	
  Description:
    This function sets the use of the Data Byte Select Pins, High and Low, for
    control with EBI or GPIO.
	
  Precondition:
    None.  
  
  Parameters:
    index -          Identifier for the device instance
    DataUpperByte -  Identifies control of Upper Data Byte for enabling
    DataLowerByte -  Identifies control of Lower Data Byte for enabling
	
  Returns:
    None.
	
  Remarks:
    None.                                                                                       
*/
void PLIB_EBI_DataEnableSet (EBI_MODULE_ID index, bool DataUpperByte, bool DataLowerByte);

// *****************************************************************************
// *****************************************************************************
// Section: EBI Peripheral Library Exists API Routines
// *****************************************************************************
// *****************************************************************************
/* The following functions indicate the existence of the features on the device. 
*/

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsBaseAddress( EBI_MODULE_ID index )

  Summary:
    Identifies whether the Base_Address feature exists on the EBI module. 

  Description:
    This function identifies whether the Base_Address feature is available on 
	the EBI module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_EBI_BaseAddressSet
    - PLIB_EBI_BaseAddressGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The Base_Address feature is supported on the device
    - false  - The Base_Address feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsBaseAddress( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsMemoryCharacteristics( EBI_MODULE_ID index )

  Summary:
    Identifies whether the MemoryCharacteristics feature exists on the EBI module. 

  Description:
    This function identifies whether the MemoryCharacteristics feature is available 
	on the EBI module.
    When this function returns true, this function is supported on the device: 
    - PLIB_EBI_MemoryCharacteristicsSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The MemoryCharacteristics feature is supported on the device
    - false  - The MemoryCharacteristics feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsMemoryCharacteristics( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsMemoryTimingConfig( EBI_MODULE_ID index )

  Summary:
    Identifies whether the MemoryTimingConfig feature exists on the EBI module. 

  Description:
    This function identifies whether the MemoryTimingConfig feature is available 
	on the EBI module.
    When this function returns true, this function is supported on the device: 
    - PLIB_EBI_MemoryTimingConfigSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The MemoryTimingConfig feature is supported on the device
    - false  - The MemoryTimingConfig feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsMemoryTimingConfig( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsReadyMode( EBI_MODULE_ID index )

  Summary:
    Identifies whether the ReadyMode feature exists on the EBI module. 

  Description:
    This function identifies whether the ReadyMode feature is available on the 
	EBI module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_EBI_ReadyModeSet
    - PLIB_EBI_ReadyModeGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ReadyMode feature is supported on the device
    - false  - The ReadyMode feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsReadyMode( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsMemoryPaging( EBI_MODULE_ID index )

  Summary:
    Identifies whether the MemoryPaging feature exists on the EBI module. 

  Description:
    This function identifies whether the MemoryPaging feature is available on the 
	EBI module.
    When this function returns true, this function is supported on the device: 
    - PLIB_EBI_MemoryPagingSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The MemoryPaging feature is supported on the device
    - false  - The MemoryPaging feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsMemoryPaging( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsPageMode( EBI_MODULE_ID index )

  Summary:
    Identifies whether the PageMode feature exists on the EBI module. 

  Description:
    This function identifies whether the PageMode feature is available on the 
	EBI module.
    When this function returns true, this function is supported on the device: 
    - PLIB_EBI_PageModeGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PageMode feature is supported on the device
    - false  - The PageMode feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsPageMode( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsPageReadTime( EBI_MODULE_ID index )

  Summary:
    Identifies whether the PageReadTime feature exists on the EBI module. 

  Description:
    This function identifies whether the PageReadTime feature is available on the 
	EBI module.
    When this function returns true, this function is supported on the device: 
    - PLIB_EBI_PageReadCycleTimeGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The PageReadTime feature is supported on the device
    - false  - The PageReadTime feature is not supported on the device

  Remarks:
    None.
*/

bool PLIB_EBI_ExistsPageReadTime( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsDataTurnAroundTime( EBI_MODULE_ID index )

  Summary:
    Identifies whether the DataTurnAroundTime feature exists on the EBI module. 

  Description:
    This function identifies whether the DataTurnAroundTime feature is available 
	on the EBI module.
    When this function returns true, this function is supported on the device: 
    - PLIB_EBI_DataTurnAroundTimeGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The DataTurnAroundTime feature is supported on the device
    - false  - The DataTurnAroundTime feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsDataTurnAroundTime( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsWritePulseWidth( EBI_MODULE_ID index )

  Summary:
    Identifies whether the WritePulseWidth feature exists on the EBI module. 

  Description:
    This function identifies whether the WritePulseWidth feature is available 
	on the EBI module.
    When this function returns true, this function is supported on the device: 
    - PLIB_EBI_WritePulseWidthGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The WritePulseWidth feature is supported on the device
    - false  - The WritePulseWidth feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsWritePulseWidth( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsAddressHoldTime( EBI_MODULE_ID index )

  Summary:
    Identifies whether the AddressHoldTime feature exists on the EBI module. 

  Description:
    This function identifies whether the AddressHoldTime feature is available 
	on the EBI module.
    When this function returns true, this function is supported on the device: 
    - PLIB_EBI_AddressHoldTimeGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The AddressHoldTime feature is supported on the device
    - false  - The AddressHoldTime feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsAddressHoldTime( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsAddressSetupTime( EBI_MODULE_ID index )

  Summary:
    Identifies whether the AddressSetupTime feature exists on the EBI module. 

  Description:
    This function identifies whether the AddressSetupTime feature is available on 
	the EBI module.
    When this function returns true, this function is supported on the device: 
    - PLIB_EBI_AddressSetupTimeGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The AddressSetupTime feature is supported on the device
    - false  - The AddressSetupTime feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsAddressSetupTime( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsReadCycleTime( EBI_MODULE_ID index )

  Summary:
    Identifies whether the ReadCycleTime feature exists on the EBI module. 

  Description:
    This function identifies whether the ReadCycleTime feature is available on 
	the EBI module.
    When this function returns true, this function is supported on the device: 
    - PLIB_EBI_ReadCycleTimeGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ReadCycleTime feature is supported on the device
    - false  - The ReadCycleTime feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsReadCycleTime( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsFlashTiming( EBI_MODULE_ID index )

  Summary:
    Identifies whether the FlashTiming feature exists on the EBI module. 

  Description:
    This function identifies whether the FlashTiming feature is available on the 
	EBI module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_EBI_FlashTimingSet
    - PLIB_EBI_FlashTimingGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The FlashTiming feature is supported on the device
    - false  - The FlashTiming feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsFlashTiming( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsStaticMemoryWidthRegister( EBI_MODULE_ID index )

  Summary:
    Identifies whether the StaticMemoryWidthRegister feature exists on the EBI module. 

  Description:
    This function identifies whether the StaticMemoryWidthRegister feature is available 
	on the EBI module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_EBI_StaticMemoryWidthRegisterSet
    - PLIB_EBI_StaticMemoryWidthRegisterGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The StaticMemoryWidthRegister feature is supported on the device
    - false  - The StaticMemoryWidthRegister feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsStaticMemoryWidthRegister( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsFlashPowerDownMode( EBI_MODULE_ID index )

  Summary:
    Identifies whether the FlashPowerDownMode feature exists on the EBI module. 

  Description:
    This function identifies whether the FlashPowerDownMode feature is available 
	on the EBI module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_EBI_FlashPowerDownModeSet
    - PLIB_EBI_FlashPowerDownModeGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The FlashPowerDownMode feature is supported on the device
    - false  - The FlashPowerDownMode feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsFlashPowerDownMode( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsControlEnable( EBI_MODULE_ID index )

  Summary:
    Identifies whether the ControlEnable feature exists on the EBI module. 

  Description:
    This function identifies whether the ControlEnable feature is available on 
	the EBI module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_EBI_ControlEnableSet
    - PLIB_EBI_ControlEnableGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ControlEnable feature is supported on the device
    - false  - The ControlEnable feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsControlEnable( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsAddressPinEnableBits( EBI_MODULE_ID index )

  Summary:
    Identifies whether the AddressPinEnableBits feature exists on the EBI module. 

  Description:
    This function identifies whether the AddressPinEnableBits feature is available 
	on the EBI module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_EBI_AddressPinEnableBitsSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The AddressPinEnableBits feature is supported on the device
    - false  - The AddressPinEnableBits feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsAddressPinEnableBits( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsReadyPin3Config( EBI_MODULE_ID index )

  Summary:
    Identifies whether the ReadyPin3Config feature exists on the EBI module. 

  Description:
    This function identifies whether the ReadyPin3Config feature is available on 
	the EBI module.
    When this function returns true, this function is supported on the device: 
    - PLIB_EBI_ReadyPin3ConfigSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ReadyPin3Config feature is supported on the device
    - false  - The ReadyPin3Config feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsReadyPin3Config( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsReadyPin2Config( EBI_MODULE_ID index )

  Summary:
    Identifies whether the ReadyPin2Config feature exists on the EBI module. 

  Description:
    This function identifies whether the ReadyPin2Config feature is available on 
	the EBI module.
    When this function returns true, this function is supported on the device: 
    - PLIB_EBI_ReadyPin2ConfigSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ReadyPin2Config feature is supported on the device
    - false  - The ReadyPin2Config feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsReadyPin2Config( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsReadyPin1Config( EBI_MODULE_ID index )

  Summary:
    Identifies whether the ReadyPin1Config feature exists on the EBI module. 

  Description:
    This function identifies whether the ReadyPin1Config feature is available 
	on the EBI module.
    When this function returns true, this function is supported on the device: 
    - PLIB_EBI_ReadyPin1ConfigSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ReadyPin1Config feature is supported on the device
    - false  - The ReadyPin1Config feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsReadyPin1Config( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsReadyPinSens( EBI_MODULE_ID index )

  Summary:
    Identifies whether the ReadyPinSens feature exists on the EBI module 

  Description:
    This function identifies whether the ReadyPinSens feature is available on 
	the EBI module.
    When this function returns true, this function is supported on the device: 
    - PLIB_EBI_ReadyPinSensSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ReadyPinSens feature is supported on the device
    - false  - The ReadyPinSens feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsReadyPinSens( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsFlashResetPin( EBI_MODULE_ID index )

  Summary:
    Identifies whether the FlashResetPin feature exists on the EBI module. 

  Description:
    This function identifies whether the FlashResetPin feature is available on 
	the EBI module.
    When this function returns true, these functions are supported on the device: 
    - PLIB_EBI_FlashResetPinSet
    - PLIB_EBI_FlashResetPinGet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The FlashResetPin feature is supported on the device
    - false  - The FlashResetPin feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsFlashResetPin( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsWriteOutputControl( EBI_MODULE_ID index )

  Summary:
    Identifies whether the WriteOutputControl feature exists on the EBI module. 

  Description:
    This function identifies whether the WriteOutputControl feature is available 
	on the EBI module.
    When this function returns true, this function is supported on the device: 
    - PLIB_EBI_WriteOutputControlSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The WriteOutputControl feature is supported on the device
    - false  - The WriteOutputControl feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsWriteOutputControl( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsByteSelectPin( EBI_MODULE_ID index )

  Summary:
    Identifies whether the ByteSelectPin feature exists on the EBI module. 

  Description:
    This function identifies whether the ByteSelectPin feature is available on 
	the EBI module.
    When this function returns true, this function is supported on the device: 
    - PLIB_EBI_ByteSelectPinSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ByteSelectPin feature is supported on the device
    - false  - The ByteSelectPin feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsByteSelectPin( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsChipSelectEnable( EBI_MODULE_ID index )

  Summary:
    Identifies whether the ChipSelectEnable feature exists on the EBI module. 

  Description:
    This function identifies whether the ChipSelectEnable feature is available 
	on the EBI module.
    When this function returns true, this function is supported on the device: 
    - PLIB_EBI_ChipSelectEnableSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The ChipSelectEnable feature is supported on the device
    - false  - The ChipSelectEnable feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsChipSelectEnable( EBI_MODULE_ID index );

//******************************************************************************
/* Function:
	PLIB_EBI_ExistsDataEnable( EBI_MODULE_ID index )

  Summary:
    Identifies whether the DataEnable feature exists on the EBI module. 

  Description:
    This function identifies whether the DataEnable feature is available on the 
	EBI module.
    When this function returns true, this function is supported on the device: 
    - PLIB_EBI_DataEnableSet

  Preconditions:
    None.

  Parameters:
    index           - Identifier for the device instance

  Returns:
    - true   - The DataEnable feature is supported on the device
    - false  - The DataEnable feature is not supported on the device

  Remarks:
    None.
*/
bool PLIB_EBI_ExistsDataEnable( EBI_MODULE_ID index );

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

//*******************************************************************************
//*******************************************************************************
#endif // #ifndef _PLIB_EBI_H
/*******************************************************************************
 End of File
*/