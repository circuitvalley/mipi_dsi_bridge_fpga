/*******************************************************************************
  USB Host HID Mouse driver implementation.

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_hid_mouse.c

  Summary:
    This file contains implementations of both private and public functions
    of the USB Host HID Mouse driver.

  Description:
    This file contains the USB host HID Mouse driver implementation. This file 
    should be included in the project if USB HID Mouse devices are to be supported.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "usb/usb_host_hid_mouse.h"
#include "usb/src/usb_host_hid_mouse_local.h"
#include "system/debug/sys_debug.h"
#include "usb/usb_host_hid.h"
#include <p32xxxx.h>

/* Mouse driver information on a per instance basis */
USB_HOST_HID_MOUSE_DATA_OBJ mouseData[USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER];
USB_HOST_HID_MOUSE_EVENT_HANDLER appMouseHandler;


// *****************************************************************************
/* Function:
    USB_HOST_HID_MOUSE_RESULT USB_HOST_HID_MOUSE_EventHandlerSet
    (
        USB_HOST_HID_MOUSE_EVENT_HANDLER appMouseHandler
    )
 
  Summary:
   Function registers application event handler with USB HID Mouse driver
  
  Description:
   Function registers application event handler with USB HID Mouse driver
  
  Remarks:
    Function registered should be of type USB_HOST_HID_MOUSE_EVENT_HANDLER.
*/

USB_HOST_HID_MOUSE_RESULT USB_HOST_HID_MOUSE_EventHandlerSet
(
    USB_HOST_HID_MOUSE_EVENT_HANDLER appMouseEventHandler
)
{
    if(NULL == appMouseEventHandler)
    {
        return USB_HOST_HID_MOUSE_RESULT_INVALID_PARAMETER;
    }
    else
    {
        appMouseHandler = appMouseEventHandler;
    }
    return USB_HOST_HID_MOUSE_RESULT_SUCCESS;
} /* End of USB_HOST_HID_MOUSE_EventHandlerSet() */


// *****************************************************************************
/* Function:
    void _USB_HOST_HID_MOUSE_EventHandler
    (
        USB_HOST_HID_OBJ_HANDLE handle,
        USB_HOST_HID_EVENT event,
        void * eventData
    )
 
  Summary:
    Mouse driver event handler function registered with USB HID client driver
  
  Description:
    Mouse driver event handler function registered with USB HID client driver
  
  Remarks:
    This is a local function and should not be called by application directly.
*/

void _USB_HOST_HID_MOUSE_EventHandler
(
    USB_HOST_HID_OBJ_HANDLE handle,
    USB_HOST_HID_EVENT event,
    void * eventData
)
{
    /* Start  of local variables */
    uint8_t loop = 0;
    /* End of local variables */
    
    if(handle != USB_HOST_HID_OBJ_HANDLE_INVALID)
    {
        switch(event)
        {
            case USB_HOST_HID_EVENT_ATTACH:
                for(loop = 0; loop < USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER;
                        loop++)
                {
                    if(!mouseData[loop].inUse)
                    {
                        /* Grab the pool */
                        mouseData[loop].inUse = true;
                        /* Reset necessary data structures */
                        mouseData[loop].handle = handle;
                        mouseData[loop].state = USB_HOST_HID_MOUSE_ATTACHED;
                        mouseData[loop].nextPingPong = false;
                        mouseData[loop].taskPingPong = false;
                        mouseData[loop].isPingReportProcessing = false;
                        mouseData[loop].isPongReportProcessing = false;
                        memset((void *)&mouseData[loop].appData, 0,
                                sizeof(USB_HOST_HID_MOUSE_DATA));
                        memset((void *)mouseData[loop].dataPing, 0,64);
                        memset((void *)mouseData[loop].dataPong, 0,64);
                        break;
                    }
                }
                if(loop != USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER)
                {
                    if(appMouseHandler != NULL)
                    {
                        appMouseHandler((USB_HOST_HID_MOUSE_HANDLE)handle,
                                USB_HOST_HID_MOUSE_EVENT_ATTACH,
                                NULL);
                    }
                }
                break;
            case USB_HOST_HID_EVENT_DETACH:
                for(loop = 0; loop < USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER;
                        loop++)
                {
                    if(mouseData[loop].inUse && 
                            (mouseData[loop].handle == handle))
                    {
                        /* Release the pool object */
                        mouseData[loop].inUse = false;
                        mouseData[loop].state = USB_HOST_HID_MOUSE_DETACHED;
                        break;
                    }
                }
                if(loop != USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER)
                {
                    if(appMouseHandler != NULL)
                    {
                        appMouseHandler((USB_HOST_HID_MOUSE_HANDLE)handle,
                                USB_HOST_HID_MOUSE_EVENT_DETACH,
                                NULL);
                    }
                }
                break;
            case USB_HOST_HID_EVENT_REPORT_RECEIVED:
                for(loop = 0; loop < USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER;
                        loop++)
                {
                    if(mouseData[loop].inUse && 
                            (mouseData[loop].handle == handle))
                    {
                        /* Found the Mouse data object */
                        break;
                    }
                }
                if(loop != USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER)
                {
                    if(mouseData[loop].nextPingPong)
                    {
                        if((mouseData[loop].isPongReportProcessing == false))
                        {
                            /* There is no ongoing report processing for this Mouse
                            driver instance for Pong buffer. */
                            mouseData[loop].nextPingPong = false;
                            mouseData[loop].isPongReportProcessing = true;
                            memcpy((void *)mouseData[loop].dataPong, (const void *)eventData,
                                    64);
                            mouseData[loop].state =
                                    USB_HOST_HID_MOUSE_REPORT_PROCESS;
                        }
                    }
                    else
                    {
                        if((mouseData[loop].isPingReportProcessing == false))
                        {
                            /* There is no ongoing report processing for this Mouse
                            driver instance for Ping buffer. */
                            mouseData[loop].nextPingPong = true;
                            mouseData[loop].isPingReportProcessing = true;
                            memcpy((void *)mouseData[loop].dataPing, (const void *)eventData,
                                    64);
                            mouseData[loop].state =
                                    USB_HOST_HID_MOUSE_REPORT_PROCESS;
                        }
                    }
                }
            default:
                break;
        }
    }
} /* End of _USB_HOST_HID_MOUSE_EventHandler() */


// *****************************************************************************
/* Function:
    void _USB_HOST_HID_MOUSE_Task(USB_HOST_HID_OBJ_HANDLE handle)
 
  Summary:
    Mouse driver task routine function registered with USB HID client driver
  
  Description:
    Mouse driver task routine function registered with USB HID client driver
  
  Remarks:
    This is a local function and should not be called by application directly.
*/

void _USB_HOST_HID_MOUSE_Task(USB_HOST_HID_OBJ_HANDLE handle)
{
    /* Start of local variables */
    USB_HOST_HID_LOCAL_ITEM localItem = {.delimiterBranch = 0};
    USB_HOST_HID_GLOBAL_ITEM globalItem = {.reportSize = 0};
    USB_HOST_HID_MAIN_ITEM mainItem = {.localItem = NULL};
    
    int64_t mouseDataBufferTemp = 0;
    int8_t * mouseDataBuffer = NULL;
    int8_t *ptr = NULL;
    int8_t dataTemp[64] ={0};
    
    uint32_t reportOffset = 0;
    uint32_t currentReportOffset = 0;
    uint32_t currentReportOffsetTemp = 0;
    uint32_t andMask = 0;
    uint32_t usage = 0;
    
    uint8_t index = 1;
    uint8_t loop = 0;
    uint8_t mouseIndex = 0;
    uint8_t i=0;
    bool tobeDone = false;
    USB_HOST_HID_RESULT result = USB_HOST_HID_RESULT_FAILURE;
    /* End of local variables */
    
    if(handle == USB_HOST_HID_OBJ_HANDLE_INVALID)
    {
        return;
    }
    for(mouseIndex = 0; mouseIndex < USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER;
                        mouseIndex++)
    {
        if(mouseData[mouseIndex].inUse && (mouseData[mouseIndex].handle == handle))
        {
            /* Found the Mouse data object */
            break;
        }
    }
    if(mouseIndex == USB_HOST_HID_USAGE_DRIVER_SUPPORT_NUMBER)
    {
        /* Mouse index corresponding to the handle not found */
        return;
    }
    mainItem.globalItem = &globalItem;
    mainItem.localItem = &localItem;
    
    switch(mouseData[mouseIndex].state)
    {
        case USB_HOST_HID_MOUSE_DETACHED:
            break;
        case USB_HOST_HID_MOUSE_ATTACHED:
            break;
        case USB_HOST_HID_MOUSE_REPORT_PROCESS:
            tobeDone = false;
            if(!mouseData[mouseIndex].taskPingPong)
            {
                if(mouseData[mouseIndex].isPingReportProcessing == true)
                {
                    mouseData[mouseIndex].taskPingPong = true;
                    /* Keep a temp backup of the data */
                    memcpy(&dataTemp,
                            (const void *)mouseData[mouseIndex].dataPing, 64);
                    /* Reset global items only once as they are applicable through out */
                    memset(&globalItem, 0,
                            (size_t)sizeof(USB_HOST_HID_GLOBAL_ITEM));
                    tobeDone = true;
                }
            }
            else
            {
                if(mouseData[mouseIndex].isPongReportProcessing == true)
                {
                    mouseData[mouseIndex].taskPingPong = false;
                    /* Keep a temp backup of the data */
                    memcpy(&dataTemp,
                            (const void *)mouseData[mouseIndex].dataPong, 64);
                    /* Reset global items only once as they are applicable through out */
                    memset(&globalItem, 0,
                            (size_t)sizeof(USB_HOST_HID_GLOBAL_ITEM));
                    tobeDone = true;
                }
            }
            if(tobeDone)
            {
                do
                {
                    /* Reset the field data except global items */
                    memset(&localItem, 0, 
                            (size_t)sizeof(USB_HOST_HID_LOCAL_ITEM));
                    memset(&(mainItem.data), 0,
                            (size_t)sizeof(USB_HID_MAIN_ITEM_OPTIONAL_DATA));
                    mainItem.tag = 0;
                    
                    result = USB_HOST_HID_MainItemGet(handle,index,&mainItem);
                    if(result == USB_HOST_HID_RESULT_SUCCESS)
                    {
                        /* Copy the data as while processing the last field
                         we have changed the data for numbered report. If
                         taskPingPong is true that means we need to process
                         Ping buffer, else Pong buffer */
                        if(mouseData[mouseIndex].taskPingPong)
                        {
                            memcpy((void *)mouseData[mouseIndex].dataPing,
                                    (const void *)&dataTemp, 64);
                        }
                        else
                        {
                            memcpy((void *)mouseData[mouseIndex].dataPong,
                                    (const void *)&dataTemp, 64);
                        }
                        if(mainItem.tag ==
                                USB_HID_MAIN_ITEM_TAG_BEGIN_COLLECTION)
                        {    
                            /* Do not change report offset as they do not
                             * create data fields. Just reset the global item
                             * data */
                            memset(&globalItem, 0,
                                    (size_t)sizeof(USB_HOST_HID_GLOBAL_ITEM));
                        }/* Main item = BEGIN COLLECTION */
                        
                        else if(mainItem.tag == 
                                USB_HID_MAIN_ITEM_TAG_END_COLLECTION)
                        {
                            /* Do not change report offset as they do not
                             *  create data fields*/
                        }/* Main item = END COLLECTION */
                        
                        else if(mainItem.tag == USB_HID_MAIN_ITEM_TAG_INPUT)
                        {
                            if(((mainItem.globalItem)->reportCount == 0))
                            {
                                index++;
                                continue;
                            }
                            if(!((mainItem.globalItem)->reportID == 0))
                            {
                                /* Numbered report */
                                if(mouseData[mouseIndex].taskPingPong)
                                {
                                    if(mouseData[mouseIndex].dataPing[0] !=
                                        (mainItem.globalItem)->reportID)
                                    {
                                        /* Report ID does not match */
                                        index++;
                                        continue;
                                    }
                                    /* Numbered Report. Shift right by 1 byte */
                                    for(loop = 0; loop <64; loop ++)
                                    {
                                        if(loop == 63)
                                        {
                                            mouseData[mouseIndex].dataPing[loop] = 0;
                                            break;
                                        }
                                        mouseData[mouseIndex].dataPing[loop] = 
                                                mouseData[mouseIndex].dataPing[loop + 1];
                                    }
                                }
                                else
                                {
                                    if(mouseData[mouseIndex].dataPong[0] !=
                                        (mainItem.globalItem)->reportID)
                                    {
                                        /* Report ID does not match */
                                        index++;
                                        continue;
                                    }
                                    /* Numbered Report. Shift right by 1 byte */
                                    for(loop = 0; loop <64; loop ++)
                                    {
                                        if(loop == 63)
                                        {
                                            mouseData[mouseIndex].dataPong[loop] = 0;
                                            break;
                                        }
                                        mouseData[mouseIndex].dataPong[loop] = 
                                                mouseData[mouseIndex].dataPong[loop + 1];
                                    }
                                }
                            }

                            if(mouseData[mouseIndex].taskPingPong)
                            {
                                mouseDataBuffer = (int8_t *)mouseData[mouseIndex].dataPing;
                            }
                            else
                            {
                                mouseDataBuffer = (int8_t *)mouseData[mouseIndex].dataPong;
                            }
                            
                            currentReportOffset = reportOffset;
                            currentReportOffsetTemp = currentReportOffset;
                            reportOffset = reportOffset + 
                                    (mainItem.globalItem)->reportCount *
                                    (mainItem.globalItem)->reportSize;
                            
                            /* Mouse button handling logic*/
                            if(((0xFF00 & (mainItem.globalItem)->usagePage) 
                                    == USB_HID_USAGE_PAGE_BUTTON) || 
                                    ((0x00FF & (mainItem.globalItem)->usagePage)
                                        == USB_HID_USAGE_PAGE_BUTTON))
                            {
                                if(mainItem.localItem->usageMinMax.valid)
                                {
                                    usage = mainItem.localItem->usageMinMax.min;
                                    loop = 0;
                                    while(usage <= 
                                            (mainItem.localItem->usageMinMax.max))
                                    {
                                        if(mouseData[mouseIndex].taskPingPong)
                                        {
                                            mouseDataBuffer = (int8_t *)mouseData[mouseIndex].dataPing;
                                        }
                                        else
                                        {
                                            mouseDataBuffer = (int8_t *)mouseData[mouseIndex].dataPong;
                                        }
                                        if(((0x00FF & usage) == USB_HID_USAGE_ID_BUTTON1) &&
                                                (loop < USB_HOST_HID_MOUSE_BUTTONS_NUMBER))
                                        {
                                            mouseDataBuffer = mouseDataBuffer + currentReportOffsetTemp/8;
                                            mouseData[mouseIndex].appData.buttonState[loop] = 
                                                (USB_HID_BUTTON_STATE)
                                                (((*mouseDataBuffer) >> (currentReportOffsetTemp % 8))
                                                        & 0x01);
                                            mouseData[mouseIndex].appData.buttonID[loop] =
                                                    USB_HID_USAGE_ID_BUTTON1;
                                            loop++;
                                        }
                                        if((0x00FF & usage) == USB_HID_USAGE_ID_BUTTON2 && 
                                        (loop < USB_HOST_HID_MOUSE_BUTTONS_NUMBER))
                                        {
                                            mouseDataBuffer = mouseDataBuffer + currentReportOffsetTemp/8;
                                            mouseData[mouseIndex].appData.buttonState[loop] = 
                                                (USB_HID_BUTTON_STATE)
                                                (((*mouseDataBuffer) >> (currentReportOffsetTemp % 8))
                                                        & 0x01);
                                            mouseData[mouseIndex].appData.buttonID[loop] =
                                                    USB_HID_USAGE_ID_BUTTON2;
                                            loop++;
                                        }
                                        if((0x00FF & usage) == USB_HID_USAGE_ID_BUTTON3 && 
                                        (loop < USB_HOST_HID_MOUSE_BUTTONS_NUMBER))
                                        {
                                            mouseDataBuffer = mouseDataBuffer + currentReportOffsetTemp/8;
                                            mouseData[mouseIndex].appData.buttonState[loop] = 
                                                (USB_HID_BUTTON_STATE)
                                                (((*mouseDataBuffer) >> currentReportOffsetTemp % 8)
                                                        & 0x01);
                                            mouseData[mouseIndex].appData.buttonID[loop] =
                                                    USB_HID_USAGE_ID_BUTTON3;
                                            loop++;
                                        }
                                        if((0x00FF & usage) == USB_HID_USAGE_ID_BUTTON4 && 
                                        (loop < USB_HOST_HID_MOUSE_BUTTONS_NUMBER))
                                        {
                                            mouseDataBuffer = mouseDataBuffer + currentReportOffsetTemp/8;
                                            mouseData[mouseIndex].appData.buttonState[loop] = 
                                                (USB_HID_BUTTON_STATE)
                                                (((*mouseDataBuffer) >> currentReportOffsetTemp % 8)
                                                        & 0x01);
                                            mouseData[mouseIndex].appData.buttonID[loop] =
                                                    USB_HID_USAGE_ID_BUTTON4;
                                            loop++;
                                        }
                                        if((0x00FF & usage) == USB_HID_USAGE_ID_BUTTON5 && 
                                        (loop < USB_HOST_HID_MOUSE_BUTTONS_NUMBER))
                                        {
                                            mouseDataBuffer = mouseDataBuffer + currentReportOffsetTemp/8;
                                            mouseData[mouseIndex].appData.buttonState[loop] = 
                                                (USB_HID_BUTTON_STATE)
                                                (((*mouseDataBuffer) >> currentReportOffsetTemp % 8)
                                                        & 0x01);
                                            mouseData[mouseIndex].appData.buttonID[loop] =
                                                    USB_HID_USAGE_ID_BUTTON5;
                                            loop++;
                                        }
                                        /* Move to the next usage */
                                        usage++;
                                        /* Update the report offset */
                                        currentReportOffsetTemp = 
                                            currentReportOffsetTemp + 
                                            (mainItem.globalItem)->reportSize;
                                    } /* end of while(all usages) */
                                } /* Usage Min Max present */
                                else
                                {
                                    loop = 1;
                                    do
                                    {
                                        if(mouseData[mouseIndex].taskPingPong)
                                        {
                                            mouseDataBuffer = (int8_t *)mouseData[mouseIndex].dataPing;
                                        }
                                        else
                                        {
                                            mouseDataBuffer = (int8_t *)mouseData[mouseIndex].dataPong;
                                        }
                                        result = USB_HOST_HID_UsageGet
                                                (
                                                    handle,
                                                    index,
                                                    loop,
                                                    &usage
                                                );
                                        if(result == USB_HOST_HID_RESULT_SUCCESS)
                                        {
                                            if((0x00FF & usage) == USB_HID_USAGE_ID_BUTTON1 && 
                                            (loop < USB_HOST_HID_MOUSE_BUTTONS_NUMBER))
                                            {
                                                mouseDataBuffer = mouseDataBuffer + currentReportOffsetTemp/8;
                                                mouseData[mouseIndex].appData.buttonState[loop] = 
                                                    (USB_HID_BUTTON_STATE)
                                                    (((*mouseDataBuffer) >> currentReportOffsetTemp % 8)
                                                            & 0x01);
                                                mouseData[mouseIndex].appData.buttonID[loop] =
                                                        USB_HID_USAGE_ID_BUTTON1;
                                                loop++;
                                            }
                                            if((0x00FF & usage) == USB_HID_USAGE_ID_BUTTON2 && 
                                            (loop < USB_HOST_HID_MOUSE_BUTTONS_NUMBER))
                                            {
                                                mouseDataBuffer = mouseDataBuffer + currentReportOffsetTemp/8;
                                                mouseData[mouseIndex].appData.buttonState[loop] = 
                                                    (USB_HID_BUTTON_STATE)
                                                    (((*mouseDataBuffer) >> currentReportOffsetTemp % 8)
                                                            & 0x01);
                                                mouseData[mouseIndex].appData.buttonID[loop] =
                                                        USB_HID_USAGE_ID_BUTTON2;
                                                loop++;
                                            }
                                            if((0x00FF & usage) == USB_HID_USAGE_ID_BUTTON3 && 
                                            (loop < USB_HOST_HID_MOUSE_BUTTONS_NUMBER))
                                            {
                                                mouseDataBuffer = mouseDataBuffer + currentReportOffsetTemp/8;
                                                mouseData[mouseIndex].appData.buttonState[loop] = 
                                                    (USB_HID_BUTTON_STATE)
                                                    (((*mouseDataBuffer) >> currentReportOffsetTemp % 8)
                                                            & 0x01);
                                                mouseData[mouseIndex].appData.buttonID[loop] =
                                                        USB_HID_USAGE_ID_BUTTON3;
                                                loop++;
                                            }
                                            if((0x00FF & usage) == USB_HID_USAGE_ID_BUTTON4 && 
                                            (loop < USB_HOST_HID_MOUSE_BUTTONS_NUMBER))
                                            {
                                                mouseDataBuffer = mouseDataBuffer + currentReportOffsetTemp/8;
                                                mouseData[mouseIndex].appData.buttonState[loop] = 
                                                    (USB_HID_BUTTON_STATE)
                                                    (((*mouseDataBuffer) >> currentReportOffsetTemp % 8)
                                                            & 0x01);
                                                mouseData[mouseIndex].appData.buttonID[loop] =
                                                        USB_HID_USAGE_ID_BUTTON4;
                                                loop++;
                                            }
                                            if((0x00FF & usage) == USB_HID_USAGE_ID_BUTTON5 && 
                                            (loop < USB_HOST_HID_MOUSE_BUTTONS_NUMBER))
                                            {
                                                mouseDataBuffer = mouseDataBuffer + currentReportOffsetTemp/8;
                                                mouseData[mouseIndex].appData.buttonState[loop] = 
                                                    (USB_HID_BUTTON_STATE)
                                                    (((*mouseDataBuffer) >> currentReportOffsetTemp % 8)
                                                            & 0x01);
                                                mouseData[mouseIndex].appData.buttonID[loop] =
                                                        USB_HID_USAGE_ID_BUTTON5;
                                                loop++;
                                            }
                                            currentReportOffsetTemp = 
                                                                currentReportOffsetTemp + 
                                                                (mainItem.globalItem)->reportSize;
                                            loop++;
                                        } /* if usage found */
                                    } while(result == USB_HOST_HID_RESULT_SUCCESS);
                                    /* We have checked all the usages present
                                         in this field. Now reset the result.
                                         Otherwise the main while loop will
                                         exit. */
                                    result = USB_HOST_HID_RESULT_SUCCESS;
                                } /* Individual usage based field */
                            } /* Button page */

                            else if(((0xFF00 & (mainItem.globalItem)->usagePage)
                                        == USB_HID_USAGE_PAGE_GENERIC_DESKTOP_CONTROLS) || 
                                        ((0x00FF & (mainItem.globalItem)->usagePage)
                                        == USB_HID_USAGE_PAGE_GENERIC_DESKTOP_CONTROLS))
                            {
                                if(((mainItem.localItem)->usageMinMax.valid) == false)
                                {
                                    usage = 0;
                                    loop = 1;
                                    do
                                    {
                                        if(mouseData[mouseIndex].taskPingPong)
                                        {
                                            mouseDataBuffer = (int8_t *)mouseData[mouseIndex].dataPing;
                                        }
                                        else
                                        {
                                            mouseDataBuffer = (int8_t *)mouseData[mouseIndex].dataPong;
                                        }
                                        result = USB_HOST_HID_UsageGet
                                                (
                                                    handle,
                                                    index,
                                                    loop,
                                                    &usage
                                                );
                                        if(result == USB_HOST_HID_RESULT_SUCCESS)
                                        {
                                            /* Usage obtained */
                                            usage = usage << 16;
                                            usage = usage >> 16;
                                            if((usage == USAGE_X) || 
                                                (usage == USAGE_Y) ||
                                                    (usage == USAGE_Z))
                                            {
                                                /* X/Y/Z found*/
                                                mouseDataBuffer = mouseDataBuffer + 
                                                        currentReportOffsetTemp/8;
                                                
                                                ptr = mouseDataBuffer;
                                                mouseDataBufferTemp = 0;
                                                for (i = 0; i < 5; i++)
                                                {
                                                    mouseDataBufferTemp |= ptr[i] << (i * 8);
                                                }
                                                
                                                mouseDataBufferTemp >>= currentReportOffsetTemp % 8;
                                                mouseDataBufferTemp &= 0xFFFFFFFF;
                                                
                                                /* The logic here is AND with (2 to the power report size) - 1 */
                                                andMask = (1 << (mainItem.globalItem)->reportSize) - 1;
                                                mouseDataBufferTemp = mouseDataBufferTemp & andMask;
                                                
                                                if((mainItem.globalItem)->logicalMinimum < 0)
                                                {
                                                    mouseDataBufferTemp = 
                                                        mouseDataBufferTemp & (1 << (((mainItem.globalItem)->reportSize) - 1)) ? 
                                                        mouseDataBufferTemp | ((int64_t)-1 << ((mainItem.globalItem)->reportSize))
                                                          : mouseDataBufferTemp;
                                                }
                                                if(usage == USAGE_X)
                                                {
                                                    mouseData[mouseIndex].appData.xMovement =
                                                        (int16_t)(mouseDataBufferTemp);
                                                }
                                                else if(usage == USAGE_Y)
                                                {
                                                    mouseData[mouseIndex].appData.yMovement =
                                                        (int16_t)(mouseDataBufferTemp);
                                                }
                                                else if(usage == USAGE_Z)
                                                {
                                                    mouseData[mouseIndex].appData.zMovement =
                                                        (int16_t)(mouseDataBufferTemp);
                                                }
                                            }
                                            currentReportOffsetTemp = 
                                                        currentReportOffsetTemp + 
                                                        (mainItem.globalItem)->reportSize;
                                            loop++;
                                        }/* if usage obtained */
                                    } while(result == USB_HOST_HID_RESULT_SUCCESS);
                                    /* We have checked all the usages present
                                         in this field. Now reset the result.
                                         Otherwise the main while loop will
                                         exit. */
                                    result = USB_HOST_HID_RESULT_SUCCESS;
                                }/* if usage min max used */
                            }/* if Generic Desktop Page */
                        }/* Main item = INPUT */
                        else if(mainItem.tag ==
                                    USB_HID_MAIN_ITEM_TAG_OUTPUT)
                        {
                            /* Change report offset as they
                             * create data fields*/
                            reportOffset = reportOffset +
                                        (mainItem.globalItem)->reportCount *
                                        (mainItem.globalItem)->reportSize;
                        }
                        else if(mainItem.tag ==
                                    USB_HID_MAIN_ITEM_TAG_FEATURE)
                        {
                            /* Change report offset as they
                             * create data fields*/
                            reportOffset = reportOffset +
                                    (mainItem.globalItem)->reportCount *
                                    (mainItem.globalItem)->reportSize;
                        }
                    }/* Field found */
                    index++;
                } while(result == USB_HOST_HID_RESULT_SUCCESS);
                
                if(appMouseHandler != NULL)
                {
                    appMouseHandler((USB_HOST_HID_MOUSE_HANDLE)handle,
                                USB_HOST_HID_MOUSE_EVENT_REPORT_RECEIVED,
                                (void *)&mouseData[mouseIndex].appData);
                }
                
                if(mouseData[mouseIndex].taskPingPong)
                {
                    mouseData[mouseIndex].isPingReportProcessing = false;
                }
                else
                {
                    mouseData[mouseIndex].isPongReportProcessing = false;
                }
            }/* end of report processing */
            break;
        default:
            break;
    }
}/* End of _USB_HOST_HID_MOUSE_Task() */

