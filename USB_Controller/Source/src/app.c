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

#include "app.h"
#include "spi_dma.h"
#include <stdbool.h>


APP_DATA appData;

/* Transmit data buffer */
uint8_t  transmitDataBuffer[APP_READ_BUFFER_SIZE] APP_MAKE_BUFFER_DMA_READY;
uint8_t usb_packet_buffer[APP_READ_BUFFER_SIZE * BUFFER_COUNT] APP_MAKE_BUFFER_DMA_READY;

/* The endpoint size is 64 for FS and 512 for HS */
uint16_t endpointSize;

usb_packet_data_t packets[BUFFER_COUNT];        //linked list of packet (pointer pointing to usb_packet_buffer)

usb_packet_data_t *packet_read = &packets[0];
usb_packet_data_t *packet_write = &packets[0];
display_line_t display_spi_line_buffer[2] APP_MAKE_BUFFER_DMA_READY = {{.available_length = 0, .command = 0, .spi_busy = false, .next = &display_spi_line_buffer[1]},
                                                                  {.available_length = 0, .command = 0, .spi_busy = false, .next = &display_spi_line_buffer[0]}};

display_line_t *display_spi_line = &display_spi_line_buffer[0]; //porinter to linked list , but only one is used. WIP
volatile static uint32_t total;


void printf32(char *format, ...)
{   
    static char buffer[1024];
    va_list args;
    va_start(args, format);
    
    uint16_t n = vsprintf(buffer , format, args);
    appData.drvBufferEventComplete = false;
    DRV_USART_BufferAddWrite( appData.usartHandle, &(appData.usartBufferHandle), buffer, n);
    
    va_end(args);
    //while (appData.drvBufferEventComplete == false)  ;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/*********************************************
 * Application USB Device Layer Event Handler
 *********************************************/

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context)
{
    uint8_t * configurationValue;
    USB_SETUP_PACKET * setupPacket;
    switch(event)
    {
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:

            /* Device is reset or deconfigured. Provide LED indication.*/
            BSP_LEDOn  (APP_USB_LED_1);
            BSP_LEDOn (APP_USB_LED_2);
            BSP_LEDOff (APP_USB_LED_3);

            appData.deviceIsConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuration */
            configurationValue = (uint8_t *)eventData;
            if(*configurationValue == 1 )
            {
                /* The device is in configured state. Update LED indication */
                BSP_LEDOff  (APP_USB_LED_1);
                BSP_LEDOff (APP_USB_LED_2);
                BSP_LEDOn (APP_USB_LED_3);

                /* Reset endpoint data send & receive flag  */
                appData.deviceIsConfigured = true;
            }
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Device is suspended. Update LED indication */
            BSP_LEDOff  (APP_USB_LED_1);
            BSP_LEDOn (APP_USB_LED_2);
            BSP_LEDOn (APP_USB_LED_3);
            break;


        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS is detected. Attach the device */
            USB_DEVICE_Attach(appData.usbDevHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is removed. Detach the device */
            USB_DEVICE_Detach (appData.usbDevHandle);
            break;

        case USB_DEVICE_EVENT_CONTROL_TRANSFER_SETUP_REQUEST:
            /* This means we have received a setup packet */
            setupPacket = (USB_SETUP_PACKET *)eventData;
            if(setupPacket->bRequest == USB_REQUEST_SET_INTERFACE)
            {
                /* If we have got the SET_INTERFACE request, we just acknowledge
                 for now. This demo has only one alternate setting which is already
                 active. */
                USB_DEVICE_ControlStatus(appData.usbDevHandle,USB_DEVICE_CONTROL_STATUS_OK);
            }
            else if(setupPacket->bRequest == USB_REQUEST_GET_INTERFACE)
            {
                /* We have only one alternate setting and this setting 0. So
                 * we send this information to the host. */

                USB_DEVICE_ControlSend(appData.usbDevHandle, &appData.altSetting, 1);
            }
            else
            {
                /* We have received a request that we cannot handle. Stall it*/
                USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
            }
            break;

        case USB_DEVICE_EVENT_ENDPOINT_READ_COMPLETE:
        {
            
           /* Endpoint read is complete */ 
                packet_read->max_length = packet_read->available_data_length = max(((USB_DEVICE_EVENT_DATA_ENDPOINT_WRITE_COMPLETE *)eventData)->length, 2) - USB_HEADER_LENGTH; //get payload 
                
                LATFbits.LATF5 =  ~LATFbits.LATF5;                  //debugging stuff
                LATE = packet_read->display_packet->package_index;  //debugging stuff
     
                packet_read->is_scheduled = false;
                  
                packet_read = packet_read->next;

                appData.epDataReadPending = false;
           
                
            break;
        }
        case USB_DEVICE_EVENT_ENDPOINT_WRITE_COMPLETE:
            /* Endpoint write is complete */
            appData.epDataWritePending = false;
            break;

        /* These events are not used in this demo. */
        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}



void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    appData.usbDevHandle = USB_DEVICE_HANDLE_INVALID;
    appData.deviceIsConfigured = false;
    appData.endpointRx = (APP_EP_BULK_OUT | USB_EP_DIRECTION_OUT);
    appData.endpointTx = (APP_EP_BULK_IN | USB_EP_DIRECTION_IN);
    appData.epDataReadPending = false;
    appData.epDataWritePending = false;
    appData.altSetting = 0;
}

void APP_BufferEventHandler(DRV_USART_BUFFER_EVENT buffEvent,
                            DRV_USART_BUFFER_HANDLE hBufferEvent,
                            uintptr_t context )
{
    switch(buffEvent)
    {
        /* Buffer event is completed successfully */
        case DRV_USART_BUFFER_EVENT_COMPLETE:
        {
            if(context == APP_DRV_CONTEXT)
            {
                /* Update buffer event status */
                appData.drvBufferEventComplete = true;
            }
        }
            
            break;

        /* Buffer event has some error */
        case DRV_USART_BUFFER_EVENT_ERROR:
            break;

        /* Buffer event has aborted */
        case DRV_USART_BUFFER_EVENT_ABORT:
            break;
    }
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */
#define APP_USART_DRIVER_INDEX  DRV_USART_INDEX_0

void APP_Tasks (void )
{
    switch(appData.state)
    {
        case APP_STATE_INIT:
            /* Open the device layer */
            appData.usbDevHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0,
                    DRV_IO_INTENT_READWRITE );

            if(appData.usbDevHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.usbDevHandle,  APP_USBDeviceEventHandler, 0);
                 appData.usartHandle = DRV_USART_Open(APP_USART_DRIVER_INDEX, (DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING));
               //  DRV_USART_BufferEventHandlerSet(appData.usartHandle, APP_BufferEventHandler,APP_DRV_CONTEXT);
                SYS_INT_SourceEnable(INT_SOURCE_USART_1_ERROR);
                SYS_INT_SourceEnable(INT_SOURCE_USART_1_TRANSMIT);
                

                for (uint16_t i=0; i < BUFFER_COUNT; i++)
                {
                    
                    packets[i] = (usb_packet_data_t ){.display_packet = (usb_display_paket_t *)(usb_packet_buffer + (USB_READ_SIZE * i)), 
                                                     .available_data_length = 0, 
                                                     .next = &packets[(i+1)%BUFFER_COUNT]}; //mod to get last one point to first
                }

      
                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }
 
            TRISFbits.TRISF5 = 0;
            TRISBbits.TRISB2 = 1;
            CNPUBbits.CNPUB2 = 1;
            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device is configured */
            if(appData.deviceIsConfigured == true)
            {
                if (USB_DEVICE_ActiveSpeedGet(appData.usbDevHandle) == USB_SPEED_FULL)
                {
                    endpointSize = 64;
                }
                else if (USB_DEVICE_ActiveSpeedGet(appData.usbDevHandle) == USB_SPEED_HIGH)
                {
                    endpointSize = APP_READ_BUFFER_SIZE;
                }
                
                if (USB_DEVICE_EndpointIsEnabled(appData.usbDevHandle, appData.endpointRx) == false )
                {
                    /* Enable Read Endpoint */
                    USB_DEVICE_EndpointEnable(appData.usbDevHandle, 0, appData.endpointRx,
                            USB_TRANSFER_TYPE_BULK, endpointSize);
                }
                
                if (USB_DEVICE_EndpointIsEnabled(appData.usbDevHandle, appData.endpointTx) == false )
                {
                    /* Enable Write Endpoint */
                    USB_DEVICE_EndpointEnable(appData.usbDevHandle, 0, appData.endpointTx,
                            USB_TRANSFER_TYPE_BULK, endpointSize);
                }

                /* Device is ready to run the main task */
                appData.state = APP_STATE_MAIN_TASK;
            }
            break;

        case APP_STATE_MAIN_TASK:
        {
       
            if(!appData.deviceIsConfigured)
            {
                /* This means the device got deconfigured. Change the
                 * application state back to waiting for configuration. */
                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;

                /* Disable the endpoint*/
                USB_DEVICE_EndpointDisable(appData.usbDevHandle, appData.endpointRx);
                USB_DEVICE_EndpointDisable(appData.usbDevHandle, appData.endpointTx);
                appData.epDataReadPending = false;
                appData.epDataWritePending = false;
            }
            else if (appData.epDataReadPending == false)
            {
                usb_packet_data_t *packet = packet_read;
                USB_DEVICE_RESULT result = USB_DEVICE_RESULT_OK;
                static uint8_t buffer_handle_index = 0;
                do 
                {
                    if (!packet->is_scheduled)
                    {
                        result = USB_DEVICE_EndpointRead( appData.usbDevHandle, 
                                &appData.readTranferHandle[buffer_handle_index++ % USB_DEVICE_ENDPOINT_QUEUE_DEPTH_COMBINED],
                                appData.endpointRx, packet->display_packet, APP_READ_BUFFER_SIZE);
                        if (result == USB_DEVICE_RESULT_OK)
                        {
                            packet->is_scheduled = true;
                        }
                    }
                    
                    packet = packet->next;
                }while (result == USB_DEVICE_RESULT_OK);

                appData.epDataReadPending = true;
            }
            static uint16_t line_index;
            static uint8_t last_Frame_ID;
            if (packet_write->available_data_length  && (packet_write->max_length >= packet_write->available_data_length))
            {
                if ( display_spi_line->available_length < LINE_LENGTH )
                {          
                   
                    BSP_LEDToggle( APP_USB_LED_1 );
                    
                    if ( packet_write->display_packet->Frame_ID != last_Frame_ID
                         && packet_write->display_packet->package_index == 0 
                        && packet_write->available_data_length == packet_write->max_length )
                    {
                        display_spi_line->available_length = 0;
                        line_index =0;
                        BSP_LEDToggle( APP_USB_LED_2 );
                        
                        if ( total != 115200)   //Debugging: detects issues with missing packets
                        {
                            LATDbits.LATD9 = ~LATDbits.LATD9;       
                        }
                        total = 0;
                        
                        last_Frame_ID = packet_write->display_packet->Frame_ID;
                    }
                      
                
                    uint32_t ptr = (uint32_t)&(packet_write->display_packet->data[0]) + (packet_write->max_length  - packet_write->available_data_length);
                    uint16_t tocopy = min(LINE_LENGTH - display_spi_line->available_length, packet_write->available_data_length);
                    total = total + tocopy;


                    memcpy(&(display_spi_line->data[display_spi_line->available_length])
                            ,(void *)ptr
                            , tocopy);

                    packet_write->available_data_length = packet_write->available_data_length - tocopy;
                    display_spi_line->available_length = display_spi_line->available_length + tocopy;

                    if (!(packet_write->available_data_length))
                    {
                        packet_write->max_length = 0;

                        packet_write = packet_write->next;
                    }
                }
            }
                        
        
            if (!display_spi_line->spi_busy)        //Do not check in same if, because available_length and spi_busy seem to updating in incorrect order may be because non cached section? 
            {
                if (display_spi_line->available_length == LINE_LENGTH)
                {
                    if (line_index == 0)
                    {
                        display_spi_line->command = 0x3F;       //fpga cmd write first line(from FPGA ROM)
                    }
                    else
                    {
                        display_spi_line->command = 0x6B;       //fpga write next lines
                    }

                    line_index ++;
                    transfer_spi_dma(display_spi_line); //command and data are consecutive in memory 
                }
            }
            
            static uint8_t image_number;
            if (!PORTBbits.RB2) //test pattern key press
            {
                for (uint8_t line =0; line <240; line ++)
                {
                    if (line == 0)
                    {
                        display_spi_line->command = 0x3F;       //fpga cmd write first line(from FPGA ROM)
                        BSP_LEDToggle( APP_USB_LED_2 );
                    }
                    else
                    {
                        display_spi_line->command = 0x6B;       //fpga write next lines
                    }
                    
                    #ifdef TPTYPE_1
                        memcpy(display_spi_line->data, image1[image_number & 0x01] + (line *480), 480);
                    #else
                        memcpy(display_spi_line->data, line_tp , 480);
                    #endif
                        
                    display_spi_line->available_length = 480;
                    transfer_spi_dma(display_spi_line); //command and data are consecutive in memory  
                    while (display_spi_line->spi_busy);
                }

                image_number++;
                for(uint32_t delay=0; delay < 300000; delay++);
            }   
            
            break;
        }
        case APP_STATE_ERROR:
            break;

        default:
            break;
    }
}
 

/*******************************************************************************
 End of File
 */

