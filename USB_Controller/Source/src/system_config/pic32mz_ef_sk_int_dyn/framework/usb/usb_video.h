/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _USB_VIDEO_H    /* Guard against multiple inclusion */
#define _USB_VIDEO_H

#include <stdint.h>
#include <stdbool.h>
#include "system_config.h"
#include "usb/usb_common.h"
#include "usb/usb_chapter_9.h"
#include "usb/usb_device.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  
// *****************************************************************************
        
    
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Video Interface Class codes

  Summary:
    Identifies the Class Codes for Video interface. 

  Description:
    This constant identifies the value of the Video Interface class code. 

  Remarks:
    None.
*/
#define  USB_VIDEO_CLASS_CODE 0x0E

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Video Interface subclass codes

  Summary:
    Identifies the subclass codes for Video interface. 

  Description:
    This enumeration identifies the possible subclass codes for 
    video interface. 

  Remarks:
    The "ISC" in the enumeration member names is an acronym for Interface 
    Subclass Code.
*/


typedef enum
{
    USB_VIDEO_SUBCLASS_UNDEFINED    = 0x00,
    USB_VIDEO_VIDEOCONTROL          = 0x01,
    USB_VIDEO_VIDEOSTREAMING        = 0x02,
} USB_VIDEO_SUBCLASS_CODE; 

// *****************************************************************************
/* Video Interface Protocol codes

  Summary:
    Identifies the protocol codes for Video interface. 

  Description:
    This enumeration identifies the possible protocol codes for 
    video interface

  Remarks:
    As per USB Device class definition for Video Device release v1.0, the
    protocol code should always be 0.
*/

typedef enum
{
    USB_VIDEO_PR_PROTOCOL_UNDEFINED    = 0x0

} USB_VIDEO_PROTOCOL_CODE;

// *****************************************************************************
/* Video Class Specific Descriptor Types

  Summary:
    Identifies the Video class specific descriptor types for Video. 

  Description:
    This enumeration identifies the Video Class specific descriptor types. 

  Remarks:
    The "CS" in the enumeration member names is an acronym for Class Specific.
*/

typedef enum
{
    USB_VIDEO_CS_UNDEFINED       = 0x20,
    USB_VIDEO_CS_DEVICE          = 0x21,
    USB_VIDEO_CS_CONFIGURATION   = 0x22,
    USB_VIDEO_CS_STRING          = 0x23,
    USB_VIDEO_CS_INTERFACE       = 0x24,
    USB_VIDEO_CS_ENDPOINT        = 0x25

} USB_VIDEO_CS_DESCRIPTOR_TYPE;

// *****************************************************************************
/* Video Class Specific AC Interface Descriptor Subtypes

  Summary:
    Identifies the Video Class Specific AC Interface Descriptor Subtypes.

  Description:
    This enumeration identifies the possible Video Class Specific AC Interface
    Descriptor Subtypes.

  Remarks:
    The "CS" in the enumeration member names is an acronym for Class Specific.
    The "AC" in the enumeration member names is an acronym for Video Control.
*/

typedef enum
{
    USB_VIDEO_AC_DESCRIPTOR_UNDEFINED    = 0x00,
    USB_VIDEO_HEADER                     = 0x01,
    USB_VIDEO_INPUT_TERMINAL             = 0x02,
    USB_VIDEO_OUTPUT_TERMINAL            = 0x03,
    USB_VIDEO_MIXER_UNIT                 = 0x04,
    USB_VIDEO_SELECTOR_UNIT              = 0x05,
    USB_VIDEO_FEATURE_UNIT               = 0x06,
    USB_VIDEO_PROCESSING_UNIT            = 0x07,
    USB_VIDEO_EXTENSION_UNIT             = 0x08,

} USB_VIDEO_CS_AC_INTERFACE_DESCRIPTOR_SUBTYPE,
USB_VIDEO_V1_ENTITY_TYPE; 

// *****************************************************************************
/* Video Class Specific Terminal Types. 

  Summary:
    Identifies the Video Class Specific Video Class Specific Terminal Types.

  Description:
    This enumeration identifies the possible Video Class Specific Terminal Types.
    This is enumeration is as per the Table 2-1, 2-2 and 2-3 of the document 
    Universal Serial Bus Device Class Definition for Terminal Types. 

  Remarks:
    The "CS" in the enumeration member names is an acronym for Class Specific. 
*/
typedef enum 
{
    /* USB Terminal Types */ 
    USB_VIDEO_TERMINAL_TYPE_USB_UNDEFINED = 0x0100,
    USB_VIDEO_TERMINAL_TYPE_USB_STREAMING = 0x0101,
    USB_VIDEO_TERMINAL_TYPE_USB_VENDOR_SPECIFIC = 0x01FF,
         
    /* Input Terminal types */ 
    USB_VIDEO_TERMINAL_TYPE_INPUT_UNDEFINED = 0x0200, 
    USB_VIDEO_TERMINAL_TYPE_INPUT_MICROPHONE = 0x0201,
    USB_VIDEO_TERMINAL_TYPE_INPUT_MICROPHONE_DESKTOP = 0x0202,
    USB_VIDEO_TERMINAL_TYPE_INPUT_MICROPHONE_PERSONAL = 0x0203,
    USB_VIDEO_TERMINAL_TYPE_INPUT_MICROPHONE_OMNI = 0x0204,
    USB_VIDEO_TERMINAL_TYPE_INPUT_MICROPHONE_ARRAY = 0x0205,
    USB_VIDEO_TERMINAL_TYPE_INPUT_MICROPHONE_ARRAY_PROCESSING = 0x0206,
       
    /* Output Terminal types */         
    USB_VIDEO_TERMINAL_TYPE_OUTPUT_UNDEFINED = 0x0300,
    USB_VIDEO_TERMINAL_TYPE_OUTPUT_SPEAKER = 0x0301,
    USB_VIDEO_TERMINAL_TYPE_OUTPUT_HEADPHONES = 0x0302,
    USB_VIDEO_TERMINAL_TYPE_OUTPUT_HMD = 0x0303,
    USB_VIDEO_TERMINAL_TYPE_OUTPUT_SPEAKER_DESKTOP = 0x0304,
    USB_VIDEO_TERMINAL_TYPE_OUTPUT_SPEAKER_ROOM = 0x0305,
    USB_VIDEO_TERMINAL_TYPE_OUTPUT_SPEAKER_COMM = 0x0306,
    USB_VIDEO_TERMINAL_TYPE_OUTPUT_SPEAKER_LFE = 0x0307
            
}USB_VIDEO_V1_TERMINAL_TYPE; 

// *****************************************************************************
/* Video Class Specific AS Interface Descriptor Subtypes

  Summary:
    Identifies the Video Class Specific AS Interface Descriptor Subtypes. 

  Description:
    This enumeration identifies the possible Video Class Specific AS Interface 
    Descriptor Subtypes. 

  Remarks:
    The "CS" in the enumeration member names is an acronym for Class Specific.
    The "AS" in the enumeration member names is an acronym for Video Streaming.
*/

typedef enum
{
    USB_VIDEO_AS_DESCRIPTOR_UNDEFINED    = 0x00,
    USB_VIDEO_AS_GENERAL                 = 0x01,
    USB_VIDEO_FORMAT_TYPE                = 0x02,
    USB_VIDEO_FORMAT_SPECIFIC            = 0x03

} USB_VIDEO_CS_AS_INTERFACE_DESCRIPTOR_SUBTYPE;

// *****************************************************************************
/* Video Processing Unit Process Types

  Summary:
    Identifies the Video Process Unit Process Types.

  Description:
    This enumeration identifies the possible Video Process Unit Process types. 

  Remarks:
    None.
*/

typedef enum
{
    USB_VIDEO_PROCESS_UNDEFINED             = 0x00,
    USB_VIDEO_UP_DOWNMIX_PROCESS            = 0x01,
    USB_VIDEO_DOLBY_PROLOGIC_PROCESS        = 0x02,
    USB_VIDEO_3D_STEREO_EXTENDER_PROCESS    = 0x03,
    USB_VIDEO_REVERBERATION_PROCESS         = 0x04,
    USB_VIDEO_CHORUS_PROCESS                = 0x05,   
    USB_VIDEO_DYN_RANGE_COMP_PROCESS        = 0x06

} USB_VIDEO_PROCESSING_UNIT_PROCESS_TYPE;

// *****************************************************************************
/* Video Class Specific Endpoint Descriptor Subtypes.

  Summary:
    Identifies the Video Class Specific Endpoint Descriptor Subtypes.

  Description:
    This enumeration identifies the possible Video Class Specific Endpoint 
    Descriptor Subtypes.

  Remarks:
    The "CS" in the enumeration member names is an acronym for Class Specific.
*/

typedef enum
{
    USB_VIDEO_DESCRIPTOR_UNDEFINED    = 0x00,
    USB_VIDEO_EP_GENERAL              = 0x01

} USB_VIDEO_CS_ENDPOINT_DESCRIPTOR_SUBTYPE;

// *****************************************************************************
/* Video Class Specific Request Codes.

  Summary:
    Identifies the Video Class Specific Request Codes.

  Description:
    This enumeration identifies the possible Video Class Specific Request codes.

  Remarks:
    The "CS" in the enumeration member names is an acronym for Class Specific.
*/

typedef enum
{
    USB_VIDEO_CS_REQUEST_CODE_UNDEFINED = 0x00,
    USB_VIDEO_CS_SET_CUR   = 0x01,
    USB_VIDEO_CS_GET_CUR   = 0x81,
    USB_VIDEO_CS_SET_MIN   = 0x02,
    USB_VIDEO_CS_GET_MIN   = 0x82,
    USB_VIDEO_CS_SET_MAX   = 0x03,
    USB_VIDEO_CS_GET_MAX   = 0x83,
    USB_VIDEO_CS_SET_RES   = 0x04,
    USB_VIDEO_CS_GET_RES   = 0x84,
    USB_VIDEO_CS_SET_MEM   = 0x05,
    USB_VIDEO_CS_GET_MEM   = 0x85,
    USB_VIDEO_CS_GET_INFO  = 0x86,
    USB_VIDEO_CS_GET_DEF   = 0x87,
    USB_VIDEO_CS_GET_STAT  = 0xFF

} USB_VIDEO_CS_REQUEST_CODE;

// *****************************************************************************
/* Video Terminal Control Selectors.

  Summary:
    Identifies the Video Terminal Control Selectors.

  Description:
    This enumeration identifies the possible Video Terminal Control Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_VIDEO_TE_CONTROL_UNDEFINED      = 0x00,
    USB_VIDEO_COPY_PROTECT_CONTROL      = 0x01

} USB_VIDEO_TERMINAL_CONTROL_SELECTOR;

// *****************************************************************************
/* Video Feature Unit Control Selector.

  Summary:
    Identifies the Video Feature Unit Control Selector.

  Description:
    This enumeration identifies the possible Video Feature Unit Control
    Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_VIDEO_FU_CONTROL_UNDEFINED        = 0x00,
    USB_VIDEO_MUTE_CONTROL                = 0x01,
    USB_VIDEO_VOLUME_CONTROL              = 0x02,
    USB_VIDEO_BASS_CONTROL                = 0x03,
    USB_VIDEO_MID_CONTROL                 = 0x04,
    USB_VIDEO_TREBLE_CONTROL              = 0x05,
    USB_VIDEO_GRAPHIC_EQUALIZER_CONTROL   = 0x06,
    USB_VIDEO_AUTOMATIC_GAIN_CONTROL      = 0x07,
    USB_VIDEO_DELAY_CONTROL               = 0x08,
    USB_VIDEO_BASS_BOOST_CONTROL          = 0x09,
    USB_VIDEO_LOUDNESS_CONTROL            = 0x0A

} USB_VIDEO_FEATURE_UNIT_CONTROL_SELECTORS;    

// *****************************************************************************
/* Video Up/Down-mix Processing Unit Control Selector.

  Summary:
    Identifies the Video Up/Down-mix Processing Unit Control Selector.

  Description:
    This enumeration identifies the possible Video Up/Down-mix Processing Unit 
    Control Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_VIDEO_UD_CONTROL_UNDEFINED     = 0x00,
    USB_VIDEO_UD_ENABLE_CONTROL        = 0x01,
    USB_VIDEO_UD_MODE_SELECT_CONTROL   = 0x02

} USB_VIDEO_UP_DOWN_MIX_PROCESSING_UNIT_CONTROL_SELECTORS;

// *****************************************************************************
/* Video Dolby Prologic Processing Unit Control Selector.

  Summary:
    Identifies the Video Dolby Prologic Processing Unit Control Selector.

  Description:
    This enumeration identifies the possible Video Dolby Prologic Processing Unit 
    Control Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_VIDEO_DP_CONTROL_UNDEFINED      = 0x00,
    USB_VIDEO_DP_ENABLE_CONTROL         = 0x01,
    USB_VIDEO_DP_MODE_SELECT_CONTROL    = 0x02

} USB_VIDEO_DOLBY_PROLOGIC_PROCESSING_UNIT_CONTROL_SELECTOR;

// *****************************************************************************
/* Video 3D Stereo Extender Processing Unit Control Selector.

  Summary:
    Identifies the Video 3D Stereo Extender Processing Unit Control Selector.

  Description:
    This enumeration identifies the possible Video 3D Stereo Extender Processing
    Unit Control Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_VIDEO_3D_CONTROL_UNDEFINED      = 0x00,
    USB_VIDEO_3D_ENABLE_CONTROL         = 0x01,
    USB_VIDEO_SPACIOUSNESS_CONTROL      = 0x02

} USB_VIDEO_3D_STEREO_EXTENDER_PROCESSING_UNIT_CONTROL_SELECTOR;

// *****************************************************************************
/* Video Reverberation Processing Unit Control Selector.

  Summary:
    Identifies the Video Reverberation Processing Unit Control Selector.

  Description:
    This enumeration identifies the possible Video Reverberation Processing Unit
    Control Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_VIDEO_RV_CONTROL_UNDEFINED      = 0x00,
    USB_VIDEO_RV_ENABLE_CONTROL         = 0x01,
    USB_VIDEO_REVERB_LEVEL_CONTROL      = 0x02,
    USB_VIDEO_REVERB_TIME_CONTROL       = 0x03,
    USB_VIDEO_REVERB_FEEDBACK_CONTROL   = 0x04

} USB_VIDEO_REVERBERATION_PROCESSING_UNIT_CONTROL_SELECTORS;

// *****************************************************************************
/* Video Chorus Processing Unit Control Selector.

  Summary:
    Identifies the Video Chorus Processing Unit Control Selector.

  Description:
    This enumeration identifies the possible Video Chorus Processing Unit 
    Control Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_VIDEO_CH_CONTROL_UNDEFINED  = 0x00,
    USB_VIDEO_CH_ENABLE_CONTROL     = 0x01,
    USB_VIDEO_CHORUS_LEVEL_CONTROL  = 0x02,
    USB_VIDEO_CHORUS_RATE_CONTROL   = 0x03,
    USB_VIDEO_CHORUS_DEPTH_CONTROL  = 0x04

} USB_VIDEO_CHORUS_PROCESSING_UNIT_CONTROL_SELECTORS;

// *****************************************************************************
/* Video Dynamic Range Compressor Processing Unit Control Selector.

  Summary:
    Identifies the Video Dynamic Range Compressor Processing Unit Control
    Selector.

  Description:
    This enumeration identifies the possible Video Dynamic Range Compressor 
    Processing Unit Control Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_VIDEO_DR_CONTROL_UNDEFINED      = 0x00,
    USB_VIDEO_DR_ENABLE_CONTROL         = 0x01,
    USB_VIDEO_COMPRESSION_RATE_CONTROL  = 0x02,
    USB_VIDEO_MAXAMPL_CONTROL           = 0x03,
    USB_VIDEO_THRESHOLD_CONTROL         = 0x04,
    USB_VIDEO_ATTACK_TIME               = 0x05,
    USB_VIDEO_RELEASE_TIME              = 0x06

} USB_VIDEO_DYNAMIC_RANGE_COMPRESSOR_PROCESSING_UNIT_CONTROL_SELECTORS;

// *****************************************************************************
/* Video Extension Unit Control Selector.

  Summary:
    Identifies the Video Extension Unit Control Selector.

  Description:
    This enumeration identifies the possible Video Extension Unit Control
    Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_VIDEO_XU_CONTROL_UNDEFINED  = 0x00,
    USB_VIDEO_XU_ENABLE_CONTROL     = 0x01

} USB_VIDEO_EXTENSION_UNIT_CONTROL_SELECTORS;

// *****************************************************************************
/* Video Endpoint Control Selector.

  Summary:
    Identifies the Video Endpoint Control Selector.

  Description:
    This enumeration identifies the possible Video Endpoint Control Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_VIDEO_EP_CONTROL_UNDEFINED  = 0x00,
    USB_VIDEO_SAMPLING_FREQ_CONTROL = 0x01,
    USB_VIDEO_PITCH_CONTROL         = 0x02

} USB_VIDEO_ENDPOINT_CONTROL_SELECTORS;

// *****************************************************************************
/* USB Video Copy Protect Control Codes.

  Summary:
    Identifies the Copy Control codes for USB Video Class.

  Description:
    This enumeration identifies the possible Copy Control Protect codes for USB
    Video Class. Refer the document "Universal Serial Bus Device Class
    Definition for Video Devices revision 1.0" Section 5.2.2.1.3.1.

  Remarks:
    None.
*/
typedef enum
{
    /* Copying is permitted without restriction.  The material is either not
       copyrighted or the copyright is not asserted */
    USB_VIDEO_CPL0 = 0x00 ,

    /* One generation of copies can be made. The material is copyright protected
       and is the original */
    USB_VIDEO_CPL1 = 0x01 ,

    /* The material is copyright protected and no digital copying is permitted
       */
    USB_VIDEO_CPL2 = 0x02

} USB_VIDEO_COPY_PROTECT_CONTROL_PARAMETER;

// *****************************************************************************
/* USB Video Format Codes.

  Summary:
    Identifies the USB Video Format codes for USB Video Class.

  Description:
    This enumeration identifies the possible USB Video Format codes for USB
    Video Class. Refer the document "Universal Serial Bus Device Class Definition
    for Video Data Formats" Table A-1, Table A-2 and Table A-3.

  Remarks:
    None.
*/
typedef enum
{
    USB_VIDEO_FORMAT_TYPE_I_UNDEFINED = 0x0000,

    USB_VIDEO_FORMAT_PCM = 0x0001,

    USB_VIDEO_FORMAT_PCM8 = 0x0002,

} USB_VIDEO_FORMAT_CODE,
  USB_VIDEO_V1_FORMAT_TAG;

// *****************************************************************************
/* USB Video Status Interrupt Endpoint Status Word Format.

  Summary:
    Specifies the format of USB Video Status Interrupt Endpoint Status Word
    Format.

  Description:
    This type identifies the format of the USB Video Status Interrupt Endpoint
    Status Word. This is defined in Table 3-1 of the USB Device Class Definition 
    for Video Devices Release 1.0 document.

  Remarks:
    Always needs to be packed.
*/

typedef union __attribute__((packed))
{
    uint16_t value;

    struct
    {
        unsigned bStatusType:8;
        unsigned bOriginator:8; 
    };
    struct
    {
        unsigned originator:4;
        unsigned :2;
        unsigned memoryContentsChanged:1;
        unsigned interruptPending:1;
        unsigned :8;
    };
} USB_VIDEO_INTERRUPT_STATUS_WORD;

// *****************************************************************************
/* USB Video Class Specific Video Control Interface Header Descriptor

  Summary:
    Identifies the USB Video Class Specific Video Control Interface Header
    Descriptor.

  Description:
    This type identifies the USB Video Class Specific Video Control Interface
    Header Descriptor. This structure is as per Table 4-2 of the USB Device
    Class Definition for Video Device v1.0 document.

  Remarks:
    Always needs to be packed.
*/
   
                        /* Class specific VC Interface Header Descriptor */

typedef struct __attribute__ ((packed))
{
	/* Size of this descriptor */
    uint8_t bLength;
	
	/* Interface descriptor type */
    uint8_t bDescriptorType;
	
	/* Interface Descriptor Subtype */
    uint8_t bDescriptorSubtype;
	
    /* Video Device Class Specification Release Number in BCD format*/	
    uint16_t bcdADC;
	
    /* Total number of bytes returned for the class-specific VideoControl
       interface descriptor. Includes the combined length of this descriptor
       header and all Unit and Terminal descriptors */
    uint16_t wTotalLength;

    
    uint32_t frequency;
    /* The number of VideoStreaming and MIDIStreaming interfaces in the Video
       Interface Collection to which this VideoControl interface belongs. This
       is equal to USB_DEVICE_VIDEO_STREAMING_INTERFACES_NUMBER.*/
    uint8_t bInCollection;

    /* Note: The subsequent members of this structure should contain the
     * interface numbers of Video and MIDI streaming interfaces in the
     * collection. The total number of interface entries should match the value
     * of bInCollection member of this structure. The type of each entry should
     * be USB_DEVICE_VIDEO_STREAMING_INTERFACE_NUMBER. */

} USB_VIDEO_CS_AC_INTERFACE_HEADER_DESCRIPTOR;

// *****************************************************************************
/* Video Input Terminal Descriptor Type

  Summary:
    Identifies the Video Input Terminal Descriptor Type.

  Description:
    This type identifies the Video Input Terminal Descriptor.  This structure is
    as per Table 4-3 of the USB Device Class Definition for Video Device v1.0
    document.

  Remarks:
    Always needs to be packed.
*/

typedef struct __attribute__ ((packed))
{
    /* Size of this descriptor */
    uint8_t bLength;

    /* Interface descriptor type */
    uint8_t bDescriptorType;

    /* Interface Descriptor Subtype */
    uint8_t bDescriptorSubtype;

    /* Unique Terminal Identifier Constant */
    uint8_t bTerminalID;

    /* Terminal Type */
    uint16_t wTerminalType;

    /* ID of the associated Output Terminal */
    uint8_t bAssocTerminal;

    /* Number of channels in the terminal output */
    uint8_t bNrChannels;

    /* Spatial location of the logical channels */
    uint16_t wChannelConfig;

    /* First Logical Channel String descriptor index */
    uint8_t iChannelNames;

    /* Input Terminal String Descriptor Index */
    uint8_t iTerminal;

} USB_VIDEO_INPUT_TERMINAL_DESCRIPTOR;

// *****************************************************************************
/* Video Output Terminal Descriptor Type

  Summary:
    Identifies the Video Output Terminal Descriptor Type.

  Description:
    This type identifies the Video Output Terminal Descriptor.  This structure
    is as per Table 4-4 of the USB Device Class Definition for Video Device v1.0
    document.

  Remarks:
    Always needs to be packed.
*/

typedef struct __attribute__ ((packed))
{
    /* Size of this descriptor */
    uint8_t bLength;

    /* Interface descriptor type */
    uint8_t bDescriptorType;

    /* Interface Descriptor Subtype */
    uint8_t bDescriptorSubtype;

    /* Unique Terminal Identifier Constant */
    uint8_t bTerminalID;

    /* Terminal Type */
    uint16_t wTerminalType;

    /* ID of the associated Input Terminal */
    uint8_t bAssocTerminal;

    /* Source Unit or Terminal ID */
    uint8_t bSourceID;

    /* Output Terminal String Descriptor Index */
    uint8_t iTerminal;

} USB_VIDEO_OUTPUT_TERMINAL_DESCRIPTOR;

// *****************************************************************************
/* Video Feature Unit Descriptor Header Type

  Summary:
    Identifies the Video Feature Unit Descriptor Type.

  Description:
    This type identifies the Video Feature Unit Descriptor.  This structure
    is as per Table 4-7 of the USB Device Class Definition for Video Device v1.0
    document.

  Remarks:
    Always needs to be packed.
*/

typedef struct __attribute__ ((packed))
{
    /* Size of this descriptor */
    uint8_t bLength;

    /* Interface descriptor type */
    uint8_t bDescriptorType;

    /* Interface Descriptor Subtype */
    uint8_t bDescriptorSubtype;

    /* Constant uniquely identifying the Unit within the video function.*/
    uint8_t bUnitID;

    /* Source Unit or Terminal ID */
    uint8_t bSourceID;

    /* Size in Bytes of an element in the Control array */
    uint8_t bControlSize;

} USB_VIDEO_FEATURE_UNIT_DESCRIPTOR_HEADER;

// *****************************************************************************
/* Video Feature Unit BMA Control

  Summary:
    Identifies the Video Feature Unit BMA control type.

  Description:
    This type identifies the Video Feature Unit BMA control.  This structure
    is as per Table 4-7 of the USB Device Class Definition for Video Device v1.0
    document.

  Remarks:
    Always needs to be packed.
*/
typedef union __attribute__ ((packed))
{
    uint16_t bmaControls;

    struct 
    {
        uint8_t mute: 1;
        uint8_t volume: 1;
        uint8_t bass:1;
        uint8_t mid:1;
        uint8_t treble:1;
        uint8_t graphicEqualizer:1;
        uint8_t automaticGain:1;
        uint8_t delay:1;
        uint8_t bassBoost:1;
        uint8_t loudness:1;
        unsigned :6; 

    }; 
} USB_VIDEO_FEATURE_UNIT_BMA_CONTROLS;

// *****************************************************************************
/* USB Video Class Specific Video Streaming Interface Descriptor

  Summary:
    Identifies the USB Video Class Specific Video Streaming Interface Descriptor
    Type.

  Description:
    This type identifies the USB Video Class Specific Video Streaming Interface
    Descriptor.  This structure is as per Table 4-19 of the USB Device Class
    Definition for Video Device v1.0 document.

  Remarks:
    Always needs to be packed.
*/

typedef struct __attribute__((packed))
{
    /* Size of the descriptor in bytes */
    uint8_t bLength;

    /* CS_INTERFACE descriptor type */
    uint8_t bDescriptorType;

    /* AS_GENERAL descriptor subtype */
    uint8_t bDescriptorSubtype;

    /* Terminal ID of the terminal to which the endpoint of this interface is
     * connected */
    uint8_t bTerminalLink;

    /* Delay introduced by the data path */
    uint8_t bDelay;

    /* Video data format of this interface */
    uint16_t wFormatTag;

} USB_VIDEO_CS_AS_INTERFACE_DESCRIPTOR;

// *****************************************************************************
/* USB Video Class Specific Video Streaming Isochronous Video Data Endpoint 
   Descriptor

  Summary:
    Identifies the USB Video Class Specific Video Streaming Isochronous Video
    Data Endpoint Descriptor

  Description:
    This type identifies the USB Video Class Specific Video Streaming
    Isochronous Video Data Endpoint Descriptor Type. This structure is as per
    Table 4-21 of the USB Device Class Definition for Video Device v1.0 document.

  Remarks:
    Always needs to be packed.
*/

typedef struct __attribute__((packed))
{
    /* Size of this descriptor */
    uint8_t bLength;

    /* CS_ENDPOINT descriptor type */
    uint8_t bDescriptorType;

    /* EP_GENERAL descriptor subtype */
    uint8_t bDescriptorSubtype;

    /* Bit map indicating the mentioned control is supported by this endpoint */
    uint8_t bmAttributes;

    /* Indicates the units used for the wLockDelay field */
    uint8_t bLockDelayUnits;

    /* Indicates the time it takes this endpoint to reliably lock its internal
     * clock recovery circuitry */
    uint16_t wLockDelay;

} USB_VIDEO_CS_AS_ISOCHRONOUS_VIDEO_DATA_EP_DESCRIPTOR;    
        

// *****************************************************************************
/* USB Video Feature Unit Control Set and Get Request.

  Summary:
    Identifies the type of the USB Video Feature Unit Control Set and Get Request.

  Description:
    This type identifies the type of the USB Video Feature Unit Control Set and
    Get Request.  The application can type cast the received video class
    specific setup packet to this type in order to service the feature unit
    control Set and Get requests. This structure is as per Table 5-14 and 5-15
    of the USB Device Class Definition for Video Device v1.0 document. Refer to
    section 5.2.2.4.1 and section 5.2.2.4.2 of this document for detailed
    description and request specific interpretation of the fields.

  Remarks:
    Always needs to be packed.
*/

typedef struct __attribute__((packed))
{
    /* Request type SET or GET */
    uint8_t bmRequestType;
    
    /* Identifies the attribute to be accessed */
    uint8_t bRequest;

    /* Identifies the channel number */
    uint8_t channelNumber;

    /* Identifies the control selector */
    uint8_t controlSelector;

    /* Identifies the interface number */
    uint8_t interfaceNumber;
    
    /* Identifies the feature unit ID */
    uint8_t featureUnitId;

    /* Length of the parameter block */
    uint16_t wLength;

} USB_VIDEO_FEATURE_UNIT_CONTROL_REQUEST;

/* USB Video Control Interface Set and Get Request.

  Summary:
    Identifies the type of the USB Video Control Interface Set and Get Request.

  Description:
    This type identifies the type of the USB Video Control Interface Set and Get
    Request.  The application can type cast the received video class specific
    setup packet to this type in order to service Control attribute Set and Get
    requests. This structure is as per Table 5-1 and 5-2 of the USB Device Class
    Definition for Video Device v1.0 document. Refer to section 5.2.1.1 and
    section 5.2.1.2 of this document for detailed description and request
    specific interpretation of the fields.

  Remarks:
    Always needs to be packed.
*/

typedef struct __attribute__((packed))
{
    /* Request type SET or GET */
    uint8_t bmRequestType;

    /* Identifies the attribute to be accessed */
    uint8_t bRequest;

    /* The wValue field */
    uint16_t wValue;

    union
    {
        /* Identifies the interface number */
        uint8_t interfaceNumber;

        /* Identifies the endpoint */
        uint8_t endpoint;
    };

    /* Identifies the entity ID */
    uint8_t entityID;

    /* Length of the parameter block */
    uint16_t wLength;

} USB_VIDEO_CONTROL_INTERFACE_REQUEST;

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* USB Device Video Function Driver Index

  Summary:
    USB Device Video function driver index.

  Description:
    This definition uniquely identifies a Video Function Driver instance.

  Remarks:
    None.
*/

typedef uintptr_t USB_DEVICE_VIDEO_INDEX;

// *****************************************************************************
/* USB Device Video Function Driver Transfer Handle Definition

  Summary:
    USB Device Video Function Driver transfer handle definition.

  Description:
    This definition defines a USB Device Video Function Driver Transfer Handle.
    A Transfer Handle is owned by the application but its value is modified by
    the USB_DEVICE_VIDEO_Write, USB_DEVICE_VIDEO_Read functions. The
    transfer handle is valid for the life time of the transfer and expires when
    the transfer related event had occurred.

  Remarks:
    None.
*/

typedef uintptr_t USB_DEVICE_VIDEO_TRANSFER_HANDLE;

// *****************************************************************************
/* USB Device Video Function Driver Invalid Transfer Handle Definition
 
  Summary:
    USB Device Video Function Driver invalid transfer handle definition.

  Description:
    This definition defines a Invalid USB Device Video Function Driver Transfer
    Handle.  A Invalid Transfer Handle is returned by the
    USB_DEVICE_Video_Write , USB_DEVICE_Video_Read, functions when the
    request was not successful.

  Remarks:
    None.
*/

#define USB_DEVICE_VIDEO_TRANSFER_HANDLE_INVALID  /*DOM-IGNORE-BEGIN*/((USB_DEVICE_VIDEO_TRANSFER_HANDLE)(-1))/*DOM-IGNORE-END*/

// *****************************************************************************
/* USB Device Video Function Driver Events

  Summary:
    USB Device Video Function Driver events.

  Description:
    These events are specific to a USB Device Video Function Driver instance.
    An event may have some data associated with it. This is provided to the
    event handling function. Each event description contains details about this
    event data (pData) and other parameters passed along with the event, to the
    event handler. 
    
    Events associated with the Video Function Driver Specific Control Transfers
    require application response. The application should respond to
    these events by using the USB_DEVICE_ControlReceive,
    USB_DEVICE_ControlSend and USB_DEVICE_ControlStatus functions.
    
    Calling the USB_DEVICE_ControlStatus function with a
    USB_DEVICE_CONTROL_STATUS_ERROR will stall the control transfer request.
    The application would do this if the control transfer request is not
    supported. Calling the USB_DEVICE_ControlStatus function with a
    USB_DEVICE_CONTROL_STATUS_OK will complete the status stage of the
    control transfer request. The application would do this if the control
    transfer request is supported. 
    
    The following code shows an example of a possible event handling scheme.
    
    <code>
    
    // This code example shows all USB Video Function Driver possible events and
    // a possible scheme for handling these events. In this case event responses
    // are not deferred.
    
    void APP_USBDeviceVideoEventHandler
    (
        USB_DEVICE_VIDEO_INDEX instanceIndex ,
        USB_DEVICE_VIDEO_EVENT event ,
        void * pData,
        uintptr_t context
    )
    {
        switch (event)
        {
            case USB_DEVICE_VIDEO_EVENT_READ_COMPLETE:

                // This event indicates that a Video Read Transfer request
                // has completed. pData should be interpreted as a 
                // USB_DEVICE_VIDEO_EVENT_DATA_READ_COMPLETE pointer type.
                // This contains the transfer handle of the read transfer
                // that completed and amount of data that was read.

                break;
            
            case USB_DEVICE_VIDEO_EVENT_WRITE_COMPLETE:
 
                // This event indicates that a Video Write Transfer request
                // has completed. pData should be interpreted as a 
                // USB_DEVICE_VIDEO_EVENT_DATA_WRITE_COMPLETE pointer type.
                // This contains the transfer handle of the write transfer
                // that completed and amount of data that was written.

                break;

            case USB_DEVICE_VIDEO_EVENT_INTERFACE_SETTING_CHANGED:

                // This event occurs when the host sends Set Interface request
                // to the Video USB Device. pData will be a pointer to a
                // USB_DEVICE_VIDEO_EVENT_DATA_INTERFACE_SETTING_CHANGED. This
                // contains the interface number whose setting was
                // changed and the index of the alternate setting.
                // The application should typically enable the video function
                // if the interfaceAlternateSettting member of pData is greater
                // than 0.

                break;
            
            case USB_DEVICE_VIDEO_EVENT_CONTROL_TRANSFER_UNKNOWN:
             
                // This event indicates that the Video function driver has
                // received a control transfer which it cannot decode. pData
                // will be a pointer to USB_SETUP_PACKET type pointer. The
                // application should decode the packet and take the required
                // action using the USB_DEVICE_ControlStatus(),
                // USB_DEVICE_ControlSend() and USB_DEVICE_ControlReceive()
                // functions.

                break;

            case USB_DEVICE_VIDEO_EVENT_CONTROL_TRANSFER_DATA_SENT:
                
                // This event indicates the data send request associated with
                // the latest USB_DEVICE_ControlSend() function was
                // completed. pData will be NULL. 

            case USB_DEVICE_VIDEO_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

                // This event indicates the data receive request associated with
                // the latest USB_DEVICE_ControlReceive() function was
                // completed. pData will be NULL. The application can either
                // acknowledge the received data or reject it by calling the 
                // USB_DEVICE_ControlStatus() function. 

                break;

            case USB_DEVICE_VIDEO_EVENT_CONTROL_SET_CUR:
                
                // This event indicates that the host is trying to set the
                // current setting attribute of a control. The data type will be
                // USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_SET_CUR type.  The
                // application should identify the entity type based on the
                // entity ID. This mapping is application specific. The
                // following example assumes entity type to be a Feature Unit. 

                if(APP_EntityIdentify(((USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_SET_CUR *)pData)->entityID) 
                        == APP_VIDEO_ENTITY_FEATURE_UNIT)
                {
                    // The entity type is a feature unit. Type cast pData as
                    // a USB_VIDEO_FEATURE_UNIT_CONTROL_REQUEST type and find
                    // identify the control selector. This example shows the 
                    // handling for VOLUME control 

                    switch(((USB_VIDEO_FEATURE_UNIT_CONTROL_REQUEST *)pData)->controlSelector)
                    {
                        case USB_VIDEO_VOLUME_CONTROL:
                            // This means the host is trying to set the volume.
                            // Use the USB_DEVICE_ControlReceive() function to
                            // receive the volume settings for each channel.

                            USB_DEVICE_ControlReceive(usbDeviceHandle, volumeSetting,
                                    ((USB_VIDEO_FEATURE_UNIT_CONTROL_REQUEST *)pData)->wLength);
                        default:
                            // Only volume control is supported in this example.
                            // So everything else is stalled.
                            USB_DEVICE_ControlStatus(usbDeviceHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
                    }
                }
                break;

            case USB_DEVICE_VIDEO_EVENT_CONTROL_GET_CUR:
                
                // This event indicates that the host is trying to get the
                // current setting attribute of a control. The data type will be
                // USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_SET_CUR type.  The
                // application should identify the entity type based on the
                // entity ID. This mapping is application specific. The
                // following example assumes entity type to be a Feature Unit. 

                if(APP_EntityIdentify(((USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_SET_CUR *)pData)->entityID) 
                        == APP_VIDEO_ENTITY_FEATURE_UNIT)
                {
                    // The entity type is a feature unit. Type cast pData as
                    // a USB_VIDEO_FEATURE_UNIT_CONTROL_REQUEST type and find
                    // identify the control selector. This example shows the 
                    // handling for VOLUME control 

                    switch(((USB_VIDEO_FEATURE_UNIT_CONTROL_REQUEST *)pData)->controlSelector)
                    {
                        case USB_VIDEO_VOLUME_CONTROL:
                            // This means the host is trying to get the volume.
                            // Use the USB_DEVICE_ControlReceive() function to
                            // receive the volume settings for each channel.

                            USB_DEVICE_ControlSend(usbDeviceHandle, volumeSetting,
                                    ((USB_VIDEO_FEATURE_UNIT_CONTROL_REQUEST *)pData)->wLength);
                        default:
                            // Only volume control is supported in this example.
                            // So everything else is stalled.
                            USB_DEVICE_ControlStatus(usbDeviceHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
                    }
                }
                break;
            
            case USB_DEVICE_VIDEO_EVENT_CONTROL_SET_MAX:
            case USB_DEVICE_VIDEO_EVENT_CONTROL_SET_MIN:
            case USB_DEVICE_VIDEO_EVENT_CONTROL_SET_RES:
            case USB_DEVICE_VIDEO_EVENT_CONTROL_SET_MEM:
            case USB_DEVICE_VIDEO_EVENT_CONTROL_GET_MAX:
            case USB_DEVICE_VIDEO_EVENT_CONTROL_GET_MIN:
            case USB_DEVICE_VIDEO_EVENT_CONTROL_GET_RES:
            case USB_DEVICE_VIDEO_EVENT_CONTROL_GET_MEM:
                // In this example these request are not supported and so are
                // stalled.
                USB_DEVICE_ControlStatus(usbDeviceHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
                break;
            
            default:
                break;
        }
        
        return(USB_DEVICE_VIDEO_EVENT_RESPONSE_NONE);
    }
    
    </code>

  Remarks:
    The application can defer responses to events triggered by control
    transfers. In that, the application can respond to the control transfer
    event after exiting the event handler. This allows the application some time
    to obtain the response data rather than having to respond to the event
    immediately. Note that a USB host will typically wait for an event
    response for a finite time duration before timing out and canceling the
    event and associated transactions.  Even when deferring response, the
    application must respond promptly if such time-out have to be avoided.                                                                                         
*/

typedef enum
{
    /* This event occurs when a write operation scheduled by calling the
       USB_DEVICE_VIDEO_Write() function has completed. The pData member in
       the event handler will point to
       USB_DEVICE_VIDEO_EVENT_WRITE_COMPLETE_DATA type. */

    USB_DEVICE_VIDEO_EVENT_WRITE_COMPLETE ,

    /* This event occurs when a read operation scheduled by calling  the
       USB_DEVICE_VIDEO_Read() function has completed. The pData member in
       the event handler will point to USB_DEVICE_VIDEO_EVENT_READ_COMPLETE_DATA
       type.  */

    USB_DEVICE_VIDEO_EVENT_READ_COMPLETE ,

    /* This event occurs when the Host requests the Video USB
       device to set an alternate setting on an interface present in this video
       function. An Video USB Device will typically feature a default interface
       setting and one or more alternate interface settings. The pData member in
       the event handler will point to
       USB_DEVICE_VIDEO_EVENT_DATA_INTERFACE_SETTING_CHANGED type. This contains
       the index of the interface whose setting must be changed and the index of
       the alternate setting. The application may enable or disable video
       functions based on the interface setting. */

    USB_DEVICE_VIDEO_EVENT_INTERFACE_SETTING_CHANGED,

    /* This event occurs when the data stage of a control write transfer has
       completed. This would occur after the application would respond with a
       USB_DEVICE_ControlReceive function, which may possibly have been called
       in response to a USB_DEVICE_VIDEO_EVENT_ENTITY_SETTINGS_RECEIVED event
       This event notifies the application that the data is received from Host
       and is available at the location passed by the
       USB_DEVICE_ControlReceive function. If the received data is acceptable
       to the application, it should acknowledge the data by calling the
       USB_DEVICE_ControlStatus function with a USB_DEVICE_CONTROL_STATUS_OK
       flag.The application can reject the received data by calling the
       USB_DEVICE_ControlStatus function with the
       USB_DEVICE_CONTROL_STATUS_ERROR flag. The pData parameter will be
       NULL. */

    USB_DEVICE_VIDEO_EVENT_CONTROL_TRANSFER_DATA_RECEIVED,

    /* This event occurs when the data stage of a control read transfer has
       completed. This would occur when the application has called the
       USB_DEVICE_ControlSend function to complete the data stage of a control
       transfer. The event indicates that the data has been transmitted to the
       host. The pData parameter will be NULL.
       */

    USB_DEVICE_VIDEO_EVENT_CONTROL_TRANSFER_DATA_SENT,

    /* This event occurs when the Video function driver receives a control
       transfer request that could not be decoded by Video Function driver.The
       pData parameter will point to a USB_SETUP_PACKET type containing the
       SETUP packet.  The application must analyze this SETUP packet and use the
       USB_DEVICE_ControlSend or USB_DEVICE_ControlReceive or the
       USB_DEVICE_ControlStatus functions to advance the control transfer or
       complete it.*/

    USB_DEVICE_VIDEO_EVENT_CONTROL_TRANSFER_UNKNOWN,

    /* This event occurs when the Host sends an Video Control specific Set
       Current Setting Attribute Control Transfer request to an Video Device
       Control. The pData member in the event handler will point to
       USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_SET_CUR type. The application must
       use the entityID, interface, endpoint and the wValue field in the event
       data to determine the entity and control type and then respond to the
       control transfer with a USB_DEVICE_ControlStatus and
       USB_DEVICE_ControlReceive functions. */

    USB_DEVICE_VIDEO_EVENT_CONTROL_SET_CUR,

    /* This event occurs when the Host sends an Video Control specific Set
       Minimum Setting Attribute Control Transfer request to an Video Device
       Control.  The pData member in the event handler will point to
       USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_SET_MIN type. The application must
       use the entityID, interface, endpoint and the wValue field in the event
       data to determine the entity and control type and then respond to the
       control transfer with a USB_DEVICE_ControlStatus and
       USB_DEVICE_ControlReceive functions. */

    USB_DEVICE_VIDEO_EVENT_CONTROL_SET_MIN,

    /* This event occurs when the Host sends an Video Control specific Set
       Maximum Setting Attribute Control Transfer request to an Video Device
       Control. The pData member in the event handler will point to
       USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_SET_MAX type. The application must
       use the entityID, interface, endpoint and the wValue field in the event
       data to determine the entity and control type and then respond to the
       control transfer with a USB_DEVICE_ControlStatus and
       USB_DEVICE_ControlReceive functions. */

    USB_DEVICE_VIDEO_EVENT_CONTROL_SET_MAX,
 
    /* This event occurs when the Host sends an Video Control specific Set
       Resolution Attribute Control Transfer request to an Video Device Control.
       The pData member in the event handler will point to
       USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_SET_RES type. The application must
       use the entityID, interface, endpoint and the wValue field in the event
       data to determine the entity and control type and then respond to the
       control transfer with a USB_DEVICE_ControlStatus
       USB_DEVICE_ControlSend and/or USB_DEVICE_ControlReceive functions. */
    
    USB_DEVICE_VIDEO_EVENT_CONTROL_SET_RES,

    /* This event occurs when the Host sends an Video Entity specific Set
       Memory Space Attribute Control Transfer request to an Video Device
       Entity.  The pData member in the event handler will point to
       USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_SET_MEM type. The application must
       use the entityID, interface, endpoint and the wValue field in the event
       data to determine the entity and control type and then respond to the
       control transfer with a USB_DEVICE_ControlStatus
       USB_DEVICE_ControlSend and/or USB_DEVICE_ControlReceive functions. */

    USB_DEVICE_VIDEO_EVENT_ENTITY_SET_MEM,

    /* This event occurs when the Host sends an Video Control specific Get
       Current Setting Attribute Control Transfer request to an Video Device
       Control. The pData member in the event handler will point to
       USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_GET_CUR type. The application must
       use the entityID, interface, endpoint and the wValue field in the event
       data to determine the entity and control type and then respond to the
       control transfer with a USB_DEVICE_ControlStatus and
       USB_DEVICE_ControlSend functions. */

    USB_DEVICE_VIDEO_EVENT_CONTROL_GET_CUR,

    /* This event occurs when the Host sends an Video Control specific Get
       Minimum Setting Attribute Control Transfer request to an Video Device
       Control.  The pData member in the event handler will point to
       USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_GET_MIN type. The application must
       use the entityID, interface, endpoint and the wValue field in the event
       data to determine the entity and control type and then respond to the
       control transfer with a USB_DEVICE_ControlStatus and
       USB_DEVICE_ControlSend functions. */

    USB_DEVICE_VIDEO_EVENT_CONTROL_GET_MIN,

    /* This event occurs when the Host sends an Video Control specific Get
       Maximum Setting Attribute Control Transfer request to an Video Device
       Control.  The pData member in the event handler will point to
       USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_GET_MAX type. The application must
       use the entityID, interface, endpoint and the wValue field in the event
       data to determine the entity and control type and then respond to the
       control transfer with a USB_DEVICE_ControlStatus and
       USB_DEVICE_ControlSend functions. */

    USB_DEVICE_VIDEO_EVENT_CONTROL_GET_MAX,

    /* This event occurs when the Host sends an Video Control specific Get
       Resolution Setting Attribute Control Transfer request to an Video Device
       Control. The pData member in the event handler will point to
       USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_GET_RES type. The application must
       use the entityID, interface, endpoint and the wValue field in the event
       data to determine the entity and control type and then respond to the
       control transfer with a USB_DEVICE_ControlStatus and
       USB_DEVICE_ControlSend functions. */

    USB_DEVICE_VIDEO_EVENT_CONTROL_GET_RES,

    /* This event occurs when the Host sends an Video Entity specific Get
       Memory Space Attribute Control Transfer request to an Video Device
       Entity.  The pData member in the event handler will point to
       USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_SET_MEM type. The application must
       use the entityID, interface, endpoint and the wValue field in the event
       data to determine the entity and control type and then respond to the
       control transfer with a USB_DEVICE_ControlStatus or
       USB_DEVICE_ControlSend functions. */
    USB_DEVICE_VIDEO_EVENT_ENTITY_GET_MEM,
            
    /* This event occurs when the Host sends a Video Entity specific Get Status
       Control Transfer request to an Video Device Entity. The pData member in
       the event handler will point to
       USB_DEVICE_VIDEO_EVENT_DATA_ENTITY_GET_STAT type. The application mus use
       the entityID, interface, endpoint and the wValue field in the event data
       to determine the entity and control type and then respond to the control
       transfer with a USB_DEVICE_ControlSend and or USB_DEVICE_ControlStatus
       functions. */
    USB_DEVICE_VIDEO_EVENT_ENTITY_GET_STAT,
            
            USB_DEVICE_VIDEO_EVENT_ENTITY_GET_INFO,            
            USB_DEVICE_VIDEO_EVENT_CONTROL_GET_DEF,

} USB_DEVICE_VIDEO_EVENT;

// *****************************************************************************
/* USB Device Video Function Driver Event Handler Response Type

  Summary:
    USB Device Video Function Driver event callback response type.

  Description:
    This is the return type of the Video Function Driver event handler.

  Remarks:
    None.
*/

typedef void USB_DEVICE_VIDEO_EVENT_RESPONSE;

// *****************************************************************************
/* USB Device Video Function Driver Event Handler Response None  

  Summary:
    USB Device Video Function Driver event handler response type none.

  Description:
    This is the definition of the Video Function Driver event handler response
    type none.

  Remarks:
    Intentionally defined to be empty.
*/

#define USB_DEVICE_VIDEO_EVENT_RESPONSE_NONE

// *****************************************************************************
/* USB Device Video Event Handler Function Pointer Type.

  Summary:
    USB Device Video event handler function pointer type.

  Description:
    This data type defines the required function signature USB Device Video
    Function Driver event handling callback function. The application must
    register a pointer to an Video Function Driver events handling function whose
    function signature (parameter and return value types) match the types
    specified by this function pointer in order to receive event call backs from
    the Video Function Driver. The function driver will invoke this function with
    event relevant parameters. The description of the event handler function
    parameters is given here.

    instanceIndex           - Instance index of the Video Function Driver that
                              generated the event.

    event                   - Type of event generated.

    pData                   - This parameter should be typecast to an event 
                              specific pointer type based on the event that 
                              has occurred. Refer to the 
                              USB_DEVICE_VIDEO_EVENT enumeration description for
                              more details.

    context                 - Value identifying the context of the application 
                              that registered the event handling function.

  Remarks:
    The event handler function executes in the USB interrupt context when the
    USB Device Stack is configured for interrupt based operation. It is not
    advisable to call blocking functions or computationally intensive functions
    in the event handler. Where the response to a control transfer related event
    requires extended processing, the response to the control transfer should be
    deferred and the event handler should be allowed to complete execution.
*/

typedef USB_DEVICE_VIDEO_EVENT_RESPONSE (*USB_DEVICE_VIDEO_EVENT_HANDLER )
(
    USB_DEVICE_VIDEO_INDEX instanceIndex ,
    USB_DEVICE_VIDEO_EVENT event ,
    void * pData,
    uintptr_t context
);

// *****************************************************************************
/* USB Device Video Function Driver USB Device Video Result enumeration.

  Summary:
    USB Device Video Function Driver USB Device Video result enumeration.

  Description:
    This enumeration lists the possible USB Device Video Function Driver
    operation results. 

  Remarks:
    None.
 */

typedef enum
{
    /* The operation was successful */
    USB_DEVICE_VIDEO_RESULT_OK 
        /* DOM-IGNORE-BEGIN */ = USB_ERROR_NONE /* DOM-IGNORE-END */,

    /* The transfer queue is full and no new transfers can be scheduled */
    USB_DEVICE_VIDEO_RESULT_ERROR_TRANSFER_QUEUE_FULL 
        /* DOM-IGNORE-BEGIN */ = USB_ERROR_IRP_QUEUE_FULL /* DOM-IGNORE-END */,

    /* The specified instance is not provisioned in the system */
    USB_DEVICE_VIDEO_RESULT_ERROR_INSTANCE_INVALID 
        /* DOM-IGNORE-BEGIN */ = USB_ERROR_DEVICE_FUNCTION_INSTANCE_INVALID /* DOM-IGNORE-END */,

    /* The specified instance is not configured yet */
    USB_DEVICE_VIDEO_RESULT_ERROR_INSTANCE_NOT_CONFIGURED 
        /* DOM-IGNORE-BEGIN */ = USB_ERROR_ENDPOINT_NOT_CONFIGURED /* DOM-IGNORE-END */,

    /* The event handler provided is NULL */
    USB_DEVICE_VIDEO_RESULT_ERROR_PARAMETER_INVALID  
        /* DOM-IGNORE-BEGIN */ =  USB_ERROR_PARAMETER_INVALID /* DOM-IGNORE-END */,

    /* Interface number passed to the read or write function is invalid. */
    USB_DEVICE_VIDEO_RESULT_ERROR_INVALID_INTERFACE_ID,

    /* A NULL buffer was specified in the read or write function */
    USB_DEVICE_VIDEO_RESULT_ERROR_INVALID_BUFFER,
    
    /* Transfer terminated because host halted the endpoint */
    USB_DEVICE_VIDEO_RESULT_ERROR_ENDPOINT_HALTED
        /* DOM-IGNORE-BEGIN */ = USB_ERROR_ENDPOINT_HALTED /* DOM-IGNORE-END */,

    /* Transfer terminated by host because of a stall clear */
    USB_DEVICE_VIDEO_RESULT_ERROR_TERMINATED_BY_HOST
        /* DOM-IGNORE-BEGIN */ = USB_ERROR_TRANSFER_TERMINATED_BY_HOST /* DOM-IGNORE-END */,

    /* General Error */
    USB_DEVICE_VIDEO_RESULT_ERROR

} USB_DEVICE_VIDEO_RESULT;

// *****************************************************************************
/* USB Device Video Function Driver Read and Write Complete Event Data.

  Summary:
    USB Device Video Function Driver video read and write complete event data.

  Description:
    This data type defines the data structure returned by the driver along with
    USB_DEVICE_VIDEO_EVENT_READ_COMPLETE, USB_DEVICE_VIDEO_EVENT_WRITE_COMPLETE,
    events.

  Remarks:
    None.
*/

typedef struct
{
    /* Transfer handle associated with this
     * read or write request */
    USB_DEVICE_VIDEO_TRANSFER_HANDLE handle;

    /* Indicates the amount of data (in bytes) that was
     * read or written */
    uint16_t length;

    /* Interface Number */
    uint8_t interfaceNum;
    
    /* Completion status of the transfer */
    USB_DEVICE_VIDEO_RESULT status;
}
USB_DEVICE_VIDEO_EVENT_DATA_WRITE_COMPLETE,
USB_DEVICE_VIDEO_EVENT_DATA_READ_COMPLETE;

// *****************************************************************************
/* USB Device Video Function Driver Set and Get request data.

  Summary:
    USB Device Video Function Driver set and get request data.

  Description:
    This data type defines the data structure returned by the driver along with the
    USB_DEVICE_VIDEO_EVENT_CONTROL_SET_XXX,
    USB_DEVICE_VIDEO_EVENT_ENTITY_SET_MEM,
    USB_DEVICE_VIDEO_EVENT_CONTROL_GET_XXX and
    USB_DEVICE_VIDEO_EVENT_ENTITY_GET_MEM events.

  Remarks:
    None.
*/

typedef USB_VIDEO_CONTROL_INTERFACE_REQUEST USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_SET_CUR;
typedef USB_VIDEO_CONTROL_INTERFACE_REQUEST USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_SET_MIN;
typedef USB_VIDEO_CONTROL_INTERFACE_REQUEST USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_SET_MAX;
typedef USB_VIDEO_CONTROL_INTERFACE_REQUEST USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_SET_RES;
typedef USB_VIDEO_CONTROL_INTERFACE_REQUEST USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_SET_MEM;
typedef USB_VIDEO_CONTROL_INTERFACE_REQUEST USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_GET_CUR;
typedef USB_VIDEO_CONTROL_INTERFACE_REQUEST USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_GET_MIN;
typedef USB_VIDEO_CONTROL_INTERFACE_REQUEST USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_GET_MAX;
typedef USB_VIDEO_CONTROL_INTERFACE_REQUEST USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_GET_RES;
typedef USB_VIDEO_CONTROL_INTERFACE_REQUEST USB_DEVICE_VIDEO_EVENT_DATA_CONTROL_GET_MEM;
typedef USB_VIDEO_CONTROL_INTERFACE_REQUEST USB_DEVICE_VIDEO_EVENT_DATA_ENTITY_GET_STAT;

// *****************************************************************************
/* USB Device Video Function Driver Alternate Interface Setting Event Data.

  Summary:
    USB Device Video Function Driver alternate interface setting event data.

  Description:
    This data type defines the data structure returned by the driver along with
    USB_DEVICE_VIDEO_EVENT_DATA_INTERFACE_SETTING_CHANGED.

  Remarks:
    None.
*/

typedef struct
{
    /* Interface number of the interface who setting is to be changed */
    uint8_t interfaceNumber;

    /* Alternate setting number */
    uint8_t interfaceAlternateSetting;

} USB_DEVICE_VIDEO_EVENT_DATA_INTERFACE_SETTING_CHANGED;

// *****************************************************************************
// *****************************************************************************
// Section: USB Device Video Interface Function Definitions
// *****************************************************************************
// *****************************************************************************

//*******************************************************************************
/*
  Function:
    USB_DEVICE_VIDEO_RESULT USB_DEVICE_VIDEO_EventHandlerSet
    (
        USB_DEVICE_VIDEO_INDEX instance ,
        USB_DEVICE_VIDEO_EVENT_HANDLER eventHandler ,
        uintptr_t context
    );
    
  Summary:
    This function registers an event handler for the specified Video function
    driver instance. 

  Description:
    This function registers a event handler for the specified Video function
    driver instance. This function should be called by the application when it
    receives a SET CONFIGURATION event from the device layer. The application
    must register an event handler with the function driver in order to receive
    and respond to function driver specific events and control transfers. If the
    event handler is not registered, the device layer will stall function driver
    specific commands and the USB device may not function.
   
  Precondition:
    This function should be called when the function driver has been initialized
    as a result of a set configuration.
    
  Parameters:
    instance  - Instance of the Video Function Driver.
    eventHandler - A pointer to event handler function.
    context - Application specific context that is returned in the event handler.
  
  Returns:
    - USB_DEVICE_VIDEO_RESULT_OK - The operation was successful
    - USB_DEVICE_VIDEO_RESULT_ERROR_INSTANCE_INVALID - The specified instance does 
      not exist.
    - USB_DEVICE_VIDEO_RESULT_ERROR_PARAMETER_INVALID - The eventHandler parameter is 
      NULL
    
  Example:
    <code>
    // The following code shows an example for registering an event handler. The
    // application specifies the context parameter as a pointer to an
    // application object (appObject) that should be associated with this 
    // instance of the Video function driver.
    
    USB_DEVICE_VIDEO_RESULT result;
    
    USB_DEVICE_VIDEO_EVENT_RESPONSE APP_USBDeviceVIDEOEventHandler 
    (
        USB_DEVICE_VIDEO_INDEX instanceIndex ,
        USB_DEVICE_VIDEO_EVENT event ,
        void* pData,
        uintptr_t context
    )
    {
        // Event Handling comes here

        switch(event) 
        {
            ...
        }

        return(USB_DEVICE_VIDEO_EVENT_RESPONSE_NONE);
    }

    result = USB_DEVICE_VIDEO_EventHandlerSet ( USB_DEVICE_VIDEO_INSTANCE_0 ,
                &APP_USBDeviceVIDEOEventHandler, (uintptr_t) &appObject);

    if(USB_DEVICE_VIDEO_RESULT_OK != result)
    {
        // Do error handling here
    }
  
    </code>

  Remarks:
    None.
*/

USB_DEVICE_VIDEO_RESULT USB_DEVICE_VIDEO_EventHandlerSet
(
    USB_DEVICE_VIDEO_INDEX instanceIndex ,
    USB_DEVICE_VIDEO_EVENT_HANDLER eventHandler ,
    uintptr_t context
);

//**************************************************************************
/*
  Function:
    USB_DEVICE_VIDEO_RESULT USB_DEVICE_VIDEO_Write
    (
        USB_DEVICE_VIDEO_INDEX instance ,
        USB_DEVICE_VIDEO_TRANSFER_HANDLE* transferHandle,
        uint8_t interfaceNum ,
        void * data ,
        size_t size
    );
 
  Summary:
    This function requests a data write to the USB Device Video Function
    Driver Layer.
  
  Description:
    This function requests a data write to the USB Device Video Function
    Driver Layer. The function places a requests with driver, the request
    will get serviced as data is requested by the USB Host. A handle to the
    request is returned in the transferHandle parameter. The termination of
    the request is indicated by the USB_DEVICE_VIDEO_EVENT_WRITE_COMPLETE
    event. The amount of data written and the transfer handle associated
    with the request is returned along with the event in writeCompleteData 
    member of the pData parameter in the event handler. 
    
    The transfer handle expires when event handler for the
    USB_DEVICE_VIDEO_EVENT_WRITE_COMPLETE exits. If the write request could not
    be accepted, the function returns an error code and transferHandle will
    contain the value USB_DEVICE_VIDEO_TRANSFER_HANDLE_INVALID.
 
  Precondition:
    The function driver should have been configured.
  
  Parameters:
    instance -        USB Device Video Function Driver instance.
    transferHandle -  Pointer to a USB_DEVICE_VIDEO_TRANSFER_HANDLE type of
                      variable. This variable will contain the transfer
                      handle in case the write request was successful.
    interfaceNum -    The USB Video streaming interface number on which the
                      write request is to placed.
    data  -           pointer to the data buffer contains the data to be written. 
                      In case of PIC32MZ device, this buffer should be located in 
                      coherent memory and should be aligned a 16 byte boundary.
    size -            Size of the data buffer.

  Return:
    - USB_DEVICE_VIDEO_RESULT_OK - The read request was successful. transferHandle
    contains a valid transfer handle.
    - USB_DEVICE_VIDEO_RESULT_ERROR_TRANSFER_QUEUE_FULL - internal request queue
    is full. The write request could not be added.
    - USB_DEVICE_VIDEO_RESULT_ERROR_INSTANCE_NOT_CONFIGURED - The specified
    instance is not configured yet.
    - USB_DEVICE_VIDEO_RESULT_ERROR_INSTANCE_INVALID - The specified instance
    was not provisioned in the application and is invalid.
  
  Example:
    <code>

    // Shows an example of how to write video data to the video streaming
    // interface . This assumes that device is configured and the video
    // streaming interface is 1.

    USB_DEVICE_VIDEO_INDEX instanceIndex;
    USB_DEVICE_VIDEO_TRANSFER_HANDLE transferHandle;
    unit8_t interfaceNumber;
    unit8_t txBuffer[192]; // Use this attribute for PIC32MZ __attribute__((coherent, aligned(16)))
    USB_DEVICE_VIDEO_RESULT writeRequestResult;
    
    instanceIndex = 0; //specify the Video Function driver instance number.
    interfaceNumber = 1; //Specify the Video Streaming interface number.
    
    writeRequestResult = USB_DEVICE_VIDEO_Write ( instanceIndex, &transferHandle,
                                interfaceNumber, &txBuffer, 192);

    if(USB_DEVICE_VIDEO_RESULT_OK != writeRequestResult)
    {
        //Do Error handling here
    }

    // The completion of the write request will be indicated by the
    // USB_DEVICE_VIDEO_EVENT_WRITE_COMPLETE event. The transfer handle
    // and transfer size is provided along with this event.

    </code>

  Remarks:
    While the using the Video Function Driver with the PIC32MZ USB module, the
    video buffer provided to the USB_DEVICE_VIDEO_Write function should be
    placed in coherent memory and aligned at a 16 byte boundary.  This can be
    done by declaring the buffer using the  __attribute__((coherent,
    aligned(16))) attribute. An example is shown here                                                                   

    <code>
    uint8_t data[256] __attribute__((coherent, aligned(16)));
    </code>                                                                  
*/

USB_DEVICE_VIDEO_RESULT USB_DEVICE_VIDEO_Write
(
    USB_DEVICE_VIDEO_INDEX instanceIndex,
    USB_DEVICE_VIDEO_TRANSFER_HANDLE * transferHandle,
    uint8_t interfaceNumber,
    void * data,
    size_t size
);

//***************************************************************************
/*
  Function:
    USB_DEVICE_VIDEO_RESULT USB_DEVICE_VIDEO_Read
    (
        USB_DEVICE_VIDEO_INDEX instanceIndex ,
        USB_DEVICE_VIDEO_TRANSFER_HANDLE* transferHandle,
        uint8_t interfaceNum ,
        void * data ,
        size_t size
    );
  
  Summary:
    This function requests a data read from the USB Device Video Function
    Driver Layer.
  
  Description:
    This function requests a data read from the USB Device Video Function
    Driver Layer. The function places a requests with driver, the request
    will get serviced as data is made available by the USB Host. A handle
    to the request is returned in the transferHandle parameter. The
    termination of the request is indicated by the
    USB_DEVICE_VIDEO_EVENT_READ_COMPLETE event. The amount of data read and
    the transfer handle associated with the request is returned along with
    the event. The transfer handle expires when event handler for the
    USB_DEVICE_VIDEO_EVENT_READ_COMPLETE exits. If the read request could
    not be accepted, the function returns an error code and transferHandle
    will contain the value USB_DEVICE_VIDEO_TRANSFER_HANDLE_INVALID.

  Conditions:
    The function driver should have been configured.
  
  Parameters:
    instance -        USB Device Video Function Driver instance.
    transferHandle -  Pointer to a USB_DEVICE_VIDEO_TRANSFER_HANDLE type of
                      variable. This variable will contain the transfer
                      handle in case the read request was successful.
    interfaceNum -    The USB Video streaming interface number on which read
                      request is to placed.
    data  -           pointer to the data buffer where read data will be stored. 
                      In case of PIC32MZ device, this buffer should be located in 
                      coherent memory and should be aligned a 16 byte boundary.
    size -            Size of the data buffer. Refer to the description section
                      for more details on how the size affects the transfer.

  Return:
    - USB_DEVICE_VIDEO_RESULT_OK - The read request was successful. transferHandle
    contains a valid transfer handle.
    - USB_DEVICE_VIDEO_RESULT_ERROR_TRANSFER_QUEUE_FULL - internal request queue
    is full. The read request could not be added.
    - USB_DEVICE_VIDEO_RESULT_ERROR_INSTANCE_NOT_CONFIGURED - The specified
    instance is not configured yet.
    - USB_DEVICE_VIDEO_RESULT_ERROR_INSTANCE_INVALID - The specified instance
    was not provisioned in the application and is invalid.

  Example:
    <code>
    
    // Shows an example of how to read. This assumes that
    // device had been configured. The example attempts to read
    // data from interface 1.
   
    USB_DEVICE_VIDEO_INDEX instanceIndex;
    USB_DEVICE_VIDEO_TRANSFER_HANDLE transferHandle;
    unit8_t interfaceNumber;
    unit8_t rxBuffer[192]; // Use this attribute for PIC32MZ __attribute__((coherent, aligned(16)))
    USB_DEVICE_VIDEO_RESULT readRequestResult; 

    instanceIndex = 0; //specify the Video Function driver instance number.
    interfaceNumber = 1; //Specify the Video Streaming interface number.
    
    readRequestResult = USB_DEVICE_VIDEO_Read ( instanceIndex, &transferHandle,
                            interfaceNumber, &rxBuffer, 192);

    if(USB_DEVICE_VIDEO_RESULT_OK != readRequestResult)
    {
        //Do Error handling here
    }

    // The completion of the read request will be indicated by the
    // USB_DEVICE_VIDEO_EVENT_READ_COMPLETE event. The transfer handle
    // and the amount of data read will be returned along with the 
    // event.

    </code>

  Remarks:
    While the using the Video Function Driver with PIC32MZ USB module, the video
    buffer provided to the USB_DEVICE_VIDEO_Read function should be placed in
    coherent memory and aligned at a 16 byte boundary.  This can be done by
    declaring the buffer using the  __attribute__((coherent, aligned(16)))
    attribute. An example is shown here                                                                   

    <code>
    uint8_t data[256] __attribute__((coherent, aligned(16)));
    </code>
*/

USB_DEVICE_VIDEO_RESULT USB_DEVICE_VIDEO_Read
(
    USB_DEVICE_VIDEO_INDEX instanceIndex,
    USB_DEVICE_VIDEO_TRANSFER_HANDLE* transferHandle,
    uint8_t interfaceNumber,
    void * data,
    size_t size
);

//******************************************************************************
/* Function:
    USB_DEVICE_VIDEO_RESULT USB_DEVICE_VIDEO_TransferCancel
    (
        USB_DEVICE_VIDEO_INDEX instanceIndex,
        USB_DEVICE_VIDEO_TRANSFER_HANDLE transferHandle
    );

  Summary:
    This function cancels a scheduled Video Device data transfer.

  Description:
    This function cancels a scheduled Video Device data transfer. The transfer
    could have been scheduled  using the USB_DEVICE_VIDEO_Read,
    USB_DEVICE_VIDEO_Write, or the USB_DEVICE_VIDEO_SerialStateNotificationSend 
	functions. If a transfer is still in the queue and its processing has not 
	started, the transfer is canceled completely. A transfer that is in progress 
	may or may not get canceled depending on the transaction that is presently in 
	progress. If the last transaction of the transfer is in progress, the transfer 
	will not be canceled.  If it is not the last transaction in progress, the in-progress 
	will be allowed to complete. Pending transactions will be canceled. The first 
	transaction of an in progress transfer cannot be canceled.

  Precondition:
    The USB Device should be in a configured state.

  Parameters:
    instanceIndex  - VIDEO Function Driver instance index.
    transferHandle - Transfer handle of the transfer to be canceled.

  Returns:
    - USB_DEVICE_VIDEO_RESULT_OK    - The transfer will be canceled completely or
                                      partially.
    - USB_DEVICE_VIDEO_RESULT_ERROR - The transfer could not be canceled because it has
                                      either completed, the transfer handle is invalid
                                      or the last transaction is in progress.

  Example:
    <code>

    // The following code snippet cancels a VIDEO transfer.

    USB_DEVICE_VIDEO_TRANSFER_HANDLE transferHandle;
    USB_DEVICE_VIDEO_RESULT result;

    result = USB_DEVICE_VIDEO_TransferCancel(instanceIndex, transferHandle);

    if(USB_DEVICE_VIDEO_RESULT_OK == result)
    {
        // The transfer cancellation was either completely or
        // partially successful.
    }

    </code>

  Remarks:
    None.
*/

USB_DEVICE_VIDEO_RESULT USB_DEVICE_VIDEO_TransferCancel
(
    USB_DEVICE_VIDEO_INDEX instanceIndex,
    USB_DEVICE_VIDEO_TRANSFER_HANDLE transferHandle
);

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Types. This section is specific to PIC32 implementation
//          of the USB Device Video Function Driver
// *****************************************************************************
// *****************************************************************************

/*DOM-IGNORE-BEGIN*/extern const USB_DEVICE_FUNCTION_DRIVER videoFunctionDriver;/*DOM-IGNORE-END*/

// *****************************************************************************
/* USB Device Video Function Driver Function Pointer

  Summary:
    USB Device Video Function Driver function pointer.

  Description:
    This is the USB Device Video Function Driver function pointer. This should
    registered with the device layer in the function driver registration table.

  Remarks:
    None.
*/

#define USB_DEVICE_VIDEO_FUNCTION_DRIVER /*DOM-IGNORE-BEGIN*/&videoFunctionDriver/*DOM-IGNORE-END*/

// *****************************************************************************
/* USB Device Video Function Driver Initialization Data Structure

  Summary:
    USB Device Video Function Driver initialization data structure.

  Description:
    This data structure must be defined for every instance of the Video Function 
    Driver. It is passed to the Video function driver, by the Device Layer,
    at the time of initialization. The funcDriverInit member of the 
    Device Layer Function Driver registration table entry must point to this
    data structure for an instance of the Video function driver. 

  Remarks:
    The queue sizes that are specified in this data structure are also affected
    by the USB_DEVICE_VIDEO_QUEUE_DEPTH_COMBINED configuration macro.
*/

typedef struct 
{
    /* Size of the read queue for this instance
     * of the Video function driver */
    size_t queueSizeRead;
    
    /* Size of the write queue for this instance
     * of the Video function driver */
    size_t queueSizeWrite;

} USB_DEVICE_VIDEO_INIT;

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END



#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
