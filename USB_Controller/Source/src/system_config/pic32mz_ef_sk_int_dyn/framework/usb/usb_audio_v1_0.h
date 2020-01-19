/*******************************************************************************
  USB Audio class definitions

  Company:
    Microchip Technology Inc.

  File Name:
    usb_audio_v1_0.h

  Summary:
    USB Audio class definitions.

  Description:
    This file describes the Audio v1.0 class specific definitions.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to  you  the  right  to  use,  modify,  copy  and  distribute
Software only when embedded on a Microchip  microcontroller  or  digital  signal
controller  that  is  integrated  into  your  product  or  third  party  product
(pursuant to the  sublicense  terms  in  the  accompanying  license  agreement).

You should refer  to  the  license  agreement  accompanying  this  Software  for
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
// DOM-IGNORE-END

#ifndef _USB_AUDIO_H_
#define _USB_AUDIO_H_

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

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
/* Audio Interface Class codes

  Summary:
    Identifies the Class Codes for Audio interface. 

  Description:
    This constant identifies the value of the Audio Interface class code. 

  Remarks:
    None.
*/
#define  USB_AUDIO_CLASS_CODE 0x01

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Audio Interface subclass codes

  Summary:
    Identifies the subclass codes for Audio interface. 

  Description:
    This enumeration identifies the possible subclass codes for 
    audio interface. 

  Remarks:
    The "ISC" in the enumeration member names is an acronym for Interface 
    Subclass Code.
*/


typedef enum
{
    USB_AUDIO_SUBCLASS_UNDEFINED    = 0x00,
    USB_AUDIO_AUDIOCONTROL          = 0x01,
    USB_AUDIO_AUDIOSTREAMING        = 0x02,
    USB_AUDIO_MIDISTREAMING         = 0x03

} USB_AUDIO_SUBCLASS_CODE; 

// *****************************************************************************
/* Audio Interface Protocol codes

  Summary:
    Identifies the protocol codes for Audio interface. 

  Description:
    This enumeration identifies the possible protocol codes for 
    audio interface

  Remarks:
    As per USB Device class definition for Audio Device release v1.0, the
    protocol code should always be 0.
*/

typedef enum
{
    USB_AUDIO_PR_PROTOCOL_UNDEFINED    = 0x0

} USB_AUDIO_PROTOCOL_CODE;

// *****************************************************************************
/* Audio Class Specific Descriptor Types

  Summary:
    Identifies the Audio class specific descriptor types for Audio. 

  Description:
    This enumeration identifies the Audio Class specific descriptor types. 

  Remarks:
    The "CS" in the enumeration member names is an acronym for Class Specific.
*/

typedef enum
{
    USB_AUDIO_CS_UNDEFINED       = 0x20,
    USB_AUDIO_CS_DEVICE          = 0x21,
    USB_AUDIO_CS_CONFIGURATION   = 0x22,
    USB_AUDIO_CS_STRING          = 0x23,
    USB_AUDIO_CS_INTERFACE       = 0x24,
    USB_AUDIO_CS_ENDPOINT        = 0x25

} USB_AUDIO_CS_DESCRIPTOR_TYPE;

// *****************************************************************************
/* Audio Class Specific AC Interface Descriptor Subtypes

  Summary:
    Identifies the Audio Class Specific AC Interface Descriptor Subtypes.

  Description:
    This enumeration identifies the possible Audio Class Specific AC Interface
    Descriptor Subtypes.

  Remarks:
    The "CS" in the enumeration member names is an acronym for Class Specific.
    The "AC" in the enumeration member names is an acronym for Audio Control.
*/

typedef enum
{
    USB_AUDIO_AC_DESCRIPTOR_UNDEFINED    = 0x00,
    USB_AUDIO_HEADER                     = 0x01,
    USB_AUDIO_INPUT_TERMINAL             = 0x02,
    USB_AUDIO_OUTPUT_TERMINAL            = 0x03,
    USB_AUDIO_MIXER_UNIT                 = 0x04,
    USB_AUDIO_SELECTOR_UNIT              = 0x05,
    USB_AUDIO_FEATURE_UNIT               = 0x06,
    USB_AUDIO_PROCESSING_UNIT            = 0x07,
    USB_AUDIO_EXTENSION_UNIT             = 0x08,

} USB_AUDIO_CS_AC_INTERFACE_DESCRIPTOR_SUBTYPE,
USB_AUDIO_V1_ENTITY_TYPE; 

// *****************************************************************************
/* Audio Class Specific Terminal Types. 

  Summary:
    Identifies the Audio Class Specific Audio Class Specific Terminal Types.

  Description:
    This enumeration identifies the possible Audio Class Specific Terminal Types.
    This is enumeration is as per the Table 2-1, 2-2 and 2-3 of the document 
    Universal Serial Bus Device Class Definition for Terminal Types. 

  Remarks:
    The "CS" in the enumeration member names is an acronym for Class Specific. 
*/
typedef enum 
{
    /* USB Terminal Types */ 
    USB_AUDIO_TERMINAL_TYPE_USB_UNDEFINED = 0x0100,
    USB_AUDIO_TERMINAL_TYPE_USB_STREAMING = 0x0101,
    USB_AUDIO_TERMINAL_TYPE_USB_VENDOR_SPECIFIC = 0x01FF,
         
    /* Input Terminal types */ 
    USB_AUDIO_TERMINAL_TYPE_INPUT_UNDEFINED = 0x0200, 
    USB_AUDIO_TERMINAL_TYPE_INPUT_MICROPHONE = 0x0201,
    USB_AUDIO_TERMINAL_TYPE_INPUT_MICROPHONE_DESKTOP = 0x0202,
    USB_AUDIO_TERMINAL_TYPE_INPUT_MICROPHONE_PERSONAL = 0x0203,
    USB_AUDIO_TERMINAL_TYPE_INPUT_MICROPHONE_OMNI = 0x0204,
    USB_AUDIO_TERMINAL_TYPE_INPUT_MICROPHONE_ARRAY = 0x0205,
    USB_AUDIO_TERMINAL_TYPE_INPUT_MICROPHONE_ARRAY_PROCESSING = 0x0206,
       
    /* Output Terminal types */         
    USB_AUDIO_TERMINAL_TYPE_OUTPUT_UNDEFINED = 0x0300,
    USB_AUDIO_TERMINAL_TYPE_OUTPUT_SPEAKER = 0x0301,
    USB_AUDIO_TERMINAL_TYPE_OUTPUT_HEADPHONES = 0x0302,
    USB_AUDIO_TERMINAL_TYPE_OUTPUT_HMD = 0x0303,
    USB_AUDIO_TERMINAL_TYPE_OUTPUT_SPEAKER_DESKTOP = 0x0304,
    USB_AUDIO_TERMINAL_TYPE_OUTPUT_SPEAKER_ROOM = 0x0305,
    USB_AUDIO_TERMINAL_TYPE_OUTPUT_SPEAKER_COMM = 0x0306,
    USB_AUDIO_TERMINAL_TYPE_OUTPUT_SPEAKER_LFE = 0x0307
            
}USB_AUDIO_V1_TERMINAL_TYPE; 

// *****************************************************************************
/* Audio Class Specific AS Interface Descriptor Subtypes

  Summary:
    Identifies the Audio Class Specific AS Interface Descriptor Subtypes. 

  Description:
    This enumeration identifies the possible Audio Class Specific AS Interface 
    Descriptor Subtypes. 

  Remarks:
    The "CS" in the enumeration member names is an acronym for Class Specific.
    The "AS" in the enumeration member names is an acronym for Audio Streaming.
*/

typedef enum
{
    USB_AUDIO_AS_DESCRIPTOR_UNDEFINED    = 0x00,
    USB_AUDIO_AS_GENERAL                 = 0x01,
    USB_AUDIO_FORMAT_TYPE                = 0x02,
    USB_AUDIO_FORMAT_SPECIFIC            = 0x03

} USB_AUDIO_CS_AS_INTERFACE_DESCRIPTOR_SUBTYPE;

// *****************************************************************************
/* Audio Processing Unit Process Types

  Summary:
    Identifies the Audio Process Unit Process Types.

  Description:
    This enumeration identifies the possible Audio Process Unit Process types. 

  Remarks:
    None.
*/

typedef enum
{
    USB_AUDIO_PROCESS_UNDEFINED             = 0x00,
    USB_AUDIO_UP_DOWNMIX_PROCESS            = 0x01,
    USB_AUDIO_DOLBY_PROLOGIC_PROCESS        = 0x02,
    USB_AUDIO_3D_STEREO_EXTENDER_PROCESS    = 0x03,
    USB_AUDIO_REVERBERATION_PROCESS         = 0x04,
    USB_AUDIO_CHORUS_PROCESS                = 0x05,   
    USB_AUDIO_DYN_RANGE_COMP_PROCESS        = 0x06

} USB_AUDIO_PROCESSING_UNIT_PROCESS_TYPE;

// *****************************************************************************
/* Audio Class Specific Endpoint Descriptor Subtypes.

  Summary:
    Identifies the Audio Class Specific Endpoint Descriptor Subtypes.

  Description:
    This enumeration identifies the possible Audio Class Specific Endpoint 
    Descriptor Subtypes.

  Remarks:
    The "CS" in the enumeration member names is an acronym for Class Specific.
*/

typedef enum
{
    USB_AUDIO_DESCRIPTOR_UNDEFINED    = 0x00,
    USB_AUDIO_EP_GENERAL              = 0x01

} USB_AUDIO_CS_ENDPOINT_DESCRIPTOR_SUBTYPE;

// *****************************************************************************
/* Audio Class Specific Request Codes.

  Summary:
    Identifies the Audio Class Specific Request Codes.

  Description:
    This enumeration identifies the possible Audio Class Specific Request codes.

  Remarks:
    The "CS" in the enumeration member names is an acronym for Class Specific.
*/

typedef enum
{
    USB_AUDIO_CS_REQUEST_CODE_UNDEFINED = 0x00,
    USB_AUDIO_CS_SET_CUR   = 0x01,
    USB_AUDIO_CS_GET_CUR   = 0x81,
    USB_AUDIO_CS_SET_MIN   = 0x02,
    USB_AUDIO_CS_GET_MIN   = 0x82,
    USB_AUDIO_CS_SET_MAX   = 0x03,
    USB_AUDIO_CS_GET_MAX   = 0x83,
    USB_AUDIO_CS_SET_RES   = 0x04,
    USB_AUDIO_CS_GET_RES   = 0x84,
    USB_AUDIO_CS_SET_MEM   = 0x05,
    USB_AUDIO_CS_GET_MEM   = 0x85,
    USB_AUDIO_CS_GET_STAT  = 0xFF

} USB_AUDIO_CS_REQUEST_CODE;

// *****************************************************************************
/* Audio Terminal Control Selectors.

  Summary:
    Identifies the Audio Terminal Control Selectors.

  Description:
    This enumeration identifies the possible Audio Terminal Control Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_AUDIO_TE_CONTROL_UNDEFINED      = 0x00,
    USB_AUDIO_COPY_PROTECT_CONTROL      = 0x01

} USB_AUDIO_TERMINAL_CONTROL_SELECTOR;

// *****************************************************************************
/* Audio Feature Unit Control Selector.

  Summary:
    Identifies the Audio Feature Unit Control Selector.

  Description:
    This enumeration identifies the possible Audio Feature Unit Control
    Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_AUDIO_FU_CONTROL_UNDEFINED        = 0x00,
    USB_AUDIO_MUTE_CONTROL                = 0x01,
    USB_AUDIO_VOLUME_CONTROL              = 0x02,
    USB_AUDIO_BASS_CONTROL                = 0x03,
    USB_AUDIO_MID_CONTROL                 = 0x04,
    USB_AUDIO_TREBLE_CONTROL              = 0x05,
    USB_AUDIO_GRAPHIC_EQUALIZER_CONTROL   = 0x06,
    USB_AUDIO_AUTOMATIC_GAIN_CONTROL      = 0x07,
    USB_AUDIO_DELAY_CONTROL               = 0x08,
    USB_AUDIO_BASS_BOOST_CONTROL          = 0x09,
    USB_AUDIO_LOUDNESS_CONTROL            = 0x0A

} USB_AUDIO_FEATURE_UNIT_CONTROL_SELECTORS;    

// *****************************************************************************
/* Audio Up/Down-mix Processing Unit Control Selector.

  Summary:
    Identifies the Audio Up/Down-mix Processing Unit Control Selector.

  Description:
    This enumeration identifies the possible Audio Up/Down-mix Processing Unit 
    Control Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_AUDIO_UD_CONTROL_UNDEFINED     = 0x00,
    USB_AUDIO_UD_ENABLE_CONTROL        = 0x01,
    USB_AUDIO_UD_MODE_SELECT_CONTROL   = 0x02

} USB_AUDIO_UP_DOWN_MIX_PROCESSING_UNIT_CONTROL_SELECTORS;

// *****************************************************************************
/* Audio Dolby Prologic Processing Unit Control Selector.

  Summary:
    Identifies the Audio Dolby Prologic Processing Unit Control Selector.

  Description:
    This enumeration identifies the possible Audio Dolby Prologic Processing Unit 
    Control Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_AUDIO_DP_CONTROL_UNDEFINED      = 0x00,
    USB_AUDIO_DP_ENABLE_CONTROL         = 0x01,
    USB_AUDIO_DP_MODE_SELECT_CONTROL    = 0x02

} USB_AUDIO_DOLBY_PROLOGIC_PROCESSING_UNIT_CONTROL_SELECTOR;

// *****************************************************************************
/* Audio 3D Stereo Extender Processing Unit Control Selector.

  Summary:
    Identifies the Audio 3D Stereo Extender Processing Unit Control Selector.

  Description:
    This enumeration identifies the possible Audio 3D Stereo Extender Processing
    Unit Control Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_AUDIO_3D_CONTROL_UNDEFINED      = 0x00,
    USB_AUDIO_3D_ENABLE_CONTROL         = 0x01,
    USB_AUDIO_SPACIOUSNESS_CONTROL      = 0x02

} USB_AUDIO_3D_STEREO_EXTENDER_PROCESSING_UNIT_CONTROL_SELECTOR;

// *****************************************************************************
/* Audio Reverberation Processing Unit Control Selector.

  Summary:
    Identifies the Audio Reverberation Processing Unit Control Selector.

  Description:
    This enumeration identifies the possible Audio Reverberation Processing Unit
    Control Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_AUDIO_RV_CONTROL_UNDEFINED      = 0x00,
    USB_AUDIO_RV_ENABLE_CONTROL         = 0x01,
    USB_AUDIO_REVERB_LEVEL_CONTROL      = 0x02,
    USB_AUDIO_REVERB_TIME_CONTROL       = 0x03,
    USB_AUDIO_REVERB_FEEDBACK_CONTROL   = 0x04

} USB_AUDIO_REVERBERATION_PROCESSING_UNIT_CONTROL_SELECTORS;

// *****************************************************************************
/* Audio Chorus Processing Unit Control Selector.

  Summary:
    Identifies the Audio Chorus Processing Unit Control Selector.

  Description:
    This enumeration identifies the possible Audio Chorus Processing Unit 
    Control Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_AUDIO_CH_CONTROL_UNDEFINED  = 0x00,
    USB_AUDIO_CH_ENABLE_CONTROL     = 0x01,
    USB_AUDIO_CHORUS_LEVEL_CONTROL  = 0x02,
    USB_AUDIO_CHORUS_RATE_CONTROL   = 0x03,
    USB_AUDIO_CHORUS_DEPTH_CONTROL  = 0x04

} USB_AUDIO_CHORUS_PROCESSING_UNIT_CONTROL_SELECTORS;

// *****************************************************************************
/* Audio Dynamic Range Compressor Processing Unit Control Selector.

  Summary:
    Identifies the Audio Dynamic Range Compressor Processing Unit Control
    Selector.

  Description:
    This enumeration identifies the possible Audio Dynamic Range Compressor 
    Processing Unit Control Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_AUDIO_DR_CONTROL_UNDEFINED      = 0x00,
    USB_AUDIO_DR_ENABLE_CONTROL         = 0x01,
    USB_AUDIO_COMPRESSION_RATE_CONTROL  = 0x02,
    USB_AUDIO_MAXAMPL_CONTROL           = 0x03,
    USB_AUDIO_THRESHOLD_CONTROL         = 0x04,
    USB_AUDIO_ATTACK_TIME               = 0x05,
    USB_AUDIO_RELEASE_TIME              = 0x06

} USB_AUDIO_DYNAMIC_RANGE_COMPRESSOR_PROCESSING_UNIT_CONTROL_SELECTORS;

// *****************************************************************************
/* Audio Extension Unit Control Selector.

  Summary:
    Identifies the Audio Extension Unit Control Selector.

  Description:
    This enumeration identifies the possible Audio Extension Unit Control
    Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_AUDIO_XU_CONTROL_UNDEFINED  = 0x00,
    USB_AUDIO_XU_ENABLE_CONTROL     = 0x01

} USB_AUDIO_EXTENSION_UNIT_CONTROL_SELECTORS;

// *****************************************************************************
/* Audio Endpoint Control Selector.

  Summary:
    Identifies the Audio Endpoint Control Selector.

  Description:
    This enumeration identifies the possible Audio Endpoint Control Selectors.

  Remarks:
    None.
*/

typedef enum
{
    USB_AUDIO_EP_CONTROL_UNDEFINED  = 0x00,
    USB_AUDIO_SAMPLING_FREQ_CONTROL = 0x01,
    USB_AUDIO_PITCH_CONTROL         = 0x02

} USB_AUDIO_ENDPOINT_CONTROL_SELECTORS;

// *****************************************************************************
/* USB Audio Copy Protect Control Codes.

  Summary:
    Identifies the Copy Control codes for USB Audio Class.

  Description:
    This enumeration identifies the possible Copy Control Protect codes for USB
    Audio Class. Refer the document "Universal Serial Bus Device Class
    Definition for Audio Devices revision 1.0" Section 5.2.2.1.3.1.

  Remarks:
    None.
*/
typedef enum
{
    /* Copying is permitted without restriction.  The material is either not
       copyrighted or the copyright is not asserted */
    USB_AUDIO_CPL0 = 0x00 ,

    /* One generation of copies can be made. The material is copyright protected
       and is the original */
    USB_AUDIO_CPL1 = 0x01 ,

    /* The material is copyright protected and no digital copying is permitted
       */
    USB_AUDIO_CPL2 = 0x02

} USB_AUDIO_COPY_PROTECT_CONTROL_PARAMETER;

// *****************************************************************************
/* USB Audio Format Codes.

  Summary:
    Identifies the USB Audio Format codes for USB Audio Class.

  Description:
    This enumeration identifies the possible USB Audio Format codes for USB
    Audio Class. Refer the document "Universal Serial Bus Device Class Definition
    for Audio Data Formats" Table A-1, Table A-2 and Table A-3.

  Remarks:
    None.
*/
typedef enum
{
    USB_AUDIO_FORMAT_TYPE_I_UNDEFINED = 0x0000,

    USB_AUDIO_FORMAT_PCM = 0x0001,

    USB_AUDIO_FORMAT_PCM8 = 0x0002,

} USB_AUDIO_FORMAT_CODE,
  USB_AUDIO_V1_FORMAT_TAG;

// *****************************************************************************
/* USB Audio Status Interrupt Endpoint Status Word Format.

  Summary:
    Specifies the format of USB Audio Status Interrupt Endpoint Status Word
    Format.

  Description:
    This type identifies the format of the USB Audio Status Interrupt Endpoint
    Status Word. This is defined in Table 3-1 of the USB Device Class Definition 
    for Audio Devices Release 1.0 document.

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
} USB_AUDIO_INTERRUPT_STATUS_WORD;

// *****************************************************************************
/* USB Audio Class Specific Audio Control Interface Header Descriptor

  Summary:
    Identifies the USB Audio Class Specific Audio Control Interface Header
    Descriptor.

  Description:
    This type identifies the USB Audio Class Specific Audio Control Interface
    Header Descriptor. This structure is as per Table 4-2 of the USB Device
    Class Definition for Audio Device v1.0 document.

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
	
    /* Audio Device Class Specification Release Number in BCD format*/	
    uint16_t bcdADC;
	
    /* Total number of bytes returned for the class-specific AudioControl
       interface descriptor. Includes the combined length of this descriptor
       header and all Unit and Terminal descriptors */
    uint16_t wTotalLength;

    /* The number of AudioStreaming and MIDIStreaming interfaces in the Audio
       Interface Collection to which this AudioControl interface belongs. This
       is equal to USB_DEVICE_AUDIO_STREAMING_INTERFACES_NUMBER.*/
    uint8_t bInCollection;

    /* Note: The subsequent members of this structure should contain the
     * interface numbers of Audio and MIDI streaming interfaces in the
     * collection. The total number of interface entries should match the value
     * of bInCollection member of this structure. The type of each entry should
     * be USB_DEVICE_AUDIO_STREAMING_INTERFACE_NUMBER. */

} USB_AUDIO_CS_AC_INTERFACE_HEADER_DESCRIPTOR;

// *****************************************************************************
/* Audio Input Terminal Descriptor Type

  Summary:
    Identifies the Audio Input Terminal Descriptor Type.

  Description:
    This type identifies the Audio Input Terminal Descriptor.  This structure is
    as per Table 4-3 of the USB Device Class Definition for Audio Device v1.0
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

} USB_AUDIO_INPUT_TERMINAL_DESCRIPTOR;

// *****************************************************************************
/* Audio Output Terminal Descriptor Type

  Summary:
    Identifies the Audio Output Terminal Descriptor Type.

  Description:
    This type identifies the Audio Output Terminal Descriptor.  This structure
    is as per Table 4-4 of the USB Device Class Definition for Audio Device v1.0
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

} USB_AUDIO_OUTPUT_TERMINAL_DESCRIPTOR;

// *****************************************************************************
/* Audio Feature Unit Descriptor Header Type

  Summary:
    Identifies the Audio Feature Unit Descriptor Type.

  Description:
    This type identifies the Audio Feature Unit Descriptor.  This structure
    is as per Table 4-7 of the USB Device Class Definition for Audio Device v1.0
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

    /* Constant uniquely identifying the Unit within the audio function.*/
    uint8_t bUnitID;

    /* Source Unit or Terminal ID */
    uint8_t bSourceID;

    /* Size in Bytes of an element in the Control array */
    uint8_t bControlSize;

} USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER;

// *****************************************************************************
/* Audio Feature Unit BMA Control

  Summary:
    Identifies the Audio Feature Unit BMA control type.

  Description:
    This type identifies the Audio Feature Unit BMA control.  This structure
    is as per Table 4-7 of the USB Device Class Definition for Audio Device v1.0
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
} USB_AUDIO_FEATURE_UNIT_BMA_CONTROLS;

// *****************************************************************************
/* USB Audio Class Specific Audio Streaming Interface Descriptor

  Summary:
    Identifies the USB Audio Class Specific Audio Streaming Interface Descriptor
    Type.

  Description:
    This type identifies the USB Audio Class Specific Audio Streaming Interface
    Descriptor.  This structure is as per Table 4-19 of the USB Device Class
    Definition for Audio Device v1.0 document.

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

    /* Audio data format of this interface */
    uint16_t wFormatTag;

} USB_AUDIO_CS_AS_INTERFACE_DESCRIPTOR;

// *****************************************************************************
/* USB Audio Class Specific Audio Streaming Isochronous Audio Data Endpoint 
   Descriptor

  Summary:
    Identifies the USB Audio Class Specific Audio Streaming Isochronous Audio
    Data Endpoint Descriptor

  Description:
    This type identifies the USB Audio Class Specific Audio Streaming
    Isochronous Audio Data Endpoint Descriptor Type. This structure is as per
    Table 4-21 of the USB Device Class Definition for Audio Device v1.0 document.

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

} USB_AUDIO_CS_AS_ISOCHRONOUS_AUDIO_DATA_EP_DESCRIPTOR;

// *****************************************************************************
/* USB Audio Control Interface Set and Get Request.

  Summary:
    Identifies the type of the USB Audio Control Interface Set and Get Request.

  Description:
    This type identifies the type of the USB Audio Control Interface Set and Get
    Request.  The application can type cast the received audio class specific
    setup packet to this type in order to service Control attribute Set and Get
    requests. This structure is as per Table 5-1 and 5-2 of the USB Device Class
    Definition for Audio Device v1.0 document. Refer to section 5.2.1.1 and
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

} USB_AUDIO_CONTROL_INTERFACE_REQUEST;

// *****************************************************************************
/* USB Audio Terminal Control Set and Get Request.

  Summary:
    Identifies the type of the USB Audio Terminal Control Set and Get Request.

  Description:
    This type identifies the type of the USB Audio Terminal Control Set and Get
    Request.  The application can type cast the received audio class specific
    setup packet to this type in order to service terminal control Set and Get
    requests. This structure is as per Table 5-3 and 5-4 of the USB Device Class
    Definition for Audio Device v1.0 document. Refer to section 5.2.2.1.1 and
    section 5.2.2.1.2 of this document for detailed description and request
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

    /* This field is usually 0 */
    unsigned :8;

    /* Identifies the control to be accessed */          
    uint8_t controlSelector;

    /* Identifies the interface number */
    uint8_t interfaceNumber;
    
    /* Identifies the terminal ID */
    uint8_t terminalId;
    
    /* Length of the parameter block */
    uint16_t wLength;
    
} USB_AUDIO_TERMINAL_CONTROL_REQUEST;

// *****************************************************************************
/* USB Audio Mixer Unit Control Set and Get Request.

  Summary:
    Identifies the type of the USB Audio Mixer Unit Control Set and Get Request.

  Description:
    This type identifies the type of the USB Audio Mixer Unit Control Set and
    Get Request.  The application can type cast the received audio class
    specific setup packet to this type in order to service mixer unit control
    Set and Get requests. This structure is as per Table 5-6 and 5-7 of the USB
    Device Class Definition for Audio Device v1.0 document. Refer to section
    5.2.2.2.1 and section 5.2.2.2.2 of this document for detailed description
    and request specific interpretation of the fields.

  Remarks:
    Always needs to be packed.
*/

typedef struct __attribute__((packed))
{
    /* Request type SET or GET */
    uint8_t bmRequestType;

    /* Identifies the attribute to be accessed */
    uint8_t bRequest;

    /* Output Channel Number */
    uint8_t OCN;

    /* Input Channel Number */
    uint8_t ICN;

    /* Identifies the interface number */
    uint8_t interfaceNumber;
    
    /* Identifies the Mixer Unit ID */
    uint8_t mixerUnitID;
    
    /* Length of the parameter block */
    uint16_t wLength;

} USB_AUDIO_MIXER_UNIT_CONTROL_REQUEST;

// *****************************************************************************
/* USB Audio Selector Unit Control Set and Get Request.

  Summary:
    Identifies the type of the USB Audio Selector Unit Control Set and Get Request.

  Description:
    This type identifies the type of the USB Audio Selector Unit Control Set and
    Get Request.  The application can type cast the received audio class
    specific setup packet to this type in order to service selector unit control
    Set and Get requests. This structure is as per Table 5-11 and 5-12 of the USB
    Device Class Definition for Audio Device v1.0 document. Refer to section
    5.2.2.3.1 and section 5.2.2.3.2 of this document for detailed description
    and request specific interpretation of the fields.

  Remarks:
    Always needs to be packed.
*/

typedef struct __attribute__((packed))
{
    /* Request type SET or GET */
    uint8_t bmRequestType;

    /* Identifies the attribute to be accessed */
    uint8_t bRequest;

    /* This value is always 0 */
    unsigned :16;
    
    /* Identifies the interface number */
    uint8_t interfaceNumber;

    /* Identifies the Selector Unit ID */
    uint8_t selectorUnitId;
    
    /* Length of the parameter block */
    uint16_t wLength;

} USB_AUDIO_SELECTOR_UNIT_CONTROL_REQUEST;

// *****************************************************************************
/* USB Audio Feature Unit Control Set and Get Request.

  Summary:
    Identifies the type of the USB Audio Feature Unit Control Set and Get Request.

  Description:
    This type identifies the type of the USB Audio Feature Unit Control Set and
    Get Request.  The application can type cast the received audio class
    specific setup packet to this type in order to service the feature unit
    control Set and Get requests. This structure is as per Table 5-14 and 5-15
    of the USB Device Class Definition for Audio Device v1.0 document. Refer to
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

} USB_AUDIO_FEATURE_UNIT_CONTROL_REQUEST;

// *****************************************************************************
/* USB Audio Processing Unit Control Set and Get Request.

  Summary:
    Identifies the type of the USB Audio Processing Unit Control Set and Get
    Request.

  Description:
    This type identifies the type of the USB Audio Processing Unit Control Set
    and Get Request. The application can type cast the received audio class
    specific setup packet to this type in order to service the processing unit
    control Set and Get requests. This structure is as per Table 5-36 and 5-37
    of the USB Device Class Definition for Audio Device v1.0 document. Refer to
    section 5.2.2.5.1 and section 5.2.2.5.2 of this document for detailed
    description and request specific interpretation of the fields.

  Remarks:
    Always needs to be packed.
*/

typedef struct
{
    /* Request type Set or Get */
    uint8_t bmRequestType;

    /* Identifies the attribute to be accessed */
    uint8_t bRequest;

    /* Not used */
    unsigned :8;
    
    /* Identifies the Control Selector */          
    uint8_t controlSelector;

    /* Identifies the interface number */
    uint8_t interfaceNumber;

    /* Identifies the processing unit ID */
    uint8_t processingUnitID;
    
    /* Length of the parameter block */
    uint16_t wLength;

} USB_AUDIO_PROCESSING_UNIT_CONTROL_REQUEST;

// *****************************************************************************
/* USB Audio Mixer Unit Descriptor Header.

  Summary:
   Structure describing USB Audio Mixer unit descriptor Header.

  Description:
    This type identifies the type of the USB Audio mixer unit descriptor.  This
    structure is as per Table 5-45 USB Device Class Definition for Audio Device
    v1.0 document. This structure represents first 5 Bytes of the USB Audio mixer
    unit descriptor.

    The complete structure of the USB_AUDIO_MIXER_UNIT_DESCRIPTOR should be as
    such.

    struct __attribute__((packed))
    {
        USB_AUDIO_MIXER_UNIT_DESCRIPTOR_HEADER header;
        USB_AUDIO_MIXER_UNIT_DESCRIPTOR_SOURCE_ID mixerInputSourceID[NUMBER_OF_MIXER_INPUT_PINS];
        USB_AUDIO_MIXER_UNIT_DESCRIPTOR_CHANNEL_INFO channelInfo;
        USB_AUDIO_MIXER_UNIT_DESCRIPTOR_BMACONTROLS mixingControls[NUMBER_OF_PROGRAMMABLE_MIXING_CONTROLS];
        USB_AUDIO_MIXER_UNIT_DESCRIPTOR_FOOTER stringdesciptorIndex;
    
    } USB_AUDIO_MIXER_UNIT_DESCRIPTOR;

  Remarks:
    This structure is a part of the USB_AUDIO_MIXER_UNIT_DESCRIPTOR. The
    USB_AUDIO_MIXER_UNIT_DESCRIPTOR cannot be defined as it an open ended data
    structure.

*/

typedef struct
{
    /* Size of this descriptor, in bytes */
    uint8_t bLength;

    /* CS_INTERFACE descriptor type - constant */
    uint8_t bDescriptorType;

    /* MIXER_UNIT descriptor subtype - constant*/
    uint8_t bDescriptorSubtype;

    /* Constant uniquely identifying the Unit within the audio function.*/
    uint8_t bUnitID;

    /* Number of Input Pins of this Unit */
    uint8_t bNrInPins;

    /* This header is followed by USB_AUDIO_MIXER_UNIT_DESCRIPTOR_SOURCE_ID
       type of entries identifying the ID of the entities connected to input pin
       of the mixer. So the first entry is the ID of the entity that is
       connected to the first mixer input pin and so on.*/ 
    
} USB_AUDIO_MIXER_UNIT_DESCRIPTOR_HEADER;

// *****************************************************************************
/* USB Audio Mixer Unit Descriptor Source ID.

  Summary:
   Structure describing USB Audio Mixer unit descriptor Source ID.

  Description:
    This type identifies the type of the USB Audio mixer unit descriptor.  This
    structure is as per Table 5-45 USB Device Class Definition for Audio Device
    v1.0 document.

  Remarks:
    This structure is a part of the USB_AUDIO_MIXER_UNIT_DESCRIPTOR. The
    USB_AUDIO_MIXER_UNIT_DESCRIPTOR cannot be defined as it an open ended data
    structure.
*/

typedef struct
{
    
    /* ID of the Unit or Terminal to which the first Input Pin of this Mixer
       Unit is connected */
    uint8_t baSourceID;

} USB_AUDIO_MIXER_UNIT_DESCRIPTOR_SOURCE_ID;

// *****************************************************************************
/* USB Audio Mixer Unit Descriptor Channel Info.

  Summary:
   Structure describing USB Audio Mixer unit descriptor Channel Info.

  Description:
    This type identifies the type of the USB Audio mixer unit descriptor.  This
    structure is as per Table 5-45 USB Device Class Definition for Audio Device
    v1.0 document.

  Remarks:
    This structure is a part of the USB_AUDIO_MIXER_UNIT_DESCRIPTOR. The
    USB_AUDIO_MIXER_UNIT_DESCRIPTOR cannot be defined as it an open ended data
    structure.
*/

typedef struct
{
    /* Number of logical output channels in the Mixer’s output audio channel
       cluster */
    uint8_t bNrChannels;

    /* Describes the spatial location of the logical channels. */
    uint16_t wChannelConfig;

    /* Index of a string descriptor, describing the name of the first logical
       channel */
    uint8_t iChannelNames;

} USB_AUDIO_MIXER_UNIT_DESCRIPTOR_CHANNEL_INFO;

// *****************************************************************************
/* USB Audio Mixer Unit Descriptor bmaControls.

  Summary:
   Structure describing USB Audio Mixer unit descriptor bmaControls.

  Description:
    This type identifies the type of the USB Audio mixer unit descriptor.  This
    structure is as per Table 5-45 USB Device Class Definition for Audio Device
    v1.0 document.

  Remarks:
    This structure is a part of the USB_AUDIO_MIXER_UNIT_DESCRIPTOR. The
    USB_AUDIO_MIXER_UNIT_DESCRIPTOR cannot be defined as it an open ended data
    structure.
*/

typedef struct
{
    /* Bit map indicating which mixing Controls are programmable. */
    uint8_t bmControls;

}USB_AUDIO_MIXER_UNIT_DESCRIPTOR_BMACONTROLS;

// *****************************************************************************
/* USB Audio Mixer Unit Descriptor Footer.

  Summary:
   Structure describing USB Audio Mixer unit descriptor Footer.

  Description:
    This type identifies the type of the USB Audio mixer unit descriptor.  This
    structure is as per Table 5-45 USB Device Class Definition for Audio Device
    v1.0 document.

  Remarks:
    This structure is a part of the USB_AUDIO_MIXER_UNIT_DESCRIPTOR. The
    USB_AUDIO_MIXER_UNIT_DESCRIPTOR cannot be defined as it an open ended data
    structure.
*/

typedef struct
{
    /* Index of a string descriptor, describing the Mixer Unit. */
    uint8_t iMixer;

}USB_AUDIO_MIXER_UNIT_DESCRIPTOR_FOOTER; 


// *****************************************************************************
/* USB Audio Type 1 Format Descriptor Header.

  Summary:
   Structure describing USB Audio Type 1 Format Descriptor

  Description:
    This type identifies the type of the USB Audio type 1 Format descriptor. This
    structure is as per Table 2-1 Universal Serial Bus Device Class Definition 
    for Audio Data Formats

  Remarks:
    This structure is a part of the Type 1 Format Descriptor. The
    Type 1 Format descriptor  cannot be defined in a single structure as it is an
    open ended data structure.
*/
typedef struct  
{
    /* Size of this descriptor, in bytes */
    uint8_t bLength;

    /* CS_INTERFACE descriptor type - constant */
    uint8_t bDescriptorType;

    /* FORMAT_TYPE descriptor subtype - constant*/
    uint8_t bDescriptorSubtype;
    
    /* FORMAT_TYPE_I. Constant identifying the Format Type the AudioStreaming
       interface is using.*/
    uint8_t bFormatType; 
    
    /* Indicates the number of physical channels in the audio data stream.*/
    uint8_t bNrChannels; 
    
    /* The number of bytes occupied by one audio subframe. Can be 1, 2, 3 or 4*/
    uint8_t bSubframeSize; 
    
    /* The number of effectively used bits from the available bits in an audio subframe */ 
    uint8_t bBitResolution; 
    
    /* Indicates how the sampling frequency can be programmed:
            0: Continuous sampling frequency
            1..255: The number of discrete sampling frequencies supported by the
                    AudioStreaming interface (ns)*/
    uint8_t bSamFreqType; 
} USB_AUDIO_FORMAT_I_DESCRIPTOR_HEADER; 

// *****************************************************************************
/* USB Audio Endpoint Control Set and Get Request.

  Summary:
    Identifies the type of the USB Audio Endpoint Control Set and Get Request.

  Description:
    This type identifies the type of the USB Audio Endpoint Control Set and
    Get Request.  The application can type cast the received audio class
    specific setup packet to this type in order to service the Endpoint
    control Set and Get requests. This structure is as per Table 5-56 and 5-57
    of the USB Device Class Definition for Audio Device v1.0 document. Refer to
    section 5.2.3.2.1  and section 5.2.3.2.2 of this document for detailed
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

    /* This field is always Zero */
    unsigned :8 ;

    /* Identifies the control selector */
    uint8_t controlSelector;

    /* Identifies the Endpoint number */
    uint8_t endpointNumber;
    
    /* This field is always Zer0 */
    unsigned :8;

    /* Length of the parameter block */
    uint16_t wLength;

} USB_AUDIO_ENDPOINT_CONTROL_REQUEST;


// *****************************************************************************
/* USB Audio Cluster Channel Configuration 

  Summary:
    Identifies the type of the USB Audio Cluster Channel Configuration.

  Description:
    This type identifies the type of the USB Audio Cluster Channel Configuration.
	This structure is as per section 3.7.2.3 
    of the USB Device Class Definition for Audio Device v1.0 document. 

  Remarks:
    Always needs to be packed.
*/
typedef union  __attribute__((packed))
{
    /* Value */
	uint16_t value; 
	
	struct 
	{
		unsigned leftFront:1; 
		unsigned rightFront:1; 
		unsigned centerFront:1; 	
		unsigned LFE:1; 
		unsigned leftSurround:1; 
		unsigned rightSurround:1; 
		unsigned leftOfCenter:1; 
		unsigned rightOfCenter:1; 
		unsigned surround:1; 
		unsigned sideLeft:1; 
		unsigned sideRight:1; 
		unsigned top:1; 
		unsigned :4; 
	};

} USB_AUDIO_CHANNEL_CONFIG; 

// *****************************************************************************
/* USB Audio Volume Control Range 

  Summary:
    Identifies the type of the USB Audio Volume Control Range. 

  Description:
    This type identifies the type of the USB Audio Volume Control Range.
	This structure includes three fields for Minimum, Maximum and Resolution 
	volume control. 

  Remarks:
    Always needs to be packed.
*/
typedef struct __attribute__((packed))
{
	/* Minimum Setting */ 
	uint16_t wMIN; 
	
	/* Maximum Setting */ 
	uint16_t wMAX; 
	
	/* Resolution */ 
	uint16_t wRES; 
	
} USB_AUDIO_VOLUME_CONTROL_RANGE; 

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif

