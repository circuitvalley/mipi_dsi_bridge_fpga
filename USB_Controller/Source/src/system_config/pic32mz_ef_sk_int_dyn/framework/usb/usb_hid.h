/*******************************************************************************
  USB HID Function Driver

  Company:
    Microchip Technology Inc.

  File Name:
    usb_hid.h

  Summary:
    USB HID Definitions

  Description:
    This file contains USB HID specification definitions and included by the USB
    Stack. The application can include this file if it needs to access any of
    the HID specification definitions.
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

#ifndef USB_HID_H
#define USB_HID_H

#include <stdint.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

/* HID Class code */
#define USB_HID_CLASS_CODE               0x03

/*
 * Macro defines the tag corresponding to Input item.
 */
#define USB_HID_MAIN_ITEM_TAG_INPUT                       8

/*
 * Macro defines the tag corresponding to Output item.
 */
#define USB_HID_MAIN_ITEM_TAG_OUTPUT                      9

/*
 * Macro defines the tag corresponding to Feature item.
 */
#define USB_HID_MAIN_ITEM_TAG_FEATURE                     11

/*
 * Macro defines the tag corresponding to Collection item.
 */
#define USB_HID_MAIN_ITEM_TAG_BEGIN_COLLECTION            10

/*
 * Macro defines the tag corresponding to End Collection item.
 */
#define USB_HID_MAIN_ITEM_TAG_END_COLLECTION              12

/*
 * Macro defines the tag corresponding to Usage Page.
 */
#define USB_HID_GLOBAL_ITEM_TAG_USAGE_PAGE                0

/*
 * Macro defines the tag corresponding to Logical Minimum item.
 */
#define USB_HID_GLOBAL_ITEM_TAG_LOGICAL_MINIMUM           1

/*
 * Macro defines the tag corresponding to Logical Maximum item.
 */
#define USB_HID_GLOBAL_ITEM_TAG_LOGICAL_MAXIMUM           2

/*
 * Macro defines the tag corresponding to Physical Minimum item.
 */
#define USB_HID_GLOBAL_ITEM_TAG_PHY_MINIMUM               3

/*
 * Macro defines the tag corresponding to Physical Maximum item.
 */
#define USB_HID_GLOBAL_ITEM_TAG_PHY_MAXIMUM               4

/*
 * Macro defines the tag corresponding to Unit Exponent item.
 */
#define USB_HID_GLOBAL_ITEM_TAG_UNIT_EXPONENT             5

/*
 * Macro defines the tag corresponding to Unit item.
 */
#define USB_HID_GLOBAL_ITEM_TAG_UNIT                      6

/*
 * Macro defines the tag corresponding to Report Size item.
 */
#define USB_HID_GLOBAL_ITEM_TAG_REPORT_SIZE               7

/*
 * Macro defines the tag corresponding to Report ID item.
 */
#define USB_HID_GLOBAL_ITEM_TAG_REPORT_ID                 8

/*
 * Macro defines the tag corresponding to Report Count item.
 */
#define USB_HID_GLOBAL_ITEM_TAG_REPORT_COUNT              9

/*
 * Macro defines the tag corresponding to Push item.
 */
#define USB_HID_GLOBAL_ITEM_TAG_PUSH                      10

/*
 * Macro defines the tag corresponding to Pop item.
 */
#define USB_HID_GLOBAL_ITEM_TAG_POP                       11

/*
 * Macro defines local item tag corresponding to Usage.
 */
#define USB_HID_LOCAL_ITEM_TAG_USAGE                      0

/*
 * Macro defines local item tag corresponding to Usage Minimum.
 */
#define USB_HID_LOCAL_ITEM_TAG_USAGE_MINIMUM              1

/*
 * Macro defines local item tag corresponding to Usage Maximum.
 */
#define USB_HID_LOCAL_ITEM_TAG_USAGE_MAXIMUM              2

/*
 * Macro defines local item tag corresponding to Designator index.
 */
#define USB_HID_LOCAL_ITEM_TAG_DESIGNATOR_INDEX           3

/*
 * Macro defines local item tag corresponding to Designator Minimum.
 */
#define USB_HID_LOCAL_ITEM_TAG_DESIGNATOR_MINIMUM         4

/*
 * Macro defines local item tag corresponding to Designator Maximum.
 */
#define USB_HID_LOCAL_ITEM_TAG_DESIGNATOR_MAXIMUM         5

/*
 * Macro defines local item tag corresponding to String index.
 */
#define USB_HID_LOCAL_ITEM_TAG_STRING_INDEX               7

/*
 * Macro defines local item tag corresponding to String Minimum.
 */
#define USB_HID_LOCAL_ITEM_TAG_STRING_MINIMUM              8

/*
 * Macro defines local item tag corresponding to String Maximum.
 */
#define USB_HID_LOCAL_ITEM_TAG_STRING_MAXIMUM              9

/*
 * Macro defines local item tag corresponding to Delimiter.
 */
#define USB_HID_LOCAL_ITEM_TAG_DELIMITER                  10

#define USB_HID_USAGE_MOUSE 2
#define USAGE_X 0x30
#define USAGE_Y 0x31
#define USAGE_Z 0x32

typedef enum
{
    USB_HID_USAGE_ID_BUTTON1   = 1,
    USB_HID_USAGE_ID_BUTTON2   = 2,
    USB_HID_USAGE_ID_BUTTON3,
    USB_HID_USAGE_ID_BUTTON4,
    USB_HID_USAGE_ID_BUTTON5
            
} USB_HID_BUTTON_ID;

typedef enum
{
    USB_HID_BUTTON_RELEASED   = 0,
	USB_HID_BUTTON_PRESSED
            
} USB_HID_BUTTON_STATE;


typedef enum
{
    USB_HID_KEY_RELEASED   = 0,
	USB_HID_KEY_PRESSED
            
} USB_HID_KEY_EVENT;

typedef struct __attribute__((packed))
{
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint16_t    bcdHID;
    uint8_t     bCountryCode;
    uint8_t     bNumDescriptors;
    uint8_t     bReportDescriptorType;
    uint16_t    wItemLength;

} USB_HID_DESCRIPTOR;

typedef struct __attribute__((packed))
{
    uint8_t     bDescriptorType;
    uint16_t    wDescriptorLength;

} USB_HID_DESCRIPTOR_SUBORDINATE;

typedef struct __attribute__((packed))
{
    unsigned    bSize :2;
    unsigned    bType :2;
    unsigned    bTag  :4;

} USB_HID_REPORT_ITEM_HEADER_SHORT;

typedef struct __attribute__((packed))
{
    unsigned    bSize :2;
    unsigned    bType :2;
    unsigned    bTag  :4;
    uint8_t     bDataSize;
    uint8_t     bLongItemTag;

} USB_HID_REPORT_ITEM_HEADER_LONG;

typedef enum
{
    USB_HID_DESCRIPTOR_TYPES_HID                                            = 0x21,
    USB_HID_DESCRIPTOR_TYPES_REPORT                                         = 0x22,
    USB_HID_DESCRIPTOR_TYPES_PHYSICAL_DESCRIPTOR                            = 0x23
    /* Reserved                                                         = 0x24-0x2F */

} USB_HID_DESCRIPTOR_TYPES;

typedef enum
{
    USB_HID_REQUESTS_GET_REPORT                                             = 0x01,
    USB_HID_REQUESTS_GET_IDLE                                               = 0x02,
    USB_HID_REQUESTS_GET_PROTOCOL                                           = 0x03,
    /* Reserved                                                         = 0x04-0x08 */
    USB_HID_REQUESTS_SET_REPORT                                             = 0x09,
    USB_HID_REQUESTS_SET_IDLE                                               = 0x0A,
    USB_HID_REQUESTS_SET_PROTOCOL                                           = 0x0B

} USB_HID_REQUESTS;

typedef enum
{
    USB_HID_REPORT_TYPE_INPUT                                               = 0x01,
    USB_HID_REPORT_TYPE_OUTPUT                                              = 0x02,
    USB_HID_REPORT_TYPE_FEATURE                                             = 0x03
    /* Reserved                                                         = 0x04-0xFF */
} USB_HID_REPORT_TYPE;



typedef enum
{
    USB_HID_REPORT_ITEM_HEADER_BTYPE_MAIN                                   = 0,
    USB_HID_REPORT_ITEM_HEADER_BTYPE_GLOBAL                                 = 1,
    USB_HID_REPORT_ITEM_HEADER_BTYPE_LOCAL                                  = 2,
    USB_HID_REPORT_ITEM_HEADER_BTYPE_RESERVED                               = 3
} USB_HID_REPORT_ITEM_HEADER_BTYPE;

typedef enum
{
    USB_HID_BOOT_PROTOCOL                                   = 0,
    USB_HID_REPORT_PROTOCOL                                 = 1,
} USB_HID_PROTOCOL_TYPE;

typedef enum
{
    USB_HID_COLLECTION_PHYSICAL                             = 0,
	USB_HID_COLLECTION_APPLICATION,
	USB_HID_COLLECTION_LOGICAL,
	USB_HID_COLLECTION_REPORT,
	USB_HID_COLLECTION_NAMED_ARRAY,
	USB_HID_COLLECTION_USAGE_SWITCH,
	USB_HID_COLLECTION_USAGE_MODIFIER
} USB_HID_COLLECTION_TYPE;


typedef struct
{
    uint32_t isConstant : 1;
    uint32_t isVariable : 1;
    uint32_t isRelative : 1;
    uint32_t isWrap : 1;
    uint32_t isNonLinear : 1;
    uint32_t isNoPreferredState : 1;
    uint32_t isNulllState : 1;
    uint32_t reserved : 1;
    uint32_t isBufferedBytes : 1;
    uint32_t reserved1 : 23;
    
} USB_HID_INPUT_ITEM_OPTIONAL_DATA;

typedef struct
{
    uint32_t isConstant : 1;
    uint32_t isVariable : 1;
    uint32_t isRelative : 1;
    uint32_t isWrap : 1;
    uint32_t isNonLinear : 1;
    uint32_t isNoPreferredState : 1;
    uint32_t isNulllState : 1;
    uint32_t isVolatile : 1;
    uint32_t isBufferedBytes : 1;
    uint32_t reserved1 : 23;
    
} USB_HID_OUTPUT_ITEM_OPTIONAL_DATA;

typedef struct
{
    uint32_t isConstant : 1;
    uint32_t isVariable : 1;
    uint32_t isRelative : 1;
    uint32_t isWrap : 1;
    uint32_t isNonLinear : 1;
    uint32_t isNoPreferredState : 1;
    uint32_t isNulllState : 1;
    uint32_t isVolatile : 1;
    uint32_t isBufferedBytes : 1;
    uint32_t reserved1 : 23;
    
} USB_HID_FEATURE_ITEM_OPTIONAL_DATA;

typedef struct
{
    USB_HID_COLLECTION_TYPE collectionType;

} USB_HID_COLLECTION_ITEM_OPTIONAL_DATA;

typedef union
{
    uint8_t data1Byte[4];
    uint16_t data2Bytes[2];
    uint32_t data4Bytes;
    
    USB_HID_INPUT_ITEM_OPTIONAL_DATA inputOptionalData;    
    USB_HID_OUTPUT_ITEM_OPTIONAL_DATA  outputOptionalData;
    USB_HID_FEATURE_ITEM_OPTIONAL_DATA  featureOptionalData;
    USB_HID_COLLECTION_ITEM_OPTIONAL_DATA  collectionOptionalData;

} USB_HID_MAIN_ITEM_OPTIONAL_DATA;


typedef enum
{
    USB_HID_COUNTRY_CODE_NOT_SUPPORTED                                      = 0,
    USB_HID_COUNTRY_CODE_ARABIC                                             = 1,
    USB_HID_COUNTRY_CODE_BELGIAN                                            = 2,
    USB_HID_COUNTRY_CODE_CANADIAN_BILINGUAL                                 = 3,
    USB_HID_COUNTRY_CODE_CANADIAN_FRENCH                                    = 4,
    USB_HID_COUNTRY_CODE_CZECH_REPUBLIC                                     = 5,
    USB_HID_COUNTRY_CODE_DANISH                                             = 6,
    USB_HID_COUNTRY_CODE_FINNISH                                            = 7,
    USB_HID_COUNTRY_CODE_FRENCH                                             = 8,
    USB_HID_COUNTRY_CODE_GERMAN                                             = 9,
    USB_HID_COUNTRY_CODE_GREEK                                              = 10,
    USB_HID_COUNTRY_CODE_HEBREW                                             = 11,
    USB_HID_COUNTRY_CODE_HUNGARY                                            = 12,
    USB_HID_COUNTRY_CODE_INTERNATIONAL_ISO                                  = 13,
    USB_HID_COUNTRY_CODE_ITALIAN                                            = 14,
    USB_HID_COUNTRY_CODE_JAPAN_KATAKANA                                     = 15,
    USB_HID_COUNTRY_CODE_KOREAN                                             = 16,
    USB_HID_COUNTRY_CODE_LATIN_AMERICAN                                     = 17,
    USB_HID_COUNTRY_CODE_NETHERLANDS_DUTCH                                  = 18,
    USB_HID_COUNTRY_CODE_NORWEGIAN                                          = 19,
    USB_HID_COUNTRY_CODE_PERSIAN_FARSI                                      = 20,
    USB_HID_COUNTRY_CODE_POLAND                                             = 21,
    USB_HID_COUNTRY_CODE_PORTUGUESE                                         = 22,
    USB_HID_COUNTRY_CODE_RUSSIA                                             = 23,
    USB_HID_COUNTRY_CODE_SLOVAKIA                                           = 24,
    USB_HID_COUNTRY_CODE_SPANISH                                            = 25,
    USB_HID_COUNTRY_CODE_SWEDISH                                            = 26,
    USB_HID_COUNTRY_CODE_SWISS_FRENCH                                       = 27,
    USB_HID_COUNTRY_CODE_SWISS_GERMAN                                       = 28,
    USB_HID_COUNTRY_CODE_SWITZERLAND                                        = 29,
    USB_HID_COUNTRY_CODE_TAIWAN                                             = 30,
    USB_HID_COUNTRY_CODE_TURKISH_Q                                          = 31,
    USB_HID_COUNTRY_CODE_UK                                                 = 32,
    USB_HID_COUNTRY_CODE_US                                                 = 33,
    USB_HID_COUNTRY_CODE_YUGOSLAVIA                                         = 34,
    USB_HID_COUNTRY_CODE_TURKISH_F                                          = 35
    /* Reserved                                                         = 36-255 */
} USB_HID_COUNTRY_CODE;

/* As defined in section 4.2 of the HID v1.11 specification. */
typedef enum
{
    USB_HID_SUBCLASS_CODE_NO_SUBCLASS                                           = 0x00,
    USB_HID_SUBCLASS_CODE_BOOT_INTERFACE_SUBCLASS                               = 0x01
    /* Reserved                                                                 = 0x02-0xFF */
} USB_HID_SUBCLASS_CODE;

/* As defined in section 4.3 of the HID v1.11 specification. */
typedef enum
{
    USB_HID_PROTOCOL_CODE_NONE                                                  = 0x00,
    USB_HID_PROTOCOL_CODE_KEYBOARD                                              = 0x01,
    USB_HID_PROTOCOL_CODE_MOUSE                                                 = 0x02
    /* Reserved                                                                 = 0x03-0xFF */
} USB_HID_PROTOCOL_CODE;

typedef enum
{
    USB_HID_USAGE_PAGE_UNDEFINED                                                = 0x00,
    USB_HID_USAGE_PAGE_GENERIC_DESKTOP_CONTROLS                                 = 0x01,
    USB_HID_USAGE_PAGE_SIMULATION_CONTROLS                                      = 0x02,
    USB_HID_USAGE_PAGE_VR_CONTROLS                                              = 0x03,
    USB_HID_USAGE_PAGE_SPORT_CONTROLS                                           = 0x04,
    USB_HID_USAGE_PAGE_GAME_CONTROLS                                            = 0x05,
    USB_HID_USAGE_PAGE_GENERIC_DEVICE_CONTROLS                                  = 0x06,
    USB_HID_USAGE_PAGE_KEYBOARD_KEYPAD                                          = 0x07,
    USB_HID_USAGE_PAGE_LEDS                                                     = 0x08,
    USB_HID_USAGE_PAGE_BUTTON                                                   = 0x09,
    USB_HID_USAGE_PAGE_ORDINAL                                                  = 0x0A,
    USB_HID_USAGE_PAGE_TELEPHONY                                                = 0x0B,
    USB_HID_USAGE_PAGE_CONSUMER                                                 = 0x0C,
    USB_HID_USAGE_PAGE_DIGITIZER                                                = 0x0D,
    /* Reserved                                                                 = 0x0E */
    USB_HID_USAGE_PAGE_PID_PAGE                                                 = 0x0F,
    USB_HID_USAGE_PAGE_UNICODE                                                  = 0x10,
    /* Reserved                                                                 = 0x11-13 */
    USB_HID_USAGE_PAGE_ALPHANUMERIC_DISPLAY                                     = 0x14,
    /* Reserved                                                                 = 0x15-3f */
    USB_HID_USAGE_PAGE_MEDICAL_INSTRUMENTS                                      = 0x40,
    /* Reserved                                                                 = 0x41-7f */
    USB_HID_USAGE_PAGE_MONITOR                                                  = 0x80,
    USB_HID_USAGE_PAGE_MONITOR_ENUMERATED_VALUES                                = 0x81,
    USB_HID_USAGE_PAGE_MONITOR_VESA_VIRTUAL_CONTROLS                            = 0x82,
    USB_HID_USAGE_PAGE_MONITOR_RESERVED                                         = 0x83,
    USB_HID_USAGE_PAGE_POWER_DEVICE                                             = 0x84,
    USB_HID_USAGE_PAGE_BATTERY_SYSTEM                                           = 0x85,
    USB_HID_USAGE_PAGE_POWER_PAGE_3                                             = 0x86,
    USB_HID_USAGE_PAGE_POWER_PAGE_4                                             = 0x87,
    /* Reserved                                                                 = 0x88-8B */
    USB_HID_USAGE_PAGE_BAR_CODE_SCANNER                                         = 0x8C,
    USB_HID_USAGE_PAGE_WEIGHING_DEVICES                                         = 0x8D,
    USB_HID_USAGE_PAGE_MAGNETIC_STRIPE_READING_DEVICES                          = 0x8E,
    USB_HID_USAGE_PAGE_RESERVED_POINT_OF_SALE_PAGES                             = 0x8F,
    USB_HID_USAGE_PAGE_CAMERA_CONTROL_PAGE                                      = 0x90,
    USB_HID_USAGE_PAGE_ARCADE_PAGE                                              = 0x91,
    /* Reserved                                                                 = 0x92-FEFF */
    /* Vendor-defined                                                           = 0xFF00-FFFF */
    USB_HID_USAGE_PAGE_VENDOR_START                                             = 0xFF00,
    USB_HID_USAGE_PAGE_VENDOR_END                                               = 0xFFFF
} USB_HID_USAGE_PAGE;

typedef enum
{
    USB_HID_GENERIC_DESKTOP_UNDEFINED                                = 0x00,
    USB_HID_GENERIC_DESKTOP_POINTER                                  = 0x01,
    USB_HID_GENERIC_DESKTOP_MOUSE                                    = 0x02,
    /* Reserved                                                                 = 0x03 */
    USB_HID_GENERIC_DESKTOP_JOYSTICK                                 = 0x04,
    USB_HID_GENERIC_DESKTOP_GAME_PAD                                 = 0x05,
    USB_HID_GENERIC_DESKTOP_KEYBOARD                                 = 0x06,
    USB_HID_GENERIC_DESKTOP_KEYPAD                                   = 0x07,
    USB_HID_GENERIC_DESKTOP_MULTI_AXIS_CONTROLLER                    = 0x08,
    USB_HID_GENERIC_DESKTOP_TABLET_PC_SYSTEM_CONTROLS                = 0x09,

    USB_HID_GENERIC_DESKTOP_X                                        = 0x30,
    USB_HID_GENERIC_DESKTOP_Y                                        = 0x31,
    USB_HID_GENERIC_DESKTOP_Z                                        = 0x32,
    USB_HID_GENERIC_DESKTOP_RX                                       = 0x33,
    USB_HID_GENERIC_DESKTOP_RY                                       = 0x34,
    USB_HID_GENERIC_DESKTOP_RZ                                       = 0x35,
    USB_HID_GENERIC_DESKTOP_SLIDER                                   = 0x36,
    USB_HID_GENERIC_DESKTOP_DIAL                                     = 0x37,
    USB_HID_GENERIC_DESKTOP_WHEEL                                    = 0x38,
    USB_HID_GENERIC_DESKTOP_HAT_SWITCH                               = 0x39,
    USB_HID_GENERIC_DESKTOP_COUNTED_BUFFER                           = 0x3A,
    USB_HID_GENERIC_DESKTOP_BYTE_COUNT                               = 0x3B,
    USB_HID_GENERIC_DESKTOP_MOTION_WAKEUP                            = 0x3C,
    USB_HID_GENERIC_DESKTOP_START                                    = 0x3D,
    USB_HID_GENERIC_DESKTOP_SELECT                                   = 0x3E,

    USB_HID_GENERIC_DESKTOP_VX                                       = 0x40,
    USB_HID_GENERIC_DESKTOP_VY                                       = 0x41,
    USB_HID_GENERIC_DESKTOP_VZ                                       = 0x42,
    USB_HID_GENERIC_DESKTOP_VBRX                                     = 0x43,
    USB_HID_GENERIC_DESKTOP_VBRY                                     = 0x44,
    USB_HID_GENERIC_DESKTOP_VBRZ                                     = 0x45,
    USB_HID_GENERIC_DESKTOP_VNO                                      = 0x46,
    USB_HID_GENERIC_DESKTOP_FEATURE_NOTIFICATION                     = 0x47,
    USB_HID_GENERIC_DESKTOP_RESOLUTION_MULTIPLIER                    = 0x48,

    USB_HID_GENERIC_DESKTOP_SYSTEM_CONTROL                           = 0x80,
    USB_HID_GENERIC_DESKTOP_SYSTEM_POWER_DOWN                        = 0x81,
    USB_HID_GENERIC_DESKTOP_SYSTEM_SLEEP                             = 0x82,
    USB_HID_GENERIC_DESKTOP_WAKE_UP                                  = 0x83,
    USB_HID_GENERIC_DESKTOP_CONTEXT_MENU                             = 0x84,
    USB_HID_GENERIC_DESKTOP_MAIN_MENU                                = 0x85,
    USB_HID_GENERIC_DESKTOP_APP_MENU                                 = 0x86,
    USB_HID_GENERIC_DESKTOP_MENU_HELP                                = 0x87,
    USB_HID_GENERIC_DESKTOP_MENU_EXIT                                = 0x88,
    USB_HID_GENERIC_DESKTOP_MENU_SELECT                              = 0x89,
    USB_HID_GENERIC_DESKTOP_MENU_RIGHT                               = 0x8A,
    USB_HID_GENERIC_DESKTOP_MENU_LEFT                                = 0x8B,
    USB_HID_GENERIC_DESKTOP_MENU_UP                                  = 0x8C,
    USB_HID_GENERIC_DESKTOP_MENU_DOWN                                = 0x8D,
    USB_HID_GENERIC_DESKTOP_MENU_COLD_RESTART                        = 0x8E,
    USB_HID_GENERIC_DESKTOP_MENU_WARM_RESTART                        = 0x8F,
    USB_HID_GENERIC_DESKTOP_DPAD_UP                                  = 0x90,
    USB_HID_GENERIC_DESKTOP_DPAD_DOWN                                = 0x91,
    USB_HID_GENERIC_DESKTOP_DPAD_RIGHT                               = 0x92,
    USB_HID_GENERIC_DESKTOP_DPAD_LEFT                                = 0x93,

    USB_HID_GENERIC_DESKTOP_SYSTEM_DOCK                              = 0xA0,
    USB_HID_GENERIC_DESKTOP_SYSTEM_UNDOCK                            = 0xA1,
    USB_HID_GENERIC_DESKTOP_SYSTEM_SETUP                             = 0xA2,
    USB_HID_GENERIC_DESKTOP_SYSTEM_BREAK                             = 0xA3,
    USB_HID_GENERIC_DESKTOP_SYSTEM_DEBUGGER_BREAK                    = 0xA4,
    USB_HID_GENERIC_DESKTOP_APPLICATION_BREAK                        = 0xA5,
    USB_HID_GENERIC_DESKTOP_APPLICATION_DEBUGGER_BREAK               = 0xA6,
    USB_HID_GENERIC_DESKTOP_SYSTEM_SPEAKER_MUTE                      = 0xA7,
    USB_HID_GENERIC_DESKTOP_SYSTEM_HIBERNATE                         = 0xA8,

    USB_HID_GENERIC_DESKTOP_SYSTEM_DISPLAY_INVERT                    = 0xB0,
    USB_HID_GENERIC_DESKTOP_SYSTEM_DISPLAY_INTERNAL                  = 0xB1,
    USB_HID_GENERIC_DESKTOP_SYSTEM_DISPLAY_EXTERNAL                  = 0xB2,
    USB_HID_GENERIC_DESKTOP_SYSTEM_DISPLAY_BOTH                      = 0xB3,
    USB_HID_GENERIC_DESKTOP_SYSTEM_DISPLAY_DUAL                      = 0xB4,
    USB_HID_GENERIC_DESKTOP_SYSTEM_DISPLAY_TOGGLE_INT_EXT            = 0xB5,
    USB_HID_GENERIC_DESKTOP_SYSTEM_DISPLAY_SWAP_PRIMARY_SECONDARY    = 0xB6,
    USB_HID_GENERIC_DESKTOP_SYSTEM_DISPLAY_LCD_AUTOSCALE             = 0xB7
    /* Reserved                                                                 = 0xB6-0xFFFF*/

} USB_HID_GENERIC_DESKTOP;

typedef enum
{
    USB_HID_SIMULATION_CONTROLS_UNDEFINED                            = 0x00,
    USB_HID_SIMULATION_CONTROLS_FLIGHT_SIMULATION_DEVICE             = 0x01,
    USB_HID_SIMULATION_CONTROLS_AUTOMOBILE_SIMULATION_DEVICE         = 0x02,
    USB_HID_SIMULATION_CONTROLS_TANK_SIMULATION_DEVICE               = 0x03,
    USB_HID_SIMULATION_CONTROLS_SPACESHIP_SIMULATION_DEVICE          = 0x04,
    USB_HID_SIMULATION_CONTROLS_SUBMARINE_SIMULATION_DEVICE          = 0x05,
    USB_HID_SIMULATION_CONTROLS_SAILING_SIMULATION_DEVICE            = 0x06,
    USB_HID_SIMULATION_CONTROLS_MOTORCYCLE_SIMULATION_DEVICE         = 0x07,
    USB_HID_SIMULATION_CONTROLS_SPORTS_SIMLUATION_DEVICE             = 0x08,
    USB_HID_SIMULATION_CONTROLS_AIRPLANE_SIMULATION_DEVICE           = 0x09,
    USB_HID_SIMULATION_CONTROLS_HELICOPTER_SIMULATION_DEVICE         = 0x0A,
    USB_HID_SIMULATION_CONTROLS_MAGIC_CARPET_SIMULATION_DEVICE       = 0x0B,
    USB_HID_SIMULATION_CONTROLS_BICYCLE_SIMULATION_DEVICE            = 0x0C,
    /* Reserved                                                                 = 0x0D-0x1F */
    USB_HID_SIMULATION_CONTROLS_FLIGHT_CONTROL_STICK                 = 0x20,
    USB_HID_SIMULATION_CONTROLS_FLIGHT_STICK                         = 0x21,
    USB_HID_SIMULATION_CONTROLS_CYCLIC_CONTROL                       = 0x22,
    USB_HID_SIMULATION_CONTROLS_CYCLIC_TRIM                          = 0x23,
    USB_HID_SIMULATION_CONTROLS_FLIGHT_YOKE                          = 0x24,
    USB_HID_SIMULATION_CONTROLS_TRACK_CONTROL                        = 0x25,
    /* Reserved                                                                 = 0x26-0xAF */
    USB_HID_SIMULATION_CONTROLS_AILERON                              = 0xB0,
    USB_HID_SIMULATION_CONTROLS_AILERON_TRIM                         = 0xB1,
    USB_HID_SIMULATION_CONTROLS_ANTI_TORQUE_CONTROL                  = 0xB2,
    USB_HID_SIMULATION_CONTROLS_AUTOPILOT_ENABLE                     = 0xB3,
    USB_HID_SIMULATION_CONTROLS_CHAFF_RELEASE                        = 0xB4,
    USB_HID_SIMULATION_CONTROLS_COLLECTIVE_CONTROL                   = 0xB5,
    USB_HID_SIMULATION_CONTROLS_DIVE_BRAKE                           = 0xB6,
    USB_HID_SIMULATION_CONTROLS_ELECTRONIC_COUNTERMEASURES           = 0xB7,
    USB_HID_SIMULATION_CONTROLS_ELEVATOR                             = 0xB8,
    USB_HID_SIMULATION_CONTROLS_ELEVATOR_TRIM                        = 0xB9,
    USB_HID_SIMULATION_CONTROLS_RUDDER                               = 0xBA,
    USB_HID_SIMULATION_CONTROLS_THROTTLE                             = 0xBB,
    USB_HID_SIMULATION_CONTROLS_FLIGHT_COMMUNICATIONS                = 0xBC,
    USB_HID_SIMULATION_CONTROLS_FLARE_RELEASE                        = 0xBD,
    USB_HID_SIMULATION_CONTROLS_LANDING_GEAR                         = 0xBE,
    USB_HID_SIMULATION_CONTROLS_TOE_BRAKE                            = 0xBF,
    USB_HID_SIMULATION_CONTROLS_TRIGGER                              = 0xC0,
    USB_HID_SIMULATION_CONTROLS_WEAPONS_ARM                          = 0xC1,
    USB_HID_SIMULATION_CONTROLS_WEAPONS_SELECT                       = 0xC2,
    USB_HID_SIMULATION_CONTROLS_WING_FLAPS                           = 0xC3,
    USB_HID_SIMULATION_CONTROLS_ACCELERATOR                          = 0xC4,
    USB_HID_SIMULATION_CONTROLS_BRAKE                                = 0xC5,
    USB_HID_SIMULATION_CONTROLS_CLUTCH                               = 0xC6,
    USB_HID_SIMULATION_CONTROLS_SHIFTER                              = 0xC7,
    USB_HID_SIMULATION_CONTROLS_STEERING                             = 0xC8,
    USB_HID_SIMULATION_CONTROLS_TURRET_DIRECTION                     = 0xC9,
    USB_HID_SIMULATION_CONTROLS_BARREL_ELEVATION                     = 0xCA,
    USB_HID_SIMULATION_CONTROLS_DIVE_PLANE                           = 0xCB,
    USB_HID_SIMULATION_CONTROLS_BALLAST                              = 0xCC,
    USB_HID_SIMULATION_CONTROLS_BICYCLE_CRANK                        = 0xCD,
    USB_HID_SIMULATION_CONTROLS_HANDLE_BARS                          = 0xCE,
    USB_HID_SIMULATION_CONTROLS_FRONT_BRAKE                          = 0xCF,
    USB_HID_SIMULATION_CONTROLS_REAR_BRAKE                           = 0xD0
    /* Reserved                                                                 = 0xD1-0xFFFF*/
} USB_HID_SIMULATION_CONTROLS;


typedef enum
{
    USB_HID_VR_CONTROLS_UNIDENTIFIED                                 = 0x00,
    USB_HID_VR_CONTROLS_BELT                                         = 0x01,
    USB_HID_VR_CONTROLS_BODY_SUIT                                    = 0x02,
    USB_HID_VR_CONTROLS_FLEXOR                                       = 0x03,
    USB_HID_VR_CONTROLS_GLOVE                                        = 0x04,
    USB_HID_VR_CONTROLS_HEAD_TRACKER                                 = 0x05,
    USB_HID_VR_CONTROLS_HEAD_MOUNTED_DISPLAY                         = 0x06,
    USB_HID_VR_CONTROLS_HAND_TRACKER                                 = 0x07,
    USB_HID_VR_CONTROLS_OCULOMETER                                   = 0x08,
    USB_HID_VR_CONTROLS_VEST                                         = 0x09,
    USB_HID_VR_CONTROLS_ANIMATRONIC_DEVICE                           = 0x0A,
    /* Reserved                                                                 = 0x0B-0x1F */
    USB_HID_VR_CONTROLS_STEREO_ENABLE                                = 0x20,
    USB_HID_VR_CONTROLS_DISPLAY_ENABLE                               = 0x21
    /* Reserved                                                                 = 0x22-0xFFFF */
} USB_HID_VR_CONTROLS;

typedef enum
{
    USB_HID_SPORT_CONTROLS_UNIDENTIFIED                              = 0x00,
    USB_HID_SPORT_CONTROLS_BASEBALL_BAT                              = 0x01,
    USB_HID_SPORT_CONTROLS_GOLF_CLUB                                 = 0x02,
    USB_HID_SPORT_CONTROLS_ROWING_MACHINE                            = 0x03,
    USB_HID_SPORT_CONTROLS_TREADMILL                                 = 0x04,
    /* Reserved                                                                 = 0x05-0x2F */
    USB_HID_SPORT_CONTROLS_OAR                                       = 0x30,
    USB_HID_SPORT_CONTROLS_SLOPE                                     = 0x31,
    USB_HID_SPORT_CONTROLS_RATE                                      = 0x32,
    USB_HID_SPORT_CONTROLS_STICK_SPEED                               = 0x33,
    USB_HID_SPORT_CONTROLS_STICK_FACE_ANGLE                          = 0x34,
    USB_HID_SPORT_CONTROLS_STICK_HEEL_TOE                            = 0x35,
    USB_HID_SPORT_CONTROLS_STICK_FOLLOW_THROUGH                      = 0x36,
    USB_HID_SPORT_CONTROLS_STICK_TEMPO                               = 0x37,
    USB_HID_SPORT_CONTROLS_STICK_TYPE                                = 0x38,
    USB_HID_SPORT_CONTROLS_STICK_HEIGHT                              = 0x39,
    /* Reserved                                                                 = 0x3A-4F */
    USB_HID_SPORT_CONTROLS_PUTTER                                    = 0x50,
    USB_HID_SPORT_CONTROLS_1_IRON                                    = 0x51,
    USB_HID_SPORT_CONTROLS_2_IRON                                    = 0x52,
    USB_HID_SPORT_CONTROLS_3_IRON                                    = 0x53,
    USB_HID_SPORT_CONTROLS_4_IRON                                    = 0x54,
    USB_HID_SPORT_CONTROLS_5_IRON                                    = 0x55,
    USB_HID_SPORT_CONTROLS_6_IRON                                    = 0x56,
    USB_HID_SPORT_CONTROLS_7_IRON                                    = 0x57,
    USB_HID_SPORT_CONTROLS_8_IRON                                    = 0x58,
    USB_HID_SPORT_CONTROLS_9_IRON                                    = 0x59,
    USB_HID_SPORT_CONTROLS_10_IRON                                   = 0x5A,
    USB_HID_SPORT_CONTROLS_11_IRON                                   = 0x5B,
    USB_HID_SPORT_CONTROLS_SAND_WEDGE                                = 0x5C,
    USB_HID_SPORT_CONTROLS_LOFT_WEDGE                                = 0x5D,
    USB_HID_SPORT_CONTROLS_POWER_WEDGE                               = 0x5E,
    USB_HID_SPORT_CONTROLS_1_WOOD                                    = 0x5F,
    USB_HID_SPORT_CONTROLS_3_WOOD                                    = 0x60,
    USB_HID_SPORT_CONTROLS_5_WOOD                                    = 0x61,
    USB_HID_SPORT_CONTROLS_7_WOOD                                    = 0x62,
    USB_HID_SPORT_CONTROLS_9_WOOD                                    = 0x63
    /* Reserved                                                                 = 0x64-0xFFFF */
} USB_HID_SPORT_CONTROLS;

typedef enum
{
    USB_HID_GAME_CONTROLS_UNDEFINED                                  = 0x00,
    USB_HID_GAME_CONTROLS_3D_GAME_CONTROLLER                         = 0x01,
    USB_HID_GAME_CONTROLS_PINBALL_DEVICE                             = 0x02,
    USB_HID_GAME_CONTROLS_GUN_DEVICE                                 = 0x03,
    /* Reserved                                                                 = 0x04-0x1F */
    USB_HID_GAME_CONTROLS_POINT_OF_VIEW                              = 0x20,
    USB_HID_GAME_CONTROLS_TURN_RIGHT_LEFT                            = 0x21,
    USB_HID_GAME_CONTROLS_PITCH_FORWARD_BACKWARD                     = 0x22,
    USB_HID_GAME_CONTROLS_ROLL_RIGHT_LEFT                            = 0x23,
    USB_HID_GAME_CONTROLS_MOVE_RIGHT_LEFT                            = 0x24,
    USB_HID_GAME_CONTROLS_MOVE_FORWARD_BACKWARD                      = 0x25,
    USB_HID_GAME_CONTROLS_MOVE_UP_DOWN                               = 0x26,
    USB_HID_GAME_CONTROLS_LEAN_RIGHT_LEFT                            = 0x27,
    USB_HID_GAME_CONTROLS_LEAN_FORWARD_BACKWARD                      = 0x28,
    USB_HID_GAME_CONTROLS_HEIGHT_OF_POV                              = 0x29,
    USB_HID_GAME_CONTROLS_FLIPPER                                    = 0x2A,
    USB_HID_GAME_CONTROLS_SECONDARY_FLIPPER                          = 0x2B,
    USB_HID_GAME_CONTROLS_BUMP                                       = 0x2C,
    USB_HID_GAME_CONTROLS_NEW_GAME                                   = 0x2D,
    USB_HID_GAME_CONTROLS_SHOOT_BALL                                 = 0x2E,
    USB_HID_GAME_CONTROLS_PLAYER                                     = 0x2F,
    USB_HID_GAME_CONTROLS_GUN_BOLT                                   = 0x30,
    USB_HID_GAME_CONTROLS_GUN_CLIP                                   = 0x31,
    USB_HID_GAME_CONTROLS_GUN_SELECTOR                               = 0x32,
    USB_HID_GAME_CONTROLS_GUN_SINGLE_SHOT                            = 0x33,
    USB_HID_GAME_CONTROLS_GUN_BURST                                  = 0x34,
    USB_HID_GAME_CONTROLS_GUN_AUTOMATIC                              = 0x35,
    USB_HID_GAME_CONTROLS_GUN_SAFETY                                 = 0x36,
    USB_HID_GAME_CONTROLS_GAMEPAD_FIRE_JUMP                          = 0x37,
    /* Unspecified in spec                                                      = 0x38 */
    USB_HID_GAME_CONTROLS_GAMEPAD_TRIGGER                            = 0x39
    /* Reserved                                                                 = 0x3A-0xFFFF */
} USB_HID_GAME_CONTROLS;

typedef enum
{
    USB_HID_GENERIC_DEVICE_CONTROLS_UNIDENTIFIED                     = 0x00,
    /* Reserved                                                                 = 0x01-0x1F */
    USB_HID_GENERIC_DEVICE_CONTROLS_BATTERY_STRENGTH                 = 0x20,
    USB_HID_GENERIC_DEVICE_CONTROLS_WIRELESS_CHANNEL                 = 0x21,
    USB_HID_GENERIC_DEVICE_CONTROLS_WIRELESS_ID                      = 0x22,
    USB_HID_GENERIC_DEVICE_CONTROLS_DISCOVER_WIRELESS_CONTROL        = 0x23,
    USB_HID_GENERIC_DEVICE_CONTROLS_SECURITY_CODE_CHARACTER_ENTERED  = 0x24,
    USB_HID_GENERIC_DEVICE_CONTROLS_SECURITY_CODE_CHARACTER_ERASED   = 0x25,
    USB_HID_GENERIC_DEVICE_CONTROLS_SECURITY_CODE_CLEARED            = 0x26
    /* Reserved                                                                 = 0x27-0xFFFF */
} USB_HID_GENERIC_DEVICE_CONTROLS;

typedef enum
{
    USB_HID_KEYBOARD_KEYPAD_RESERVED_NO_EVENT_INDICATED                      = 0x00,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_ERROR_ROLL_OVER                         = 0x01,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_POST_FAIL                               = 0x02,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_ERROR_UNDEFINED                         = 0x03,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_A                                       = 0x04,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_B                                       = 0x05,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_C                                       = 0x06,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_D                                       = 0x07,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_E                                       = 0x08,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F                                       = 0x09,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_G                                       = 0x0A,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_H                                       = 0x0B,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_I                                       = 0x0C,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_J                                       = 0x0D,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_K                                       = 0x0E,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_L                                       = 0x0F,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_M                                       = 0x10,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_N                                       = 0x11,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_O                                       = 0x12,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_P                                       = 0x13,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_Q                                       = 0x14,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_R                                       = 0x15,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_S                                       = 0x16,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_T                                       = 0x17,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_U                                       = 0x18,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_V                                       = 0x19,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_W                                       = 0x1A,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_X                                       = 0x1B,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_Y                                       = 0x1C,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_Z                                       = 0x1D,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_1_AND_EXCLAMATION_POINT                 = 0x1E,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_2_AND_AT                                = 0x1F,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_3_AND_HASH                              = 0x20,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_4_AND_DOLLAR                            = 0x21,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_5_AND_PERCENT                           = 0x22,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_6_AND_CARROT                            = 0x23,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_7_AND_AMPERSAND                         = 0x24,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_8_AND_ASTERISK                          = 0x25,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_9_AND_OPEN_PARENTHESIS                  = 0x26,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_0_AND_CLOSE_PARENTHESIS                 = 0x27,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_RETURN_ENTER                            = 0x28,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_ESCAPE                                  = 0x29,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_DELETE                                  = 0x2A,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_TAB                                     = 0x2B,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_SPACEBAR                                = 0x2C,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_MINUS_AND_UNDERSCORE                    = 0x2D,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_EQUAL_AND_PLUS                          = 0x2E,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_OPEN_BRACKET_AND_OPEN_CURLY_BRACE       = 0x2F,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_CLOSE_BRACKET_AND_CLOSE_CURLY_BRACE     = 0x30,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_BACK_SLASH_AND_PIPE                     = 0x31,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_NON_US_HASH_AND_TILDE                   = 0x32,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_SEMICOLON_AND_COLON                     = 0x33,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_APOSTROPHE_AND_QUOTE                    = 0x34,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_GRAVE_ACCENT_AND_TILDE                  = 0x35,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_COMMA_AND_LESS_THAN                     = 0x36,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_PERIOD_AND_GREATER_THAN                 = 0x37,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_FORWARD_SLASH_AND_QUESTION_MARK         = 0x38,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_CAPS_LOCK                               = 0x39,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F1                                      = 0x3A,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F2                                      = 0x3B,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F3                                      = 0x3C,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F4                                      = 0x3D,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F5                                      = 0x3E,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F6                                      = 0x3F,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F7                                      = 0x40,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F8                                      = 0x41,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F9                                      = 0x42,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F10                                     = 0x43,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F11                                     = 0x44,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F12                                     = 0x45,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_PRINT_SCREEN                            = 0x46,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_SCROLL_LOCK                             = 0x47,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_PAUSE                                   = 0x48,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_INSERT                                  = 0x49,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_HOME                                    = 0x4A,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_PAGE_UP                                 = 0x4B,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_DELETE_FORWARD                          = 0x4C,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_END                                     = 0x4D,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_PAGE_DOWN                               = 0x4E,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_RIGHT_ARROW                             = 0x4F,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_LEFT_ARROW                              = 0x50,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_DOWN_ARROW                              = 0x51,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_UP_ARROW                                = 0x52,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_NUM_LOCK_AND_CLEAR                        = 0x53,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_BACK_SLASH                                = 0x54,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_ASTERISK                                  = 0x55,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_MINUS                                     = 0x56,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_PLUS                                      = 0x57,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_ENTER                                     = 0x58,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_1_AND_END                                 = 0x59,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_2_AND_DOWN_ARROW                          = 0x5A,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_3_AND_PAGE_DOWN                           = 0x5B,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_4_AND_LEFT_ARROW                          = 0x5C,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_5                                         = 0x5D,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_6_AND_RIGHT_ARROW                         = 0x5E,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_7_AND_HOME                                = 0x5F,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_8_AND_UP_ARROW                            = 0x60,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_9_AND_PAGE_UP                             = 0x61,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_0_AND_INSERT                              = 0x62,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_PERIOD_AND_DELETE                         = 0x63,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_NON_US_FORWARD_SLASH_AND_PIPE           = 0x64,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_APPLICATION                             = 0x65,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_POWER                                   = 0x66,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_EQUAL_SIZE                              = 0x67,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F13                                     = 0x68,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F14                                     = 0x69,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F15                                     = 0x6A,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F16                                     = 0x6B,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F17                                     = 0x6C,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F18                                     = 0x6D,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F19                                     = 0x6E,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F20                                     = 0x6F,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F21                                     = 0x70,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F22                                     = 0x71,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F23                                     = 0x72,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_F24                                     = 0x73,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_EXECUTE                                 = 0x74,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_HELP                                    = 0x75,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_MENU                                    = 0x76,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_SELECT                                  = 0x77,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_STOP                                    = 0x78,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_AGAIN                                   = 0x79,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_UNDO                                    = 0x7A,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_CUT                                     = 0x7B,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_COPY                                    = 0x7C,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_PASTE                                   = 0x7D,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_FIND                                    = 0x7E,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_MUTE                                    = 0x7F,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_VOLUME_UP                               = 0x80,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_VOLUME_DOWN                             = 0x81,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_LOCKING_CAPS_LOCK                       = 0x82,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_LOCKING_NUM_LOCK                        = 0x83,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_LOCKING_SCROLL_LOCK                     = 0x84,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_COMMA                                     = 0x85,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_EQUAL_SIGN                                = 0x86,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_INTERNATIONAL1                          = 0x87,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_INTERNATIONAL2                          = 0x88,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_INTERNATIONAL3                          = 0x89,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_INTERNATIONAL4                          = 0x8A,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_INTERNATIONAL5                          = 0x8B,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_INTERNATIONAL6                          = 0x8C,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_INTERNATIONAL7                          = 0x8D,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_INTERNATIONAL8                          = 0x8E,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_INTERNATIONAL9                          = 0x8F,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_LANG1                                   = 0x90,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_LANG2                                   = 0x91,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_LANG3                                   = 0x92,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_LANG4                                   = 0x93,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_LANG5                                   = 0x94,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_LANG6                                   = 0x95,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_LANG7                                   = 0x96,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_LANG8                                   = 0x97,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_LANG9                                   = 0x98,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_ALTERNATE_ERASE                         = 0x99,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_SYS_REQ_ATTENTION                       = 0x9A,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_CANCEL                                  = 0x9B,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_CLEAR                                   = 0x9C,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_PRIOR                                   = 0x9D,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_RETURN                                  = 0x9E,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_SEPARATOR                               = 0x9F,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_OUT                                     = 0xA0,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_OPER                                    = 0xA1,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_CLEAR_AGAIN                             = 0xA2,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_CR_SEL_PROPS                            = 0xA3,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_EX_SEL                                  = 0xA4,
    /* Reserved                                                                         = 0xA5-0xAF */
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_00                                        = 0xB0,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_000                                       = 0xB1,
    USB_HID_KEYBOARD_KEYPAD_THOUSANDS_SEPARATOR                              = 0xB2,
    USB_HID_KEYBOARD_KEYPAD_DECIMAL_SEPARATOR                                = 0xB3,
    USB_HID_KEYBOARD_KEYPAD_CURRENCY_UNIT                                    = 0xB4,
    USB_HID_KEYBOARD_KEYPAD_CURRENTY_SUB_UNIT                                = 0xB5,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_OPEN_PARENTHESIS                          = 0xB6,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_CLOSE_PARENTHESIS                         = 0xB7,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_OPEN_CURLY_BRACE                          = 0xB8,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_CLOSE_CURLY_BRACE                         = 0xB9,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_TAB                                       = 0xBA,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_BACKSPACE                                 = 0xBB,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_A                                         = 0xBC,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_B                                         = 0xBD,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_C                                         = 0xBE,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_D                                         = 0xBF,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_E                                         = 0xC0,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_F                                         = 0xC1,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_XOR                                       = 0xC2,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_CARROT                                    = 0xC3,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_PERCENT_SIGN                              = 0xC4,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_LESS_THAN                                 = 0xC5,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_GREATER_THAN                              = 0xC6,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_AMPERSAND                                 = 0xC7,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_DOUBLE_AMPERSAND                          = 0xC8,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_PIPE                                      = 0xC9,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_DOUBLE_PIPE                               = 0xCA,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_COLON                                     = 0xCB,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_HASH                                      = 0xCC,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_SPACE                                     = 0xCD,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_AT                                        = 0xCE,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_EXCLAMATION_POINT                         = 0xCF,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_MEMORY_STORE                              = 0xD0,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_MEMORY_RECALL                             = 0xD1,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_MEMORY_CLEAR                              = 0xD2,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_MEMORY_ADD                                = 0xD3,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_MEMORY_SUBTRACT                           = 0xD4,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_MEMORY_MULTIPLY                           = 0xD5,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_MEMORY_DIVIDE                             = 0xD6,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_PLUS_MINUS                                = 0xD7,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_CLEAR                                     = 0xD8,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_CLEAR_ENTRY                               = 0xD9,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_BINARY                                    = 0xDA,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_OCTAL                                     = 0xDB,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_DECIMAL                                   = 0xDC,
    USB_HID_KEYBOARD_KEYPAD_KEYPAD_HEXADECIMAL                               = 0xDD,
    /* Reserved                                                                         = 0xDE-0xDF */
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_LEFT_CONTROL                            = 0xE0,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_LEFT_SHIFT                              = 0xE1,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_LEFT_ALT                                = 0xE2,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_LEFT_GUI                                = 0xE3,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_RIGHT_CONTROL                           = 0xE4,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_RIGHT_SHIFT                             = 0xE5,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_RIGHT_ALT                               = 0xE6,
    USB_HID_KEYBOARD_KEYPAD_KEYBOARD_RIGHT_GUI                               = 0xE7
    //0xE8-0xFFFF reserved
} USB_HID_KEYBOARD_KEYPAD;

typedef enum
{
    USB_HID_LED_UNDEFINED                                            = 0x00,
    USB_HID_LED_NUM_LOCK                                             = 0x01,
    USB_HID_LED_CAPS_LOCK                                            = 0x02,
    USB_HID_LED_SCROLL_LOCK                                          = 0x03,
    USB_HID_LED_COMPOSE                                              = 0x04,
    USB_HID_LED_KANA                                                 = 0x05,
    USB_HID_LED_POWER                                                = 0x06,
    USB_HID_LED_SHIFT                                                = 0x07,
    USB_HID_LED_DO_NOT_DISTURB                                       = 0x08,
    USB_HID_LED_MUTE                                                 = 0x09,
    USB_HID_LED_TONE_ENABLE                                          = 0x0A,
    USB_HID_LED_HIGH_CUT_FILTER                                      = 0x0B,
    USB_HID_LED_LOW_CUT_FILTER                                       = 0x0C,
    USB_HID_LED_EQUALIZER_ENABLE                                     = 0x0D,
    USB_HID_LED_SOUND_FIELD_ON                                       = 0x0E,
    USB_HID_LED_SURROUND_ON                                          = 0x0F,
    USB_HID_LED_REPEAT                                               = 0x10,
    USB_HID_LED_STEREO                                               = 0x11,
    USB_HID_LED_SAMPLING_RATE_DETECT                                 = 0x12,
    USB_HID_LED_SPINNING                                             = 0x13,
    USB_HID_LED_CAV                                                  = 0x14,
    USB_HID_LED_CLV                                                  = 0x15,
    USB_HID_LED_RECORDING_FORMAT_DETECT                              = 0x16,
    USB_HID_LED_OFF_HOOK                                             = 0x17,
    USB_HID_LED_RING                                                 = 0x18,
    USB_HID_LED_MESSAGE_WAITING                                      = 0x19,
    USB_HID_LED_DATA_MODE                                            = 0x1A,
    USB_HID_LED_BATTERY_OPERATION                                    = 0x1B,
    USB_HID_LED_BATTERY_OK                                           = 0x1C,
    USB_HID_LED_BATTERY_LOW                                          = 0x1D,
    USB_HID_LED_SPEAKER                                              = 0x1E,
    USB_HID_LED_HEAD_SET                                             = 0x1F,
    USB_HID_LED_HOLD                                                 = 0x20,
    USB_HID_LED_MICROPHONE                                           = 0x21,
    USB_HID_LED_COVERAGE                                             = 0x22,
    USB_HID_LED_NIGHT_MODE                                           = 0x23,
    USB_HID_LED_SEND_CALLS                                           = 0x24,
    USB_HID_LED_CALL_PICKUP                                          = 0x25,
    USB_HID_LED_CONFERENCE                                           = 0x26,
    USB_HID_LED_STANDBY                                              = 0x27,
    USB_HID_LED_CAMERA_ON                                            = 0x28,
    USB_HID_LED_CAMERA_OFF                                           = 0x29,
    USB_HID_LED_ON_LINE                                              = 0x2A,
    USB_HID_LED_OFF_LINE                                             = 0x2B,
    USB_HID_LED_BUSY                                                 = 0x2C,
    USB_HID_LED_READY                                                = 0x2D,
    USB_HID_LED_PAPER_OUT                                            = 0x2E,
    USB_HID_LED_PAPER_JAM                                            = 0x2F,
    USB_HID_LED_REMOTE                                               = 0x30,
    USB_HID_LED_FORWARD                                              = 0x31,
    USB_HID_LED_REVERSE                                              = 0x32,
    USB_HID_LED_STOP                                                 = 0x33,
    USB_HID_LED_REWIND                                               = 0x34,
    USB_HID_LED_FAST_FORWARD                                         = 0x35,
    USB_HID_LED_PLAY                                                 = 0x36,
    USB_HID_LED_PAUSE                                                = 0x37,
    USB_HID_LED_RECORD                                               = 0x38,
    USB_HID_LED_ERROR                                                = 0x39,
    USB_HID_LED_USAGE_SELECTED_INDICATOR                             = 0x3A,
    USB_HID_LED_USAGE_IN_USE_INDICATOR                               = 0x3B,
    USB_HID_LED_USAGE_MULTI_MODE_INDICATOR                           = 0x3C,
    USB_HID_LED_INDICATOR_ON                                         = 0x3D,
    USB_HID_LED_INDICATOR_FLASH                                      = 0x3E,
    USB_HID_LED_INDICATOR_SLOW_BLINK                                 = 0x3F,
    USB_HID_LED_INDICATOR_FAST_BLINK                                 = 0x40,
    USB_HID_LED_INDICATOR_OFF                                        = 0x41,
    USB_HID_LED_FLASH_ON_TIME                                        = 0x42,
    USB_HID_LED_SLOW_BLINK_ON_TIME                                   = 0x43,
    USB_HID_LED_SLOW_BLINK_OFF_TIME                                  = 0x44,
    USB_HID_LED_FAST_BLINK_ON_TIME                                   = 0x45,
    USB_HID_LED_FAST_BLINK_OFF_TIME                                  = 0x46,
    USB_HID_LED_USAGE_INDICATOR_COLOR                                = 0x47,
    USB_HID_LED_INDICATOR_RED                                        = 0x48,
    USB_HID_LED_INDICATOR_GREEN                                      = 0x49,
    USB_HID_LED_INDICATOR_AMBER                                      = 0x4A,
    USB_HID_LED_GENERIC_INDICATOR                                    = 0x4B,
    USB_HID_LED_SYSTEM_SUSPEND                                       = 0x4C,
    USB_HID_LED_EXTERNAL_POWER_CONNECTED                             = 0x4D
    /* Reserved                                                                 = 0x4E-0xFFFF */
} USB_HID_LED;

typedef enum
{
    USB_HID_TELEPHONY_DEVICE_UNASSIGNED                              = 0x00,
    USB_HID_TELEPHONY_DEVICE_PHONE                                   = 0x01,
    USB_HID_TELEPHONY_DEVICE_ANSWERING_MACHINE                       = 0x02,
    USB_HID_TELEPHONY_DEVICE_MESSAGE_CONTROLS                        = 0x03,
    USB_HID_TELEPHONY_DEVICE_HANDSET                                 = 0x04,
    USB_HID_TELEPHONY_DEVICE_HEADSET                                 = 0x05,
    USB_HID_TELEPHONY_DEVICE_TELEPHONY_KEY_PAD                       = 0x06,
    USB_HID_TELEPHONY_DEVICE_PROGRAMMABLE_BUTTON                     = 0x07,
    /* Reserved                                                                 = 0x08-0x1F */
    USB_HID_TELEPHONY_DEVICE_HOOK_SWITCH                             = 0x20,
    USB_HID_TELEPHONY_DEVICE_FLASH                                   = 0x21,
    USB_HID_TELEPHONY_DEVICE_FEATURE                                 = 0x22,
    USB_HID_TELEPHONY_DEVICE_HOLD                                    = 0x23,
    USB_HID_TELEPHONY_DEVICE_REDIAL                                  = 0x24,
    USB_HID_TELEPHONY_DEVICE_TRANSFER                                = 0x25,
    USB_HID_TELEPHONY_DEVICE_DROP                                    = 0x26,
    USB_HID_TELEPHONY_DEVICE_PARK                                    = 0x27,
    USB_HID_TELEPHONY_DEVICE_FORWARD_CALLS                           = 0x28,
    USB_HID_TELEPHONY_DEVICE_ALTERNATE_FUNCTION                      = 0x29,
    USB_HID_TELEPHONY_DEVICE_LINE                                    = 0x2A,
    USB_HID_TELEPHONY_DEVICE_SPEAKER_PHONE                           = 0x2B,
    USB_HID_TELEPHONY_DEVICE_CONFERENCE                              = 0x2C,
    USB_HID_TELEPHONY_DEVICE_RING_ENABLE                             = 0x2D,
    USB_HID_TELEPHONY_DEVICE_RING_SELECT                             = 0x2E,
    USB_HID_TELEPHONY_DEVICE_PHONE_MUTE                              = 0x2F,
    USB_HID_TELEPHONY_DEVICE_CALLER_ID                               = 0x30,
    USB_HID_TELEPHONY_DEVICE_SEND                                    = 0x31,
    /* Reserved                                                                 = 0x32-0x4F */
    USB_HID_TELEPHONY_DEVICE_SPEED_DIAL                              = 0x50,
    USB_HID_TELEPHONY_DEVICE_STORE_NUMBER                            = 0x51,
    USB_HID_TELEPHONY_DEVICE_RECALL_NUMBER                           = 0x52,
    USB_HID_TELEPHONY_DEVICE_PHONE_DIRECTORY                         = 0x53,
    /* Reserved                                                                 = 0x54-0x6F */
    USB_HID_TELEPHONY_DEVICE_VOICE_MAIL                              = 0x70,
    USB_HID_TELEPHONY_DEVICE_SCREEN_CALLS                            = 0x71,
    USB_HID_TELEPHONY_DEVICE_DO_NOT_DISTURB                          = 0x72,
    USB_HID_TELEPHONY_DEVICE_MESSAGE                                 = 0x73,
    USB_HID_TELEPHONY_DEVICE_ANSWER_ON_OFF                           = 0x74,
    /* Reserved                                                                 = 0x75-0x8F */
    USB_HID_TELEPHONY_DEVICE_INSIDE_DIAL_TONE                        = 0x90,
    USB_HID_TELEPHONY_DEVICE_OUTSIDE_DIAL_TONE                       = 0x91,
    USB_HID_TELEPHONY_DEVICE_INSIDE_RING_TONE                        = 0x92,
    USB_HID_TELEPHONY_DEVICE_OUTSIDE_RING_TONE                       = 0x93,
    USB_HID_TELEPHONY_DEVICE_PRIORITY_RING_TONE                      = 0x94,
    USB_HID_TELEPHONY_DEVICE_INSIDE_RINGBACK                         = 0x95,
    USB_HID_TELEPHONY_DEVICE_PRIORITY_RINGBACK                       = 0x96,
    USB_HID_TELEPHONY_DEVICE_LINE_BUSY_TONE                          = 0x97,
    USB_HID_TELEPHONY_DEVICE_REORDER_TONE                            = 0x98,
    USB_HID_TELEPHONY_DEVICE_CALL_WAITING_TONE                       = 0x99,
    USB_HID_TELEPHONY_DEVICE_CONFIRMATION_TONE_1                     = 0x9A,
    USB_HID_TELEPHONY_DEVICE_CONFIRMATION_TONE_2                     = 0x9B,
    USB_HID_TELEPHONY_DEVICE_TONES_OFF                               = 0x9C,
    USB_HID_TELEPHONY_DEVICE_OUTSIDE_RINGBACK                        = 0x9D,
    USB_HID_TELEPHONY_DEVICE_RINGER                                  = 0x9E,
    /* Reserved                                                                 = 0x9F-0xAF */
    USB_HID_TELEPHONY_DEVICE_PHONE_KEY_0                             = 0xB0,
    USB_HID_TELEPHONY_DEVICE_PHONE_KEY_1                             = 0xB1,
    USB_HID_TELEPHONY_DEVICE_PHONE_KEY_2                             = 0xB2,
    USB_HID_TELEPHONY_DEVICE_PHONE_KEY_3                             = 0xB3,
    USB_HID_TELEPHONY_DEVICE_PHONE_KEY_4                             = 0xB4,
    USB_HID_TELEPHONY_DEVICE_PHONE_KEY_5                             = 0xB5,
    USB_HID_TELEPHONY_DEVICE_PHONE_KEY_6                             = 0xB6,
    USB_HID_TELEPHONY_DEVICE_PHONE_KEY_7                             = 0xB7,
    USB_HID_TELEPHONY_DEVICE_PHONE_KEY_8                             = 0xB8,
    USB_HID_TELEPHONY_DEVICE_PHONE_KEY_9                             = 0xB9,
    USB_HID_TELEPHONY_DEVICE_PHONE_KEY_STAR                          = 0xBA,
    USB_HID_TELEPHONY_DEVICE_PHONE_KEY_POUND                         = 0xBB,
    USB_HID_TELEPHONY_DEVICE_PHONE_KEY_A                             = 0xBC,
    USB_HID_TELEPHONY_DEVICE_PHONE_KEY_B                             = 0xBD,
    USB_HID_TELEPHONY_DEVICE_PHONE_KEY_C                             = 0xBE,
    USB_HID_TELEPHONY_DEVICE_PHONE_KEY_D                             = 0xBF
    /* Reserved                                                                 = 0xC0-0xFFFF */
} USB_HID_TELEPHONY_DEVICE;

typedef enum
{
    USB_HID_CONSUMER_UNASSIGNED                                         = 0x00,
    USB_HID_CONSUMER_CONSUMER_CONTROL                                   = 0x01,
    USB_HID_CONSUMER_NUMERIC_KEY_PAD                                    = 0x02,
    USB_HID_CONSUMER_PROGRAMMABLE_BUTTONS                               = 0x03,
    USB_HID_CONSUMER_MICROPHONE                                         = 0x04,
    USB_HID_CONSUMER_HEADPHONE                                          = 0x05,
    USB_HID_CONSUMER_GRAPHIC_EQUALIZER                                  = 0x06,
    /* Reserved                                                         = 0x07-0x1F */
    USB_HID_CONSUMER_PLUS_10                                            = 0x20,
    USB_HID_CONSUMER_PLUS_100                                           = 0x21,
    USB_HID_CONSUMER_AM_PM                                              = 0x22,
    /* Reserved                                                         = 0x23-0x2F */
    USB_HID_CONSUMER_POWER                                              = 0x30,
    USB_HID_CONSUMER_RESET                                              = 0x31,
    USB_HID_CONSUMER_SLEEP                                              = 0x32,
    USB_HID_CONSUMER_SLEEP_AFTER                                        = 0x33,
    USB_HID_CONSUMER_SLEEP_MODE                                         = 0x34,
    USB_HID_CONSUMER_ILLUMINATION                                       = 0x35,
    USB_HID_CONSUMER_FUNCTION_BUTTONS                                   = 0x36,
    /* Reserved                                                         = 0x37-0x3F */
    USB_HID_CONSUMER_MENU                                               = 0x40,
    USB_HID_CONSUMER_MENU_PICK                                          = 0x41,
    USB_HID_CONSUMER_MENU_UP                                            = 0x42,
    USB_HID_CONSUMER_MENU_DOWN                                          = 0x43,
    USB_HID_CONSUMER_MENU_LEFT                                          = 0x44,
    USB_HID_CONSUMER_MENU_RIGHT                                         = 0x45,
    USB_HID_CONSUMER_MENU_ESCAPE                                        = 0x46,
    USB_HID_CONSUMER_MENU_VALUE_INCREASE                                = 0x47,
    USB_HID_CONSUMER_MENU_VALUE_DECREASE                                = 0x48,
    /* Reserved                                                         = 0x49-0x5F */
    USB_HID_CONSUMER_DATA_ON_SCREEN                                     = 0x60,
    USB_HID_CONSUMER_CLOSED_CAPTION                                     = 0x61,
    USB_HID_CONSUMER_CLOSED_CAPTION_SELECT                              = 0x62,
    USB_HID_CONSUMER_VCR_TV                                             = 0x63,
    USB_HID_CONSUMER_BROADCAST_MODE                                     = 0x64,
    USB_HID_CONSUMER_SNAPSHOT                                           = 0x65,
    USB_HID_CONSUMER_STILL                                              = 0x66,
    /* Reserved                                                         = 0x67-0x7F */
    USB_HID_CONSUMER_SELECTION                                          = 0x80,
    USB_HID_CONSUMER_ASSIGN_SELECTION                                   = 0x81,
    USB_HID_CONSUMER_MODE_STEP                                          = 0x82,
    USB_HID_CONSUMER_RECALL_LAST                                        = 0x83,
    USB_HID_CONSUMER_ENTER_CHANNEL                                      = 0x84,
    USB_HID_CONSUMER_ORDER_MOVIE                                        = 0x85,
    USB_HID_CONSUMER_CHANNEL                                            = 0x86,
    USB_HID_CONSUMER_MEDIA_SELECTION                                    = 0x87,
    USB_HID_CONSUMER_MEDIA_SELECT_COMPUTER                              = 0x88,
    USB_HID_CONSUMER_MEDIA_SELECT_TV                                    = 0x89,
    USB_HID_CONSUMER_MEDIA_SELECT_WWW                                   = 0x8A,
    USB_HID_CONSUMER_MEDIA_SELECT_DVD                                   = 0x8B,
    USB_HID_CONSUMER_MEDIA_SELECT_TELEPHONE                             = 0x8C,
    USB_HID_CONSUMER_MEDIA_SELECT_PROGRAM_GUIDE                         = 0x8D,
    USB_HID_CONSUMER_MEDIA_SELECT_VIDEO_PHONE                           = 0x8E,
    USB_HID_CONSUMER_MEDIA_SELECT_GAMES                                 = 0x8F,
    USB_HID_CONSUMER_MEDIA_SELECT_MESSAGES                              = 0x90,
    USB_HID_CONSUMER_MEDIA_SELECT_CD                                    = 0x91,
    USB_HID_CONSUMER_MEDIA_SELECT_VCR                                   = 0x92,
    USB_HID_CONSUMER_MEDIA_SELECT_TUNER                                 = 0x93,
    USB_HID_CONSUMER_QUIT                                               = 0x94,
    USB_HID_CONSUMER_HELP                                               = 0x95,
    USB_HID_CONSUMER_MEDIA_SELECT_TAPE                                  = 0x96,
    USB_HID_CONSUMER_MEDIA_SELECT_CABLE                                 = 0x97,
    USB_HID_CONSUMER_MEDIA_SELECT_SATELLITE                             = 0x98,
    USB_HID_CONSUMER_MEDIA_SELECT_SECURITY                              = 0x99,
    USB_HID_CONSUMER_MEDIA_SELECT_HOME                                  = 0x9A,
    USB_HID_CONSUMER_MEDIA_SELECT_CALL                                  = 0x9B,
    USB_HID_CONSUMER_CHANNEL_INCREMENT                                  = 0x9C,
    USB_HID_CONSUMER_CHANNEL_DECREMENT                                  = 0x9D,
    USB_HID_CONSUMER_MEDIA_SELECT_SAP                                   = 0x9E,
    /* Reserved                                                         = 0x9F */
    USB_HID_CONSUMER_VCR_PLUS                                           = 0xA0,
    USB_HID_CONSUMER_ONCE                                               = 0xA1,
    USB_HID_CONSUMER_DAILY                                              = 0xA2,
    USB_HID_CONSUMER_WEEKLY                                             = 0xA3,
    USB_HID_CONSUMER_MONTHLY                                            = 0xA4,
    /* Reserved                                                         = 0xA5-0xAF */
    USB_HID_CONSUMER_PLAY                                               = 0xB0,
    USB_HID_CONSUMER_PAUSE                                              = 0xB1,
    USB_HID_CONSUMER_RECORD                                             = 0xB2,
    USB_HID_CONSUMER_FAST_FORWARD                                       = 0xB3,
    USB_HID_CONSUMER_REWIND                                             = 0xB4,
    USB_HID_CONSUMER_SCAN_NEXT_TRACK                                    = 0xB5,
    USB_HID_CONSUMER_SCAN_PREVIOUS_TRACK                                = 0xB6,
    USB_HID_CONSUMER_STOP                                               = 0xB7,
    USB_HID_CONSUMER_EJECT                                              = 0xB8,
    USB_HID_CONSUMER_RANDOM_PLAY                                        = 0xB9,
    USB_HID_CONSUMER_SELECT_DISC                                        = 0xBA,
    USB_HID_CONSUMER_ENTER_DISC                                         = 0xBB,
    USB_HID_CONSUMER_REPEAT                                             = 0xBC,
    USB_HID_CONSUMER_TRACKING                                           = 0xBD,
    USB_HID_CONSUMER_TRACK_NORMAL                                       = 0xBE,
    USB_HID_CONSUMER_SLOW_TRACKING                                      = 0xBF,
    USB_HID_CONSUMER_FRAME_FORWARD                                      = 0xC0,
    USB_HID_CONSUMER_FRAME_BACK                                         = 0xC1,
    USB_HID_CONSUMER_MARK                                               = 0xC2,
    USB_HID_CONSUMER_CLEAR_MARK                                         = 0xC3,
    USB_HID_CONSUMER_REPEAT_FROM_MARK                                   = 0xC4,
    USB_HID_CONSUMER_RETURN_TO_MARK                                     = 0xC5,
    USB_HID_CONSUMER_SEARCH_MARK_FORWARD                                = 0xC6,
    USB_HID_CONSUMER_SEARCH_MARK_BACKWARDS                              = 0xC7,
    USB_HID_CONSUMER_COUNTER_RESET                                      = 0xC8,
    USB_HID_CONSUMER_SHOW_COUNTER                                       = 0xC9,
    USB_HID_CONSUMER_TRACKING_INCREMENT                                 = 0xCA,
    USB_HID_CONSUMER_TRACKING_DECREMENT                                 = 0xCB,
    USB_HID_CONSUMER_STOP_EJECT                                         = 0xCC,
    USB_HID_CONSUMER_PLAY_PAUSE                                         = 0xCD,
    USB_HID_CONSUMER_PLAY_SKIP                                          = 0xCE,
    /* Reserved                                                         = 0xCF-0xDF */
    USB_HID_CONSUMER_VOLUME                                             = 0xE0,
    USB_HID_CONSUMER_BALANCE                                            = 0xE1,
    USB_HID_CONSUMER_MUTE                                               = 0xE2,
    USB_HID_CONSUMER_BASS                                               = 0xE3,
    USB_HID_CONSUMER_TREBLE                                             = 0xE4,
    USB_HID_CONSUMER_BASS_BOOST                                         = 0xE5,
    USB_HID_CONSUMER_SURROUND_MODE                                      = 0xE6,
    USB_HID_CONSUMER_LOUDNESS                                           = 0xE7,
    USB_HID_CONSUMER_MPX                                                = 0xE8,
    USB_HID_CONSUMER_VOLUME_INCREMENT                                   = 0xE9,
    USB_HID_CONSUMER_VOLUME_DECREMENT                                   = 0xEA,
    /* Reserved                                                         = 0xEB-0xEF */
    USB_HID_CONSUMER_SPEED_SELECT                                       = 0xF0,
    USB_HID_CONSUMER_PLAYBACK_SPEED                                     = 0xF1,
    USB_HID_CONSUMER_STANDARD_PLAY                                      = 0xF2,
    USB_HID_CONSUMER_LONG_PLAY                                          = 0xF3,
    USB_HID_CONSUMER_EXTENDED_PLAY                                      = 0xF4,
    USB_HID_CONSUMER_SLOW                                               = 0xF5,
    /* Reserved                                                         = 0xF6-0xFF */
    USB_HID_CONSUMER_FAN_ENABLE                                         = 0x100,
    USB_HID_CONSUMER_FAN_SPEED                                          = 0x101,
    USB_HID_CONSUMER_LIGHT_ENABLE                                       = 0x102,
    USB_HID_CONSUMER_LIGHT_ILLUMINATION_LEVEL                           = 0x103,
    USB_HID_CONSUMER_CLIMATE_CONTROL_ENABLE                             = 0x104,
    USB_HID_CONSUMER_ROOM_TEMPERATURE                                   = 0x105,
    USB_HID_CONSUMER_SECURITY_ENABLE                                    = 0x106,
    USB_HID_CONSUMER_FIRE_ALARM                                         = 0x107,
    USB_HID_CONSUMER_POLICE_ALARM                                       = 0x108,
    USB_HID_CONSUMER_PROXIMITY                                          = 0x109,
    USB_HID_CONSUMER_MOTION                                             = 0x10A,
    USB_HID_CONSUMER_DURESS_ALARM                                       = 0x10B,
    USB_HID_CONSUMER_HOLDUP_ALARM                                       = 0x10C,
    USB_HID_CONSUMER_MEDICAL_ALARM                                      = 0x10D,
    /* Reserved                                                         = 0x10E-0x14F */
    USB_HID_CONSUMER_BALANCE_RIGHT                                      = 0x150,
    USB_HID_CONSUMER_BALANCE_LEFT                                       = 0x151,
    USB_HID_CONSUMER_BASS_INCREMENT                                     = 0x152,
    USB_HID_CONSUMER_BASS_DECREMENT                                     = 0x153,
    USB_HID_CONSUMER_TREBLE_INCREMENT                                   = 0x154,
    USB_HID_CONSUMER_TREBLE_DECREMENT                                   = 0x155,
    /* Reserved                                                         = 0x156-0x15F */
    USB_HID_CONSUMER_SPEAKER_SYSTEM                                     = 0x160,
    USB_HID_CONSUMER_CHANNEL_LEFT                                       = 0x161,
    USB_HID_CONSUMER_CHANNEL_RIGHT                                      = 0x162,
    USB_HID_CONSUMER_CHANNEL_CENTER                                     = 0x163,
    USB_HID_CONSUMER_CHANNEL_FRONT                                      = 0x164,
    USB_HID_CONSUMER_CHANNEL_CENTER_FRONT                               = 0x165,
    USB_HID_CONSUMER_CHANNEL_SIDE                                       = 0x166,
    USB_HID_CONSUMER_CHANNEL_SURROUND                                   = 0x167,
    USB_HID_CONSUMER_CHANNEL_LOW_FREQUENCY_ENHANCEMENT                  = 0x168,
    USB_HID_CONSUMER_CHANNEL_TOP                                        = 0x169,
    USB_HID_CONSUMER_CHANNEL_UNKNOWN                                    = 0x16A,
    /* Reserved                                                         = 0x16B-0x16F */
    USB_HID_CONSUMER_SUBCHANNEL                                         = 0x170,
    USB_HID_CONSUMER_SUBCHANNEL_INCREMENT                               = 0x171,
    USB_HID_CONSUMER_SUBCHANNEL_DECREMENT                               = 0x172,
    USB_HID_CONSUMER_ALTERNATE_AUDIO_INCREMENT                          = 0x173,
    USB_HID_CONSUMER_ALTERNATE_AUDIO_DECREMENT                          = 0x174,
    /* Reserved                                                         = 0x175-0x17F */
    USB_HID_CONSUMER_APPLICATION_LAUNCH_BUTTONS                         = 0x180,
    USB_HID_CONSUMER_AL_LAUNCH_BUTTON_CONFIGURATION_TOOL                = 0x181,
    USB_HID_CONSUMER_AL_PROGRAMMABLE_BUTTON_CONFIGURATION               = 0x182,
    USB_HID_CONSUMER_AL_CONSUMER_CONTROL_CONFIGURATION                  = 0x183,
    USB_HID_CONSUMER_AL_WORD_PROCESSOR                                  = 0x184,
    USB_HID_CONSUMER_AL_TEXT_EDITOR                                     = 0x185,
    USB_HID_CONSUMER_AL_SPREADSHEET                                     = 0x186,
    USB_HID_CONSUMER_AL_GRAPHICS_EDITOR                                 = 0x187,
    USB_HID_CONSUMER_AL_PRESENTATION_APP                                = 0x188,
    USB_HID_CONSUMER_AL_DATABASE_APP                                    = 0x189,
    USB_HID_CONSUMER_AL_EMAIL_READER                                    = 0x18A,
    USB_HID_CONSUMER_AL_NEWSREADER                                      = 0x18B,
    USB_HID_CONSUMER_AL_VOICEMAIL                                       = 0x18C,
    USB_HID_CONSUMER_AL_CONTACT_ADDRESS_BOOK                            = 0x18D,
    USB_HID_CONSUMER_AL_CALENDAR_SCHEDULE                               = 0x18E,
    USB_HID_CONSUMER_AL_TASK_PROJECT_MANAGER                            = 0x18F,
    USB_HID_CONSUMER_AL_LOG_JOURNAL_TIMECARD                            = 0x190,
    USB_HID_CONSUMER_AL_CHECKBOOK_FINANCE                               = 0x191,
    USB_HID_CONSUMER_AL_CALCULATOR                                      = 0x192,
    USB_HID_CONSUMER_AL_AV_CAPTURE_PLAYBACK                             = 0x193,
    USB_HID_CONSUMER_AL_LOCAL_MACHINE_BROWSER                           = 0x194,
    USB_HID_CONSUMER_AL_LAN_WAN_BROWSER                                 = 0x195,
    USB_HID_CONSUMER_AL_INTERNET_BROWSER                                = 0x196,
    USB_HID_CONSUMER_AL_REMOTE_NETOWORKING_ISP_CONNECT                  = 0x197,
    USB_HID_CONSUMER_AL_NETWORK_CONFERENCE                              = 0x198,
    USB_HID_CONSUMER_AL_NETOWRK_CHAT                                    = 0x199,
    USB_HID_CONSUMER_AL_TELEPHONY_DIALER                                = 0x19A,
    USB_HID_CONSUMER_AL_LOGON                                           = 0x19B,
    USB_HID_CONSUMER_AL_LOGOFF                                          = 0x19C,
    USB_HID_CONSUMER_AL_LOGON_LOGOFF                                    = 0x19D,
    USB_HID_CONSUMER_AL_TERMINAL_LOCK_SCREENSAVER                       = 0x19E,
    USB_HID_CONSUMER_AL_CONTROL_PANEL                                   = 0x19F,
    USB_HID_CONSUMER_AL_COMMAND_LINE_PROCESSOR_RUN                      = 0x1A0,
    USB_HID_CONSUMER_AL_PROCESS_TASK_MANAGER                            = 0x1A1,
    USB_HID_CONSUMER_AL_SELECT_TASK_APPLICATION                         = 0x1A2,
    USB_HID_CONSUMER_AL_NEXT_TASK_APPLICATION                           = 0x1A3,
    USB_HID_CONSUMER_AL_PREVIOUS_TASK_APPLICATION                       = 0x1A4,
    USB_HID_CONSUMER_AL_PREEMPTIVE_HALT_TASK_APPLICATION                = 0x1A5,
    USB_HID_CONSUMER_AL_INTEGRATED_HELP_CENTER                          = 0x1A6,
    USB_HID_CONSUMER_AL_DOCUMENTS                                       = 0x1A7,
    USB_HID_CONSUMER_AL_THESAURUS                                       = 0x1A8,
    USB_HID_CONSUMER_AL_DICTIONARY                                      = 0x1A9,
    USB_HID_CONSUMER_AL_DESKTOP                                         = 0x1AA,
    USB_HID_CONSUMER_AL_SPELL_CHECK                                     = 0x1AB,
    USB_HID_CONSUMER_AL_GRAMMER_CHECK                                   = 0x1AC,
    USB_HID_CONSUMER_AL_WIRELESS_STATUS                                 = 0x1AD,
    USB_HID_CONSUMER_AL_KEYBOARD_LAYOUT                                 = 0x1AE,
    USB_HID_CONSUMER_AL_VIRUS_PROTECTION                                = 0x1AF,
    USB_HID_CONSUMER_AL_ENCRYPTION                                      = 0x1B0,
    USB_HID_CONSUMER_AL_SCREEN_SAVER                                    = 0x1B1,
    USB_HID_CONSUMER_AL_ALARMS                                          = 0x1B2,
    USB_HID_CONSUMER_AL_CLOCK                                           = 0x1B3,
    USB_HID_CONSUMER_AL_FILE_BROWSER                                    = 0x1B4,
    USB_HID_CONSUMER_AL_POWER_STATUS                                    = 0x1B5,
    USB_HID_CONSUMER_AL_IMAGE_BROWSER                                   = 0x1B6,
    USB_HID_CONSUMER_AL_AUDIO_BROWSER                                   = 0x1B7,
    USB_HID_CONSUMER_AL_MOVIE_BROWSER                                   = 0x1B8,
    USB_HID_CONSUMER_AL_DIGITAL_RIGHTS_MANAGER                          = 0x1B9,
    USB_HID_CONSUMER_AL_DIGITAL_WALLET                                  = 0x1BA,
    /* Reserved                                                         = 0x1BB */
    USB_HID_CONSUMER_AL_INSTANT_MESSAGING                               = 0x1BC,
    USB_HID_CONSUMER_AL_OEM_FEATURES_TIPS_TUTORIAL_BROWSER              = 0x1BD,
    USB_HID_CONSUMER_AL_OEM_HELP                                        = 0x1BE,
    USB_HID_CONSUMER_AL_ONLINE_COMMUNITY                                = 0x1BF,
    USB_HID_CONSUMER_AL_ENTERTAINMENT_CONTENT_BROWSER                   = 0x1C0,
    USB_HID_CONSUMER_AL_ONLINE_SHOPPING_BROWSER                         = 0x1C1,
    USB_HID_CONSUMER_AL_SMARTCARD_INFORMATION_HELP                      = 0x1C2,
    USB_HID_CONSUMER_AL_MARKET_MONITOR_FINANCE_BROWSER                  = 0x1C3,
    USB_HID_CONSUMER_AL_CUSTOMIZED_CORPORATE_NEWS_BROWSER               = 0x1C4,
    USB_HID_CONSUMER_AL_ONLINE_ACTIVITY_BROWSER                         = 0x1C5,
    USB_HID_CONSUMER_AL_RESEARCH_SERACH_BROWSER                         = 0x1C6,
    USB_HID_CONSUMER_AL_AUDIO_PLAYER                                    = 0x1C7,
    /* Reserved                                                         = 0x1C8-0x1FF */
    USB_HID_CONSUMER_GENERIC_GUI_APPLICATION_CONTROLS                   = 0x200,
    USB_HID_CONSUMER_AC_NEW                                             = 0x201,
    USB_HID_CONSUMER_AC_OPEN                                            = 0x202,
    USB_HID_CONSUMER_AC_CLOSE                                           = 0x203,
    USB_HID_CONSUMER_AC_EXIT                                            = 0x204,
    USB_HID_CONSUMER_AC_MAXIMIZE                                        = 0x205,
    USB_HID_CONSUMER_AC_MINIMIZE                                        = 0x206,
    USB_HID_CONSUMER_AC_SAVE                                            = 0x207,
    USB_HID_CONSUMER_AC_PRINT                                           = 0x208,
    USB_HID_CONSUMER_AC_PROPERTIES                                      = 0x209,
    /* Unspecified                                                      = 0x20A-0x219 */
    USB_HID_CONSUMER_AC_UNDO                                            = 0x21A,
    USB_HID_CONSUMER_AC_COPY                                            = 0x21B,
    USB_HID_CONSUMER_AC_CUT                                             = 0x21C,
    USB_HID_CONSUMER_AC_PASTE                                           = 0x21D,
    USB_HID_CONSUMER_AC_SELECT_ALL                                      = 0x21E,
    USB_HID_CONSUMER_AC_FIND                                            = 0x21F,
    USB_HID_CONSUMER_AC_FIND_AND_REPLACE                                = 0x220,
    USB_HID_CONSUMER_AC_SEARCH                                          = 0x221,
    USB_HID_CONSUMER_AC_GO_TO                                           = 0x222,
    USB_HID_CONSUMER_AC_HOME                                            = 0x223,
    USB_HID_CONSUMER_AC_BACK                                            = 0x224,
    USB_HID_CONSUMER_AC_FORWARD                                         = 0x225,
    USB_HID_CONSUMER_AC_STOP                                            = 0x226,
    USB_HID_CONSUMER_AC_REFRESH                                         = 0x227,
    USB_HID_CONSUMER_AC_PREVIOUS_LINK                                   = 0x228,
    USB_HID_CONSUMER_AC_NEXT_LINK                                       = 0x229,
    USB_HID_CONSUMER_AC_BOOKMARKS                                       = 0x22A,
    USB_HID_CONSUMER_AC_HISTORY                                         = 0x22B,
    USB_HID_CONSUMER_AC_SUBSCRIPTIONS                                   = 0x22C,
    USB_HID_CONSUMER_AC_ZOOM_IN                                         = 0x22D,
    USB_HID_CONSUMER_AC_ZOOM_OUT                                        = 0x22E,
    USB_HID_CONSUMER_AC_ZOOM                                            = 0x22F,
    USB_HID_CONSUMER_AC_FULL_SCREEN_VIEW                                = 0x230,
    USB_HID_CONSUMER_AC_NORMAL_VIEW                                     = 0x231,
    USB_HID_CONSUMER_AC_VIEW_TOGGLE                                     = 0x232,
    USB_HID_CONSUMER_AC_SCROLL_UP                                       = 0x233,
    USB_HID_CONSUMER_AC_SCROLL_DOWN                                     = 0x234,
    USB_HID_CONSUMER_AC_SCROLL                                          = 0x235,
    USB_HID_CONSUMER_AC_PAN_LEFT                                        = 0x236,
    USB_HID_CONSUMER_AC_PAN_RIGHT                                       = 0x237,
    USB_HID_CONSUMER_AC_PAN                                             = 0x238,
    USB_HID_CONSUMER_AC_NEW_WINDOW                                      = 0x239,
    USB_HID_CONSUMER_AC_TILE_HORIZONTALLY                               = 0x23A,
    USB_HID_CONSUMER_AC_TILE_VERTICALLY                                 = 0x23B,
    USB_HID_CONSUMER_AC_FORMAT                                          = 0x23C,
    USB_HID_CONSUMER_AC_EDIT                                            = 0x23D,
    USB_HID_CONSUMER_AC_BOLD                                            = 0x23E,
    USB_HID_CONSUMER_AC_ITALICS                                         = 0x23F,
    USB_HID_CONSUMER_AC_UNDERLINE                                       = 0x240,
    USB_HID_CONSUMER_AC_STRIKETHROUGH                                   = 0x241,
    USB_HID_CONSUMER_AC_SUBSCRIPT                                       = 0x242,
    USB_HID_CONSUMER_AC_SUPERSCRIPT                                     = 0x243,
    USB_HID_CONSUMER_AC_ALL_CAPS                                        = 0x244,
    USB_HID_CONSUMER_AC_ROTATE                                          = 0x245,
    USB_HID_CONSUMER_AC_RESIZE                                          = 0x246,
    USB_HID_CONSUMER_AC_FLIP_HORIZONTAL                                 = 0x247,
    USB_HID_CONSUMER_AC_FLIP_VERTICALLY                                 = 0x248,
    USB_HID_CONSUMER_AC_MIRROR_HORIZONTAL                               = 0x249,
    USB_HID_CONSUMER_AC_MIRROR_VERTICAL                                 = 0x24A,
    USB_HID_CONSUMER_AC_FONT_SELECT                                     = 0x24B,
    USB_HID_CONSUMER_AC_FONT_COLOR                                      = 0x24C,
    USB_HID_CONSUMER_AC_FONT_SIZE                                       = 0x24D,
    USB_HID_CONSUMER_AC_JUSTIFY_LEFT                                    = 0x24E,
    USB_HID_CONSUMER_AC_JUSTIFY_CENTER_H                                = 0x24F,
    USB_HID_CONSUMER_AC_JUSTIFY_RIGHT                                   = 0x250,
    USB_HID_CONSUMER_AC_JUSTIFY_BLOCK_H                                 = 0x251,
    USB_HID_CONSUMER_AC_JUSTIFY_TOP                                     = 0x252,
    USB_HID_CONSUMER_AC_JUSTIFY_CENTER_V                                = 0x253,
    USB_HID_CONSUMER_AC_JUSTIFY_BOTTOM                                  = 0x254,
    USB_HID_CONSUMER_AC_JUSTIFY_BLOCK_V                                 = 0x255,
    USB_HID_CONSUMER_AC_INDENT_DESCREASE                                = 0x256,
    USB_HID_CONSUMER_AC_INDENT_INCREASE                                 = 0x257,
    USB_HID_CONSUMER_AC_NUMBERED_LIST                                   = 0x258,
    USB_HID_CONSUMER_AC_RESTART_NUMBERING                               = 0x259,
    USB_HID_CONSUMER_AC_BULLETED_LIST                                   = 0x25A,
    USB_HID_CONSUMER_AC_PROMOTE                                         = 0x25B,
    USB_HID_CONSUMER_AC_DEMOTE                                          = 0x25C,
    USB_HID_CONSUMER_AC_YES                                             = 0x25D,
    USB_HID_CONSUMER_AC_NO                                              = 0x25E,
    USB_HID_CONSUMER_AC_CANCEL                                          = 0x25F,
    USB_HID_CONSUMER_AC_CATALOG                                         = 0x260,
    USB_HID_CONSUMER_AC_BUY_CHECKOUT                                    = 0x261,
    USB_HID_CONSUMER_AC_ADD_TO_CART                                     = 0x262,
    USB_HID_CONSUMER_AC_EXPAND                                          = 0x263,
    USB_HID_CONSUMER_AC_EXPAND_ALL                                      = 0x264,
    USB_HID_CONSUMER_AC_COLLAPSE                                        = 0x265,
    USB_HID_CONSUMER_AC_COLLAPSE_ALL                                    = 0x266,
    USB_HID_CONSUMER_AC_PRINT_PREVIEW                                   = 0x267,
    USB_HID_CONSUMER_AC_PASTE_SPECIAL                                   = 0x268,
    USB_HID_CONSUMER_AC_INSERT_MODE                                     = 0x269,
    USB_HID_CONSUMER_AC_DELETE                                          = 0x26A,
    USB_HID_CONSUMER_AC_LOCK                                            = 0x26B,
    USB_HID_CONSUMER_AC_UNLOCK                                          = 0x26C,
    USB_HID_CONSUMER_AC_PROTECT                                         = 0x26D,
    USB_HID_CONSUMER_AC_UNPROTECT                                       = 0x26E,
    USB_HID_CONSUMER_AC_ATTACH_COMMENT                                  = 0x26F,
    USB_HID_CONSUMER_AC_DELETE_COMMENT                                  = 0x270,
    USB_HID_CONSUMER_AC_VIEW_COMMENT                                    = 0x271,
    USB_HID_CONSUMER_AC_SELECT_WORD                                     = 0x272,
    USB_HID_CONSUMER_AC_SELECT_SENTENCE                                 = 0x273,
    USB_HID_CONSUMER_AC_SELECT_PARAGRAPH                                = 0x274,
    USB_HID_CONSUMER_AC_SELECT_COLUMN                                   = 0x275,
    USB_HID_CONSUMER_AC_SELECT_ROW                                      = 0x276,
    USB_HID_CONSUMER_AC_SELECT_TABLE                                    = 0x277,
    USB_HID_CONSUMER_AC_SELECT_OBJECT                                   = 0x278,
    USB_HID_CONSUMER_AC_SELECT_REDO_REPEAT                              = 0x279,
    USB_HID_CONSUMER_AC_SORT                                            = 0x27A,
    USB_HID_CONSUMER_AC_SORT_ASCENDING                                  = 0x27B,
    USB_HID_CONSUMER_AC_SORT_DESCENDING                                 = 0x27C,
    USB_HID_CONSUMER_AC_FILTER                                          = 0x27D,
    USB_HID_CONSUMER_AC_SET_CLOCK                                       = 0x27E,
    USB_HID_CONSUMER_AC_VIEW_CLOCK                                      = 0x27F,
    USB_HID_CONSUMER_AC_SELECT_TIME_ZONE                                = 0x280,
    USB_HID_CONSUMER_AC_EDIT_TIME_ZONE                                  = 0x281,
    USB_HID_CONSUMER_AC_SET_ALARM                                       = 0x282,
    USB_HID_CONSUMER_AC_CLEAR_ALARM                                     = 0x283,
    USB_HID_CONSUMER_AC_SNOOZE_ALARM                                    = 0x284,
    USB_HID_CONSUMER_AC_RESET_ALARM                                     = 0x285,
    USB_HID_CONSUMER_AC_SYNCHRONIZE                                     = 0x286,
    USB_HID_CONSUMER_AC_SEND_RECEIVE                                    = 0x287,
    USB_HID_CONSUMER_AC_SEND_TO                                         = 0x288,
    USB_HID_CONSUMER_AC_REPLY                                           = 0x289,
    USB_HID_CONSUMER_AC_REPLY_ALL                                       = 0x28A,
    USB_HID_CONSUMER_AC_FORWARD_MSG                                     = 0x28B,
    USB_HID_CONSUMER_AC_SEND                                            = 0x28C,
    USB_HID_CONSUMER_AC_ATTACH_FILE                                     = 0x28D,
    USB_HID_CONSUMER_AC_UPLOAD                                          = 0x28E,
    USB_HID_CONSUMER_AC_DOWNLOAD_SAVE_TARGET_AS                         = 0x28F,
    USB_HID_CONSUMER_AC_SET_BORDERS                                     = 0x290,
    USB_HID_CONSUMER_AC_INSERT_ROW                                      = 0x291,
    USB_HID_CONSUMER_AC_INSERT_COLUMN                                   = 0x292,
    USB_HID_CONSUMER_AC_INSERT_FILE                                     = 0x293,
    USB_HID_CONSUMER_AC_INSERT_PICTURE                                  = 0x294,
    USB_HID_CONSUMER_AC_INSERT_OBJECT                                   = 0x295,
    USB_HID_CONSUMER_AC_INSERT_SYMBOL                                   = 0x296,
    USB_HID_CONSUMER_AC_SAVE_AND_CLOSE                                  = 0x297,
    USB_HID_CONSUMER_AC_RENAME                                          = 0x298,
    USB_HID_CONSUMER_AC_MERGE                                           = 0x299,
    USB_HID_CONSUMER_AC_SPLIT                                           = 0x29A,
    USB_HID_CONSUMER_AC_DISTRIBUTE_HORIZONTALLY                         = 0x29B,
    USB_HID_CONSUMER_AC_DISTRIBUTE_VERTICALLY                           = 0x29C
    /* Reserved                                                         = 0x29D-0xFFFF */
} USB_HID_CONSUMER;

typedef enum
{
    USB_HID_DIGITIZERS_UNDEFINED                                        = 0x00,
    USB_HID_DIGITIZERS_DIGITIZER                                        = 0x01,
    USB_HID_DIGITIZERS_PEN                                              = 0x02,
    USB_HID_DIGITIZERS_LIGHT_PEN                                        = 0x03,
    USB_HID_DIGITIZERS_TOUCH_SCREEN                                     = 0x04,
    USB_HID_DIGITIZERS_TOUCH_PAD                                        = 0x05,
    USB_HID_DIGITIZERS_WHITE_BOARD                                      = 0x06,
    USB_HID_DIGITIZERS_COORDINATE_MEASURING_MACHINE                     = 0x07,
    USB_HID_DIGITIZERS_3D_DIGITIZER                                     = 0x08,
    USB_HID_DIGITIZERS_STEREO_PLOTTER                                   = 0x09,
    USB_HID_DIGITIZERS_ARTICULATED_ARM                                  = 0x0A,
    USB_HID_DIGITIZERS_ARMATURE                                         = 0x0B,
    USB_HID_DIGITIZERS_MULTIPLE_POINT_DIGITIZER                         = 0x0C,
    USB_HID_DIGITIZERS_FREE_SPACE_WAND                                  = 0x0D,
    /* Reserved                                                         = 0x0E-0x1F */
    USB_HID_DIGITIZERS_STYLUS                                           = 0x20,
    USB_HID_DIGITIZERS_PUCK                                             = 0x21,
    USB_HID_DIGITIZERS_FINGER                                           = 0x22,
    /* Reserved                                                         = 0x23-0x2F */
    USB_HID_DIGITIZERS_TIP_PRESSURE                                     = 0x30,
    USB_HID_DIGITIZERS_BARREL_PRESSURE                                  = 0x31,
    USB_HID_DIGITIZERS_IN_RANGE                                         = 0x32,
    USB_HID_DIGITIZERS_TOUCH                                            = 0x33,
    USB_HID_DIGITIZERS_UNTOUCH                                          = 0x34,
    USB_HID_DIGITIZERS_TAP                                              = 0x35,
    USB_HID_DIGITIZERS_QUALITY                                          = 0x36,
    USB_HID_DIGITIZERS_DATA_VALID                                       = 0x37,
    USB_HID_DIGITIZERS_TRANSDUCER_INDEX                                 = 0x38,
    USB_HID_DIGITIZERS_TABLET_FUNCTION_KEYS                             = 0x39,
    USB_HID_DIGITIZERS_PROGRAMMING_CHANGE_KEYS                          = 0x3A,
    USB_HID_DIGITIZERS_BATTERY_STRENGTH                                 = 0x3B,
    USB_HID_DIGITIZERS_INVERT                                           = 0x3C,
    USB_HID_DIGITIZERS_X_TILT                                           = 0x3D,
    USB_HID_DIGITIZERS_Y_TILT                                           = 0x3E,
    USB_HID_DIGITIZERS_AZIMUTH                                          = 0x3F,
    USB_HID_DIGITIZERS_ALTITUDE                                         = 0x40,
    USB_HID_DIGITIZERS_TWIST                                            = 0x41,
    USB_HID_DIGITIZERS_TIP_SWITCH                                       = 0x42,
    USB_HID_DIGITIZERS_SECONDARY_TIP_SWITCH                             = 0x43,
    USB_HID_DIGITIZERS_BARREL_SWITCH                                    = 0x44,
    USB_HID_DIGITIZERS_ERASER                                           = 0x45,
    USB_HID_DIGITIZERS_TABLET_PICK                                      = 0x46
    /* Reserved                                                         = 0x47-0xFFFF */
} USB_HID_DIGITIZERS;

typedef enum
{
    USB_HID_ALPHANUMERIC_DISPLAY_UNDEFINED                              = 0x00,
    USB_HID_ALPHANUMERIC_DISPLAY_ALPHANUMERIC_DISPLAY                   = 0x01,
    USB_HID_ALPHANUMERIC_DISPLAY_BITMAPPED_DISPLAY                      = 0x02,
    /* Reserved                                                         = 0x03-0x1F */
    USB_HID_ALPHANUMERIC_DISPLAY_DISPLAY_ATTRIBUTES_REPORT              = 0x20,
    USB_HID_ALPHANUMERIC_DISPLAY_ASCII_CHARACTER_SET                    = 0x21,
    USB_HID_ALPHANUMERIC_DISPLAY_DATA_READ_BACK                         = 0x22,
    USB_HID_ALPHANUMERIC_DISPLAY_FONT_READ_BACK                         = 0x23,
    USB_HID_ALPHANUMERIC_DISPLAY_DISPLAY_CONTROL_REPORT                 = 0x24,
    USB_HID_ALPHANUMERIC_DISPLAY_CLEAR_DISPLAY                          = 0x25,
    USB_HID_ALPHANUMERIC_DISPLAY_DISPLAY_ENABLE                         = 0x26,
    USB_HID_ALPHANUMERIC_DISPLAY_SCREEN_SAVER_DELAY                     = 0x27,
    USB_HID_ALPHANUMERIC_DISPLAY_SCREEN_SAVER_ENABLE                    = 0x28,
    USB_HID_ALPHANUMERIC_DISPLAY_VERTICAL_SCROLL                        = 0x29,
    USB_HID_ALPHANUMERIC_DISPLAY_HORIZONTAL_SCROLL                      = 0x2A,
    USB_HID_ALPHANUMERIC_DISPLAY_CHARACTER_REPORT                       = 0x2B,
    USB_HID_ALPHANUMERIC_DISPLAY_DISPLAY_DATA                           = 0x2C,
    USB_HID_ALPHANUMERIC_DISPLAY_DISPLAY_STATUS                         = 0x2D,
    USB_HID_ALPHANUMERIC_DISPLAY_STAT_NOT_READY                         = 0x2E,
    USB_HID_ALPHANUMERIC_DISPLAY_STAT_READY                             = 0x2F,
    USB_HID_ALPHANUMERIC_DISPLAY_ERR_NOT_A_LOADABLE_CHARACTER           = 0x30,
    USB_HID_ALPHANUMERIC_DISPLAY_ERR_FONT_DATA_CANNOT_BE_READ           = 0x31,
    USB_HID_ALPHANUMERIC_DISPLAY_CURSOR_POSITION_REPORT                 = 0x32,
    USB_HID_ALPHANUMERIC_DISPLAY_ROW                                    = 0x33,
    USB_HID_ALPHANUMERIC_DISPLAY_COLUMN                                 = 0x34,
    USB_HID_ALPHANUMERIC_DISPLAY_ROWS                                   = 0x35,
    USB_HID_ALPHANUMERIC_DISPLAY_COLUMNS                                = 0x36,
    USB_HID_ALPHANUMERIC_DISPLAY_CURSOR_PIXEL_POSITION                  = 0x37,
    USB_HID_ALPHANUMERIC_DISPLAY_CURSOR_MODE                            = 0x38,
    USB_HID_ALPHANUMERIC_DISPLAY_CURSOR_ENABLE                          = 0x39,
    USB_HID_ALPHANUMERIC_DISPLAY_CURSOR_BLINK                           = 0x3A,
    USB_HID_ALPHANUMERIC_DISPLAY_FONT_REPORT                            = 0x3B,
    USB_HID_ALPHANUMERIC_DISPLAY_FONT_DATA                              = 0x3C,
    USB_HID_ALPHANUMERIC_DISPLAY_CHARACTER_WIDTH                        = 0x3D,
    USB_HID_ALPHANUMERIC_DISPLAY_CHARACTER_HEIGHT                       = 0x3E,
    USB_HID_ALPHANUMERIC_DISPLAY_CHARACTER_SPACING_HORIZONTAL           = 0x3F,
    USB_HID_ALPHANUMERIC_DISPLAY_CHARACTER_SPACING_VERTICAL             = 0x40,
    USB_HID_ALPHANUMERIC_DISPLAY_UNICODE_CHARACTER_SET                  = 0x41,
    USB_HID_ALPHANUMERIC_DISPLAY_FONT_7_SEGMENT                         = 0x42,
    USB_HID_ALPHANUMERIC_DISPLAY_7_SEGMENT_DIRECT_MAP                   = 0x43,
    USB_HID_ALPHANUMERIC_DISPLAY_FONT_14_SEGMENT                        = 0x44,
    USB_HID_ALPHANUMERIC_DISPLAY_14_SEGMENT_DIRECT_MAP                  = 0x45,
    USB_HID_ALPHANUMERIC_DISPLAY_DISPLAY_BRIGHTNESS                     = 0x46,
    USB_HID_ALPHANUMERIC_DISPLAY_DISPLAY_CONTRAST                       = 0x47,
    USB_HID_ALPHANUMERIC_DISPLAY_CHARACTER_ATTRIBUTE                    = 0x48,
    USB_HID_ALPHANUMERIC_DISPLAY_ATTRIBUTE_READBACK                     = 0x49,
    USB_HID_ALPHANUMERIC_DISPLAY_ATTRIBUTE_DATA                         = 0x4A,
    USB_HID_ALPHANUMERIC_DISPLAY_CHAR_ATTR_ENHANCE                      = 0x4B,
    USB_HID_ALPHANUMERIC_DISPLAY_CHAR_ATTR_UNDERLINE                    = 0x4C,
    USB_HID_ALPHANUMERIC_DISPLAY_CHAR_ATTR_BLINK                        = 0x4D,
    /* Reserved                                                         = 0x4E-0x7F */
    USB_HID_ALPHANUMERIC_DISPLAY_BITMAP_SIZE_X                          = 0x80,
    USB_HID_ALPHANUMERIC_DISPLAY_BITMAP_SIZE_Y                          = 0x81,
    /* Reserved                                                         = 0x82 */
    USB_HID_ALPHANUMERIC_DISPLAY_BIT_DEPTH_FORMAT                       = 0x83,
    USB_HID_ALPHANUMERIC_DISPLAY_DISPLAY_ORIENTATION                    = 0x84,
    USB_HID_ALPHANUMERIC_DISPLAY_PALETTE_REPORT                         = 0x85,
    USB_HID_ALPHANUMERIC_DISPLAY_PALETTE_DATA_SIZE                      = 0x86,
    USB_HID_ALPHANUMERIC_DISPLAY_PALETTE_DATA_OFFSET                    = 0x87,
    USB_HID_ALPHANUMERIC_DISPLAY_PALETTE_DATA                           = 0x88,
    /* Unspecified                                                      = 0x89 */
    USB_HID_ALPHANUMERIC_DISPLAY_BLIT_REPORT                            = 0x8A,
    USB_HID_ALPHANUMERIC_DISPLAY_BLIT_RECTANGLE_X1                      = 0x8B,
    USB_HID_ALPHANUMERIC_DISPLAY_BLIT_RECTANGLE_Y1                      = 0x8C,
    USB_HID_ALPHANUMERIC_DISPLAY_BLIT_RECTANGLE_X2                      = 0x8D,
    USB_HID_ALPHANUMERIC_DISPLAY_BLIT_RECTANGLE_Y2                      = 0x8E,
    USB_HID_ALPHANUMERIC_DISPLAY_BLIT_DATA                              = 0x8F,
    USB_HID_ALPHANUMERIC_DISPLAY_SOFT_BUTTON                            = 0x90,
    USB_HID_ALPHANUMERIC_DISPLAY_SOFT_BUTTON_ID                         = 0x91,
    USB_HID_ALPHANUMERIC_DISPLAY_SOFT_BUTTON_SIDE                       = 0x92,
    USB_HID_ALPHANUMERIC_DISPLAY_SOFT_BUTTON_OFFSET_1                   = 0x93,
    USB_HID_ALPHANUMERIC_DISPLAY_SOFT_BUTTON_OFFSET_2                   = 0x94,
    USB_HID_ALPHANUMERIC_DISPLAY_SOFT_BUTTON_REPORT                     = 0x95
    /* Reserved                                                         = 0x96-0xFFFF */
} USB_HID_ALPHANUMERIC_DISPLAY;

typedef enum
{
    USB_HID_MEDICAL_INSTRUMENT_UNDEFINED                                = 0x00,
    USB_HID_MEDICAL_INSTRUMENT_MEDICAL_ULTRASOUND                       = 0x01,
    /* Reserved                                                         = 0x02-0x1F */
    USB_HID_MEDICAL_INSTRUMENT_VCR_ACQUISITION                          = 0x20,
    USB_HID_MEDICAL_INSTRUMENT_FREEZE_THAW                              = 0x21,
    USB_HID_MEDICAL_INSTRUMENT_CLIP_STORE                               = 0x22,
    USB_HID_MEDICAL_INSTRUMENT_UPDATE                                   = 0x23,
    USB_HID_MEDICAL_INSTRUMENT_NEXT                                     = 0x24,
    USB_HID_MEDICAL_INSTRUMENT_SAVE                                     = 0x25,
    USB_HID_MEDICAL_INSTRUMENT_PRINT                                    = 0x26,
    USB_HID_MEDICAL_INSTRUMENT_MICROPHONE_ENABLE                        = 0x27,
    /* Reserved                                                         = 0x28-0x3F */
    USB_HID_MEDICAL_INSTRUMENT_CINE                                     = 0x40,
    USB_HID_MEDICAL_INSTRUMENT_TRANSMIT_POWER                           = 0x41,
    USB_HID_MEDICAL_INSTRUMENT_VOLUME                                   = 0x42,
    USB_HID_MEDICAL_INSTRUMENT_FOCUS                                    = 0x43,
    USB_HID_MEDICAL_INSTRUMENT_DEPTH                                    = 0x44,
    /* Reserved                                                         = 0x45-0x5F */
    USB_HID_MEDICAL_INSTRUMENT_SOFT_STEP_PRIMARY                        = 0x60,
    USB_HID_MEDICAL_INSTRUMENT_SOFT_STEP_SECONDARY                      = 0x61,
    /* Reserved                                                         = 0x62-0x6F */
    USB_HID_MEDICAL_INSTRUMENT_DEPTH_GAIN_COMPENSATION                  = 0x70,
    /* Reserved                                                         = 0x71-0x7F */
    USB_HID_MEDICAL_INSTRUMENT_ZOOM_SELECT                              = 0x80,
    USB_HID_MEDICAL_INSTRUMENT_ZOOM_ADJUST                              = 0x81,
    USB_HID_MEDICAL_INSTRUMENT_SPECTRAL_DOPPLER_MODE_SELECT             = 0x82,
    USB_HID_MEDICAL_INSTRUMENT_SPECTRAL_DOPPLER_ADJUST                  = 0x83,
    USB_HID_MEDICAL_INSTRUMENT_COLOR_DOPPLER_MODE_SELECT                = 0x84,
    USB_HID_MEDICAL_INSTRUMENT_COLOR_COPPLER_ADJUST                     = 0x85,
    USB_HID_MEDICAL_INSTRUMENT_MOTION_MODE_SELECT                       = 0x86,
    USB_HID_MEDICAL_INSTRUMENT_MOTION_MODE_ADJUST                       = 0x87,
    USB_HID_MEDICAL_INSTRUMENT_2D_MODE_SELECT                           = 0x88,
    USB_HID_MEDICAL_INSTRUMENT_2D_MODE_ADJUST                           = 0x89,
    /* Reserved                                                         = 0x8A-0x9F */
    USB_HID_MEDICAL_INSTRUMENT_SOFT_CONTROL_SELECT                      = 0xA0,
    USB_HID_MEDICAL_INSTRUMENT_SOFT_CONTROL_ADJUST                      = 0xA1
    /* Reserved                                                         = 0xA2-0xFFFF */
} USB_HID_MEDICAL_INSTRUMENT;

typedef enum
{
    USB_HID_MONITOR_RESERVED                                            = 0x00,
    USB_HID_MONITOR_MONITOR_CONTROL                                     = 0x01,
    USB_HID_MONITOR_EDID_INFORMATION                                    = 0x02,
    USB_HID_MONITOR_VDIF_INFORMATION                                    = 0x03,
    USB_HID_MONITOR_VESA_VERSION                                        = 0x04
} USB_HID_MONITOR;

typedef enum
{
    /* Contiguous controls */
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_BRIGHTNESS                    = 0x10,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_CONTRAST                      = 0x12,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_RED_VIDEO_GAIN                = 0x16,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_GREEN_VIDEO_GAIN              = 0x18,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_BLUE_VIDEO_GAIN               = 0x1A,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_FOCUS                         = 0x1C,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_HORIZONTAL_POSITION           = 0x20,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_HORIZONTAL_SIZE               = 0x22,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_HORIZONTAL_PINCUSHION         = 0x24,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_HORIZONTAL_PINCUSHION_BALANCE = 0x26,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_HORIZONTAL_MISCONVERGENCE     = 0x28,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_HORIZONTAL_LINEARITY          = 0x2A,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_HORIZONTAL_LINEARITY_BALANCE  = 0x2C,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_VERTICAL_POSITION             = 0x30,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_VERITCAL_SIZE                 = 0x32,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_VERTICAL_PINCUSHION           = 0x34,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_VERTICAL_PINCUSHION_BALANCE   = 0x36,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_VERTICAL_MISCONVERGENCE       = 0x38,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_VERTICAL_LINEARITY            = 0x3A,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_VERTICAL_LINEARITY_BALANCE    = 0x3C,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_PARALLELOGRAM_DISTORTION_KEY_BALANCE  = 0x40,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_TRAPEZOIDAL_DISTORTION_KEY    = 0x42,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_TILT_ROTATION                 = 0x44,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_TOP_CORNER_DISTORTION_CONTROL = 0x46,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_TOP_CORNER_DISTORTION_BALANCE = 0x48,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_BOTTOM_CORNER_DISTORTION_CONTROL      = 0x4A,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_BOTTOM_CORNER_DISTORTION_BALANCE      = 0x4C,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_HORIZONTAL_MOIRE              = 0x56,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_VERTICAL_MOIRE                = 0x58,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_RED_VIDEO_BLACK_LEVEL         = 0x6C,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_GREEN_VIDEO_BLACK_LEVEL       = 0x6E,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_BLUE_VIDEO_BLACK_LEVEL        = 0x70,

    /* Non-contiguous controls (read/write) */
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_INPUT_LEVEL_SELECT            = 0x5E,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_INPUT_SOURCE_SELECT           = 0x60,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_ON_SCREEN_DISPLAY             = 0xCA,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_STEREOMODE                    = 0xD4,

    /* Non-contiguous controls (read-only) */
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_AUDO_SIZE_CENTER              = 0xA2,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_POLARITY_HORIZONTAL_SYNCHRONIZATION   = 0xA4,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_POLARITY_VERTICAL_SYNCHRONIZATION     = 0xA6,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_SYNCHRONIZATION_TYPE          = 0xA8,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_SCREEN_ORIENTATION            = 0xAA,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_HORIZONTAL_FREQUENCY          = 0xAC,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_VERTICAL_FREQUENCY            = 0xAE,

    /* Non-contiguous controls (write-only) */
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_DEGAUSS                       = 0x01,
    USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS_SETTINGS                      = 0xB0

} USB_HID_MONITOR_VESA_VIRTUAL_CONTROLS;


typedef enum
{
    USB_HID_BAR_CODE_SCANNER_UNDEFINED                                  = 0x00,
    USB_HID_BAR_CODE_SCANNER_BAR_CODE_BADGE_READER                      = 0x01,
    USB_HID_BAR_CODE_SCANNER_BAR_CODE_SCANNER                           = 0x02,
    USB_HID_BAR_CODE_SCANNER_DUMB_BAR_CODE_SCANNER                      = 0x03,
    USB_HID_BAR_CODE_SCANNER_CORDLESS_SCANNER_BASE                      = 0x04,
    USB_HID_BAR_CODE_SCANNER_BAR_CODE_SCANNER_CRADLE                    = 0x05,
    /* Reserved                                                         = 0x06-0x0F */
    USB_HID_BAR_CODE_SCANNER_ATTRIBUTE_REPORT                           = 0x10,
    USB_HID_BAR_CODE_SCANNER_SETTINGS_REPORT                            = 0x11,
    USB_HID_BAR_CODE_SCANNER_SCANNED_DATA_REPORT                        = 0x12,
    USB_HID_BAR_CODE_SCANNER_RAW_SCANNED_DATA_REPORT                    = 0x13,
    USB_HID_BAR_CODE_SCANNER_TRIGGER_REPORT                             = 0x14,
    USB_HID_BAR_CODE_SCANNER_STATUS_REPORT                              = 0x15,
    USB_HID_BAR_CODE_SCANNER_UPC_EAN_CONTROL_REPORT                     = 0x16,
    USB_HID_BAR_CODE_SCANNER_EAN_2_3_LABEL_CONTROL_REPORT               = 0x17,
    USB_HID_BAR_CODE_SCANNER_CODE_39_CONTROL_REPORT                     = 0x18,
    USB_HID_BAR_CODE_SCANNER_INTERLEAVED_2_OF_5_CONTROL_REPORT          = 0x19,
    USB_HID_BAR_CODE_SCANNER_STANDARD_2_OF_5_CONTROL_REPORT             = 0x1A,
    USB_HID_BAR_CODE_SCANNER_MSI_PLESSEY_CONTROL_REPORT                 = 0x1B,
    USB_HID_BAR_CODE_SCANNER_CODABAR_CONTROL_REPORT                     = 0x1C,
    USB_HID_BAR_CODE_SCANNER_CODE_128_CONTROL_REPORT                    = 0x1D,
    USB_HID_BAR_CODE_SCANNER_MISC_1D_CONTROL_REPORT                     = 0x1E,
    USB_HID_BAR_CODE_SCANNER_2D_CONTROL_REPORT                          = 0x1F,
    /* Reserved                                                         = 0x20-0x2F */
    USB_HID_BAR_CODE_SCANNER_AIMING_POINTER_MODE                        = 0x30,
    USB_HID_BAR_CODE_SCANNER_BAR_CODE_PRESENT_SENSOR                    = 0x31,
    USB_HID_BAR_CODE_SCANNER_CLASS_1A_LASER                             = 0x32,
    USB_HID_BAR_CODE_SCANNER_CLASS_2_LASER                              = 0x33,
    USB_HID_BAR_CODE_SCANNER_HEATER_PRESENT                             = 0x34,
    USB_HID_BAR_CODE_SCANNER_CONTACT_SCANNER                            = 0x35,
    USB_HID_BAR_CODE_SCANNER_ELECTRONIC_ARTICLE_SURVEILLANCE_NOTIFICATION   = 0x36,
    USB_HID_BAR_CODE_SCANNER_CONSTANT_ELECTRONIC_ARTICLE_SURVEILLANCE   = 0x37,
    USB_HID_BAR_CODE_SCANNER_ERROR_INDICATION                           = 0x38,
    USB_HID_BAR_CODE_SCANNER_FIXED_BEEPER                               = 0x39,
    USB_HID_BAR_CODE_SCANNER_GOOD_DECODE_INDICATION                     = 0x3A,
    USB_HID_BAR_CODE_SCANNER_HANDS_FREE_SCANNING                        = 0x3B,
    USB_HID_BAR_CODE_SCANNER_INTRINSICALLY_SAFE                         = 0x3C,
    USB_HID_BAR_CODE_SCANNER_KLASSE_EINS_LASER                          = 0x3D,
    USB_HID_BAR_CODE_SCANNER_LONG_RANGE_SCANNER                         = 0x3E,
    USB_HID_BAR_CODE_SCANNER_MIRROR_SPEED_CONTROL                       = 0x3F,
    USB_HID_BAR_CODE_SCANNER_NOT_ON_FILE_INDICATION                     = 0x40,
    USB_HID_BAR_CODE_SCANNER_PROGRAMMABLE_BEEPER                        = 0x41,
    USB_HID_BAR_CODE_SCANNER_TRIGGERLESS                                = 0x42,
    USB_HID_BAR_CODE_SCANNER_WAND                                       = 0x43,
    USB_HID_BAR_CODE_SCANNER_WATER_RESISTANT                            = 0x44,
    USB_HID_BAR_CODE_SCANNER_MULTI_RANGE_SCANNER                        = 0x45,
    USB_HID_BAR_CODE_SCANNER_PROXIMITY_SENSOR                           = 0x46,
    /* Reserved                                                         = 0x47-0x4C */
    USB_HID_BAR_CODE_SCANNER_FRAGMENT_DECODING                          = 0x4D,
    USB_HID_BAR_CODE_SCANNER_SCANNER_READ_CONFIDENCE                    = 0x4E,
    USB_HID_BAR_CODE_SCANNER_DATA_PREFIX                                = 0x4F,
    USB_HID_BAR_CODE_SCANNER_PREFIX_AIMI                                = 0x50,
    USB_HID_BAR_CODE_SCANNER_PREFIX_NONE                                = 0x51,
    USB_HID_BAR_CODE_SCANNER_PREFIX_PROPRIETARY                         = 0x52,
    /* Reserved                                                         = 0x53-0x54 */
    USB_HID_BAR_CODE_SCANNER_ACTIVE_TIME                                = 0x55,
    USB_HID_BAR_CODE_SCANNER_AIMING_LASER_PATTERN                       = 0x56,
    USB_HID_BAR_CODE_SCANNER_BAR_CODE_PRESENT                           = 0x57,
    USB_HID_BAR_CODE_SCANNER_BEEPER_STATE                               = 0x58,
    USB_HID_BAR_CODE_SCANNER_LASER_ON_TIME                              = 0x59,
    USB_HID_BAR_CODE_SCANNER_LASER_STATE                                = 0x5A,
    USB_HID_BAR_CODE_SCANNER_LOCKOUT_TIME                               = 0x5B,
    USB_HID_BAR_CODE_SCANNER_MOTOR_STATE                                = 0x5C,
    USB_HID_BAR_CODE_SCANNER_MOTOR_TIMEOUT                              = 0x5D,
    USB_HID_BAR_CODE_SCANNER_POWER_ON_RESET_SCANNER                     = 0x5E,
    USB_HID_BAR_CODE_SCANNER_PREVENT_READ_OF_BARCODES                   = 0x5F,
    USB_HID_BAR_CODE_SCANNER_INITIATE_BARCODE_READ                      = 0x60,
    USB_HID_BAR_CODE_SCANNER_TRIGGER_STATE                              = 0x61,
    USB_HID_BAR_CODE_SCANNER_TRIGGER_MODE                               = 0x62,
    USB_HID_BAR_CODE_SCANNER_TRIGGER_MODE_BLINKING_LASER_ON             = 0x63,
    USB_HID_BAR_CODE_SCANNER_TRIGGER_MODE_CONTINUOUS_LASER_ON           = 0x64,
    USB_HID_BAR_CODE_SCANNER_TRIGGER_MODE_LASER_ON_WHILE_PULLED         = 0x65,
    USB_HID_BAR_CODE_SCANNER_TRIGGER_MODE_LASER_STAYS_ON_AFTER_TRIGGER_RELEASE  = 0x66,
    /* Reserved                                                         = 0x67-0x6C */
    USB_HID_BAR_CODE_SCANNER_COMMIT_PARAMETERS_TO_NVM                   = 0x6D,
    USB_HID_BAR_CODE_SCANNER_PARAMETER_SCANNING                         = 0x6E,
    USB_HID_BAR_CODE_SCANNER_PARAMETERS_CHANGED                         = 0x6F,
    USB_HID_BAR_CODE_SCANNER_SET_PARAMETER_DEFAULT_VALUES               = 0x70,
    /* Reserved                                                         = 0x71-0x74 */
    USB_HID_BAR_CODE_SCANNER_SCANNER_IN_CRADLE                          = 0x75,
    USB_HID_BAR_CODE_SCANNER_SCANNER_IN_RANGE                           = 0x76,
    /* Reserved                                                         = 0x77-0x79 */
    USB_HID_BAR_CODE_SCANNER_AIM_DURATION                               = 0x7A,
    USB_HID_BAR_CODE_SCANNER_GOOD_READ_LAMP_DURATION                    = 0x7B,
    USB_HID_BAR_CODE_SCANNER_GOOD_READ_LAMP_INTENSITY                   = 0x7C,
    USB_HID_BAR_CODE_SCANNER_GOOD_READ_LED                              = 0x7D,
    USB_HID_BAR_CODE_SCANNER_GOOD_READ_TONE_FREQUENCY                   = 0x7E,
    USB_HID_BAR_CODE_SCANNER_GOOD_READ_TONE_LENGTH                      = 0x7F,
    USB_HID_BAR_CODE_SCANNER_GOOD_READ_TONE_VOLUME                      = 0x80,
    /* Reserved                                                         = 0x81 */
    USB_HID_BAR_CODE_SCANNER_NO_READ_MESSAGE                            = 0x82,
    USB_HID_BAR_CODE_SCANNER_NOT_ON_FILE_VOLUME                         = 0x83,
    USB_HID_BAR_CODE_SCANNER_POWERUP_BEEP                               = 0x84,
    USB_HID_BAR_CODE_SCANNER_SOUND_ERROR_BEEP                           = 0x85,
    USB_HID_BAR_CODE_SCANNER_SOUND_GOOD_READ_BEEP                       = 0x86,
    USB_HID_BAR_CODE_SCANNER_SOUND_NOT_ON_FILE_BEEP                     = 0x87,
    USB_HID_BAR_CODE_SCANNER_GOOD_READ_WHEN_TO_WRITE                    = 0x88,
    USB_HID_BAR_CODE_SCANNER_GRWTI_AFTER_DECODE                         = 0x89,
    USB_HID_BAR_CODE_SCANNER_GRWTI_BEEP_LAMP_AFTER_TRANSMIT             = 0x8A,
    USB_HID_BAR_CODE_SCANNER_GRWTI_NO_BEEP_LAMP_USE_AT_ALL              = 0x8B,
    /* Reserved                                                         = 0x8C-0x90 */
    USB_HID_BAR_CODE_SCANNER_BOOKLAND_EAN                               = 0x91,
    USB_HID_BAR_CODE_SCANNER_CONVERT_EAN_8_TO_13_TYPE                   = 0x92,
    USB_HID_BAR_CODE_SCANNER_CONVERT_UPC_A_TO_EAN_13                    = 0x93,
    USB_HID_BAR_CODE_SCANNER_CONVERT_UPC_E_TO_A                         = 0x94,
    USB_HID_BAR_CODE_SCANNER_EAN_13                                     = 0x95,
    USB_HID_BAR_CODE_SCANNER_EAN_8                                      = 0x96,
    USB_HID_BAR_CODE_SCANNER_EAN_99_128_MANDATORY                       = 0x97,
    USB_HID_BAR_CODE_SCANNER_EAN_99_P5_128_OPTIONAL                     = 0x98,
    /* Reserved                                                         = 0x99 */
    USB_HID_BAR_CODE_SCANNER_UPC_EAN                                    = 0x9A,
    USB_HID_BAR_CODE_SCANNER_UPC_EAN_COUPON_CODE                        = 0x9B,
    USB_HID_BAR_CODE_SCANNER_UPC_EAN_PERIODICALS                        = 0x9C,
    USB_HID_BAR_CODE_SCANNER_UPC_A                                      = 0x9D,
    USB_HID_BAR_CODE_SCANNER_UPC_A_WITH_128_MANDATORY                   = 0x9E,
    USB_HID_BAR_CODE_SCANNER_UPC_A_WITH_128_OPTIONAL                    = 0x9F,
    USB_HID_BAR_CODE_SCANNER_UPC_A_WITH_P5_OPTIONAL                     = 0xA0,
    USB_HID_BAR_CODE_SCANNER_UPC_E                                      = 0xA1,
    USB_HID_BAR_CODE_SCANNER_UPC_E1                                     = 0xA2,
    /* Reserved                                                         = 0xA2-0xA8 */
    USB_HID_BAR_CODE_SCANNER_PERIODICAL                                 = 0xA9,
    USB_HID_BAR_CODE_SCANNER_PERIODICAL_AUTO_DISCRIMINATE_PLUS_2        = 0xAA,
    USB_HID_BAR_CODE_SCANNER_PERIODICAL_ONLY_DECODE_WITH_PLUS_2         = 0xAB,
    USB_HID_BAR_CODE_SCANNER_PERIODICAL_IGNORE_PLUS_2                   = 0xAC,
    USB_HID_BAR_CODE_SCANNER_PERIODICAL_AUTO_DISCRIMINATE_PLUS_5        = 0xAD,
    USB_HID_BAR_CODE_SCANNER_PERIODICAL_ONLY_DECODE_WITH_PLUS_5         = 0xAE,
    USB_HID_BAR_CODE_SCANNER_PERIODICAL_IGNORE_PLUS_5                   = 0xAF,
    USB_HID_BAR_CODE_SCANNER_CHECK                                      = 0xB0,
    USB_HID_BAR_CODE_SCANNER_CHECK_DISABLE_PRICE                        = 0xB1,
    USB_HID_BAR_CODE_SCANNER_CHECK_ENABLE_4_DIGIT_PRICE                 = 0xB2,
    USB_HID_BAR_CODE_SCANNER_CHECK_ENABLE_5_DIGIT_PRICE                 = 0xB3,
    USB_HID_BAR_CODE_SCANNER_CHECK_ENABLE_EUROPEAN_4_DIGIT_PRICE        = 0xB4,
    USB_HID_BAR_CODE_SCANNER_CHECK_ENABLE_EUROPEAN_5_DIGIT_PRICE        = 0xB5,
    /* Reserved                                                         = 0xB6 */
    USB_HID_BAR_CODE_SCANNER_EAN_TWO_LABEL                              = 0xB7,
    USB_HID_BAR_CODE_SCANNER_EAN_THREE_LABEL                            = 0xB8,
    USB_HID_BAR_CODE_SCANNER_EAN_8_FLAG_DIGIT_1                         = 0xB9,
    USB_HID_BAR_CODE_SCANNER_EAN_8_FLAG_DIGIT_2                         = 0xBA,
    USB_HID_BAR_CODE_SCANNER_EAN_8_FLAG_DIGIT_3                         = 0xBB,
    USB_HID_BAR_CODE_SCANNER_EAN_13_FLAG_DIGIT_1                        = 0xBC,
    USB_HID_BAR_CODE_SCANNER_EAN_13_FLAG_DIGIT_2                        = 0xBD,
    USB_HID_BAR_CODE_SCANNER_EAN_13_FLAG_DIGIT_3                        = 0xBE,
    USB_HID_BAR_CODE_SCANNER_ADD_EAN_2_3_LABEL_DEFINITION               = 0xBF,
    USB_HID_BAR_CODE_SCANNER_CLEAR_ALL_EAN_2_3_LABEL_DEFINITIONS        = 0xC0,
    /* Reserved                                                         = 0xC1-0xC2 */
    USB_HID_BAR_CODE_SCANNER_CODABAR                                    = 0xC3,
    USB_HID_BAR_CODE_SCANNER_CODE_128                                   = 0xC4,
    /* Reserved                                                         = 0xC5-0xC6 */
    USB_HID_BAR_CODE_SCANNER_CODE_39                                    = 0xC7,
    USB_HID_BAR_CODE_SCANNER_CODE_93                                    = 0xC8,
    USB_HID_BAR_CODE_SCANNER_FULL_ASCII_CONVERSION                      = 0xC9,
    USB_HID_BAR_CODE_SCANNER_INTERLEAVED_2_OF_5                         = 0xCA,
    USB_HID_BAR_CODE_SCANNER_ITALIAN_PHARMACY_CODE                      = 0xCB,
    USB_HID_BAR_CODE_SCANNER_MSI_PLESSEY                                = 0xCC,
    USB_HID_BAR_CODE_SCANNER_STANDARD_2_OF_5_IATA                       = 0xCD,
    USB_HID_BAR_CODE_SCANNER_STANDARD_2_of_5                            = 0xCE,
    /* Reserved                                                         = 0xCF-0xD2 */
    USB_HID_BAR_CODE_SCANNER_TRANSMIT_START_STOP                        = 0xD3,
    USB_HID_BAR_CODE_SCANNER_TRI_OPTIC                                  = 0xD4,
    USB_HID_BAR_CODE_SCANNER_UCC_EAN_128                                = 0xD5,
    USB_HID_BAR_CODE_SCANNER_CHECK_DIGIT                                = 0xD6,
    USB_HID_BAR_CODE_SCANNER_CHECK_DIGIT_DISABLE                        = 0xD7,
    USB_HID_BAR_CODE_SCANNER_CHECK_DIGIT_ENABLE_INTERLEAVED_2_OF_5_OPCC = 0xD8,
    USB_HID_BAR_CODE_SCANNER_CHECK_DIGIT_ENABLE_INTERLEAVED_2_OF_5_USS  = 0xD9,
    USB_HID_BAR_CODE_SCANNER_CHECK_DIGIT_ENABLE_STANDARD_2_OF_5_OPCC    = 0xDA,
    USB_HID_BAR_CODE_SCANNER_CHECK_DIGIT_ENABLE_STANDARD_2_OF_5_USS     = 0xDB,
    USB_HID_BAR_CODE_SCANNER_CHECK_DIGIT_ENABLE_ONE_MSI_PLESSEY         = 0xDC,
    USB_HID_BAR_CODE_SCANNER_CHECK_DIGIT_ENABLE_TWO_MSI_PLESSEY         = 0xDD,
    USB_HID_BAR_CODE_SCANNER_CHECK_DIGIT_CODABAR_ENABLE                 = 0xDE,
    USB_HID_BAR_CODE_SCANNER_CHECK_DIGIT_CODE_39_ENABLE                 = 0xDF,
    /* Reserved                                                         = 0xE0-0xEF */
    USB_HID_BAR_CODE_SCANNER_TRANSMIT_CHECK_DIGIT                       = 0xF0,
    USB_HID_BAR_CODE_SCANNER_DISABLE_CHECK_DIGIT_TRANSMIT               = 0xF1,
    USB_HID_BAR_CODE_SCANNER_ENABLE_CHECK_DIGIT_TRANSMIT                = 0xF2,
    /* Reserved                                                         = 0xF3-0xFA */
    USB_HID_BAR_CODE_SCANNER_SYMBOLOGY_IDENTIFIER_1                     = 0xFB,
    USB_HID_BAR_CODE_SCANNER_SYMBOLOGY_IDENTIFIER_2                     = 0xFC,
    USB_HID_BAR_CODE_SCANNER_SYMBOLOTY_IDNETIFIER_3                     = 0xFD,
    USB_HID_BAR_CODE_SCANNER_DECODED_DATA                               = 0xFE,
    USB_HID_BAR_CODE_SCANNER_DECODE_DATA_CONTINUED                      = 0xFF,
    USB_HID_BAR_CODE_SCANNER_BAR_SPACE_DATA                             = 0x100,
    USB_HID_BAR_CODE_SCANNER_SCANNER_DATA_ACCURACY                      = 0x101,
    USB_HID_BAR_CODE_SCANNER_RAW_DATA_POLARITY                          = 0x102,
    USB_HID_BAR_CODE_SCANNER_POLARITY_INVERTED_BAR_CODE                 = 0x103,
    USB_HID_BAR_CODE_SCANNER_POLARITY_NORMAL_BAR_CODE                   = 0x104,
    /* Reserved                                                         = 0x105 */
    USB_HID_BAR_CODE_SCANNER_MINIMUM_LENGTH_TO_DECODE                   = 0x106,
    USB_HID_BAR_CODE_SCANNER_MAXIMUM_LENGTH_TO_DECODE                   = 0x107,
    USB_HID_BAR_CODE_SCANNER_FIRST_DISCRETE_LENGTH_TO_DECODE            = 0x108,
    USB_HID_BAR_CODE_SCANNER_SECOND_DISCRETE_LENGTH_TO_DECODE           = 0x109,
    USB_HID_BAR_CODE_SCANNER_DATA_LENGTH_METHOD                         = 0x10A,
    USB_HID_BAR_CODE_SCANNER_DL_METHOD_READ_ANY                         = 0x10B,
    USB_HID_BAR_CODE_SCANNER_DL_METHOD_CHECK_IN_RANGE                   = 0x10C,
    USB_HID_BAR_CODE_SCANNER_DL_METHOD_CHECK_FOR_DISCRETE               = 0x10D,
    /* Reserved                                                         = 0x10E-0x10F */
    USB_HID_BAR_CODE_SCANNER_AZTEC_CODE                                 = 0x110,
    USB_HID_BAR_CODE_SCANNER_BC412                                      = 0x111,
    USB_HID_BAR_CODE_SCANNER_CHANNEL_CODE                               = 0x112,
    USB_HID_BAR_CODE_SCANNER_CODE_16                                    = 0x113,
    USB_HID_BAR_CODE_SCANNER_CODE_32                                    = 0x114,
    USB_HID_BAR_CODE_SCANNER_CODE_49                                    = 0x115,
    USB_HID_BAR_CODE_SCANNER_CODE_ONE                                   = 0x116,
    USB_HID_BAR_CODE_SCANNER_COLORCODE                                  = 0x117,
    USB_HID_BAR_CODE_SCANNER_DATA_MATRIX                                = 0x118,
    USB_HID_BAR_CODE_SCANNER_MAXICODE                                   = 0x119,
    USB_HID_BAR_CODE_SCANNER_MICROPDF                                   = 0x11A,
    USB_HID_BAR_CODE_SCANNER_PDF417                                     = 0x11B,
    USB_HID_BAR_CODE_SCANNER_POSICODE                                   = 0x11C,
    USB_HID_BAR_CODE_SCANNER_QR_CODE                                    = 0x11D,
    USB_HID_BAR_CODE_SCANNER_SUPERCODE                                  = 0x11E,
    USB_HID_BAR_CODE_SCANNER_ULTRA_CODE                                 = 0x11F,
    USB_HID_BAR_CODE_SCANNER_USD5_SLUG_CODE                             = 0x120,
    USB_HID_BAR_CODE_SCANNER_VERICODE                                   = 0x121
    /* Reserved                                                         = 0x122-0xFFFF */
} USB_HID_BAR_CODE_SCANNER;

typedef enum
{
    USB_HID_WEIGHING_DEVICES_UNDEFINED                                  = 0x00,
    USB_HID_WEIGHING_DEVICES_WEIGHING_DEVICES                           = 0x01,
    /* Reserved                                                         = 0x02-0x1F */
    USB_HID_WEIGHING_DEVICES_SCALE_DEVICE                               = 0x20,
    USB_HID_WEIGHING_DEVICES_SCALE_CLASS_I_METRIC                       = 0x21,
    USB_HID_WEIGHING_DEVICES_SCALE_CLASS_I_METRIC_SEL                   = 0x22,
    USB_HID_WEIGHING_DEVICES_SCALE_CLASS_II_METRIC                      = 0x23,
    USB_HID_WEIGHING_DEVICES_SCALE_CLASS_III_METRIC                     = 0x24,
    USB_HID_WEIGHING_DEVICES_SCALE_CLASS_IIIL_METRIC                    = 0x25,
    USB_HID_WEIGHING_DEVICES_SCALE_CLASS_IV_METRIC                      = 0x26,
    USB_HID_WEIGHING_DEVICES_SCALE_CLASS_III_ENGLISH                    = 0x27,
    USB_HID_WEIGHING_DEVICES_SCALE_CLASS_IIIL_ENGLISH                   = 0x28,
    USB_HID_WEIGHING_DEVICES_SCALE_CLASS_IV_ENGLISH                     = 0x29,
    USB_HID_WEIGHING_DEVICES_SCALE_CLASS_GENERIC                        = 0x2A,
    /* Reserved                                                         = 0x2B-0x2F */
    USB_HID_WEIGHING_DEVICES_SCALE_ATTRIBUTE_REPORT                     = 0x30,
    USB_HID_WEIGHING_DEVICES_SCALE_CONTROL_REPORT                       = 0x31,
    USB_HID_WEIGHING_DEVICES_SCALE_DATA_REPORT                          = 0x32,
    USB_HID_WEIGHING_DEVICES_SCALE_STATUS_REPORT                        = 0x33,
    USB_HID_WEIGHING_DEVICES_SCALE_WEIGHT_LIMIT_REPORT                  = 0x34,
    USB_HID_WEIGHING_DEVICES_SCALE_STATISTICS_REPORT                    = 0x35,
    /* Reserved                                                         = 0x36-0x3F */
    USB_HID_WEIGHING_DEVICES_DATA_WEIGHT                                = 0x40,
    USB_HID_WEIGHING_DEVICES_DATA_SCALING                               = 0x41,
    /* Reserved                                                         = 0x42-0x4F */
    USB_HID_WEIGHING_DEVICES_WEIGHT_UNIT                                = 0x50,
    USB_HID_WEIGHING_DEVICES_WEIGHT_UNIT_MILLIGRAM                      = 0x51,
    USB_HID_WEIGHING_DEVICES_WEIGHT_UNIT_GRAM                           = 0x52,
    USB_HID_WEIGHING_DEVICES_WEIGHT_UNIT_KILOGRAM                       = 0x53,
    USB_HID_WEIGHING_DEVICES_WEIGHT_UNIT_CARATS                         = 0x54,
    USB_HID_WEIGHING_DEVICES_WEIGHT_UNIT_TAELS                          = 0x55,
    USB_HID_WEIGHING_DEVICES_WEIGHT_UNIT_GRAINS                         = 0x56,
    USB_HID_WEIGHING_DEVICES_WEIGHT_UNIT_PENNYWEIGHTS                   = 0x57,
    USB_HID_WEIGHING_DEVICES_WEIGHT_UNIT_METRIC_TON                     = 0x58,
    USB_HID_WEIGHING_DEVICES_WEIGHT_UNIT_AVOIR_TON                      = 0x59,
    USB_HID_WEIGHING_DEVICES_WEIGHT_UNIT_TROY_OUNCE                     = 0x5A,
    USB_HID_WEIGHING_DEVICES_WEIGHT_UNIT_OUNCE                          = 0x5B,
    USB_HID_WEIGHING_DEVICES_WEIGHT_UNIT_POUND                          = 0x5C,
    /* Reserved                                                         = 0x5D-0x5F */
    USB_HID_WEIGHING_DEVICES_CALIBRATION_COUNT                          = 0x60,
    USB_HID_WEIGHING_DEVICES_REZERO_COUNT                               = 0x61,
    /* Reserved                                                         = 0x62-0x6F */
    USB_HID_WEIGHING_DEVICES_SCALE_STATUS                               = 0x70,
    USB_HID_WEIGHING_DEVICES_SCALE_STATUS_FAULT                         = 0x71,
    USB_HID_WEIGHING_DEVICES_SCALE_STATUS_STABLE_AT_CENTER_OF_ZERO      = 0x72,
    USB_HID_WEIGHING_DEVICES_SCALE_STATUS_IN_MOTION                     = 0x73,
    USB_HID_WEIGHING_DEVICES_SCALE_STATUS_WEIGHT_STABLE                 = 0x74,
    USB_HID_WEIGHING_DEVICES_SCALE_STATUS_UNDER_ZERO                    = 0x75,
    USB_HID_WEIGHING_DEVICES_SCALE_STATUS_OVER_WEIGHT_LIMIT             = 0x76,
    USB_HID_WEIGHING_DEVICES_SCALE_STATUS_REQUIRES_CALIBRATION          = 0x77,
    USB_HID_WEIGHING_DEVICES_SCALE_STATUS_REQUIRES_REZEROING            = 0x78,
    /* Reserved                                                         = 0x79-0x7F */
    USB_HID_WEIGHING_DEVICES_ZERO_SCALE                                 = 0x80,
    USB_HID_WEIGHING_DEVICES_ENFORCED_ZERO_RETURN                       = 0x81
    /* Reserved                                                         = 0x82-0xFFFF */
} USB_HID_WEIGHING_DEVICES;

typedef enum
{
    USB_HID_MAGNETIC_STRIPE_READING_DEVICES_UNDEFINED                   = 0x00,
    USB_HID_MAGNETIC_STRIPE_READING_DEVICES_MSR_DEVICE_READ_ONLY        = 0x01,
    /* Reserved                                                         = 0x02-0x10 */
    USB_HID_MAGNETIC_STRIPE_READING_DEVICES_TRACK_1_LENGTH              = 0x11,
    USB_HID_MAGNETIC_STRIPE_READING_DEVICES_TRACK_2_LENGTH              = 0x12,
    USB_HID_MAGNETIC_STRIPE_READING_DEVICES_TRACK_3_LENGTH              = 0x13,
    USB_HID_MAGNETIC_STRIPE_READING_DEVICES_TRACK_JIS_LENGTH            = 0x14,
    /* Reserved                                                         = 0x15-0x1F */
    USB_HID_MAGNETIC_STRIPE_READING_DEVICES_TRACK_DATA                  = 0x20,
    USB_HID_MAGNETIC_STRIPE_READING_DEVICES_TRACK_1_DATA                = 0x21,
    USB_HID_MAGNETIC_STRIPE_READING_DEVICES_TRACK_2_DATA                = 0x22,
    USB_HID_MAGNETIC_STRIPE_READING_DEVICES_TRACK_3_DATA                = 0x23,
    USB_HID_MAGNETIC_STRIPE_READING_DEVICES_TRACK_JIS_DATA              = 0x24
    /* Reserved                                                         = 0x25-0xFFFF */
} USB_HID_MAGNETIC_STRIPE_READING_DEVICES;

typedef enum
{
    USB_HID_POWER_DEVICE_UNDEFINED                                      = 0x00,
    USB_HID_POWER_DEVICE_INAME                                          = 0x01,
    USB_HID_POWER_DEVICE_PRESENT_STATUS                                 = 0x02,
    USB_HID_POWER_DEVICE_CHANGED_STATUS                                 = 0x03,
    USB_HID_POWER_DEVICE_UPS                                            = 0x04,
    USB_HID_POWER_DEVICE_POWER_SUPPLY                                   = 0x05,
    /* Reserved                                                         = 0x06-0x0F */
    USB_HID_POWER_DEVICE_BATTERY_SYSTEM                                 = 0x10,
    USB_HID_POWER_DEVICE_BATTERY_SYSTEM_ID                              = 0x11,
    USB_HID_POWER_DEVICE_BATTERY                                        = 0x12,
    USB_HID_POWER_DEVICE_BATTERY_ID                                     = 0x13,
    USB_HID_POWER_DEVICE_CHARGER                                        = 0x14,
    USB_HID_POWER_DEVICE_CHARGER_ID                                     = 0x15,
    USB_HID_POWER_DEVICE_POWER_CONVERTER                                = 0x16,
    USB_HID_POWER_DEVICE_POWER_CONVERTER_ID                             = 0x17,
    USB_HID_POWER_DEVICE_OUTLET_SYSTEM                                  = 0x18,
    USB_HID_POWER_DEVICE_OUTLET_SYSTEM_ID                               = 0x19,
    USB_HID_POWER_DEVICE_INPUT                                          = 0x1A,
    USB_HID_POWER_DEVICE_INPUT_ID                                       = 0x1B,
    USB_HID_POWER_DEVICE_OUTPUT                                         = 0x1C,
    USB_HID_POWER_DEVICE_OUTPUT_ID                                      = 0x1D,
    USB_HID_POWER_DEVICE_FLOW                                           = 0x1E,
    USB_HID_POWER_DEVICE_FLOW_ID                                        = 0x1F,
    USB_HID_POWER_DEVICE_OUTLET                                         = 0x20,
    USB_HID_POWER_DEVICE_OUTLET_ID                                      = 0x21,
    USB_HID_POWER_DEVICE_GANG                                           = 0x22,
    USB_HID_POWER_DEVICE_GANG_ID                                        = 0x23,
    USB_HID_POWER_DEVICE_POWER_SUMMARY                                  = 0x24,
    USB_HID_POWER_DEVICE_POWER_SUMMARY_ID                               = 0x25,
    /* Reserved                                                         = 0x26-0x2F */
    USB_HID_POWER_DEVICE_VOLTAGE                                        = 0x30,
    USB_HID_POWER_DEVICE_CURRENT                                        = 0x31,
    USB_HID_POWER_DEVICE_FREQUENCY                                      = 0x32,
    USB_HID_POWER_DEVICE_APPARENT_POWER                                 = 0x33,
    USB_HID_POWER_DEVICE_ACTIVE_POWER                                   = 0x34,
    USB_HID_POWER_DEVICE_PERCENT_LOAD                                   = 0x35,
    USB_HID_POWER_DEVICE_TEMPERATURE                                    = 0x36,
    USB_HID_POWER_DEVICE_HUMIDITY                                       = 0x37,
    USB_HID_POWER_DEVICE_BAD_COUNT                                      = 0x38,
    /* Reserved                                                         = 0x39-0x3F */
    USB_HID_POWER_DEVICE_CONFIG_VOLTAGE                                 = 0x40,
    USB_HID_POWER_DEVICE_CONFIG_CURRENT                                 = 0x41,
    USB_HID_POWER_DEVICE_CONFIG_FREQUENCY                               = 0x42,
    USB_HID_POWER_DEVICE_CONFIG_APPARENT_POWER                          = 0x43,
    USB_HID_POWER_DEVICE_CONFIG_ACTIVE_POWER                            = 0x44,
    USB_HID_POWER_DEVICE_CONFIG_PERCENT_LOAD                            = 0x45,
    USB_HID_POWER_DEVICE_CONFIG_TEMPERATURE                             = 0x46,
    USB_HID_POWER_DEVICE_CONFIG_HUMIDITY                                = 0x47,
    /* Reserved                                                         = 0x48-0x4F */
    USB_HID_POWER_DEVICE_SWITCH_ON_CONTROL                              = 0x50,
    USB_HID_POWER_DEVICE_SWITCH_OFF_CONTROL                             = 0x51,
    USB_HID_POWER_DEVICE_TOGGLE_CONTROL                                 = 0x52,
    USB_HID_POWER_DEVICE_LOW_VOLTAGE_TRANSFER                           = 0x53,
    USB_HID_POWER_DEVICE_HIGH_VOLTAGE_TRANSFER                          = 0x54,
    USB_HID_POWER_DEVICE_DELAY_BEFORE_REBOOT                            = 0x55,
    USB_HID_POWER_DEVICE_DELAY_BEFORE_STARTUP                           = 0x56,
    USB_HID_POWER_DEVICE_DELAY_BEFORE_SHUTDOWN                          = 0x57,
    USB_HID_POWER_DEVICE_TEST                                           = 0x58,
    USB_HID_POWER_DEVICE_MODULE_RESET                                   = 0x59,
    USB_HID_POWER_DEVICE_AUDIBLE_ALARM_CONTROL                          = 0x5A,
    /* Reserved                                                         = 0x5B-0x5F */
    USB_HID_POWER_DEVICE_PRESENT                                        = 0x60,
    USB_HID_POWER_DEVICE_GOOD                                           = 0x61,
    USB_HID_POWER_DEVICE_INTERNAL_FAILURE                               = 0x62,
    USB_HID_POWER_DEVICE_VOLTAGE_OUT_OF_RANGE                           = 0x63,
    USB_HID_POWER_DEVICE_FREQUENCY_OUT_OF_RANGE                         = 0x64,
    USB_HID_POWER_DEVICE_OVERLOAD                                       = 0x65,
    USB_HID_POWER_DEVICE_OVER_CHARGED                                   = 0x66,
    USB_HID_POWER_DEVICE_OVER_TEMPERATURE                               = 0x67,
    USB_HID_POWER_DEVICE_SHUTDOWN_REQUESTED                             = 0x68,
    USB_HID_POWER_DEVICE_SHUTDOWN_IMMINENT                              = 0x69,
    /* Reserved                                                         = 0x6A */
    USB_HID_POWER_DEVICE_SWITCH_ON_OFF                                  = 0x6B,
    USB_HID_POWER_DEVICE_SWITCHABLE                                     = 0x6C,
    USB_HID_POWER_DEVICE_USED                                           = 0x6D,
    USB_HID_POWER_DEVICE_BOOST                                          = 0x6E,
    USB_HID_POWER_DEVICE_BUCK                                           = 0x6F,
    USB_HID_POWER_DEVICE_INITIALIZED                                    = 0x70,
    USB_HID_POWER_DEVICE_TESTED                                         = 0x71,
    USB_HID_POWER_DEVICE_AWAITING_POWER                                 = 0x72,
    USB_HID_POWER_DEVICE_COMMUNICATION_LOST                             = 0x73,
    /* Reserved                                                         = 0x74-0xFC */
    USB_HID_POWER_DEVICE_IMANUFACTURER                                  = 0xFD,
    USB_HID_POWER_DEVICE_IPRODUCT                                       = 0xFE,
    USB_HID_POWER_DEVICE_ISERIAL_NUMBER                                 = 0xFF
} USB_HID_POWER_DEVICE;

typedef enum
{
    USB_HID_BATTERY_SYSTEM_UNDEFINED                                    = 0x00,
    USB_HID_BATTERY_SYSTEM_SMB_BATTERY_MODE                             = 0x01,
    USB_HID_BATTERY_SYSTEM_SMB_BATTERY_STATUS                           = 0x02,
    USB_HID_BATTERY_SYSTEM_SMB_ALARM_WARNING                            = 0x03,
    USB_HID_BATTERY_SYSTEM_SMB_CHARGER_MODE                             = 0x04,
    USB_HID_BATTERY_SYSTEM_SMB_CHARGER_STATUS                           = 0x05,
    USB_HID_BATTERY_SYSTEM_SMB_CHARGER_SPEC_INFO                        = 0x06,
    USB_HID_BATTERY_SYSTEM_SMB_SELECTOR_STATE                           = 0x07,
    USB_HID_BATTERY_SYSTEM_SMB_SELECTOR_PRESETS                         = 0x08,
    USB_HID_BATTERY_SYSTEM_SMB_SELECTOR_INFO                            = 0x09,
    /* Reserved                                                         = 0x0A-0x0F */
    USB_HID_BATTERY_SYSTEM_OPTIONAL_MFG_FUNCTION_1                      = 0x10,
    USB_HID_BATTERY_SYSTEM_OPTIONAL_MFG_FUNCTION_2                      = 0x11,
    USB_HID_BATTERY_SYSTEM_OPTIONAL_MFG_FUNCTION_3                      = 0x12,
    USB_HID_BATTERY_SYSTEM_OPTIONAL_MFG_FUNCTION_4                      = 0x13,
    USB_HID_BATTERY_SYSTEM_OPTIONAL_MFG_FUNCTION_5                      = 0x14,
    USB_HID_BATTERY_SYSTEM_CONNECTION_TO_SMBUS                          = 0x15,
    USB_HID_BATTERY_SYSTEM_OUTPUT_CONNECTION                            = 0x16,
    USB_HID_BATTERY_SYSTEM_CHARGER_CONNECTION                           = 0x17,
    USB_HID_BATTERY_SYSTEM_BATTERY_INSERTION                            = 0x18,
    USB_HID_BATTERY_SYSTEM_USENEXT                                      = 0x19,
    USB_HID_BATTERY_SYSTEM_OK_TO_USE                                    = 0x1A,
    USB_HID_BATTERY_SYSTEM_BATTERY_SUPPORTED                            = 0x1B,
    USB_HID_BATTERY_SYSTEM_SELECTOR_REVISION                            = 0x1C,
    USB_HID_BATTERY_SYSTEM_CHARGING_INDICATOR                           = 0x1D,
    /* Reserved                                                         = 0x1E-0x27 */
    USB_HID_BATTERY_SYSTEM_MANUFACTURER_ACCESS                          = 0x28,
    USB_HID_BATTERY_SYSTEM_REMAINING_CAPACITY_LIMIT                     = 0x29,
    USB_HID_BATTERY_SYSTEM_REMAINING_TIME_LIMIT                         = 0x2A,
    USB_HID_BATTERY_SYSTEM_AT_RATE                                      = 0x2B,
    USB_HID_BATTERY_SYSTEM_CAPACITY_MODE                                = 0x2C,
    USB_HID_BATTERY_SYSTEM_BROADCAST_TO_CHARGER                         = 0x2D,
    USB_HID_BATTERY_SYSTEM_PRIMARY_BATTERY                              = 0x2E,
    USB_HID_BATTERY_SYSTEM_CHARGE_CONTROLLER                            = 0x2F,
    /* Reserved                                                         = 0x30-0x3F */
    USB_HID_BATTERY_SYSTEM_TERMINATE_CHARGE                             = 0x40,
    USB_HID_BATTERY_SYSTEM_TERMINATE_DISCHARGE                          = 0x41,
    USB_HID_BATTERY_SYSTEM_BELOW_REMAINING_CAPACITY_LIMIT               = 0x42,
    USB_HID_BATTERY_SYSTEM_REMAINING_TIME_LIMIT_EXPIRED                 = 0x43,
    USB_HID_BATTERY_SYSTEM_CHARGING                                     = 0x44,
    USB_HID_BATTERY_SYSTEM_DISCHARGING                                  = 0x45,
    USB_HID_BATTERY_SYSTEM_FULLY_CHARGED                                = 0x46,
    USB_HID_BATTERY_SYSTEM_FULLY_DISCHARGED                             = 0x47,
    USB_HID_BATTERY_SYSTEM_CONDITIONING_FLAG                            = 0x48,
    USB_HID_BATTERY_SYSTEM_AT_RATE_OK                                   = 0x49,
    USB_HID_BATTERY_SYSTEM_SMB_ERROR_CODE                               = 0x4A,
    USB_HID_BATTERY_SYSTEM_NEED_REPLACEMENT                             = 0x4B,
    /* Reserved                                                         = 0x4C-0x5F */
    USB_HID_BATTERY_SYSTEM_AT_RATE_TIME_TO_FULL                         = 0x60,
    USB_HID_BATTERY_SYSTEM_AT_RATE_TIME_TO_EMPTY                        = 0x61,
    USB_HID_BATTERY_SYSTEM_AVERAGE_CURRENT                              = 0x62,
    USB_HID_BATTERY_SYSTEM_MAXERROR                                     = 0x63,
    USB_HID_BATTERY_SYSTEM_RELATIVE_STATE_OF_CHARGE                     = 0x64,
    USB_HID_BATTERY_SYSTEM_ABSOLUTE_STATE_OF_CHARGE                     = 0x65,
    USB_HID_BATTERY_SYSTEM_REMAINING_CAPACITY                           = 0x66,
    USB_HID_BATTERY_SYSTEM_FULL_CHARGE_CAPACITY                         = 0x67,
    USB_HID_BATTERY_SYSTEM_RUN_TIME_TO_EMPTY                            = 0x68,
    USB_HID_BATTERY_SYSTEM_AVERAGE_TIME_TO_EMPTY                        = 0x69,
    USB_HID_BATTERY_SYSTEM_AVERAGE_TIME_TO_FULL                         = 0x6A,
    USB_HID_BATTERY_SYSTEM_CYCLE_COUNT                                  = 0x6B,
    /* Reserved                                                         = 0x6C-0x7F */
    USB_HID_BATTERY_SYSTEM_BATT_PACK_MODEL_LEVEL                        = 0x80,
    USB_HID_BATTERY_SYSTEM_INTERNAL_CHARGE_CONTROLLER                   = 0x81,
    USB_HID_BATTERY_SYSTEM_PRIMARY_BATTERY_SUPPORT                      = 0x82,
    USB_HID_BATTERY_SYSTEM_DESIGN_CAPACITY                              = 0x83,
    USB_HID_BATTERY_SYSTEM_SPECIFICATION_INFO                           = 0x84,
    USB_HID_BATTERY_SYSTEM_MANUFACTURER_DATE                            = 0x85,
    USB_HID_BATTERY_SYSTEM_SERIAL_NUMBER                                = 0x86,
    USB_HID_BATTERY_SYSTEM_IMANUFACTURER_NAME                           = 0x87,
    USB_HID_BATTERY_SYSTEM_IDEVICE_NAME                                 = 0x88,
    USB_HID_BATTERY_SYSTEM_IDEVICE_CHEMISTERY                           = 0x89,
    USB_HID_BATTERY_SYSTEM_MANUFACTURER_DATA                            = 0x8A,
    USB_HID_BATTERY_SYSTEM_RECHARGABLE                                  = 0x8B,
    USB_HID_BATTERY_SYSTEM_WARNING_CAPACITY_LIMIT                       = 0x8C,
    USB_HID_BATTERY_SYSTEM_CAPACITY_GRANULARITY_1                       = 0x8D,
    USB_HID_BATTERY_SYSTEM_CAPACITY_GRANULARITY_2                       = 0x8E,
    USB_HID_BATTERY_SYSTEM_IOEM_INFORMATION                             = 0x8F,
    /* Reserved                                                         = 0x90-0xBF */
    USB_HID_BATTERY_SYSTEM_INHIBIT_CHARGE                               = 0xC0,
    USB_HID_BATTERY_SYSTEM_ENABLE_POLLING                               = 0xC1,
    USB_HID_BATTERY_SYSTEM_RESET_TO_ZERO                                = 0xC2,
    /* Reserved                                                         = 0xC3-0xCF */
    USB_HID_BATTERY_SYSTEM_AC_PRESENT                                   = 0xD0,
    USB_HID_BATTERY_SYSTEM_BATTERY_PRESENT                              = 0xD1,
    USB_HID_BATTERY_SYSTEM_POWER_FAIL                                   = 0xD2,
    USB_HID_BATTERY_SYSTEM_ALARM_INHIBITED                              = 0xD3,
    USB_HID_BATTERY_SYSTEM_THERMISTOR_UNDER_RANGE                       = 0xD4,
    USB_HID_BATTERY_SYSTEM_THERMISTOR_HOT                               = 0xD5,
    USB_HID_BATTERY_SYSTEM_THERMISTOR_COLD                              = 0xD6,
    USB_HID_BATTERY_SYSTEM_THERMISTOR_OVER_RANGE                        = 0xD7,
    USB_HID_BATTERY_SYSTEM_VOLTAGE_OUT_OF_RANGE                         = 0xD8,
    USB_HID_BATTERY_SYSTEM_CURRENT_OUT_OF_RANGE                         = 0xD9,
    USB_HID_BATTERY_SYSTEM_CURRENT_NOT_REGULATED                        = 0xDA,
    USB_HID_BATTERY_SYSTEM_VOLTAGE_NOT_REGULATED                        = 0xDB,
    USB_HID_BATTERY_SYSTEM_MASTER_MODE                                  = 0xDC,
    /* Reserved                                                         = 0xDD-0xEF */
    USB_HID_BATTERY_SYSTEM_CHARGER_SELECTOR_SUPPORT                     = 0xF0,
    USB_HID_BATTERY_SYSTEM_CHARGER_SPEC                                 = 0xF1,
    USB_HID_BATTERY_SYSTEM_LEVEL_2                                      = 0xF2,
    USB_HID_BATTERY_SYSTEM_LEVEL_3                                      = 0xF3
    /* Reserved                                                         = 0xF2-0xFF */
} USB_HID_BATTERY_SYSTEM;

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif
