/*******************************************************************************
  USB Audio class definitions

  Company:
    Microchip Technology Inc.

  File Name:
    usb_audio_v2_0.h

  Summary:
    USB Audio class definitions

  Description:
    This file describes the Audio v2.0 class specific definitions.
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

#ifndef _USB_AUDIO_v2_0_H_
#define _USB_AUDIO_v2_0_H_

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
/* A.1 Audio Function Class code */
#define  USB_AUDIO_V2_CLASS_CODE USB_AUDIO_V2

/* A.4 Audio Interface class code */
#define USB_AUDIO_V2 0x01

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************
/* Appendix A. Audio Device Class Codes from the USB Audio 2.0 specification */

/*A.3 Audio Function Protocol code*/
typedef enum
{
    USB_AUDIO_V2_PR_PROTOCOL_UNDEFINED  = 0x0,
    USB_AUDIO_V2_AF_VERSION_02_00       = 0x20
} USB_AUDIO_V2_PROTOCOL_CODE;

/*A.5 Audio Interface Subclass code*/
typedef enum
{
    USB_AUDIO_V2_SUBCLASS_UNDEFINED = 0x00,
    USB_AUDIO_V2_AUDIOCONTROL       = 0x01,
    USB_AUDIO_V2_AUDIOSTREAMING     = 0x02,
    USB_AUDIO_V2_MIDISTREAMING      = 0x03
} USB_AUDIO_V2_SUBCLASS_CODE, USB_AUDIO_V2_INTERFACE_SUBCLASS_CODE; 

/*A.6 Audio Interface Protocol code*/
typedef enum
{
    USB_AUDIO_V2_INTERFACE_PROTOCOL_UNDEFINED   = 0x00,
    USB_AUDIO_V2_IP_VERSION_02_00               = 0x20
} USB_AUDIO_V2_INTERFACE_PROTOCOL_CODE;

/*A.7 Audio Function category codes*/
typedef enum
{
    USB_AUDIO_V2_FUNCTION_SUBCLASS_UNDEFINED    = 0x00,
    USB_AUDIO_V2_DESKTOP_SPEAKER                = 0x01,
    USB_AUDIO_V2_HOME_THEATER                   = 0x02,
    USB_AUDIO_V2_MICROPHONE                     = 0x03,
    USB_AUDIO_V2_HEADSET                        = 0x04,
    USB_AUDIO_V2_TELEPHONE                      = 0x05,
    USB_AUDIO_V2_CONVERTER                      = 0x06,
    USB_AUDIO_V2_VOICE_SOUND_RECORDER           = 0x07,
    USB_AUDIO_V2_IO_BOX                         = 0x08,
    USB_AUDIO_V2_MUSICAL_INSTRUMENT             = 0x09,
    USB_AUDIO_V2_PRO_AUDIO                      = 0x0A,
    USB_AUDIO_V2_AUDIO_VIDEO                    = 0x0B,
    USB_AUDIO_V2_CONTROL_PANEL                  = 0x0C,
    USB_AUDIO_V2_OTHER                          = 0xFF
}USB_AUDIO_V2_FUNCTION_CATEGORY_CODE;

/*A.8 Audio Class-specific Descriptor types*/
typedef enum
{
    USB_AUDIO_V2_CS_UNDEFINED       = 0x20,
    USB_AUDIO_V2_CS_DEVICE          = 0x21,
    USB_AUDIO_V2_CS_CONFIGURATION   = 0x22,
    USB_AUDIO_V2_CS_STRING          = 0x23,
    USB_AUDIO_V2_CS_INTERFACE       = 0x24,
    USB_AUDIO_V2_CS_ENDPOINT        = 0x25

} USB_AUDIO_V2_CS_DESCRIPTOR_TYPE;

/*A.9 Audio Class-Specific AC Interface Descriptor Subtypes*/
typedef enum
{
    USB_AUDIO_V2_AC_DESCRIPTOR_UNDEFINED    = 0x00,
    USB_AUDIO_V2_HEADER                     = 0x01,
    USB_AUDIO_V2_INPUT_TERMINAL             = 0x02,
    USB_AUDIO_V2_OUTPUT_TERMINAL            = 0x03,
    USB_AUDIO_V2_MIXER_UNIT                 = 0x04,
    USB_AUDIO_V2_SELECTOR_UNIT              = 0x05,
    USB_AUDIO_V2_FEATURE_UNIT               = 0x06,
    USB_AUDIO_V2_EFFECT_UNIT                = 0x07,
    USB_AUDIO_V2_PROCESSING_UNIT            = 0x08,
    USB_AUDIO_V2_EXTENSION_UNIT             = 0x09,
    USB_AUDIO_V2_CLOCK_SOURCE               = 0x0A,
    USB_AUDIO_V2_CLOCK_SELECTOR             = 0x0B,
    USB_AUDIO_V2_CLOCK_MULTIPLIER           = 0x0C,
    USB_AUDIO_V2_SAMPLE_RATE_CONVERTER      = 0x0D
} USB_AUDIO_V2_CS_AC_INTERFACE_DESCRIPTOR_SUBTYPE;

/* A.10 Audio Class-Specific AS Interface Descriptor Subtypes*/
typedef enum
{
    USB_AUDIO_V2_AS_DESCRIPTOR_UNDEFINED    = 0x00,
    USB_AUDIO_V2_AS_GENERAL                 = 0x01,
    USB_AUDIO_V2_FORMAT_TYPE                = 0x02,
    USB_AUDIO_V2_ENCODER                    = 0x03,
    USB_AUDIO_V2_DECODER                    = 0x04
} USB_AUDIO_V2_CS_AC_AS_INTERFACE_DESCRIPTOR_SUBTYPE;

/* A.11 Effect Unit Effect Types */
typedef enum
{
    USB_AUDIO_V2_EFFECT_UNDEFINED           = 0x00,
    USB_AUDIO_V2_PARAM_EQ_SECTION_EFFECT    = 0x01,
    USB_AUDIO_V2_REVERBERATION_EFFECT       = 0x02,
    USB_AUDIO_V2_MOD_DELAY_EFFECT           = 0x03,
    USB_AUDIO_V2_DYN_RANGE_COMP_EFFECT      = 0x04
} USB_AUDIO_V2_EFFECT_UNIT_TYPE;

/* A.12 Processing Unit Process Type */
typedef enum
{
    USB_AUDIO_V2_PROCESS_UNDEFINED      = 0x00,
    USB_AUDIO_V2_UP_DOWNMIX_PROCESS     = 0x01,
    USB_AUDIO_V2_DOLBY_PROLOGIC_PROCESS = 0x02,
    USB_AUDIO_V2_STEREO_EXTENDER_PROCESS= 0x03
}USB_AUDIO_V2_PROCESSING_UNIT_PROCESS_TYPE;

/* A.13 Audio Class-Specific Endpoint Descriptor Subtypes */
typedef enum
{
    USB_AUDIO_V2_DESCRIPTOR_UNDEFINED   = 0x00,
    USB_AUDIO_V2_EP_GENERAL             = 0x01
}USB_AUDIO_V2_CS_AC_ENDPOINT_DESCIPTOR_SUBTYPE;

/* A.14  Audio Class-Specific Request Codes */
typedef enum
{
    USB_AUDIO_V2_REQUEST_CODE_UNDEFINED = 0x00,
    USB_AUDIO_V2_CUR                    = 0x01,
    USB_AUDIO_V2_RANGE                  = 0x02,
    USB_AUDIO_V2_MEM                    = 0x03
} USB_AUDIO_V2_CS_AC_REQUEST_CODE;

/* A.15 Encoder type codes */
typedef enum
{
    USB_AUDIO_V2_ENCODER_UNDEFINED  = 0x00,
    USB_AUDIO_V2_OTHER_ENCODER      = 0x01,
    USB_AUDIO_V2_MPEG_ENCODER       = 0x02,
    USB_AUDIO_V2_AC_3_ENCODER       = 0x03,
    USB_AUDIO_V2_WMA_ENCODER        = 0x04,
    USB_AUDIO_V2_DTS_ENCODER        = 0x05
} USB_AUDIO_V2_EBCODER_TYPE_CODE;

/* A.16 Decoder type codes */
typedef enum
{
    USB_AUDIO_V2_DECODER_UNDEFINED  = 0x00,
    USB_AUDIO_V2_OTHER_DECODER      = 0x01,
    USB_AUDIO_V2_MPEG_DECODER       = 0x02,
    USB_AUDIO_V2_AC_3_DECODER       = 0x03,
    USB_AUDIO_V2_WMA_DECODER        = 0x04,
    USB_AUDIO_V2_DTS_DECODER        = 0x05
} USB_AUDIO_V2_DECODER_TYPE_CODE;

/* A.17 Control Selector Codes */
/* A.17.1 Clock Source Control Selectors */
typedef enum
{
    USB_AUDIO_V2_CS_CONTROL_UNDEFINED   = 0x00,
    USB_AUDIO_V2_CS_SAM_FREQ_CONTROL    = 0x01,
    USB_AUDIO_V2_CS_CLOCK_VALID_CONTROL = 0x02
} USB_AUDIO_V2_CLOCK_SOURCE_CONTROL_SELECTOR;

/* A.17.2 Clock Selector Control Selectors */
typedef enum
{
    USB_AUDIO_V2_CX_CONTROL_UNDEFINED       = 0x00,
    USB_AUDIO_V2_CX_CLOCK_SELECTOR_CONTROL  = 0x01
} USB_AUDIO_V2_CLOCK_SELECTOR_CONTROL_SELECTOR;

/* A.17.3 Clock Multiplier Control Selectors */
typedef enum
{
    USB_AUDIO_V2_CM_CONTROL_UNDEFINED   = 0x00,
    USB_AUDIO_V2_CM_NUMERATOR_CONTROL   = 0x01,
    USB_AUDIO_V2_CM_DENOMINATOR_CONTROL = 0x02
} USB_AUDIO_V2_CLOCK_MULTIPLIER_CONTROL_SELECTOR;

/* A.17.4 Terminal Control Selectors */
typedef enum
{
    USB_AUDIO_V2_TE_CONTROL_UNDEFINED       = 0x00,
    USB_AUDIO_V2_TE_COPY_PROTECT_CONTROL    = 0x01,
    USB_AUDIO_V2_TE_CONNECTOR_CONTROL       = 0x02,
    USB_AUDIO_V2_TE_OVERLOAD_CONTROL        = 0x03,
    USB_AUDIO_V2_TE_CLUSTER_CONTROL         = 0x04,
    USB_AUDIO_V2_TE_UNDERFLOW_CONTROL       = 0x05,
    USB_AUDIO_V2_TE_OVERFLOW_CONTROL        = 0x06,
    USB_AUDIO_V2_TE_LATENCY_CONTROL         = 0x07
} USB_AUDIO_V2_TERMINAL_CONTROL_SELECTOR;

/* A.17.5 Mixer Control Selectors */
typedef enum
{
    USB_AUDIO_V2_MU_CONTROL_UNDEFINED   = 0x00,
    USB_AUDIO_V2_MU_MIXER_CONTROL       = 0x01,
    USB_AUDIO_V2_MU_CLUSTER_CONTROL     = 0x02,
    USB_AUDIO_V2_MU_UNDERFLOW_CONTROL   = 0x03,
    USB_AUDIO_V2_MU_OVERFLOW_CONTROL    = 0x04,
    USB_AUDIO_V2_MU_LATENCY_CONTROL     = 0x05
} USB_AUDIO_V2_MIXER_CONTROL_SELECTOR;

/* A.17.6 Selector Control Selectors */
typedef enum
{
    USB_AUDIO_V2_SU_CONTROL_UNDEFINED   = 0x00,
    USB_AUDIO_V2_SU_SELECTOR_CONTROL    = 0x01,
    USB_AUDIO_V2_SU_LATENCY_CONTROL     = 0x02
} USB_AUDIO_V2_SELECTOR_CONTROL_SELECTOR;

/* A.17.7 Feature Unit Control Selectors */
typedef enum
{
    USB_AUDIO_V2_FCS_FU_CONTROL_UNDEFINED       = 0x00,
    USB_AUDIO_V2_FCS_MUTE_CONTROL               = 0x01,
    USB_AUDIO_V2_FCS_VOLUME_CONTROL             = 0x02,
    USB_AUDIO_V2_FCS_BASS_CONTROL               = 0x03,
    USB_AUDIO_V2_FCS_MID_CONTROL                = 0x04,
    USB_AUDIO_V2_FCS_TREBLE_CONTROL             = 0x05,
    USB_AUDIO_V2_FCS_GRAPHIC_EQUALIZER_CONTROL  = 0x06,
    USB_AUDIO_V2_FCS_AUTOMATIC_GAIN_CONTROL     = 0x07,
    USB_AUDIO_V2_FCS_DELAY_CONTROL              = 0x08,
    USB_AUDIO_V2_FCS_BASS_BOOST_CONTROL         = 0x09,
    USB_AUDIO_V2_FCS_LOUDNESS_CONTROL           = 0x0A,
    USB_AUDIO_V2_FCS_INPUT_GAIN_CONTROL         = 0x0B,
    USB_AUDIO_V2_FCS_INPUT_GAIN_PAD_CONTROL     = 0x0C,
    USB_AUDIO_V2_FCS_PHASE_INVERTER_CONTROL     = 0x0D,
    USB_AUDIO_V2_FCS_UNDERFLOW_CONTROL          = 0x0E,
    USB_AUDIO_V2_FCS_OVERFLOW_CONTROL           = 0x0F,
    USB_AUDIO_V2_FCS_LATENCY_CONTROL            = 0x10
}USB_AUDIO_V2_FEATURE_UNIT_CONTROL_SELECTORS;

/* A.17.8 Effect Unit Control Selectors */
/* A.17.8.1 Parametric Equalizer Section Unit Control Selectors */
typedef enum
{
    USB_AUDIO_V2_PE_CONTROL_UNDEFINED   = 0x00,
    USB_AUDIO_V2_PE_ENABLE_CONTROL      = 0x01,
    USB_AUDIO_V2_PE_CENTERFREQ_CONTROL  = 0x02,
    USB_AUDIO_V2_PE_QFACTOR_CONTROL     = 0x03,
    USB_AUDIO_V2_PE_GAIN_CONTROL        = 0x04,
    USB_AUDIO_V2_PE_UNDERFLOW_CONTROL   = 0x05,
    USB_AUDIO_V2_PE_OVERFLOW_CONTROL    = 0x06,
    USB_AUDIO_V2_PE_LATENCY_CONTROL     = 0x07
} USB_AUDIO_V2_EFFECT_UNIT_PE_CONTROL_SELECTOR;

/* A.17.8.2 Reverberation Effect Unit Control Selector */
typedef enum
{
    USB_AUDIO_V2_RV_CONTROL_UNDEFINED       = 0x00,
    USB_AUDIO_V2_RV_ENABLE_CONTROL          = 0x01,
    USB_AUDIO_V2_RV_TYPE_CONTROL            = 0x02,
    USB_AUDIO_V2_RV_LEVEL_CONTROL           = 0x03,
    USB_AUDIO_V2_RV_TIME_CONTROL            = 0x04,
    USB_AUDIO_V2_RV_FEEDBACK_CONTROL        = 0x05,
    USB_AUDIO_V2_RV_PREDELAY_CONTROL        = 0x06,
    USB_AUDIO_V2_RV_DENSITY_CONTROL         = 0x07,
    USB_AUDIO_V2_RV_HIFREQ_ROLLOFF_CONTROL  = 0x08,
    USB_AUDIO_V2_RV_UNDERFLOW_CONTROL       = 0x09,
    USB_AUDIO_V2_RV_OVERFLOW_CONTROL        = 0x0A,
    USB_AUDIO_V2_RV_LATENCY_CONTROL         = 0x0B
} USB_AUDIO_V2_EFFECT_UNIT_REVERBERATION_CONTROL_SELECTOR;

/* A.17.8.3 Modulation Delay Effect Unit Control Selectors */
typedef enum
{
    USB_AUDIO_V2_MD_CONTROL_UNDEFINED   = 0x00,
    USB_AUDIO_V2_MD_ENABLE_CONTROL      = 0x01,
    USB_AUDIO_V2_MD_BALANCE_CONTROL     = 0x02,
    USB_AUDIO_V2_MD_RATE_CONTROL        = 0x03,
    USB_AUDIO_V2_MD_DEPTH_CONTROL       = 0x04,
    USB_AUDIO_V2_MD_TIME_CONTROL        = 0x05,
    USB_AUDIO_V2_MD_FEEDBACK_CONTROL    = 0x06,
    USB_AUDIO_V2_MD_UNDERFLOW_CONTROL   = 0x07,
    USB_AUDIO_V2_MD_OVERFLOW_CONTROL    = 0x08,
    USB_AUDIO_V2_MD_LATENCY_CONTROL     = 0x09        
} USB_AUDIO_V2_EFFECT_UNIT_MODULATION_DELAY_CONTROL_SELECTOR;

/* A.17.8.4 Dynamic Range Compressor Effect Unit Control Selector */
typedef enum
{
    USB_AUDIO_V2_DR_CONTROL_UNDEFINED           = 0x00,
    USB_AUDIO_V2_DR_ENABLE_CONTROL              = 0x01,
    USB_AUDIO_V2_DR_COMPRESSION_RATE_CONTROL    = 0x02,
    USB_AUDIO_V2_DR_MAXAMPL_CONTROL             = 0x03,
    USB_AUDIO_V2_DR_THRESHOLD_CONTROL           = 0x04,
    USB_AUDIO_V2_DR_ATTACK_TIME_CONTROL         = 0x05,
    USB_AUDIO_V2_DR_RELEASE_TIME_CONTROL        = 0x06,
    USB_AUDIO_V2_DR_UNDERFLOW_CONTROL           = 0x07,
    USB_AUDIO_V2_DR_OVERFLOW_CONTROL            = 0x08,
    USB_AUDIO_V2_DR_LATENCY_CONTROL             = 0x09
} USB_AUDIO_V2_EFFECT_UNIT_DR_CE_CONTROL_SELECTOR;

/* A.17.9 Processing Unit Control Selectors */
/* A.17.9.1 Up/Down-mix Processing Unit Control Selectors */
typedef enum
{
    USB_AUDIO_V2_UD_CONTROL_UNDEFINED   = 0x00,
    USB_AUDIO_V2_UD_ENABLE_CONTROL      = 0x01,
    USB_AUDIO_V2_UD_MODE_SELECT_CONTROL = 0x02,
    USB_AUDIO_V2_UD_CLUSTER_CONTROL     = 0x03,
    USB_AUDIO_V2_UD_UNDERFLOW_CONTROL   = 0x04,
    USB_AUDIO_V2_UD_OVERFLOW_CONTROL    = 0x05,
    USB_AUDIO_V2_UD_LATENCY_CONTROL     = 0x06
} USB_AUDIO_V2_PROCESSING_UNIT_UD_MIX_CONTROL_SELECTOR;

/*A.17.9.2 Dolby Prologic? Processing Unit Control Selectors */
typedef enum
{
    USB_AUDIO_V2_DP_CONTROL_UNDEFINED   = 0x00,
    USB_AUDIO_V2_DP_ENABLE_CONTROL      = 0x01,
    USB_AUDIO_V2_DP_MODE_SELECT_CONTROL = 0x02,
    USB_AUDIO_V2_DP_CLUSTER_CONTROL     = 0x03,
    USB_AUDIO_V2_DP_UNDERFLOW_CONTROL   = 0x04,
    USB_AUDIO_V2_DP_OVERFLOW_CONTROL    = 0x05,
    USB_AUDIO_V2_DP_LATENCY_CONTROL     = 0x06
} USB_AUDIO_V2_PROCESSING_UNIT_DOLBY_CONTROL_SELECTOR;

/* A.17.9.3 Stereo Extender Processing Unit Control Selectors */
typedef enum
{
    USB_AUDIO_V2_ST_EXT_CONTROL_UNDEFINED   = 0x00,
    USB_AUDIO_V2_ST_EXT_ENABLE_CONTROL      = 0x01,
    USB_AUDIO_V2_ST_EXT_WIDTH_CONTROL       = 0x02,
    USB_AUDIO_V2_ST_EXT_UNDERFLOW_CONTROL   = 0x03,
    USB_AUDIO_V2_ST_EXT_OVERFLOW_CONTROL    = 0x04,
    USB_AUDIO_V2_ST_EXT_LATENCY_CONTROL     = 0x04
} USB_AUDIO_V2_PROCESSING_UNIT_STEREO_EXT_CONTROL_SELECTOR;

/* A.17.10 Extension Unit Control Selectors */
typedef enum
{
    USB_AUDIO_V2_XU_CONTROL_UNDEFINED   = 0x00,
    USB_AUDIO_V2_XU_ENABLE_CONTROL      = 0x01,
    USB_AUDIO_V2_XU_CLUSTER_CONTROL     = 0x02,
    USB_AUDIO_V2_XU_UNDERFLOW_CONTROL   = 0x03,
    USB_AUDIO_V2_XU_OVERFLOW_CONTROL    = 0x04,
    USB_AUDIO_V2_XU_LATENCY_CONTROL     = 0x05
} USB_AUDIO_V2_EXTENSION_UNIT_CONTROL_SELECTOR;

/* A.17.11 AudioStreaming Interface Control Selectors */
typedef enum
{
    USB_AUDIO_V2_AS_CONTROL_UNDEFINED           = 0x00,
    USB_AUDIO_V2_AS_ACT_ALT_SETTING_CONTROL     = 0x01,
    USB_AUDIO_V2_AS_VAL_ALT_SETTINGS_CONTROL    = 0x02,
    USB_AUDIO_V2_AS_AUDIO_DATA_FORMAT_CONTROL   = 0x03
} USB_AUDIO_V2_AUDIO_STREAMING_CONTROL_SELECTOR;

/* A.17.12 Encoder Control Selectors */
typedef enum
{
    USB_AUDIO_V2_EN_CONTROL_UNDEFINED       = 0x00,
    USB_AUDIO_V2_EN_BIT_RATE_CONTROL        = 0x01,
    USB_AUDIO_V2_EN_QUALITY_CONTROL         = 0x02,
    USB_AUDIO_V2_EN_VBR_CONTROL             = 0x03,
    USB_AUDIO_V2_EN_TYPE_CONTROL            = 0x04,
    USB_AUDIO_V2_EN_UNDERFLOW_CONTROL       = 0x05, 
    USB_AUDIO_V2_EN_OVERFLOW_CONTROL        = 0x06,
    USB_AUDIO_V2_EN_ENCODER_ERROR_CONTROL   = 0x07,
    USB_AUDIO_V2_EN_PARAM1_CONTROL          = 0x08, 
    USB_AUDIO_V2_EN_PARAM2_CONTROL          = 0x09,
    USB_AUDIO_V2_EN_PARAM3_CONTROL          = 0x0A, 
    USB_AUDIO_V2_EN_PARAM4_CONTROL          = 0x0B, 
    USB_AUDIO_V2_EN_PARAM5_CONTROL          = 0x0C, 
    USB_AUDIO_V2_EN_PARAM6_CONTROL          = 0x0D, 
    USB_AUDIO_V2_EN_PARAM7_CONTROL          = 0x0E, 
    USB_AUDIO_V2_EN_PARAM8_CONTROL          = 0x0F
} USB_AUDIO_V2_ENCODER_CONTROL_SELECTOR;

/* A.17.13 Decoder Control Selectors */
/* A.17.13.1 MPEG Decoder Control Selectors */
typedef enum
{
    USB_AUDIO_V2_MDC_CONTROL_UNDEFINED      = 0x00, 
    USB_AUDIO_V2_MDC_DUAL_CHANNEL_CONTROL   = 0x01, 
    USB_AUDIO_V2_MDC_SECOND_STEREO_CONTROL  = 0x02, 
    USB_AUDIO_V2_MDC_MULTILINGUAL_CONTROL   = 0x03, 
    USB_AUDIO_V2_MDC_DYN_RANGE_CONTROL      = 0x04,
    USB_AUDIO_V2_MDC_SCALING_CONTROL        = 0x05, 
    USB_AUDIO_V2_MDC_HILO_SCALING_CONTROL   = 0x06, 
    USB_AUDIO_V2_MDC_UNDERFLOW_CONTROL      = 0x07, 
    USB_AUDIO_V2_MDC_OVERFLOW_CONTROL       = 0x08, 
    USB_AUDIO_V2_MDC_DECODER_ERROR_CONTROL  = 0x09
} USB_AUDIO_V2_MPEG_DECODER_CONTROL_SELECTOR;

/* A.17.13.2 AC-3 Decoder Control Selectors */
typedef enum
{
    USB_AUDIO_V2_AD_CONTROL_UNDEFINED       = 0x00, 
    USB_AUDIO_V2_AD_MODE_CONTROL            = 0x01, 
    USB_AUDIO_V2_AD_DYN_RANGE_CONTROL       = 0x02, 
    USB_AUDIO_V2_AD_SCALING_CONTROL         = 0x03, 
    USB_AUDIO_V2_AD_HILO_SCALING_CONTROL    = 0x04, 
    USB_AUDIO_V2_AD_UNDERFLOW_CONTROL       = 0x05, 
    USB_AUDIO_V2_AD_OVERFLOW_CONTROL        = 0x06, 
    USB_AUDIO_V2_AD_DECODER_ERROR_CONTROL   = 0x07 
} USB_AUDIO_V2_AC_3_DECODER_CONTROL_SELECTOR;

/* A.17.13.3 WMA Decoder Control Selectors */
typedef enum
{
    USB_AUDIO_V2_WD_CONTROL_UNDEFINED       = 0x00, 
    USB_AUDIO_V2_WD_UNDERFLOW_CONTROL       = 0x01, 
    USB_AUDIO_V2_WD_OVERFLOW_CONTROL        = 0x02, 
    USB_AUDIO_V2_WD_DECODER_ERROR_CONTROL   = 0x03 
} USB_AUDIO_V2_WMA_DECODER_CONTROL_SELECTOR;

/* A.17.13.4 DTS Decoder Control Selectors */
typedef enum
{
    USB_AUDIO_V2_DD_CONTROL_UNDEFINED       = 0x00,
    USB_AUDIO_V2_DD_UNDERFLOW_CONTROL       = 0x01, 
    USB_AUDIO_V2_DD_OVERFLOW_CONTROL        = 0x02, 
    USB_AUDIO_V2_DD_DECODER_ERROR_CONTROL   = 0x03 
} USB_AUDIO_V2_DTS_DECODER_CONTROL_SELECTOR;

/* A.17.14 Endpoint Control Selector */
typedef enum
{
    USB_AUDIO_V2_EP_CONTROL_UNDEFINED       = 0x00, 
    USB_AUDIO_V2_EP_PITCH_CONTROL           = 0x01, 
    USB_AUDIO_V2_EP_DATA_OVERRUN_CONTROL    = 0x02, 
    USB_AUDIO_V2_EP_DATA_UNDERRUN_CONTROL   = 0x03
} USB_AUDIO_V2_ENDPOINT_CONTROL_SELECTOR;

// From USB 2.0, additional USB descriptor types
#define USB_AUDIO_V2_DESCRIPTOR_DEBUG            0x0A    
#define USB_AUDIO_V2_DESCRIPTOR_IA               0x0B    

/* Additional Audio Device Class codes */

/* A.1 Format Type Codes */
#define USB_AUDIO_V2_FORMAT_TYPE_UNDEFINED       0x00
#define USB_AUDIO_V2_FORMAT_TYPE_I               0x01
#define USB_AUDIO_V2_FORMAT_TYPE_II              0x02
#define USB_AUDIO_V2_FORMAT_TYPE_III             0x03
#define USB_AUDIO_V2_FORMAT_TYPE_IV              0x04
#define USB_AUDIO_V2_EXT_FORMAT_TYPE_I           0x81
#define USB_AUDIO_V2_EXT_FORMAT_TYPE_II          0x82
#define USB_AUDIO_V2_EXT_FORMAT_TYPE_III         0x83

/* A.3 Side Band Protocol Codes */
#define USB_AUDIO_V2_PROTOCOL_UNDEFINED          0x00
#define USB_AUDIO_V2_PRESS_TIMESTAMP_PROTOCOL    0x01

// *****************************************************************************
/* Universal Serial Bus  Device Class Definition for Terminal Types */

/* 2.1 USB Terminal Types */
/* Terminal Types that describe Terminals that handle signals carried over USB */
#define USB_AUDIO_V2_TERMTYPE_UNDEFINED               0x0100
#define USB_AUDIO_V2_TERMTYPE_USB_STREAMING           0x0101
#define USB_AUDIO_V2_TERMTYPE_VENDOR_SPECIFIC         0x01FF

/* 2.2 Input Terminal Types */
/* Terminal Types that describe Terminals that are designed to record sounds */

typedef enum
{
    USB_AUDIO_V2_IN_UDEFINED                            = 0x0200,
    USB_AUDIO_V2_IN_TERM_MICROPHONE                     = 0x0201,
    USB_AUDIO_V2_IN_TERM_DESKTOP_MICROPHONE             = 0x0202,
    USB_AUDIO_V2_IN_TERM_PERSONAL_MICROPHONE            = 0x0203,
    USB_AUDIO_V2_IN_TERM_OMNIDIRECTIONAL_MICROPHONE     = 0x0204,
    USB_AUDIO_V2_IN_TERM_MICROPHONE_ARRAY               = 0x0205,
    USB_AUDIO_V2_IN_TERM_PROCESSING_MICROPHONE_ARRAY    = 0x0206
} USB_AUDIO_V2_INPUT_TERMINAL_TYPES;

/* 2.3 Output Terminal Types */
/* These Terminal Types describe Terminals that produce audible signals that are
 *  intended to be heard by the user of the audio function */

typedef enum
{
    USB_AUDIO_V2_OUT_TERM_SPEAKER                     = 0x0301,
    USB_AUDIO_V2_OUT_TERM_HEADPHONES                  = 0x0302,
    USB_AUDIO_V2_OUT_TERM_HEAD_MOUNTED_DISPLAY        = 0x0303,
    USB_AUDIO_V2_OUT_TERM_DESKTOPSPEAKER              = 0x0304,
    USB_AUDIO_V2_OUT_TERM_ROOM_SPEAKER                = 0x0305,
    USB_AUDIO_V2_OUT_TERM_COMMUNICATION_SPEAKER       = 0x0306,
    USB_AUDIO_V2_OUT_TERM_LOW_FREQ_EFFECTS_SPEAKER    = 0x0307
} USB_AUDIO_V2_OUTPUT_TERMINAL_TYPES;

/*2.4 Bi-directional Terminal Types */
/* These Terminal Types describe an Input and an Output Terminal for voice 
 * communication that are closely related.*/

typedef enum
{
    USB_AUDIO_V2_BI_DIR_UNDEFINED               = 0x0400,
    USB_AUDIO_V2_BI_DIR_HANDSET                 = 0x0401,
    USB_AUDIO_V2_BI_DIR_HEADSET                 = 0x0402,
    USB_AUDIO_V2_BI_DIR_SPEAKERPHONE            = 0x0403,
    USB_AUDIO_V2_BI_DIR_ECHO_SUP_SPEAKERPHONE   = 0x0404,
    USB_AUDIO_V2_BI_DIR_ECHO_CNCL_SPEAKERPHONE  = 0x0405
} USB_AUDIO_V2_BI_DIR_TERMINAL_TYPES;

// *****************************************************************************
// Structures

// 1-byte Control RANGE Parameter Block
typedef struct
{
    uint8_t   bMin;
    uint8_t   bMax;
    uint8_t   bRes;

} USB_AUDIO_V2_LAYOUT_1_RANGE_PARAMETER_BLOCK;

// 2-byte Control RANGE Parameter Block
typedef struct
{
    uint16_t   wMin;
    uint16_t   wMax;
    uint16_t   wRes;

} USB_AUDIO_V2_LAYOUT_2_RANGE_PARAMETER_BLOCK;

// 4-byte Control RANGE Parameter Block
typedef struct
{
    uint32_t   dMin;
    uint32_t   dMax;
    uint32_t   dRes;

} USB_AUDIO_V2_LAYOUT_3_RANGE_PARAMETER_BLOCK;

// *****************************************************************************
/* USB Audio Interrupt Data Message Format.

  Summary:
    Specifies the format of USB Audio Interrupt Data Message Format.

  Description:
    This type identifies the format of the USB Audio Interrupt Data Message. 
    This is defined in Table 6-1 of the USB Device Class definition 
    for Audio Devices Release 2.0 document.

  Remarks:
    Always needs to be packed.
*/

typedef union __attribute__((packed))
{
    /* Bitmap 
     *  D0: Vendor-specific.
     *  D1: Interface or Endpoint
     *  D7..2: Reserved for future extensions. Must be set to 0. */
    uint8_t bInfo;
    
    /* The attribute that caused the interrupt */
    uint8_t bAttribute;
    
    /* CS in the high byte and CN or MCN in the low byte. */
    uint16_t wValue;
    
    /* Entity ID or zero in the high byte and Interface or Endpoint in the 
     * low byte. */
    uint16_t wIndex;

} USB_AUDIO_V2_INTERRUPT_DATA_MESSAGE;

// *****************************************************************************
/* USB Audio Class Specific Audio Control Interface Header Descriptor

  Summary:
    Identifies the USB Audio Class Specific Audio Control Interface Header
    Descriptor.

  Description:
    This type identifies the USB Audio Class Specific Audio Control Interface
    Header Descriptor. This structure is as per Table 4-5 of the USB Device
    Class definition for Audio Device v2.0 document.

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
    
    /* Constant, indicating the primary use of this audio function, as intended 
     * by the manufacturer. See Appendix A.7, "Audio Function Category Codes" */
    uint8_t bCategory;
	
    /* Total number of bytes returned for the class-specific AudioControl
       interface descriptor. Includes the combined length of this descriptor
       header and all Unit and Terminal descriptors */
    uint16_t wTotalLength;

    /* Bitmap 
     *  D1..0: Latency Control
     *  D7..2: Reserved. Must be set to 0. */
    uint8_t bmControls;

} USB_AUDIO_V2_CS_AC_INTERFACE_HEADER_DESCRIPTOR;

// *****************************************************************************
/* Audio Input Terminal Descriptor Type

  Summary:
    Identifies the Audio Input Terminal Descriptor Type.

  Description:
    This type identifies the Audio Input Terminal Descriptor.  This structure is
    as per Table 4-9 of the USB Device Class Definition for Audio Device v2.0
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
    
    /* ID of the Clock Entity to which this Input Terminal is connected. */
    uint8_t bCSourceID;

    /* Number of channels in the terminal output */
    uint8_t bNrChannels;

    /* Spatial location of the logical channels */
    uint32_t bmChannelConfig;

    /* First Logical Channel String descriptor index */
    uint8_t iChannelNames;
    
    /* bmControls
     *  D1..0: Copy Protect Control
     *  D3..2: Connector Control
     *  D5..4: Overload Control
     *  D7..6: Cluster Control
     *  D9..8: Underflow Control
     *  D11..10: Overflow Control 
     * D15..12: Reserved. Must be set to 0. */
    uint16_t bmControls;

    /* Input Terminal String Descriptor Index */
    uint8_t iTerminal;

} USB_AUDIO_V2_INPUT_TERMINAL_DESCRIPTOR;

// *****************************************************************************
/* Audio Output Terminal Descriptor Type

  Summary:
    Identifies the Audio Output Terminal Descriptor Type.

  Description:
    This type identifies the Audio Output Terminal Descriptor.  This structure
    is as per Table 4-10 of the USB Device Class Definition for Audio Device v2.0
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
    
    /* ID of the Clock Entity to which this Output Terminal is connected.*/
    uint8_t bCSourceID;
    
    /* bmControls 
     *  D1..0: Copy Protect Control
     *  D3..2: Connector Control
     *  D5..4: Overload Control
     *  D7..6: Underflow Control 
     *  D9..8: Overflow Control 
     *  D15..10: Reserved. Must be set to 0.*/
    uint16_t bmControls;
    
    /* Output Terminal String Descriptor Index */
    uint8_t iTerminal;

} USB_AUDIO_V2_OUTPUT_TERMINAL_DESCRIPTOR;

// *****************************************************************************
/* USB Audio Class Specific Audio Streaming Interface Descriptor

  Summary:
    Identifies the USB Audio Class Specific Audio Streaming Interface Descriptor
    Type.

  Description:
    This type identifies the USB Audio Class Specific Audio Streaming Interface
    Descriptor.  This structure is as per Table 4-27 of the USB Device Class
    definition for Audio Device v2.0 document.

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
    
    /* bmControls 
     *  D1..0: Active Alternate Setting Control
     *  D3..2: Valid Alternate Settings Control 
     *  D7..4: Reserved. Must be set to 0. */
    uint8_t bmControls;
    
    /* Constant identifying the Format Type the AudioStreaming interface 
     * is using. */
    uint8_t bFormatType;
    
    /* The Audio Data Format(s) that can be used to communicate with this 
     * interface. See the USB Audio Data Formats document for further details.*/
    uint32_t bmFormats;
    
    /* Number of physical channels in the AS Interface audio channel cluster. */
    uint8_t bNrChannels;
    
    /* Describes the spatial location of the physical channels.*/
    uint32_t bmChannelConfig;

    /* Index of a string descriptor, describing the name of the first 
     * physical channel.  */
    uint8_t iChannelNames;

} USB_AUDIO_V2_CS_AS_INTERFACE_DESCRIPTOR;

// *****************************************************************************
/* USB Audio Class Specific Audio Streaming Isochronous Audio Data Endpoint 
   Descriptor

  Summary:
    Identifies the USB Audio Class Specific Audio Streaming Isochronous Audio
    Data Endpoint Descriptor

  Description:
    This type identifies the USB Audio Class Specific Audio Streaming
    Isochronous Audio Data Endpoint Descriptor Type. This structure is as per
    Table 4-34 of the USB Device Class Definition for Audio Device v2.0 document.

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

    /* Bit map indicating the mentioned control is supported by this endpoint 
     *  Bit D7 indicates a requirement for wMaxPacketSize packets. 
     *  D7: MaxPacketsOnly */
    uint8_t bmAttributes;
    
    /* bmControls 
     *  D1..0: Pitch Control
     *  D3..2: Data Overrun Control
     *  D5..4: Data Underrun Control 
     *  D7..6: Reserved. Must be set to 0*/
    uint8_t bmControls;

    /* Indicates the units used for the wLockDelay field 
     *  0: Undefined
     *  1: Milliseconds
     *  2: Decoded PCM samples
     *  3..255: Reserved */
    uint8_t bLockDelayUnits;

    /* Indicates the time it takes this endpoint to reliably lock its internal
     * clock recovery circuitry */
    uint16_t wLockDelay;

} USB_AUDIO_V2_CS_AS_ISOCHRONOUS_AUDIO_DATA_EP_DESCRIPTOR;

// *****************************************************************************
/* USB Audio Control Interface Set and Get Request.

  Summary:
    Identifies the type of the USB Audio Control Interface Set and Get Request.

  Description:
    This type identifies the type of the USB Audio Control Interface Set and Get
    Request.  The application can type cast the received audio class specific
    setup packet to this type in order to service Control attribute Set and Get
    requests. This structure is as per Table 5-1 of the USB Device Class
    Definition for Audio Device v2.0 document. Refer to section 5.2.1 and
    section 5.2.2 of this document for detailed description and request
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

} USB_AUDIO_V2_CONTROL_INTERFACE_REQUEST;

// *****************************************************************************
/* USB Audio Clock Select Control Request.

  Summary:
    Identifies the type of the USB Audio Clock select Control Set and Get Request.

  Description:
    This type identifies the type of the USB Audio Terminal Control Set and Get
    Request.  The application can type cast the received audio class specific
    setup packet to this type in order to service terminal control Set and Get
    requests. This structure is as per Table 5-1 of the USB Device Class
    Definition for Audio Device v2.0 document. Refer to section 5.2.5.2 of this 
    document for detailed description and request specific interpretation 
    of the fields.

  Remarks:
    Always needs to be packed.
*/
typedef struct
{

    /* Direction of the request, Set or Get */
    unsigned direction : 1;
    
    /* Class specific request */
    unsigned type: 2;
    
    /* Recipient of the request, Interface or Isochronous ep */
    unsigned Recipient: 5;
    
    /* Identifies the attribute to be accessed */
    uint8_t bRequest;
    
    /* Identifies the channel number */
    uint8_t channelNumber;
    
    /* Identifies the control selector to be accessed */
    uint8_t controlSelector;
    
    /* The interface ID */
    uint8_t interfaceId;
    
    /* The Clock select ID */
    uint8_t ClockSelectId;
    
    /* Length of the parameter block */
    uint16_t wLength;
    
} USB_AUDIO_V2_CLOCKSELECT_CONTROL_REQUEST;

// *****************************************************************************
/* USB Audio Clock Source Control Request.

  Summary:
    Identifies the type of the USB Audio Clock Source Control Set and Get Request.

  Description:
    This type identifies the type of the USB Audio Terminal Control Set and Get
    Request.  The application can type cast the received audio class specific
    setup packet to this type in order to service terminal control Set and Get
    requests. This structure is as per Table 5-1 of the USB Device Class
    Definition for Audio Device v2.0 document. Refer to section 5.2.5.1 of this 
    document for detailed description and request specific interpretation 
    of the fields.

  Remarks:
    Always needs to be packed.
*/
typedef struct
{
    /* Direction of the request, Get or Set */
    unsigned direction : 1;
    
    /* Class specific request */
    unsigned type: 2;
    
    /* Recipient of the request, Interface or Isochronous ep */
    unsigned Recipient: 5;
    
    /* Identifies the attribute to be accessed */
    uint8_t bRequest;
    
    /* Identifies the channel number */
    uint8_t channelNumber;
    
    /* Identifies the control selector to be accessed */
    uint8_t controlSelector;
    
    /* The interface ID */
    uint8_t interfaceId;
    
    /* The Clock select ID */
    uint8_t ClockSourceId;
    
    /* Length of the parameter block */
    uint16_t wLength;
    
} USB_AUDIO_V2_CLOCKSOURCE_CONTROL_REQUEST;

// *****************************************************************************
/* USB Audio Terminal Control Set and Get Request.

  Summary:
    Identifies the type of the USB Audio Terminal Control Set and Get Request.

  Description:
    This type identifies the type of the USB Audio Terminal Control Set and Get
    Request.  The application can type cast the received audio class specific
    setup packet to this type in order to service terminal control Set and Get
    requests. This structure is as per Table 5-3 and 5-4 of the USB Device Class
    Definition for Audio Device v2.0 document. Refer to section 5.2.2.1.1 and
    section 5.2.2.1.2 of this document for detailed description and request
    specific interpretation of the fields.

  Remarks:
    Always needs to be packed.
*/

typedef struct __attribute__((packed))
{
    /* Direction of the request, Get or Set */
    unsigned direction : 1;
    
    /* Class specific request */
    unsigned type: 2;
    
    /* Recipient of the request, Interface or Isochronous ep */
    unsigned Recipient: 5;

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
    
} USB_AUDIO_V2_TERMINAL_CONTROL_REQUEST;

// *****************************************************************************
/* USB Audio Mixer Unit Control Set and Get Request.

  Summary:
    Identifies the type of the USB Audio Mixer Unit Control Set and Get Request.

  Description:
    This type identifies the type of the USB Audio Mixer Unit Control Set and
    Get Request.  The application can type cast the received audio class
    specific setup packet to this type in order to service mixer unit control
    Set and Get requests. This structure is as per Table 5-6 and 5-7 of the USB
    Device Class Definition for Audio Device v2.0 document. Refer to section
    5.2.2.2.1 and section 5.2.2.2.2 of this document for detailed description
    and request specific interpretation of the fields.

  Remarks:
    Always needs to be packed.
*/

typedef struct __attribute__((packed))
{
    /* Direction of the request, Get or Set */
    unsigned direction : 1;
    
    /* Class specific request */
    unsigned type: 2;
    
    /* Recipient of the request, Interface or Isochronous ep */
    unsigned Recipient: 5;

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

} USB_AUDIO_V2_MIXER_UNIT_CONTROL_REQUEST;

// *****************************************************************************
/* USB Audio Selector Unit Control Set and Get Request.

  Summary:
    Identifies the type of the USB Audio Selector Unit Control Set and Get Request.

  Description:
    This type identifies the type of the USB Audio Selector Unit Control Set and
    Get Request.  The application can type cast the received audio class
    specific setup packet to this type in order to service selector unit control
    Set and Get requests. This structure is as per Table 5-11 and 5-12 of the USB
    Device Class Definition for Audio Device v2.0 document. Refer to section
    5.2.2.3.1 and section 5.2.2.3.2 of this document for detailed description
    and request specific interpretation of the fields.

  Remarks:
    Always needs to be packed.
*/

typedef struct __attribute__((packed))
{
    /* Direction of the request, Get or Set */
    unsigned direction : 1;
    
    /* Class specific request */
    unsigned type: 2;
    
    /* Recipient of the request, Interface or Isochronous ep */
    unsigned Recipient: 5;

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

} USB_AUDIO_V2_SELECTOR_UNIT_CONTROL_REQUEST;

// *****************************************************************************
/* USB Audio Feature Unit Control Set and Get Request.

  Summary:
    Identifies the type of the USB Audio Feature Unit Control Set and Get Request.

  Description:
    This type identifies the type of the USB Audio Feature Unit Control Set and
    Get Request.  The application can type cast the received audio class
    specific setup packet to this type in order to service the feature unit
    control Set and Get requests. This structure is as per Table 5-14 and 5-15
    of the USB Device Class Definition for Audio Device v2.0 document. Refer to
    section 5.2.2.4.1 and section 5.2.2.4.2 of this document for detailed
    description and request specific interpretation of the fields.

  Remarks:
    Always needs to be packed.
*/

typedef struct __attribute__((packed))
{
    /* Direction of the request, Get or Set */
    unsigned direction : 1;
    
    /* Class specific request */
    unsigned type: 2;
    
    /* Recipient of the request, Interface or Isochronous ep */
    unsigned Recipient: 5;
    
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
} USB_AUDIO_V2_FEATURE_CONTROL_REQUEST;

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
    of the USB Device Class Definition for Audio Device v2.0 document. Refer to
    section 5.2.2.5.1 and section 5.2.2.5.2 of this document for detailed
    description and request specific interpretation of the fields.

  Remarks:
    Always needs to be packed.
*/

typedef struct
{
    /* Direction of the request, Get or Set */
    unsigned direction : 1;
    
    /* Class specific request */
    unsigned type: 2;
    
    /* Recipient of the request, Interface or Isochronous ep */
    unsigned Recipient: 5;

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

} USB_AUDIO_V2_PROCESSING_UNIT_CONTROL_REQUEST;

// *****************************************************************************
/* USB Audio Mixer Unit Descriptor Header.

  Summary:
   Structure describing USB Audio Mixer unit descriptor Header.

  Description:
    This type identifies the type of the USB Audio mixer unit descriptor.  This
    structure is as per Table 4-11 USB Device Class Definition for Audio Device
    v2.0 document. This structure represents the first 5 Bytes of the USB Audio mixer
    unit descriptor.

    The complete structure of the USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR should be as
    such.

    struct __attribute__((packed))
    {
        USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR_HEADER header;
        USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR_SOURCE_ID mixerInputSourceID[NUMBER_OF_MIXER_INPUT_PINS];
        USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR_CHANNEL_INFO channelInfo;
        USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR_BMACONTROLS mixingControls[NUMBER_OF_PROGRAMMABLE_MIXING_CONTROLS];
        USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR_FOOTER stringdesciptorIndex;
    
    } USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR;

  Remarks:
    This structure is a part of the USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR. The
    USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR cannot be defined as it an open ended data
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

    /* This header is followed by USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR_SOURCE_ID
       type of entries identifying the ID of the entities connected to input pin
       of the mixer. So the first entry is the ID of the entity that is
       connected to the first mixer input pin and so on.*/ 
    
} USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR_HEADER;

// *****************************************************************************
/* USB Audio Mixer Unit Descriptor Source ID.

  Summary:
   Structure describing USB Audio Mixer unit descriptor Source ID.

  Description:
    This type identifies the type of the USB Audio mixer unit descriptor.  This
    structure is as per Table 5-45 USB Device Class Definition for Audio Device
    v2.0 document.

  Remarks:
    This structure is a part of the USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR. The
    USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR cannot be defined as it an open ended data
    structure.
*/

typedef struct
{
    
    /* ID of the Unit or Terminal to which the first Input Pin of this Mixer
       Unit is connected */
    uint8_t baSourceID;

} USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR_SOURCE_ID;

// *****************************************************************************
/* USB Audio Mixer Unit Descriptor Channel Info.

  Summary:
   Structure describing USB Audio Mixer unit descriptor Channel Info.

  Description:
    This type identifies the type of the USB Audio mixer unit descriptor.  This
    structure is as per Table 4-11 USB Device Class Definition for Audio Device
    v2.0 document.

  Remarks:
    This structure is a part of the USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR. The
    USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR cannot be defined as it an open ended data
    structure.
*/

typedef struct
{
    /* Number of logical output channels in the Mixer's output audio channel
       cluster */
    uint8_t bNrChannels;

    /* Describes the spatial location of the logical channels. */
    uint16_t wChannelConfig;

    /* Index of a string descriptor, describing the name of the first logical
       channel */
    uint8_t iChannelNames;

} USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR_CHANNEL_INFO;

// *****************************************************************************
/* USB Audio Mixer Unit Descriptor bmaControls.

  Summary:
   Structure describing USB Audio Mixer unit descriptor bmaControls.

  Description:
    This type identifies the type of the USB Audio mixer unit descriptor.  This
    structure is as per Table 4-11 USB Device Class Definition for Audio Device
    v2.0 document.

  Remarks:
    This structure is a part of the USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR. The
    USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR cannot be defined as it an open ended data
    structure.
*/

typedef struct
{
    /* Bit map indicating which mixing Controls are programmable. */
    uint8_t bmControls;

}USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR_BMACONTROLS;

// *****************************************************************************
/* USB Audio Mixer Unit Descriptor Footer.

  Summary:
   Structure describing USB Audio Mixer unit descriptor Footer.

  Description:
    This type identifies the type of the USB Audio mixer unit descriptor.  This
    structure is as per Table 4-11 USB Device Class Definition for Audio Device
    v2.0 document.

  Remarks:
    This structure is a part of the USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR. The
    USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR cannot be defined as it an open ended data
    structure.
*/

typedef struct
{
    /* Index of a string descriptor, describing the Mixer Unit. */
    uint8_t iMixer;

}USB_AUDIO_V2_MIXER_UNIT_DESCRIPTOR_FOOTER;

// *****************************************************************************
/* Audio Feature Unit Descriptor Header Type

  Summary:
    Identifies the Audio Feature Unit Descriptor Type.

  Description:
    This type identifies the Audio Feature Unit Descriptor.  This structure
    is as per Table 4-13 of the USB Device Class Definition for Audio Device v2.0
    document. The header consists of the first 5 bytes of the Feature Unite descriptor
    The complete structure of the USB_AUDIO_V2_FEATURE_UNIT_DESCRIPTOR should be as
    such.

    struct __attribute__((packed))
    {
        USB_AUDIO_V2_FEATURE_UNIT_DESCRIPTOR_HEADER header;
        USB_AUDIO_V2_FEATURE_UNIT_DESCRIPTOR_BMACONTROLS FeatureControls[NUMBER_OF_LOGICAL_CHANNELS_CONTROLS];
        USB_AUDIO_V2_FEATURE_UNIT_DESCRIPTOR_FOOTER stringdesciptorIndex;
    
    } USB_AUDIO_V2_FEATURE_UNIT_DESCRIPTOR;

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

} USB_AUDIO_V2_FEATURE_UNIT_DESCRIPTOR_HEADER;

// *****************************************************************************
/* Audio Feature Unit BMA Control

  Summary:
    Identifies the Audio Feature Unit BMA control type.

  Description:
    This type identifies the Audio Feature Unit BMA control.  This structure
    is as per Table 4-13 of the USB Device Class Definition for Audio Device v2.0
    document.

  Remarks:
    Always needs to be packed.
*/
typedef union __attribute__ ((packed))
{
    uint32_t bmaControls;

    struct 
    {
        uint8_t mute: 2;
        uint8_t volume: 2;
        uint8_t bass:2;
        uint8_t mid:2;
        uint8_t treble:2;
        uint8_t graphicEqualizer:2;
        uint8_t automaticGain:2;
        uint8_t delay:2;
        uint8_t bassBoost:2;
        uint8_t loudness:2;
        uint8_t inputGain:2;
        uint8_t inputGainPad:2;
        uint8_t phaseInverter:2;
        uint8_t underflow:2;
        uint8_t overflow:2;
        uint8_t reserved:2;
    }; 
} USB_AUDIO_V2_FEATURE_UNIT_BMA_CONTROLS;

// *****************************************************************************
/* USB Audio Feature Unit Descriptor Footer.

  Summary:
   Structure describing USB Audio Feature unit descriptor Footer.

  Description:
    This type identifies the type of the USB Audio feature unit descriptor.  This
    structure is as per Table 4-13 USB Device Class Definition for Audio Device
    v2.0 document.

  Remarks:
    This structure is a part of the USB_AUDIO_V2_FEATURE_UNIT_DESCRIPTOR. The
    USB_AUDIO_V2_FEATURE_UNIT_DESCRIPTOR cannot be defined as it an open ended data
    structure.
*/

typedef struct
{
    /* Index of a string descriptor, describing the feature Unit. */
    uint8_t iFeature;

}USB_AUDIO_V2_FEATURE_UNIT_DESCRIPTOR_FOOTER;

// *****************************************************************************
//DOM-IGNORE-BEGIN
/* Definitions for backward compatibility with MPLAB Harmony 1.05 release */
/* A.1 Audio Function Class Code */
#define AUDIO_V2_FUNCTION              AUDIO_V2

/* A.2 Audio Function Subclass Codes */
#define AUDIO_V2_FUNCTION_SUBCLASS_UNDEFINED 0x00

/* A.3 Audio Function Protocol Codes */
#define AUDIO_V2_FUNCTION_PROTOCOL_UNDEFINED 0x00
#define AUDIO_V2_AF_VERSION_02_00            AUDIO_V2_IP_VERSION_02_00

/* A.4 Audio Interface Class Code */
#define AUDIO_V2                       0x01

/* A.5 Audio Interface Subclass Codes */
#define AUDIO_V2_AUDIOCONTROL                0x01
#define AUDIO_V2_AUDIOSTREAMING              0x02
#define AUDIO_V2_MIDISTREAMING               0x03

/* A.6 Audio Interface Protocol Codes */
#define AUDIO_V2_IP_VERSION_02_00            0x20

/* A.7 Audio Function Category Codes */
#define AUDIO_V2_FUNCTION_SUBCLASS_UNDEFINED 0x00
#define AUDIO_V2_DESKTOP_SPEAKER             0x01
#define AUDIO_V2_HOME_THEATER                0x02
#define AUDIO_V2_MICROPHONE                  0x03
#define AUDIO_V2_HEADSET                     0x04
#define AUDIO_V2_TELEPHONE                   0x05
#define AUDIO_V2_CONVERTER                   0x06
#define AUDIO_V2_VOICE_SOUND_RECORDER        0x07
#define AUDIO_V2_IO_BOX                      0x08
#define AUDIO_V2_MUSICAL_INTRUMENT           0x09
#define AUDIO_V2_PRO_AUDIO                   0x0A
#define AUDIO_V2_AUDIO_VIDEO                 0x0B
#define AUDIO_V2_CONTROL_PANEL               0x0C
#define AUDIO_V2_OTHER                       0xFF

/* A.8 Audio Class-Specific Descriptor Types */
#define AUDIO_V2_CS_UNDEFINED                0x20
#define AUDIO_V2_CS_DEVICE                   0x21
#define AUDIO_V2_CS_CONFIGURATION            0x22
#define AUDIO_V2_CS_STRING                   0x23
#define AUDIO_V2_CS_INTERFACE                0x24
#define AUDIO_V2_CS_ENDPOINT                 0x25

/* A.9 Audio Class-Specific AC Interface Descriptor Subtypes */
#define AUDIO_V2_AC_DESCRIPTOR_UNDEFINED     0x00
#define AUDIO_V2_HEADER                      0x01
#define AUDIO_V2_INPUT_TERMINAL              0x02
#define AUDIO_V2_OUTPUT_TERMINAL             0x03
#define AUDIO_V2_MIXER_UNIT                  0x04
#define AUDIO_V2_SELECTOR_UNIT               0x05
#define AUDIO_V2_FEATURE_UNIT                0x06
#define AUDIO_V2_EFFECT_UNIT                 0x07
#define AUDIO_V2_PROCESSING_UNIT             0x08
#define AUDIO_V2_EXTENSION_UNIT              0x09
#define AUDIO_V2_CLOCK_SOURCE                0x0A
#define AUDIO_V2_CLOCK_SELECTOR              0x0B
#define AUDIO_V2_CLOCK_MULTIPLIER            0x0C
#define AUDIO_V2_SAMPLE_RATE_CONVERTER       0x0D

/* A.10 Audio Class Specific AS Interface Descriptor Subtypes */
#define AUDIO_V2_AS_DESCRIPTOR_UNDEFINED     0x00
#define AUDIO_V2_AS_GENERAL                  0x01
#define AUDIO_V2_FORMAT_TYPE                 0x02
#define AUDIO_V2_ENCODER                     0x03
#define AUDIO_V2_DECODER                     0x04

/* A.11 Effect Unit Effect Types */
#define AUDIO_V2_EFFECT_UNDEFINED            0x00
#define AUDIO_V2_PARAM_EQ_SECTION_EFFECT     0x01
#define AUDIO_V2_REVERBERATION_EFFECT        0x02
#define AUDIO_V2_MOD_DELAY_EFFECT            0x03
#define AUDIO_V2_DYN_RANGE_COMP_EFFECT       0x04

/* A.12 Processing Unit Process Types */
#define AUDIO_V2_PROCESS_UNDEFINED           0x00
#define AUDIO_V2_UP_DOWNMIX_PROCESS          0x01
#define AUDIO_V2_DOLBY_PROLOGIC_PROCESS      0x02
#define AUDIO_V2_STEREO_EXTENDER_PROCESS     0x03

/* A.13 Audio Class-Specific Endpoint Descriptor Subtypes */
#define AUDIO_V2_DESCRIPTOR_UNDEFINED        0x00
#define AUDIO_V2_EP_GENERAL                  0x01

/* A.14 Audio Class-Specific Request Codes */
#define AUDIO_V2_REQUEST_CODE_UNDEFINED      0x00
#define AUDIO_V2_CUR                         0x01
#define AUDIO_V2_RANGE                       0x02
#define AUDIO_V2_MEM                         0x03

/* A.15 Encoder Type Codes */
#define AUDIO_V2_ENCODER_UNDEFINED           0x00
#define AUDIO_V2_OTHER_ENCODER               0x01
#define AUDIO_V2_MPEG_ENCODER                0x02
#define AUDIO_V2_AC_3_ENCODER                0x03
#define AUDIO_V2_WMA_ENCODER                 0x04
#define AUDIO_V2_DTS_ENCODER                 0x05

/* A.17 Control Selector Codes */
/* A.17.1 Clock Source Control Selectors */
#define AUDIO_V2_CS_CONTROL_UNDEFINED        0x00
#define AUDIO_V2_CS_SAM_FREQ_CONTROL         0x01
#define AUDIO_V2_CS_CLOCK_VALID_CONTROL      0x02

/* A.17.2 Clock Selector Control Selectors */
#define AUDIO_V2_CX_CONTROL_UNDEFINED        0x00
#define AUDIO_V2_CX_CLOCK_SELECTOR_CONTROL   0x01

#define AUDIO_V2_FU_CONTROL_UNDEFINED        0x00
#define AUDIO_V2_FU_MUTE_CONTROL             0x01
#define AUDIO_V2_FU_VOLUME_CONTROL           0x02

/* A.17.11 Audio Streaming Interface Control Selectors */
#define AUDIO_V2_AS_CONTROL_UNDEFINED        0x00
#define AUDIO_V2_AS_ACT_ALT_SETTING_CONTROL  0x01
#define AUDIO_V2_AS_VAL_ALT_SETTINGS_CONTROL 0x02
#define AUDIO_V2_AS_AUDIO_V2_DATA_FORMAT_CONTROL 0x03

/* Audio Class-Specific Descriptor Types */
#define AUDIO_V2_CS_INTERFACE                0x24
#define AUDIO_V2_CS_ENDPOINT                 0x25


/* Audio Class-Specific AS Interface Descriptor Subtypes */
#define AUDIO_V2_FORMAT_TYPE                 0x02


/***********************************************************************/
/* Universal Serial Bus Device Class Definition for Audio Data Formats */

/* A.1 Format Type Codes */
#define AUDIO_V2_FORMAT_TYPE_UNDEFINED       0x00
#define AUDIO_V2_FORMAT_TYPE_I               0x01
#define AUDIO_V2_FORMAT_TYPE_II              0x02
#define AUDIO_V2_FORMAT_TYPE_III             0x03
#define AUDIO_V2_FORMAT_TYPE_IV              0x04
#define AUDIO_V2_EXT_FORMAT_TYPE_I           0x81
#define AUDIO_V2_EXT_FORMAT_TYPE_II          0x82
#define AUDIO_V2_EXT_FORMAT_TYPE_III         0x83

/* A.3 Side Band Protocol Codes */
#define AUDIO_V2_PROTOCOL_UNDEFINED          0x00
#define AUDIO_V2_PRESS_TIMESTAMP_PROTOCOL    0x01


/***********************************************************************/
/* Universal Serial Bus  Device Class Definition for Terminal Types */

/* 2.2 Input Terminal Types */
/* Terminal Types that describe Terminals that are designed to record sounds */
#define AUDIO_V2_INPUT_UDEFINED              0x0200
#define AUDIO_V2_MICROPHONE_                 0x0201
#define AUDIO_V2_DESKTOP_MICROPHONE          0x0202
#define AUDIO_V2_PERSONAL_MICROPHONE         0x0203
#define AUDIO_V2_OMNIDIRECTIONAL_MICROPHONE  0x0204
#define AUDIO_V2_MICROPHONE_ARRAY            0x0205
#define AUDIO_V2_PROCESSING_MICROPHONE_ARRAY 0x0206

/* 2.3 Output Terminal Types */
/* These Terminal Types describe Terminals that produce audible signals that are intended to
 * be heard by the user of the audio function */
#define AUDIO_V2_SPEAKER                     0x0301
#define AUDIO_V2_HEADPHONES                  0x0302
#define AUDIO_V2_HEAD_MOUNTED_DISPLAY        0x0303
#define AUDIO_V2_DESKTOPSPEAKER              0x0304
#define AUDIO_V2_ROOM_SPEAKER                0x0305
#define AUDIO_V2_COMMUNICATION_SPEAKER       0x0306
#define AUDIO_V2_LOW_FREQ_EFFECTS_SPEAKER    0x0307


#define AUDIO_V2_FORMAT_TYPE_I               0x01

/* A.2 AudioData Format Bit Allocation in the bmFormats field */
/* A.2.1 Audio Data Format Type I Bit Allocations */
#define AUDIO_V2_PCM                         0x00000001
#define AUDIO_V2_PCM8                        0x00000002
#define AUDIO_V2_IEEE_FLOAT                  0x00000004
/* 0x08 to 0x4000000 Reserved */
#define AUDIO_V2_TYPE_1_RAW_DATA             0x80 /* TODO */
//DOM-IGNORE-END

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END


// End file _USBAUDIO20
#endif
