#ifndef _CST_EEPROM_H_
#define _CST_EEPROM_H_

#define EE_VERSION_MAJOR              0x0E
#define EE_VERSION_MINOR              0x0F

//                                    0x10
#define EE_DEVICE_SLEEP_TIMEOUT       0x11
#define EE_DEAD_RECKONING_TIME        0x12
#define EE_CONFIGBITS                 0x13
#define EE_BATTERY_OKAY               0x14
#define EE_BATTERY_WARN               0x15
#define EE_BATTERY_CRITICAL           0x16
#define EE_TX_HOLDOFF                 0x1D
#define EE_TIME_SOURCE_ADDRESS        0x1E
#define EE_BASE_ADDR                  0x1F
#define EE_HORN_THRESHOLD             0x20
#define EE_BRAKE_THRESHOLD            0x21
#define EE_BRAKE_LOW_THRESHOLD        0x22
#define EE_BRAKE_HIGH_THRESHOLD       0x23
#define EE_PRESSURE_CONFIG            0x24
#define EE_ALERTER_TIMEOUT            0x25

// 20 configs * 128 bytes = 2560 bytes
//  +128 bytes for global = 2688 bytes

//        Total available = 4096 bytes

#define MAX_CONFIGS      20

// Put working (scratchspace) config at the end of EEPROM space
#define WORKING_CONFIG   31

#define CONFIG_START                  0x80
#define CONFIG_SIZE                   0x80

#define CONFIG_OFFSET(cfgNum)         (((cfgNum - 1) * CONFIG_SIZE) + CONFIG_START)

#define EE_LOCO_ADDRESS               (0x00 + CONFIG_OFFSET(WORKING_CONFIG))
//      EE_LOCO_ADDRESS                0x01
#define EE_HORN_FUNCTION              (0x02 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_BELL_FUNCTION              (0x03 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_BRAKE_FUNCTION             (0x04 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_AUX_FUNCTION               (0x05 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_ENGINE_ON_FUNCTION         (0x06 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_ENGINE_OFF_FUNCTION        (0x07 + CONFIG_OFFSET(WORKING_CONFIG))

#define EE_FRONT_DIM1_FUNCTION        (0x08 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_FRONT_DIM2_FUNCTION        (0x09 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_FRONT_HEADLIGHT_FUNCTION   (0x0A + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_FRONT_DITCH_FUNCTION       (0x0B + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_REAR_DIM1_FUNCTION         (0x0C + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_REAR_DIM2_FUNCTION         (0x0D + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_REAR_HEADLIGHT_FUNCTION    (0x0E + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_REAR_DITCH_FUNCTION        (0x0F + CONFIG_OFFSET(WORKING_CONFIG))

#define EE_UP_BUTTON_FUNCTION         (0x10 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_DOWN_BUTTON_FUNCTION       (0x11 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_THR_UNLOCK_FUNCTION        (0x12 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_BRAKE_OFF_FUNCTION         (0x13 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_REV_SWAP_FUNCTION          (0x14 + CONFIG_OFFSET(WORKING_CONFIG))
//                                     0x15
#define EE_BRAKE_PULSE_WIDTH          (0x16 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_OPTIONBITS                 (0x17 + CONFIG_OFFSET(WORKING_CONFIG))

#define EE_FORCE_FUNC_ON              (0x18 + CONFIG_OFFSET(WORKING_CONFIG))
//      EE_FORCE_FUNC_ON               0x19
//      EE_FORCE_FUNC_ON               0x1A
//      EE_FORCE_FUNC_ON               0x1B
#define EE_FORCE_FUNC_OFF             (0x1C + CONFIG_OFFSET(WORKING_CONFIG))
//      EE_FORCE_FUNC_OFF              0x1D
//      EE_FORCE_FUNC_OFF              0x1E
//      EE_FORCE_FUNC_OFF              0x1F

#define EE_NOTCH_SPEEDSTEP            (0x20 + CONFIG_OFFSET(WORKING_CONFIG))
//      EE_NOTCH_SPEEDSTEP             0x21
//      EE_NOTCH_SPEEDSTEP             0x22
//      EE_NOTCH_SPEEDSTEP             0x23
//      EE_NOTCH_SPEEDSTEP             0x24
//      EE_NOTCH_SPEEDSTEP             0x25
//      EE_NOTCH_SPEEDSTEP             0x26
//      EE_NOTCH_SPEEDSTEP             0x27

#define EE_COMPRESSOR_FUNCTION        (0x30 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_BRAKE_TEST_FUNCTION        (0x31 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_NEUTRAL_FUNCTION           (0x32 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_ALERTER_FUNCTION           (0x33 + CONFIG_OFFSET(WORKING_CONFIG))

#define EE_UP2_BUTTON_FUNCTION        (0x34 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_DOWN2_BUTTON_FUNCTION      (0x35 + CONFIG_OFFSET(WORKING_CONFIG))

#endif
