#ifndef _CST_EEPROM_H_
#define _CST_EEPROM_H_

#define EE_VERSION_MAJOR              0x0E
#define EE_VERSION_MINOR              0x0F

#define EE_CURRENT_CONFIG             0x10
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

// 20 configs * 128 bytes = 2560 bytes
//  +128 bytes for global = 2688 bytes

//        Total available = 4096 bytes

#define MAX_CONFIGS      20

// Put working (scratchspace) config at the end of EEPROM space
#define WORKING_CONFIG   31

#define CONFIG_START                  0x80
#define CONFIG_SIZE                   0x80


// These are offsets from CONFIG_START
#define EE_LOCO_ADDRESS               (0x00 + configOffset)
//      EE_LOCO_ADDRESS                0x01
#define EE_HORN_FUNCTION              (0x02 + configOffset)
#define EE_BELL_FUNCTION              (0x03 + configOffset)
#define EE_BRAKE_FUNCTION             (0x04 + configOffset)
#define EE_AUX_FUNCTION               (0x05 + configOffset)
#define EE_ENGINE_ON_FUNCTION         (0x06 + configOffset)
#define EE_ENGINE_OFF_FUNCTION        (0x07 + configOffset)

#define EE_FRONT_DIM1_FUNCTION        (0x08 + configOffset)
#define EE_FRONT_DIM2_FUNCTION        (0x09 + configOffset)
#define EE_FRONT_HEADLIGHT_FUNCTION   (0x0A + configOffset)
#define EE_FRONT_DITCH_FUNCTION       (0x0B + configOffset)
#define EE_REAR_DIM1_FUNCTION         (0x0C + configOffset)
#define EE_REAR_DIM2_FUNCTION         (0x0D + configOffset)
#define EE_REAR_HEADLIGHT_FUNCTION    (0x0E + configOffset)
#define EE_REAR_DITCH_FUNCTION        (0x0F + configOffset)

#define EE_UP_BUTTON_FUNCTION         (0x10 + configOffset)
#define EE_DOWN_BUTTON_FUNCTION       (0x11 + configOffset)
#define EE_THR_UNLOCK_FUNCTION        (0x12 + configOffset)
#define EE_BRAKE_OFF_FUNCTION         (0x13 + configOffset)
#define EE_REV_SWAP_FUNCTION          (0x14 + configOffset)

#define EE_BRAKE_PULSE_WIDTH          (0x16 + configOffset)
#define EE_OPTIONBITS                 (0x17 + configOffset)

#define EE_FUNC_FORCE_ON              (0x18 + configOffset)
//      EE_FUNC_FORCE_ON               0x19
//      EE_FUNC_FORCE_ON               0x1A
//      EE_FUNC_FORCE_ON               0x1B
#define EE_FUNC_FORCE_OFF             (0x1C + configOffset)
//      EE_FUNC_FORCE_OFF              0x1D
//      EE_FUNC_FORCE_OFF              0x1E
//      EE_FUNC_FORCE_OFF              0x1F

#define EE_NOTCH_SPEEDSTEP                (0x20 + configOffset)
//      EE_NOTCH_SPEEDSTEP                 0x21
//      EE_NOTCH_SPEEDSTEP                 0x22
//      EE_NOTCH_SPEEDSTEP                 0x23
//      EE_NOTCH_SPEEDSTEP                 0x24
//      EE_NOTCH_SPEEDSTEP                 0x25
//      EE_NOTCH_SPEEDSTEP                 0x26
//      EE_NOTCH_SPEEDSTEP                 0x27


inline uint16_t calculateConfigOffset(uint8_t cfgNum)
{
	return(((cfgNum - 1) * CONFIG_SIZE) + CONFIG_START);
}

inline uint8_t calculateConfigNumber(uint16_t cfgOffset)
{
	return(((cfgOffset - CONFIG_START) / CONFIG_SIZE) + 1);
}


#endif

