/*
 * Copyright (c) 2018-2022 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <stdint.h>

#include <cstddef>

extern "C" {

/*********CAN ID Defines***********/

// CANSpark API class;
const int CMD_API_SETPNT_SET = 0x001;
const int CMD_API_DC_SET = 0x002;
const int CMD_API_SPD_SET = 0x012;
const int CMD_API_SMART_VEL_SET = 0x013;
const int CMD_API_POS_SET = 0x032;
const int CMD_API_VOLT_SET = 0x042;
const int CMD_API_CURRENT_SET = 0x043;
const int CMD_API_SMARTMOTION_SET = 0x052;
const int CMD_API_STAT0 = 0x060;
const int CMD_API_STAT1 = 0x061;
const int CMD_API_STAT2 = 0x062;
const int CMD_API_STAT3 = 0x063;
const int CMD_API_STAT4 = 0x064;
const int CMD_API_STAT5 = 0x065;
const int CMD_API_STAT6 = 0x066;
const int CMD_API_CLEAR_FAULTS = 0x06E;
const int CMD_API_DRV_STAT = 0x06A;
const int CMD_API_BURN_FLASH = 0x072;
const int CMD_API_SET_FOLLOWER = 0x073;
const int CMD_API_FACTORY_DEFAULT = 0x074;
const int CMD_API_FACTORY_RESET = 0x075;
const int CMD_API_IDENTIFY = 0x076;
const int CMD_API_NACK = 0x080;
const int CMD_API_ACK = 0x081;
const int CMD_API_BROADCAST = 0x090;
const int CMD_API_HEARTBEAT = 0x092;
const int CMD_API_SYNC = 0x093;
const int CMD_API_ID_QUERY = 0x094;
const int CMD_API_ID_ASSIGN = 0x095;
const int CMD_API_FIRMWARE = 0x098;
const int CMD_API_ENUM = 0x099;
const int CMD_API_LOCK = 0x09B;
const int CMD_API_LOCKB = 0x0B1;
const int CMD_API_NONRIO_HB = 0x0B2;

const int CMD_API_SWDL_BOOTLOADER = 0x1FF;
const int CMD_API_SWDL_DATA = 0x09C;
const int CMD_API_SWDL_CHKSUM = 0x09D;
const int CMD_API_SWDL_RETRANSMIT = 0x09E;

const int CMD_API_MECH_POS = 0x0A0;
const int CMD_API_I_ACCUM = 0x0A2;
const int CMD_API_ANALOG_POS = 0x0A3;
const int CMD_API_ALT_ENC_POS = 0x0A4;
const int CMD_API_PARAM_ACCESS = 0x300;

// CANSpark Full IDs
const int EXT_ID_API_STAT0 = 0x2051800;
const int EXT_ID_API_STAT1 = 0x2051840;
const int EXT_ID_API_STAT2 = 0x2051880;
const int EXT_ID_API_STAT3 = 0x20518C0;
const int EXT_ID_API_STAT4 = 0x2051900;
const int EXT_ID_API_STAT5 = 0x2051940;
const int EXT_ID_API_STAT6 = 0x2051980;

// Note: This is dependant on __attribute__((PACKED)) and processor is
// little-endian

#ifdef _WIN32
#define PACKED
#pragma pack(push, 1)
#else
#define PACKED __attribute__((__packed__))
#endif

typedef struct PACKED {
    float setpoint;
    int16_t auxSetpoint;
    uint32_t pidSlot : 2;
    uint8_t arbFFUnits : 1;
    uint32_t rsvd0 : 5;
    uint32_t rsvd1 : 8;
} frc_dataframe_setpoint_out_t;

typedef struct PACKED {
    uint16_t updateRate;
    uint8_t statusSelect0;
    uint8_t statusSelect1;
    uint8_t statusSelect2;
    uint8_t statusSelect3;
} frc_dataframe_statusConfig_out_t;

typedef struct PACKED {
    uint32_t parameter;
    uint8_t parameterType;
} frc_dataframe_setParam_out_t;

typedef struct PACKED {
    uint32_t parameter0;
    uint8_t parameterType;
    uint8_t parameterResponse;
} frc_dataframe_getParam_in_t;

typedef struct PACKED {
    uint8_t magicNum0;
    uint8_t magicNum1;
} frc_dataframe_burnFlash_out_t;

typedef struct PACKED {
    uint32_t followerID;
    uint32_t followerCfg;
} frc_dataframe_follower_out_t;

typedef struct PACKED {
    uint8_t firmwareMajor;
    uint8_t firmwareMinor;
    uint16_t firmwareBuild;
    uint8_t debugBuild;
    uint8_t hardwareRevision;
} frc_dataframe_firmware_in_t;

typedef struct PACKED {
    int16_t appliedOutput;
    uint16_t faults;
    uint16_t stickyFaults;
    uint8_t sensorInv : 1;
    uint8_t setpointInv : 1;
    uint8_t lock : 2;
    uint8_t mtrType : 1;
    uint8_t isFollower : 1;
    uint8_t roboRIO : 1;
    uint8_t rsvd0 : 1;
} frc_dataframe_status0_in_t;

typedef struct PACKED {
    int32_t sensorVel;
    uint8_t mtrTemp;
    uint16_t mtrVoltage : 12;
    uint16_t mtrCurrent : 12;
} frc_dataframe_status1_in_t;

typedef struct PACKED {
    int32_t sensorPos;
    int32_t iAccum;
} frc_dataframe_status2_in_t;

typedef struct PACKED {
    uint32_t analogVoltage : 10;
    int32_t analogVel : 22;
    int32_t analogPos;
} frc_dataframe_status3_in_t;

typedef struct PACKED {
    float altEncoderVelocity;
    float altEncoderPosition;
} frc_dataframe_status4_in_t;

typedef struct PACKED {
    float dutyCyclePosition;
    uint16_t dutyCycleAbsoluteValue;
    uint8_t rsvd0;
    uint8_t dutyCycleStatus;
} frc_dataframe_status5_in_t;

typedef struct PACKED {
    float dutyCycleVelocity;
    uint16_t dutyCycleFrequency;
} frc_dataframe_status6_in_t;

typedef struct PACKED {
    uint16_t userParam0ID;
    uint16_t userParam1ID;
    uint16_t userParam2ID;
    uint16_t userParam3ID;
} frc_dataframe_userStatus_in_t;

typedef struct PACKED {
    uint16_t DRVStat0;
    uint16_t DRVStat1;
    uint16_t faults;
    uint16_t stickyFaults;
} frc_dataframe_drvStatus_in_t;

typedef union {
    frc_dataframe_setpoint_out_t setpointOut;
    frc_dataframe_statusConfig_out_t statusConfigOut;
    frc_dataframe_setParam_out_t setParamOut;
    frc_dataframe_burnFlash_out_t burnFlashOut;
    frc_dataframe_follower_out_t followerOut;
    frc_dataframe_firmware_in_t firmwareIn;
    frc_dataframe_status0_in_t status0In;
    frc_dataframe_status1_in_t status1In;
    frc_dataframe_status2_in_t status2In;
    frc_dataframe_status3_in_t status3In;
    frc_dataframe_status4_in_t status4In;
    frc_dataframe_status5_in_t status5In;
    frc_dataframe_status6_in_t status6In;
    frc_dataframe_userStatus_in_t userStatusIn;
    frc_dataframe_drvStatus_in_t drvStatusIn;
    frc_dataframe_getParam_in_t getParamIn;
    uint8_t data[8];
} frc_dataframe_t;

typedef enum {
    deviceBroadcast = 0,
    robotController,
    motorController,
    relayController,
    gyroSensor,
    accelerometerSensor,
    ultrasonicSensor,
    gearToothSensor,
    powerDistribution,
    pneumaticsController,
    miscCANDevice,
    IOBreakout,
    dev_rsvd12,
    dev_rsvd13,
    dev_rsvd14,
    dev_rsvd15,
    dev_rsvd16,
    dev_rsvd17,
    dev_rsvd18,
    dev_rsvd19,
    dev_rsvd20,
    dev_rsvd21,
    dev_rsvd22,
    dev_rsvd23,
    dev_rsvd24,
    dev_rsvd25,
    dev_rsvd26,
    dev_rsvd27,
    dev_rsvd28,
    dev_rsvd29,
    dev_rsvd30,
    firmwareUpdate = 31
} frc_deviceType_t;

typedef enum {
    manufacturerBroadcast = 0,
    NI = 1,
    LM = 2,  // (TI)
    DEKA = 3,
    CTRE = 4,
    REV = 5,
    Grapple = 6,
    MindSensors = 7,
    TeamUse = 8
} frc_manufacturer_t;

typedef struct PACKED {
    uint16_t deviceNumber : 6;
    uint16_t api : 10;
    frc_manufacturer_t manufacturer : 8;
    frc_deviceType_t deviceType : 5;
    uint8_t rsvd : 3;  // these are DNC
} frc_frameIDFields_t;

typedef union {
    frc_frameIDFields_t fields;
    uint32_t raw;
} frc_frameID_t;

typedef struct {
    frc_frameID_t id;
    frc_dataframe_t dataframe;
    size_t length;
} frc_frame_t;

#ifdef _WIN32
#pragma pack(pop)
#endif

#undef PACKED

}  // extern "C"
