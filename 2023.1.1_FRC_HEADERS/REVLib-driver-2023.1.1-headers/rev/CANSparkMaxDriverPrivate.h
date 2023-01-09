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

#include <hal/Types.h>

#include "rev/CANSparkMaxDriver.h"

extern "C" {

struct c_SparkMax_Obj {
    std::mutex* m_mutex;  // TODO(Noah): Use consistently

    c_SparkMax_FirmwareVersion m_firmwareVersion;
    c_SparkMax_DataPortConfig m_dataPortConfig;

    bool m_inverted;

    int m_deviceId;

    int m_canTimeoutMs;
    int m_controlFramePeriod;
    int m_status0PeriodMs;
    int m_status1PeriodMs;
    int m_status2PeriodMs;
    int m_status3PeriodMs;
    int m_status4PeriodMs;
    int m_status5PeriodMs;
    int m_status6PeriodMs;

    float m_sensorRangeMin;
    float m_sensorRangeMax;

    HAL_CANHandle m_canHandle;

    int m_activeSetpointApi;

    c_SIM_SparkMax_handle m_simDevice;
};

}  // extern "C"
