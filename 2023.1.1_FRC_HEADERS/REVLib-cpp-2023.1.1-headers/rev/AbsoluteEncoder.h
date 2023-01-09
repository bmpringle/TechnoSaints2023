/*
 * Copyright (c) 2021-2023 REV Robotics
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

#ifdef _MSC_VER
// Disable deprecation warnings for this file when using VS compiler
#pragma warning(disable : 4996)
#endif

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#include "rev/CANEncoder.h"

namespace rev {

class AbsoluteEncoder : public MotorFeedbackSensor {
    friend class SparkMaxAbsoluteEncoder;

public:
    virtual ~AbsoluteEncoder() {}

    /**
     * Get the position of the motor. This returns the native units
     * of 'rotations' by default, and can be changed by a scale factor
     * using setPositionConversionFactor().
     *
     * @return Number of rotations of the motor
     */
    virtual double GetPosition() const = 0;

    /**
     * Get the velocity of the motor. This returns the native units
     * of 'rotations per second' by default, and can be changed by a scale
     * factor using setVelocityConversionFactor().
     *
     * @return Number of rotations per second of the motor
     */
    virtual double GetVelocity() const = 0;

    /**
     * Set the conversion factor for position of the encoder. Multiplied by the
     * native output units to give you position
     *
     * @param factor The conversion factor to multiply the native units by
     *
     * @return REVLibError::kOk if successful
     */
    virtual REVLibError SetPositionConversionFactor(double factor) = 0;

    /**
     * Get the conversion factor for position of the encoder. Multiplied by the
     * native output units to give you position
     *
     * @return The conversion factor for position
     */
    virtual double GetPositionConversionFactor() const = 0;

    /**
     * Set the conversion factor for velocity of the encoder. Multiplied by the
     * native output units to give you velocity
     *
     * @param factor The conversion factor to multiply the native units by
     *
     * @return REVLibError::kOk if successful
     */
    virtual REVLibError SetVelocityConversionFactor(double factor) = 0;

    /**
     * Get the conversion factor for velocity of the encoder. Multiplied by the
     * native output units to give you velocity
     *
     * @return The conversion factor for velocity
     */
    virtual double GetVelocityConversionFactor() const = 0;

    /**
     * Set the phase of the AbsoluteEncoder so that it is set to be in phase
     * with the motor itself
     *
     * @param inverted The phase of the encoder
     *
     * @return REVLibError::kOk if successful
     */
    virtual REVLibError SetInverted(bool inverted) = 0;

    /**
     * Get the phase of the AbsoluteEncoder
     *
     * @return The phase of the encoder
     */
    virtual bool GetInverted() const = 0;

    /**
     * Set the average sampling depth for an absolute encoder. This is a bit
     * size and should be either 1, 2, 4, 8, 16, 32, 64, or 128
     *
     * @param depth The average sampling depth of 1, 2, 4, 8, 16, 32, 64, or 128
     *
     * @return REVLibError::kOk if successful
     */
    virtual REVLibError SetAverageDepth(uint32_t depth) = 0;

    /**
     * Get the average sampling depth for an absolute encoder
     *
     * @return The average sampling depth
     */
    virtual uint32_t GetAverageDepth() const = 0;

#if 0
    /**
     * Set the sample delta for an absolute encoder
     *
     * @param delta
     * @return REVLibError::kOk if successful
     */
    virtual REVLibError SetSampleDelta(uint32_t delta) = 0;

    /**
     * Get the sample delta for an absolute encoder
     *
     * @return uint32_t
     */
    virtual uint32_t GetSampleDelta() const = 0;
#endif

    /**
     * Set the zero offset for an absolute encoder. The offset should include
     * the position conversion factor
     *
     * @param offset The zero offset with the position conversion factor applied
     *
     * @return REVLibError::kOk if successful
     */
    virtual REVLibError SetZeroOffset(double offset) = 0;

    /**
     * Get the zero offset for an absolute encoder. The offset will include the
     * position conversion factor
     *
     * @return The zero offset of the absolute encoder with the position
     * conversion factor applied
     */
    virtual double GetZeroOffset() const = 0;

private:
    AbsoluteEncoder() {}

    virtual int GetSparkMaxFeedbackDeviceID() const = 0;
};

}  // namespace rev

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
