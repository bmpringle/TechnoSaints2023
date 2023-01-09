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

#include <stdint.h>

#include "rev/AbsoluteEncoder.h"
#include "rev/REVLibError.h"

namespace rev {

class CANSparkMax;

/**
 * Get an instance of this class by using CANSparkMax::GetEncoder() or
 * CANSparkMax::GetEncoder(SparkMaxRelativeEncoder::Type, int).
 */
class SparkMaxAbsoluteEncoder : public AbsoluteEncoder {
    // Friend to allow construction
    friend class CANSparkMax;

public:
    /** The type of encoder connected to a SPARK MAX */
    enum class Type { kDutyCycle };

    SparkMaxAbsoluteEncoder(SparkMaxAbsoluteEncoder&& rhs) = default;
    SparkMaxAbsoluteEncoder& operator=(SparkMaxAbsoluteEncoder&& rhs) = default;

    SparkMaxAbsoluteEncoder(const SparkMaxAbsoluteEncoder& rhs) = default;

    ~SparkMaxAbsoluteEncoder() override = default;

    /**
     * Get the position of the motor. This returns the native units
     * of 'rotations' by default, and can be changed by a scale factor
     * using setPositionConversionFactor().
     *
     * @return Number of rotations of the motor
     */
    double GetPosition() const override;

    /**
     * Get the velocity of the motor. This returns the native units
     * of 'rotations per second' by default, and can be changed by a scale
     * factor using setVelocityConversionFactor().
     *
     * @return Number of rotations per second of the motor
     */
    double GetVelocity() const override;

    /**
     * Set the conversion factor for position of the encoder. Multiplied by the
     * native output units to give you position
     *
     * @param factor The conversion factor to multiply the native units by
     *
     * @return REVLibError::kOk if successful
     */
    REVLibError SetPositionConversionFactor(double factor) override;

    /**
     * Get the conversion factor for position of the encoder. Multiplied by the
     * native output units to give you position
     *
     * @return The conversion factor for position
     */
    double GetPositionConversionFactor() const override;

    /**
     * Set the conversion factor for velocity of the encoder. Multiplied by the
     * native output units to give you velocity
     *
     * @param factor The conversion factor to multiply the native units by
     *
     * @return REVLibError::kOk if successful
     */
    REVLibError SetVelocityConversionFactor(double factor) override;

    /**
     * Get the conversion factor for velocity of the encoder. Multiplied by the
     * native output units to give you velocity
     *
     * @return The conversion factor for velocity
     */
    double GetVelocityConversionFactor() const override;

    /**
     * Set the phase of the AbsoluteEncoder so that it is set to be in phase
     * with the motor itself
     *
     * @param inverted The phase of the encoder
     *
     * @return REVLibError::kOk if successful
     */
    REVLibError SetInverted(bool inverted) override;

    /**
     * Get the phase of the AbsoluteEncoder
     *
     * @return The phase of the encoder
     */
    bool GetInverted() const override;

    /**
     * Set the average sampling depth for an absolute encoder. This is a bit
     * size and should be either 1, 2, 4, 8, 16, 32, 64, or 128
     *
     * @param depth The average sampling depth of 1, 2, 4, 8, 16, 32, 64, or 128
     *
     * @return REVLibError::kOk if successful
     */
    REVLibError SetAverageDepth(uint32_t depth) override;

    /**
     * Get the average sampling depth for an absolute encoder
     *
     * @return The average sampling depth
     */
    uint32_t GetAverageDepth() const override;

#if 0
    /**
     * Set the sample delta for an absolute encoder
     *
     * @param delta
     * @return REVLibError::kOk if successful
     */
    REVLibError SetSampleDelta(uint32_t delta) override;

    /**
     * Get the sample delta for an absolute encoder
     *
     * @return uint32_t
     */
    uint32_t GetSampleDelta() const override;
#endif

    /**
     * Set the zero offset for an absolute encoder. The offset should include
     * the position conversion factor
     *
     * @param offset The zero offset with the position conversion factor applied
     *
     * @return REVLibError::kOk if successful
     */
    REVLibError SetZeroOffset(double offset) override;

    /**
     * Get the zero offset for an absolute encoder. The offset will include the
     * position conversion factor
     *
     * @return The zero offset of the absolute encoder with the position
     * conversion factor applied
     */
    double GetZeroOffset() const override;

private:
    CANSparkMax* m_device;

    explicit SparkMaxAbsoluteEncoder(CANSparkMax& device, Type Type);

    int GetSparkMaxFeedbackDeviceID() const override;
};

}  // namespace rev
