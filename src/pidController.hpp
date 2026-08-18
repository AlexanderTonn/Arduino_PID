#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <Arduino.h>
#include <math.h>

class PID
{
public:
    enum class Direction
    {
        DIRECT = 0,
        REVERSE = 1
    };

    /**
     * @brief Set PID gains
     *
     * Kp: proportional gain
     * Ki: integral gain [1/s]
     * Kd: derivative gain [s]
     *
     * @note Changing the tunings does NOT reset the controller.
     *       This avoids jumps in the output during runtime tuning.
     */
    auto setTunings(
        double Kp,
        double Ki,
        double Kd) -> void;

    /**
     * @brief Calculate PID output
     *
     * @param setpoint Desired value
     * @param actual   Current process value
     *
     * @return Current controller output
     */
    auto calc(
        double setpoint,
        double actual) -> double;

    /**
     * @brief Reset internal PID states
     */
    auto reset() -> void;

    /**
     * @brief Set controller output limits
     */
    auto setLimits(
        double min,
        double max) -> void;

    /**
     * @brief Set calculation interval
     *
     * @param sampleTimeMs Interval in milliseconds
     */
    auto setSampletime(
        uint32_t sampleTimeMs) -> void;

    /**
     * @brief Set controller direction
     *
     * DIRECT:
     * Output increases when actual < setpoint
     *
     * REVERSE:
     * Output increases when actual > setpoint
     */
    auto setDirection(
        Direction direction) -> void;

    /**
     * @brief Set tolerance around setpoint
     *
     * Inside the tolerance range the integral term
     * is frozen. P and D remain active.
     *
     * Example:
     * setTolerance(2) -> +/- 2 %
     */
    auto setTolerance(
        uint8_t percentage) -> void;

    /**
     * @brief Set range in which the integral term is active.
     *
     * Example:
     *
     * setIntegralActivationRange(10);
     *
     * I is only active if:
     *
     * abs(setpoint - actual) <= 10
     *
     * 0 disables this restriction.
     */
    auto setIntegralActivationRange(
        double range) -> void;

    /**
     * @brief Set derivative low-pass filter time constant
     *
     * Example:
     * 0.5 = approximately 500 ms filtering
     *
     * 0 disables filtering.
     */
    auto setDerivativeFilter(
        double tauSeconds) -> void;

    /**
     * @brief Limit rate of output change
     *
     * Unit:
     * output units / second
     *
     * Example:
     *
     * Output = 0 ... 1023
     * setOutputSlewRate(100);
     *
     * -> output can change by max. 100 / second.
     *
     * 0 disables slew-rate limiting.
     */
    auto setOutputSlewRate(
        double unitsPerSecond) -> void;

    /**
     * @brief Return current PID output
     */
    auto getOutput() const -> double
    {
        return mOut;
    }

private:

    // PID gains
    double mKp = 1.0;
    double mKi = 0.0;
    double mKd = 0.0;

    // PID states
    double mIntegral = 0.0;
    double mPrevActual = 0.0;

    // Filtered derivative
    double mDerivativeFiltered = 0.0;

    // Output
    double mOut = 0.0;

    // Output limits
    double mMin = 0.0;
    double mMax = 1023.0;

    // Sample time
    uint32_t mSampleTimeMs = 100;
    uint32_t mLastComputeMs = 0;

    // Controller configuration
    double mTolerance = 0.0;

    // 0 = integral active everywhere
    double mIntegralActivationRange = 0.0;

    // Derivative filter time constant [s]
    double mDerivativeFilterTau = 0.5;

    // Maximum output change per second
    // 0 = disabled
    double mOutputSlewRate = 0.0;

    Direction mDirection = Direction::DIRECT;

    bool mInitialized = false;

    /**
     * @brief Clamp value between min and max
     */
    static auto clamp(
        double value,
        double min,
        double max) -> double
    {
        if (value > max)
            return max;

        if (value < min)
            return min;

        return value;
    }
};

#endif