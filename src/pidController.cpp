#include "pidController.hpp"

/**
 * @brief Set PID tunings
 */
auto PID::setTunings(
    const double Kp,
    const double Ki,
    const double Kd) -> void
{
    // Negative PID parameters should normally not be required.
    // Controller direction is handled separately.
    mKp = Kp >= 0.0 ? Kp : 0.0;
    mKi = Ki >= 0.0 ? Ki : 0.0;
    mKd = Kd >= 0.0 ? Kd : 0.0;
}


/**
 * @brief Reset PID controller
 */
auto PID::reset() -> void
{
    mIntegral = 0.0;

    mPrevActual = 0.0;
    mDerivativeFiltered = 0.0;

    mLastComputeMs = millis();

    mInitialized = false;

    mOut = clamp(mOut, mMin, mMax);
}


/**
 * @brief Set output limits
 */
auto PID::setLimits(
    const double min,
    const double max) -> void
{
    if (max <= min)
        return;

    mMin = min;
    mMax = max;

    mOut = clamp(mOut, mMin, mMax);
}


/**
 * @brief Set sample time in milliseconds
 */
auto PID::setSampletime(
    const uint32_t sampleTimeMs) -> void
{
    if (sampleTimeMs == 0)
        return;

    mSampleTimeMs = sampleTimeMs;
}


/**
 * @brief Set controller direction
 */
auto PID::setDirection(
    const Direction direction) -> void
{
    mDirection = direction;
}


/**
 * @brief Set tolerance around setpoint
 */
auto PID::setTolerance(
    const uint8_t percentage) -> void
{
    if (percentage > 100)
        return;

    mTolerance =
        static_cast<double>(percentage) / 100.0;
}


/**
 * @brief Set area around setpoint where integral is allowed
 */
auto PID::setIntegralActivationRange(
    const double range) -> void
{
    if (range < 0.0)
        return;

    mIntegralActivationRange = range;
}


/**
 * @brief Configure derivative low-pass filter
 */
auto PID::setDerivativeFilter(
    const double tauSeconds) -> void
{
    if (tauSeconds < 0.0)
        return;

    mDerivativeFilterTau = tauSeconds;
}


/**
 * @brief Set maximum output change per second
 */
auto PID::setOutputSlewRate(
    const double unitsPerSecond) -> void
{
    if (unitsPerSecond < 0.0)
        return;

    mOutputSlewRate = unitsPerSecond;
}


/**
 * @brief Calculate PID controller
 */
auto PID::calc(
    const double setpoint,
    const double actual) -> double
{
    const uint32_t now = millis();

    /*
     * Rollover-safe time comparison.
     *
     * This also works when millis() wraps after ~49 days.
     */
    const uint32_t elapsedMs =
        static_cast<uint32_t>(now - mLastComputeMs);

    if (mInitialized &&
        elapsedMs < mSampleTimeMs)
    {
        return mOut;
    }

    /*
     * First PID call.
     *
     * We cannot calculate a derivative yet because there
     * is no previous measurement.
     */
    if (!mInitialized)
    {
        mPrevActual = actual;
        mDerivativeFiltered = 0.0;

        mLastComputeMs = now;
        mInitialized = true;

        return mOut;
    }

    /*
     * Actual elapsed time in seconds.
     *
     * Do not simply use mSampleTimeMs here because the
     * Arduino loop can have some timing jitter.
     */
    const double dt =
        static_cast<double>(elapsedMs) / 1000.0;

    mLastComputeMs = now;

    if (dt <= 0.0)
        return mOut;

    /*
     * ----------------------------------------------------
     * Controller direction
     * ----------------------------------------------------
     *
     * DIRECT:
     *
     * setpoint > actual
     * -> positive error
     * -> increase output
     *
     *
     * REVERSE:
     *
     * actual > setpoint
     * -> positive controller error
     * -> increase output
     */
    const double direction =
        (mDirection == Direction::DIRECT)
        ? 1.0
        : -1.0;

    const double rawError =
        setpoint - actual;

    const double error =
        direction * rawError;


    /*
     * ----------------------------------------------------
     * P TERM
     * ----------------------------------------------------
     */
    const double p =
        mKp * error;


    /*
     * ----------------------------------------------------
     * D TERM
     * ----------------------------------------------------
     *
     * Derivative is calculated from ACTUAL instead of ERROR.
     *
     * Advantage:
     *
     * - no derivative kick when setpoint changes
     * - controller recognizes rapid approach to setpoint
     *
     *
     * Example DIRECT:
     *
     * actual rises quickly
     * -> derivativeActual positive
     * -> D negative
     * -> output is reduced BEFORE reaching setpoint
     */
    const double derivativeActual =
        (actual - mPrevActual) / dt;

    double rawDerivative =
        -direction * derivativeActual;


    /*
     * Low-pass filter for derivative.
     *
     * First-order PT1:
     *
     * alpha = dt / (tau + dt)
     */
    if (mDerivativeFilterTau > 0.0)
    {
        const double alpha =
            dt /
            (mDerivativeFilterTau + dt);

        mDerivativeFiltered +=
            alpha *
            (rawDerivative - mDerivativeFiltered);
    }
    else
    {
        // Filtering disabled
        mDerivativeFiltered = rawDerivative;
    }

    const double d =
        mKd * mDerivativeFiltered;


    /*
     * Save current process value for next calculation.
     */
    mPrevActual = actual;


    /*
     * ----------------------------------------------------
     * INTEGRAL CONDITIONS
     * ----------------------------------------------------
     */

    /*
     * Tolerance is relative to setpoint.
     *
     * Example:
     *
     * Setpoint = 100
     * tolerance = 2 %
     *
     * -> toleranceAbsolute = 2
     */
    const double toleranceAbsolute =
        fabs(setpoint) * mTolerance;

    /*
     * Inside tolerance:
     * stop integration.
     *
     * IMPORTANT:
     *
     * P and D remain active!
     */
    const bool outsideTolerance =
        fabs(rawError) > toleranceAbsolute;


    /*
     * Optional integral activation range.
     *
     * When set to 0:
     * integral can work everywhere.
     *
     * When e.g. set to 10:
     * integral only starts when error <= 10.
     */
    bool insideIntegralRange = true;

    if (mIntegralActivationRange > 0.0)
    {
        insideIntegralRange =
            fabs(rawError)
            <= mIntegralActivationRange;
    }


    /*
     * Integrate only when:
     *
     * - Ki is actually enabled
     * - outside tolerance
     * - inside integral activation range
     */
    const bool integrationAllowed =
        (mKi > 0.0) &&
        outsideTolerance &&
        insideIntegralRange;


    /*
     * ----------------------------------------------------
     * CONDITIONAL INTEGRATION / ANTI-WINDUP
     * ----------------------------------------------------
     *
     * First create a possible new integral value.
     *
     * It is not committed until we know whether it would
     * drive the controller further into saturation.
     */
    double integralCandidate =
        mIntegral;

    if (integrationAllowed)
    {
        integralCandidate +=
            error * dt;
    }


    /*
     * Calculate candidate controller output.
     */
    const double iCandidate =
        mKi * integralCandidate;

    const double candidateOutput =
        p +
        iCandidate +
        d;


    /*
     * Detect whether integral would cause/increase saturation.
     *
     * Positive error wants to increase output.
     * Negative error wants to decrease output.
     */
    bool integralDrivesIntoSaturation = false;

    if (candidateOutput > mMax &&
        error > 0.0)
    {
        integralDrivesIntoSaturation = true;
    }

    if (candidateOutput < mMin &&
        error < 0.0)
    {
        integralDrivesIntoSaturation = true;
    }


    /*
     * Commit integral only if it does not worsen saturation.
     */
    if (integrationAllowed &&
        !integralDrivesIntoSaturation)
    {
        mIntegral =
            integralCandidate;
    }


    /*
     * Actual I term after anti-windup decision.
     */
    const double i =
        mKi * mIntegral;


    /*
     * ----------------------------------------------------
     * PID OUTPUT
     * ----------------------------------------------------
     */
    double output =
        p +
        i +
        d;


    /*
     * Hard output limits
     */
    output =
        clamp(
            output,
            mMin,
            mMax);


    /*
     * ----------------------------------------------------
     * OUTPUT SLEW RATE LIMITER
     * ----------------------------------------------------
     *
     * Prevent sudden output jumps.
     *
     * Example:
     *
     * rate = 100 / second
     * dt   = 0.1 s
     *
     * maximum change per calculation = 10
     */
    if (mOutputSlewRate > 0.0)
    {
        const double maxChange =
            mOutputSlewRate * dt;

        const double change =
            output - mOut;

        if (change > maxChange)
        {
            output =
                mOut + maxChange;
        }
        else if (change < -maxChange)
        {
            output =
                mOut - maxChange;
        }
    }


    /*
     * Safety clamp after slew-rate limiter.
     */
    mOut =
        clamp(
            output,
            mMin,
            mMax);

    return mOut;
}