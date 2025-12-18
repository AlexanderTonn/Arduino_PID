#include "pidController.hpp"
/**
 * @brief 
 * 
 * @param Kp 
 * @param Ki 
 * @param Kd 
 * @param dt 
 */
auto PID::setTunings(const double Kp, const double Ki, const double Kd, const double sampletime) -> void
{
    mKp = Kp;
    mKi = Ki;
    mKd = Kd;
    mSampletime = sampletime;

    // reset the controller if the values has been changed
    static double kpChanged = 0.0, kiChanged = 0.0, kdChanged = 0.0;
    if (mKp != kpChanged || mKi != kiChanged || mKd != kdChanged)
    {
        kpChanged = mKp;
        kiChanged = mKi;
        kdChanged = mKd;
        reset();
    }

}
/**
 * @brief Reset the PID controller
 * 
 */
auto PID::reset() -> void
{
    mIntegral = 0;
    mPrev_error = 0;
}
/**
 * @brief  Set the limits for the output signal
 * 
 * @param min 
 * @param max 
 */
auto PID::setLimits(const double min, const double max) -> void
{
    mMin = min;
    mMax = max;
}
/**
 * @brief  Set the sample time for the PID controller in milliseconds
 * 
 * @param sampletime 
 */
auto PID::setSampletime(const double sampletime) -> void
{
    mSampletime = sampletime;
}

/**
 * @brief  Calculate the PID output
 * 
 * @param setpoint 
 * @param actual 
 * @return double 
 */
auto PID::calc(const double setpoint, const double actual) -> double 
{
    auto static millisPrev = millis();
    
    if(millis() < millisPrev + mSampletime)
        return mOut; 

    auto error = setpoint - actual;
    millisPrev = millis();

    // Proportional term
    auto p = mKp * error;

    // Integral term
    mIntegral += error;
    auto i = mKi * mIntegral;

    // Derivative term
    auto derivative = (error - mPrev_error)  / mSampletime;
    auto d = mKd * derivative;

    // output
    mOut = p + i + d;

    // save for next iteration
    mPrev_error = error;

    // Limit the output signal range
    if(mOut > mMax)
    {
        mOut = mMax;
        // Prevent integral wind-up
        mIntegral -= error;
    }
    else if(mOut < mMin)
    {
        mOut = mMin;
        // Prevent integral wind-up
        mIntegral -= error;
    }

    return mOut;
}
/**
 * @brief Invert the output signal
 * @note DIRECT means the output signal will be increased if the setpoint is lower than the actual value
 * @note REVERSE means the output signal will be increased if the setpoint is higher than the actual value
 * @param PID::Direction
 */
auto PID::setDirection(PID::Direction direction = PID::Direction::DIRECT) -> void
{
    mDirection = direction;
}

auto PID::assignDirection(const double out) -> double
{
    if(mDirection == Direction::DIRECT)
    {
        return  mapD(out, mMin, mMax, mMin, mMax);
    }
    else
    {
        return mapD(out, mMin, mMax, mMax, mMin);
    }
}