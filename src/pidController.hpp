#ifndef PID_HPP
#define PID_HPP

#include <Arduino.h>

class PID
{
    public: 
        auto setTunings(double Kp, double Ki, double Kd, double dt) -> void;
        auto calc(double setpoint, double actual) -> double;
        auto reset() -> void;

    enum class Direction
    {
        DIRECT = 0,
        REVERSE = 1
    }mDirection;
        auto setDirection(Direction) -> void;
        auto setLimits(const double min, const double max) -> void;
        auto setSampletime(const double sampletime) -> void;
        auto setSetpoint(const double setpoint) -> void;
        

    private: 
        double mIntegral, mPrev_error;
        double mKp = 1.0;
        double mKi = 1.0;
        double mKd = 0.0;
        double mSampletime = 100.0;
        double mMin = 0.0, mMax = 1023.0; 
        double mOut;
        auto assignDirection(const double out) -> double;
        auto mapD(const double x, const double in_min, const double in_max, const double out_min, const double out_max) -> double
        {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        };



};

#endif // PID
