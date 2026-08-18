# Arduino PID Controller

A lightweight Arduino-compatible PID controller with additional features for smooth and stable control of slow or overshoot-prone processes.

## Features

- Proportional, integral and derivative control
- Derivative-on-measurement
- Derivative low-pass filtering
- Integral activation range
- Integral tolerance band
- Conditional integration / anti-windup
- Direct and reverse controller direction
- Configurable output limits
- Output slew-rate limiting
- Rollover-safe `millis()` timing
- Independent operation of multiple PID instances

## Basic Usage

```cpp
#include "pidController.hpp"

PID pid;

void setup()
{
    pid.setDirection(PID::Direction::DIRECT);

    pid.setLimits(0.0, 1023.0);
    pid.setSampletime(100);

    pid.setTolerance(2);
    pid.setIntegralActivationRange(10.0);

    pid.setDerivativeFilter(0.5);
    pid.setOutputSlewRate(150.0);

    pid.setTunings(
        5.0,    // Kp
        0.3,    // Ki
        2.0);   // Kd
}

void loop()
{
    const double setpoint = 100.0;
    const double actual = readProcessValue();

    const double output =
        pid.calc(setpoint, actual);

    applyControllerOutput(output);
}
```

## Documentation

Detailed documentation, controller concepts, API reference, tuning recommendations and implementation notes are available in the project Wiki:

**[Arduino PID Wiki](https://github.com/AlexanderTonn/Arduino_PID/wiki)**

The Wiki repository can also be cloned directly:

```bash
git clone https://github.com/AlexanderTonn/Arduino_PID.wiki.git
```

## Notes

PID parameters from other implementations may require retuning because this controller uses time-correct integral and derivative calculations in seconds.

For overshoot-prone systems, start with the proportional term, add derivative action for damping, and introduce only a small integral term afterwards.
