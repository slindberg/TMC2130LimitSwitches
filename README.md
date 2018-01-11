# TMC2130 Sensor-less Limit Switches

This is an Arduino sketch that configures a [TMC2130 SilentStepStick](https://github.com/watterott/SilentStepStick) to use its StallGuard feature for axis limit switches. It assumes external control of the stepper driver (enable, step, direction), and provides two active low outputs for each limit switch.
