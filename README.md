# BasicMotor

BasicMotor is a Java library designed to simplify motor control logic for FRC (FIRST Robotics Competition) robots. It provides a unified API for interacting with different types of motors, making robot code cleaner and easier to maintain.

## Features

- Unified motor interface for FRC robots
- Built-in support for simulation environments
- Integrated PID control for precise motor management
- Automatic logging of motor data using AdvantageKit for easy telemetry and debugging
- Multithreaded architecture for high-speed, responsive motor updates

## Installation

Before using BasicMotor, ensure you have the following libraries installed in your project:

- [AdvantageKit](https://docs.advantagekit.org/getting-started/installation)
- [Phoenix 6](https://v6.docs.ctr-electronics.com/en/stable/docs/installation/installation-frc.html)
- [REVLib](https://docs.revrobotics.com/revlib/install)

after installing these libraries install the BasicMotor Library with the following url:

```https://captainsoccer.github.io/MotorUtils/BasicMotor.json```

or download it manually to your vendordep folder


## Usage

To ensure motors are updated correctly, add the following code to your `robotPeriodic()` method in the `Robot.java` file, after the `CommandScheduler` runs:

```java
@Override
public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    MotorManager.getInstance().periodic();
}
```

## License

This project is licensed under the MIT License.

[![](https://jitpack.io/v/captainsoccer/MotorUtils.svg)](https://jitpack.io/#captainsoccer/MotorUtils)
