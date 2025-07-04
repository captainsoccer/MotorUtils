package com.basicMotor;

import edu.wpi.first.wpilibj.Notifier;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class MotorManager{

  /** the frequency of the PID loop (in Hz) used for the PID loop */
  public static final double PID_LOOP_HZ = 100;
  /**
   * the frequency of the profile loop (in Hz) used for the profile loop used for the run function
   * when the controller runs on the motor
   */
  public static final double PROFILE_LOOP_HZ = 50;
  /**
   * the frequency of the sensor loop (in Hz) used for the sensor loop used for the updateSensorData
   * function when the controller runs on the motor
   */
  public static final double SENSOR_LOOP_HZ = 4;

  /**
   * the idle voltage fed into the motor when it is not moving (in volts) used for duty cycle
   * calculations
   */
  public static final double motorIdleVoltage = 12;

  /** the maximum output of the motor (in volts) */
  public static final double defaultMaxMotorOutput = 13.0;

  /** the instance of the motor manager used to manage the motors */
  private static MotorManager instance;

  /** the list of motors used to manage the motors */
  private final ArrayList<MotorHandler> motors = new ArrayList<>();

  private MotorManager() {
  }

  /**
   * get the instance of the motor manager
   *
   * @return the instance of the motor manager
   */
  public static MotorManager getInstance() {
    if (instance == null) {
      instance = new MotorManager();
    }
    return instance;
  }

  /**
   * you need to call this method in the periodic method of your subsystem to log the
   */
  public void periodic() {
    for (MotorHandler motor : motors) {
      var frame = motor.frameSupplier.get();
      Logger.processInputs("Motors/" + motor.motorName, frame);
    }
  }

  /** stop the PID loop for all motors useful for replay simulation */
  public void stopSensorLoop() {
    for (MotorHandler motor : motors) {
      motor.sensorLoop.stop();
    }
  }

  /** stop the PID loop for all motors useful for replay simulation */
  public void stopPIDLoop() {
    for (MotorHandler motor : motors) {
      motor.pidLoop.stop();
    }
  }

  /** start the sensor loop for all motors if you accidentally do something dumb */
  public void startSensorLoop() {
    for (MotorHandler motor : motors) {
      motor.sensorLoop.startPeriodic(1 / SENSOR_LOOP_HZ);
    }
  }

  /** start the PID loop for all motors if you accidentally do something dumb */
  public void startPIDLoop() {
    for (MotorHandler motor : motors) {
      motor.pidLoop.startPeriodic(1 / motor.location.HZ);
    }
  }

  /**
   * register a motor with the motor manager
   *
   * @param name the name of the motor
   * @param location the location of the motor (RIO or Motor Controller)
   * @param run the function to run for the PID loop
   * @param sensorLoopFunction the function to run for the sensor loop
   * @param frameSupplier the function to get the frame for the motor
   */
  public void registerMotor(
      String name,
      ControllerLocation location,
      Runnable run,
      Runnable sensorLoopFunction,
      Supplier<LogFrame.LogFrameAutoLogged> frameSupplier) {
    var handler = new MotorHandler(name, location, run, sensorLoopFunction, frameSupplier);

    motors.add(handler);

    handler.sensorLoop.startPeriodic(1 / SENSOR_LOOP_HZ);
    handler.pidLoop.startPeriodic(1 / location.HZ);
  }

  /** a class to handle the motor */
  private static class MotorHandler {
    /** the name of the motor */
    private final String motorName;
    /** the location of the motor (RIO or Motor Controller) */
    private final ControllerLocation location;

    /** the thread to run for the PID loop */
    private final Notifier pidLoop;

    /** the thread to run for the sensor loop */
    private final Notifier sensorLoop;

    /** the function to get the frame for the motor */
    private final Supplier<LogFrame.LogFrameAutoLogged> frameSupplier;

    public MotorHandler(
        String motorName,
        ControllerLocation location,
        Runnable run,
        Runnable sensorLoopFunction,
        Supplier<LogFrame.LogFrameAutoLogged> frameSupplier) {
      this.motorName = motorName;
      this.location = location;

      pidLoop = new Notifier(run);
      sensorLoop = new Notifier(sensorLoopFunction);

      this.frameSupplier = frameSupplier;
    }
  }

  public enum ControllerLocation {
    MOTOR(PROFILE_LOOP_HZ),
    RIO(PID_LOOP_HZ);

    public final double HZ;

    ControllerLocation(double hz) {
      this.HZ = hz;
    }
  }
}
