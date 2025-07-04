package com.basicMotor;

import com.basicMotor.controllers.Controller;
import com.basicMotor.measurements.Measurements;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * A class that holds the latest frame of the controller, PID output, sensor data, and measurement
 * of the motor.
 * This is used to log the data of the motor and to provide a snapshot of the motor's
 * state.
 * this class is used for logging the data of the motor
 */
public class LogFrame {
  /** the latest frame of the controller */
  public ControllerFrame controllerFrame = ControllerFrame.EMPTY;
  /** the latest PID output of the controller */
  public PIDOutput pidOutput = PIDOutput.EMPTY;
  /** the latest sensor data of the motor */
  public SensorData sensorData = SensorData.EMPTY;
  /** the latest measurement of the motor */
  public Measurements.Measurement measurement = Measurements.Measurement.EMPTY;

  /** if the motor is at setpoint */
  public boolean atSetpoint = false;

  /** if the motor is at goal (used when using a profile) */
  public boolean atGoal = false;

  /** the torque the motor is applying (in Newton-meters) */
  public double appliedTorque = 0;

  /**
   * the record holding the sensor data of the motor
   *
   * @param temperature the temperature of the motor (in celsius)
   * @param currentDraw the current draw of the motor (in amps)
   * @param currentOutput the current output of the motor (in amps)
   * @param voltageOutput the voltage output of the motor (in volts)
   * @param voltageInput the voltage input of the motor (in volts)
   * @param powerDraw the power draw of the motor (in watts)
   * @param powerOutput the power output of the motor (in watts)
   * @param dutyCycle the duty cycle of the motor (0-1)
   */
  public record SensorData(
      double temperature,
      double currentDraw,
      double currentOutput,
      double voltageOutput,
      double voltageInput,
      double powerDraw,
      double powerOutput,
      double dutyCycle) {

    /**
     * empty sensor data used when no sensors are logged.
     */
    public static final SensorData EMPTY = new SensorData(0, 0, 0, 0, 0, 0, 0, 0);
  }

  /**
   * the record holding the PID output of the controller
   * all units are in volts
   *
   * @param pOutput the P output of the controller
   * @param iOutput the I output of the controller
   * @param dOutput the D output of the controller
   * @param totalOutput the total output of the controller
   */
  public record PIDOutput(double pOutput, double iOutput, double dOutput, double totalOutput) {
    /**
     * empty PID output used when no PID controller is active
     */
    public static final PIDOutput EMPTY = new PIDOutput(0, 0, 0, 0);
  }

  /**
   * the record holding the feedforward output of the controller
   * all units are in volts
   *
   * @param simpleFeedForward the simple feedforward output of the controller
   * @param frictionFeedForward the friction feedforward output of the controller
   * @param setPointFeedForward the setpoint feedforward output of the controller
   * @param calculatedFeedForward the calculated feedforward output of the controller
   * @param totalOutput the total output of the controller
   */
  public record FeedForwardOutput(
      double simpleFeedForward,
      double frictionFeedForward,
      double setPointFeedForward,
      double calculatedFeedForward,
      double arbitraryFeedForward,
      double totalOutput) {

    /** empty feedforward output used when no feedForward is active */
    public static final FeedForwardOutput EMPTY = new FeedForwardOutput(0, 0, 0, 0, 0);

    /**
     * Creates a new feedforward output with the given parameters
     * all units are in volts
     * @param simpleFeedForward the simple feedforward output of the controller
     * @param frictionFeedForward the friction feedforward output of the controller
     * @param setPointFeedForward the setpoint feedforward output of the controller
     * @param calculatedFeedForward the feedforward calculated from the feedforward function
     * @param arbitraryFeedForward the arbitrary feedforward output of the controller
     */
    public FeedForwardOutput(
        double simpleFeedForward,
        double frictionFeedForward,
        double setPointFeedForward,
        double calculatedFeedForward,
        double arbitraryFeedForward) {
      this(
          simpleFeedForward,
          frictionFeedForward,
          setPointFeedForward,
          calculatedFeedForward,
          arbitraryFeedForward,
          simpleFeedForward
              + frictionFeedForward
              + setPointFeedForward
              + calculatedFeedForward
              + arbitraryFeedForward);
    }
  }

  /**
   * The frame of the controller
   *
   * @param totalOutput the total output of the controller (in volts)
   * @param feedForwardOutput the feedforward output of the controller
   * @param setpoint the setpoint of the controller in units of measurement
   * @param measurement the measurement of the controller in units of measurement
   * @param error the error of the controller in units of measurement
   * @param goal the goal of the controller in units of measurement (used when using a profile)
   * @param mode the mode of the controller (position, velocity, voltage, percent output)
   */
  public record ControllerFrame(
      double totalOutput,
      FeedForwardOutput feedForwardOutput,
      double setpoint,
      double measurement,
      double error,
      double goal,
      Controller.RequestType mode) {

    /** empty controller frame used when stopping the motor */
    public static final ControllerFrame EMPTY =
        new ControllerFrame(0, FeedForwardOutput.EMPTY, 0, 0, 0, 0, Controller.RequestType.STOP);

    /**
     * Creates a new controller frame with the given parameters
     * @param output the total output of the controller (in volts)
     * @param wanted the setpoint of the controller in units of measurement
     * @param measurement the measurement of the controller in units of measurement
     * @param mode the mode of the controller (position, velocity, voltage, percent output)
     */
    public ControllerFrame(
        double output, double wanted, double measurement, Controller.RequestType mode) {
      this(output, FeedForwardOutput.EMPTY, wanted, measurement, 0, 0, mode);
    }
  }

  /**
   * the logged version of the LogFrame that implements LoggableInputs
   */
  public static class LogFrameAutoLogged extends LogFrame implements LoggableInputs, Cloneable {
    @Override
    public void toLog(LogTable table) {
      table.put("ControllerFrame", controllerFrame);
      table.put("PidOutput", pidOutput);
      table.put("SensorData", sensorData);
      table.put("Measurement", measurement);
      table.put("AtSetpoint", atSetpoint);
      table.put("AtGoal", atGoal);
      table.put("AppliedTorque", appliedTorque);
    }

    @Override
    public void fromLog(LogTable table) {
      controllerFrame = table.get("ControllerFrame", controllerFrame);
      pidOutput = table.get("PidOutput", pidOutput);
      sensorData = table.get("SensorData", sensorData);
      measurement = table.get("Measurement", measurement);
      atSetpoint = table.get("AtSetpoint", atSetpoint);
      atGoal = table.get("AtGoal", atGoal);
      appliedTorque = table.get("AppliedTorque", appliedTorque);
    }

    public LogFrameAutoLogged clone() {
      LogFrameAutoLogged copy = new LogFrameAutoLogged();
      copy.controllerFrame = this.controllerFrame;
      copy.pidOutput = this.pidOutput;
      copy.sensorData = this.sensorData;
      copy.measurement = this.measurement;
      copy.atSetpoint = this.atSetpoint;
      copy.atGoal = this.atGoal;
      copy.appliedTorque = this.appliedTorque;
      return copy;
    }
  }
}
