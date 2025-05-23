package util.BasicMotor;

import org.littletonrobotics.junction.AutoLog;
import util.BasicMotor.Controllers.Controller;
import util.BasicMotor.Gains.ControllerConstrains;
import util.BasicMotor.Measurements.Measurements;

@AutoLog
public class LogFrame {
  /** the latest frame of the controller */
  public ControllerFrame controllerFrame;
  /** the latest sensor data of the motor */
  public SensorData sensorData;
  /** the latest measurement of the motor */
  public Measurements.Measurement measurement;

  /** if the motor is at setpoint */
  public boolean atSetpoint = false;

  /** if the motor is at goal (used when using a profile) */
  public boolean atGoal = false;

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
   * @param faults the faults of the motor (not supported by all motor controllers)
   */
  public record SensorData(
      double temperature,
      double currentDraw,
      double currentOutput,
      double voltageOutput,
      double voltageInput,
      double powerDraw,
      double powerOutput,
      double dutyCycle,
      String faults) {}

  /**
   * the record holding the PID output of the controller
   *
   * @param pOutput the P output of the controller (in volts)
   * @param iOutput the I output of the controller (in volts)
   * @param dOutput the D output of the controller (in volts)
   * @param totalOutput the total output of the controller (in volts)
   */
  public record PIDOutput(double pOutput, double iOutput, double dOutput, double totalOutput) {
    public PIDOutput() {
      this(0, 0, 0, 0);
    }
  }

  /**
   * the record holding the feedforward output of the controller
   *
   * @param simpleFeedForward the simple feedforward output of the controller (in volts)
   * @param frictionFeedForward the friction feedforward output of the controller (in volts)
   * @param kV the kV output of the controller (in volts)
   * @param calculatedFeedForward the calculated feedforward output of the controller (in volts)
   * @param totalOutput the total output of the controller (in volts)
   */
  public record FeedForwardOutput(
      double simpleFeedForward,
      double frictionFeedForward,
      double kV,
      double calculatedFeedForward,
      double arbitraryFeedForward,
      double totalOutput) {
    public FeedForwardOutput() {
      this(0, 0, 0, 0, 0);
    }

    public FeedForwardOutput(
        double simpleFeedForward,
        double frictionFeedForward,
        double kV,
        double calculatedFeedForward,
        double arbitraryFeedForward) {
      this(
          simpleFeedForward,
          frictionFeedForward,
          kV,
          calculatedFeedForward,
          arbitraryFeedForward,
          simpleFeedForward
              + frictionFeedForward
              + kV
              + calculatedFeedForward
              + arbitraryFeedForward);
    }
  }

  /**
   * The frame of the controller
   *
   * @param totalOutput the total output of the controller (in volts)
   * @param pidOutput the PID output of the controller (not always used)
   * @param feedForwardOutput the feedforward output of the controller
   * @param setpoint the setpoint of the controller in units of measurement
   * @param measurement the measurement of the controller in units of measurement
   * @param error the error of the controller in units of measurement
   * @param goal the goal of the controller in units of measurement (used when using a profile)
   * @param mode the mode of the controller (position, velocity, voltage, percent output)
   */
  public record ControllerFrame(
      double totalOutput,
      PIDOutput pidOutput,
      FeedForwardOutput feedForwardOutput,
      double setpoint,
      double measurement,
      double error,
      double goal,
      Controller.RequestType mode) {

    /** empty constructor for the controller frame used when stopping the motor */
    public ControllerFrame() {
      this(0, new PIDOutput(), new FeedForwardOutput(), 0, 0, 0, 0, Controller.RequestType.STOP);
    }

    /**
     * apply the pid output to the controller frame
     *
     * @param frame the controller frame to apply the pid output to
     * @param output the pid output to apply to the controller frame
     */
    public ControllerFrame(ControllerFrame frame, PIDOutput output) {
      this(
          frame.totalOutput + output.totalOutput,
          output,
          frame.feedForwardOutput,
          frame.setpoint,
          frame.measurement,
          frame.error,
          frame.goal,
          frame.mode);
    }

    /**
     * apply the feedforward output to the controller frame
     *
     * @param frame the controller frame to apply the feedforward output to
     * @param output the feedforward output to apply to the controller frame
     * @param constraint apply the constraints to the output (clamping and deadband)
     */
    public ControllerFrame(
        ControllerFrame frame, PIDOutput output, ControllerConstrains constraint) {
      this(
          constraint.checkMotorOutput(frame.totalOutput + output.totalOutput),
          output,
          frame.feedForwardOutput,
          frame.setpoint,
          frame.measurement,
          frame.error,
          frame.goal,
          frame.mode);
    }

    public ControllerFrame(
        double output, double wanted, double measurement, Controller.RequestType mode) {
      this(output, new PIDOutput(), new FeedForwardOutput(), wanted, measurement, 0, 0, mode);
    }
  }
}
