package util.BasicMotor;

import org.littletonrobotics.junction.AutoLog;
import util.BasicMotor.Controllers.Controller;
import util.BasicMotor.Measurements.Measurements;

@AutoLog
public class LogFrame {
  /** the latest frame of the controller */
  public ControllerFrame controllerFrame;
  /** the latest PID output of the controller */
  public PIDOutput pidOutput;
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
    public static final PIDOutput EMPTY = new PIDOutput(0, 0, 0, 0);
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
    public static final FeedForwardOutput EMPTY = new FeedForwardOutput(0, 0, 0, 0, 0);

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
        new ControllerFrame(
            0, FeedForwardOutput.EMPTY, 0, 0, 0, 0, Controller.RequestType.STOP);

    public ControllerFrame(
        double output, double wanted, double measurement, Controller.RequestType mode) {
      this(output, FeedForwardOutput.EMPTY, wanted, measurement, 0, 0, mode);
    }
  }
}
