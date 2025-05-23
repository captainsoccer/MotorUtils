package util.BasicMotor;

import org.littletonrobotics.junction.AutoLog;
import util.BasicMotor.Controllers.Controller;
import util.BasicMotor.Gains.ControllerConstrains;
import util.BasicMotor.Measurements.Measurements;

@AutoLog
public class LogFrame {
  public ControllerFrame controllerFrame;
  public SensorData sensorData;
  public Measurements.Measurement measurement;

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

  public record PIDOutput(double pOutput, double iOutput, double dOutput, double totalOutput) {
    public PIDOutput() {
      this(0, 0, 0, 0);
    }
  }

  public record FeedForwardOutput(
      double simpleFeedForward,
      double frictionFeedForward,
      double kV,
      double calculatedFeedForward,
      double totalOutput) {
    public FeedForwardOutput() {
      this(0, 0, 0, 0, 0);
    }

    public FeedForwardOutput(
        double simpleFeedForward,
        double frictionFeedForward,
        double kV,
        double calculatedFeedForward) {
      this(
          simpleFeedForward,
          frictionFeedForward,
          kV,
          calculatedFeedForward,
          simpleFeedForward + frictionFeedForward + kV + calculatedFeedForward);
    }
  }

  /**
   * The frame of the controller
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

    /**
     * empty constructor for the controller frame
     * used when stopping the motor
     */
    public ControllerFrame() {
      this(0, new PIDOutput(), new FeedForwardOutput(), 0, 0, 0, 0, Controller.RequestType.STOP);
    }

    /**
     * apply the pid output to the controller frame
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
