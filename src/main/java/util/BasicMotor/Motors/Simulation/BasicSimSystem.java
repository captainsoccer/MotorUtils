package util.BasicMotor.Motors.Simulation;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import util.BasicMotor.BasicMotor;
import util.BasicMotor.Configuration.BasicMotorConfig;
import util.BasicMotor.Controllers.Controller;
import util.BasicMotor.Gains.ControllerConstrains;
import util.BasicMotor.Gains.ControllerGains;
import util.BasicMotor.Gains.CurrentLimits.CurrentLimits;
import util.BasicMotor.Gains.PIDGains;
import util.BasicMotor.LogFrame;
import util.BasicMotor.MotorManager;
import util.BasicMotor.MotorManager.ControllerLocation;

public abstract class BasicSimSystem extends BasicMotor {
  /**
   * The LinearSystemSim instance used by this BasicSimSystem.
   * this is the simulation system that simulates the motor's behavior.
   */
  protected final LinearSystemSim<N2, N1, N2> system;

  /**
   * The voltage output of the motor simulation.
   */
  private double voltageOutput = 0.0;

    /**
     * Creates a BasicSimSystem instance with the provided LinearSystemSim and name.
     *
     * @param system the LinearSystemSim to use
     * @param name   the name of the motor
     * @param gains  the controller gains for the motor
     */
  public BasicSimSystem(LinearSystemSim<N2, N1, N2> system, String name, ControllerGains gains) {
    super(gains, name, MotorManager.ControllerLocation.RIO);
    this.system = system;
  }

    /**
     * Creates a BasicSimSystem instance with the provided LinearSystemSim and configuration.
     *
     * @param system the LinearSystemSim to use
     * @param config the configuration for the motor
     */
  public BasicSimSystem(LinearSystemSim<N2, N1, N2> system, BasicMotorConfig config) {
    super(checkConfig(config));
    this.system = system;
  }

  /**
   * makes sure the configuration is valid for a simulation system.
   * This sets the location to RIO, as simulation systems are always on the RoboRIO.
   */
  private static BasicMotorConfig checkConfig(BasicMotorConfig config) {
    config.motorConfig.location = ControllerLocation.RIO;
    return config;
  }

  @Override
  protected void updatePIDGainsToMotor(PIDGains pidGains) {
    // does nothing, as this is a simulation system
  }

  @Override
  protected void updateConstraints(ControllerConstrains constraints) {
    // does nothing, as this is a simulation system
  }

  @Override
  public void setCurrentLimits(CurrentLimits currentLimits) {
    // does nothing, as this is a simulation system
  }

  @Override
  public void setIdleMode(IdleMode mode) {
    // does nothing, as this is a simulation system
  }

  @Override
  public void setMotorInverted(boolean inverted) {
    // does nothing, as this is a simulation system
  }

  @Override
  protected void stopRecordingMeasurements() {
    // does nothing, as this is a simulation system
  }

  @Override
  protected void startRecordingMeasurements(double HZ) {
    // does nothing, as this is a simulation system
  }

  @Override
  protected void setMotorFollow(BasicMotor master, boolean inverted) {
    // does nothing, as this is a simulation system
  }

  @Override
  protected void stopMotorFollow() {
    // does nothing, as this is a simulation system
  }

  @Override
  protected void setMotorOutput(double setpoint, double feedForward, Controller.RequestType mode) {
    if (mode.requiresPID())
      throw new IllegalArgumentException("Simulation system does not support PID mode.");

    double output =
        switch (mode) {
          case STOP -> 0;
          case PRECENT_OUTPUT -> setpoint * RobotController.getBatteryVoltage();
          default -> setpoint;
        };

    setOutput(output);
  }

  private void setOutput(double output) {
    voltageOutput = output;
    system.setInput(voltageOutput);
  }

  @Override
  protected void stopMotorOutput() {
    setOutput(0);
  }

  @Override
  protected LogFrame.SensorData getSensorData() {
    double voltageInput = RobotController.getBatteryVoltage();
    double voltageOutput = this.voltageOutput;

    double currentDraw = getCurrentDraw();

    double powerDraw =
        voltageInput * currentDraw; // also power output because this is a simulation system

    double currentOutput = powerDraw / voltageOutput;

    double temp = 0; // no temperature in simulation

    double dutyCycle =
        voltageOutput / voltageInput; // duty cycle is the ratio of output to input voltage

    return new LogFrame.SensorData(
        temp,
        voltageInput,
        voltageOutput,
        currentDraw,
        currentOutput,
        powerDraw,
        powerDraw,
        dutyCycle);
  }

  /**
   * Gets the current draw of the motor simulation.
   * this is abstract because different motor simulations may have different ways of calculating current draw.
   * @return the current draw in amps
   */
  protected abstract double getCurrentDraw();

  @Override
  protected LogFrame.PIDOutput getPIDLatestOutput() {
    return LogFrame.PIDOutput.EMPTY;
  }
}
