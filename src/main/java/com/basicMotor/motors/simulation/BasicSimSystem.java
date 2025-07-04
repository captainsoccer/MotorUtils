package com.basicMotor.motors.simulation;

import com.basicMotor.BasicMotor;
import com.basicMotor.configuration.BasicMotorConfig;
import com.basicMotor.controllers.Controller;
import com.basicMotor.gains.ControllerConstrains;
import com.basicMotor.gains.ControllerGains;
import com.basicMotor.gains.currentLimits.CurrentLimits;
import com.basicMotor.gains.PIDGains;
import com.basicMotor.LogFrame;
import com.basicMotor.MotorManager.ControllerLocation;
import edu.wpi.first.wpilibj.RobotController;

/**
 * this is the base class for all simulation systems in the basic motor system.
 * it forwards the functions necessary to run a simulation system.
 * and it ignores the functions that are not necessary for a simulation system.
 */
public abstract class BasicSimSystem extends BasicMotor {
  /** The voltage output of the motor simulation. */
  private double voltageOutput = 0.0;

  /**
   * Creates a BasicSimSystem instance with the provided LinearSystemSim and name.
   *
   * @param name the name of the motor
   * @param gains the controller gains for the motor
   */
  public BasicSimSystem(String name, ControllerGains gains) {
    super(gains, name, ControllerLocation.RIO);
  }

  /**
   * Creates a BasicSimSystem instance with the provided LinearSystemSim and configuration.
   *
   * @param config the configuration for the motor
   */
  public BasicSimSystem(BasicMotorConfig config) {
    super(checkConfig(config));
  }

  /**
   * makes sure the configuration is valid for a simulation system. This sets the location to RIO,
   * as simulation systems are always on the RoboRIO.
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

  /**
   * sets the output voltage of the motor simulation.
   *
   * @param output the output voltage to set
   */
  private void setOutput(double output) {
    voltageOutput = output;
    setInputVoltage(voltageOutput);
  }

  @Override
  protected void stopMotorOutput() {
    setOutput(0);
  }

  /**
   * sets the input voltage of the motor simulation. this is abstract because different motor
   * simulations may have different ways of setting input voltage.
   *
   * @param voltage the input voltage to set
   */
  protected abstract void setInputVoltage(double voltage);

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
        currentDraw,
        currentOutput,
        voltageOutput,
        voltageInput,
        powerDraw,
        powerDraw,
        dutyCycle);
  }

  /**
   * Gets the current draw of the motor simulation. this is abstract because different motor
   * simulations may have different ways of calculating current draw.
   *
   * @return the current draw in amps
   */
  protected abstract double getCurrentDraw();

  @Override
  protected LogFrame.PIDOutput getPIDLatestOutput() {
    return LogFrame.PIDOutput.EMPTY;
  }
}
