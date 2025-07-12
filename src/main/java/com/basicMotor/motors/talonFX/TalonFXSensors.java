package com.basicMotor.motors.talonFX;

import com.basicMotor.LogFrame;
import com.basicMotor.Manager.MotorManager;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * this class is used to manage the sensors of the TalonFX motor controller it updates the sensors
 * at a given refresh rate and also saves the pid output of the built-in controller for logging
 */
public class TalonFXSensors {
  /** the refresh rate of the sensors (how often to update the sensors) */
  private final double refreshHZ;
  /**
   * the location of the motor (if running on motor controller it will also update the pid output)
   */
  private final MotorManager.ControllerLocation location;

  /** the temperature of the motor controller */
  private final StatusSignal<Temperature> temperatureSignal;
  /** the current draw of the motor controller */
  private final StatusSignal<Current> supplyCurrentSignal;
  /** the current output of the motor controller */
  private final StatusSignal<Current> statorCurrentSignal;
  /** the voltage output of the motor controller */
  private final StatusSignal<Voltage> motorVoltageSignal;
  /** the voltage input of the motor controller */
  private final StatusSignal<Voltage> supplyVoltageSignal;
  /** the duty cycle of the motor controller */
  private final StatusSignal<Double> dutyCycleSignal;

  /** the total output of the pid controller */
  private final StatusSignal<Double> totalOutput;
  /** the proportional output of the pid controller */
  private final StatusSignal<Double> kpOutput;
  /** the integral output of the pid controller */
  private final StatusSignal<Double> kiOutput;
  /** the derivative output of the pid controller */
  private final StatusSignal<Double> kdOutput;

  /**
   * the status signals of the motor controller used to update the sensors at a given refresh rate
   */
  private final BaseStatusSignal[] statusSignals;

  /**
   * The latest PID output from the motor controller used for logging if the pid controller is on
   * the motor controller
   */
  private LogFrame.PIDOutput latestPIDOutput = LogFrame.PIDOutput.EMPTY;

  /**
   * whether to use wait for all or refresh all.
   */
  private boolean enablePro = false;

  /**
   * Constructor for TalonFX Sensors
   *
   * @param motor the motor to get the sensors from
   * @param refreshHZ the refresh rate of the sensors (how often to update the sensors)
   * @param location the location of the motor (if running on motor controller it will also update
   *     the pid output)
   */
  public TalonFXSensors(TalonFX motor, double refreshHZ, MotorManager.ControllerLocation location) {
    this.refreshHZ = refreshHZ;
    this.location = location;

    temperatureSignal = motor.getDeviceTemp();
    supplyCurrentSignal = motor.getSupplyCurrent();
    statorCurrentSignal = motor.getStatorCurrent();
    motorVoltageSignal = motor.getMotorVoltage();
    supplyVoltageSignal = motor.getSupplyVoltage();
    dutyCycleSignal = motor.getDutyCycle();

    kpOutput = motor.getClosedLoopProportionalOutput();
    kiOutput = motor.getClosedLoopIntegratedOutput();
    kdOutput = motor.getClosedLoopDerivativeOutput();
    totalOutput = motor.getClosedLoopOutput();

    // if the controller is on the rio
    if (location == MotorManager.ControllerLocation.RIO) {
      statusSignals =
          new BaseStatusSignal[] {
            temperatureSignal,
            supplyCurrentSignal,
            statorCurrentSignal,
            motorVoltageSignal,
            supplyVoltageSignal,
            dutyCycleSignal
          };
    } else {
      statusSignals =
          new BaseStatusSignal[] {
            temperatureSignal,
            supplyCurrentSignal,
            statorCurrentSignal,
            motorVoltageSignal,
            supplyVoltageSignal,
            dutyCycleSignal,
            totalOutput,
            kpOutput,
            kiOutput,
            kdOutput
          };
    }

    for (BaseStatusSignal signal : statusSignals) {
      signal.setUpdateFrequency(refreshHZ);
    }
  }

  /**
   * gets the sensor data from the motor controller this is used to update the sensors at a given
   * refresh rate
   *
   * @return the sensor data from the motor controller
   */
  public LogFrame.SensorData getSensorData() {
    if(enablePro) BaseStatusSignal.waitForAll(1 / (refreshHZ * 4), statusSignals);
    else BaseStatusSignal.refreshAll(statusSignals);

    double temperature = temperatureSignal.getValueAsDouble();
    double currentDraw = supplyCurrentSignal.getValueAsDouble();
    double currentOutput = statorCurrentSignal.getValueAsDouble();
    double voltageOutput = motorVoltageSignal.getValueAsDouble();
    double voltageInput = supplyVoltageSignal.getValueAsDouble();
    double powerDraw = currentDraw * voltageInput;
    double powerOutput = currentOutput * voltageOutput;
    double dutyCycle = dutyCycleSignal.getValueAsDouble();

    // updates the latest pid output if the controller is on the motor controller
    // used for logging
    if (location == MotorManager.ControllerLocation.MOTOR) {
      double pOutput = kpOutput.getValueAsDouble();
      double iOutput = kiOutput.getValueAsDouble();
      double dOutput = kdOutput.getValueAsDouble();
      double pidOutput = totalOutput.getValueAsDouble();

      latestPIDOutput = new LogFrame.PIDOutput(pOutput, iOutput, dOutput, pidOutput);
    }

    return new LogFrame.SensorData(
        temperature,
        currentDraw,
        currentOutput,
        voltageOutput,
        voltageInput,
        powerDraw,
        powerOutput,
        dutyCycle);
  }

  /**
   * sets the duty cycle refresh signal to the default rate used if the motor is a master for a
   * follower motor
   * @param defaultRate whether to set the duty cycle signal to the default rate
   */
  public void setDutyCycleToDefaultRate(boolean defaultRate) {
    dutyCycleSignal.setUpdateFrequency(defaultRate ? 100 : refreshHZ);
  }

  /**
   * sets the enable pro.
   * if pro is enabled, code will use wait for all status signals for better reliability
   * else it will use refresh all.
   * @param enable if to enable pro
   */
  public void setEnablePro(boolean enable) {
    this.enablePro = enable;
  }
  /**
   * gets the latest pid output from the motor controller this is used for logging
   *
   * @return the latest pid output from the motor controller
   */
  public LogFrame.PIDOutput getPIDLatestOutput() {
    return latestPIDOutput;
  }
}
