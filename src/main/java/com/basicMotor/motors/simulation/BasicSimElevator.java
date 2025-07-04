package com.basicMotor.motors.simulation;

import com.basicMotor.configuration.BasicMotorConfig;
import com.basicMotor.gains.ControllerConstrains;
import com.basicMotor.gains.ControllerGains;
import com.basicMotor.measurements.Measurements;
import com.basicMotor.measurements.simulationEncoder.ElevatorSimEncoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * a class that simulates an elevator system using the ElevatorSim class. it is in the basic sim
 * motor system and has all the functionality of a basic sim system. use this when you want to
 * simulate an elevator in your robot code. if you have two motors, use only one instance of this
 * class and set the position of both motors to the same value. units are in meters.
 */
public class BasicSimElevator extends BasicSimSystem {
  /** The elevator simulation instance used by this BasicSimElevator. */
  private final ElevatorSim elevator;

  /** The default measurements for the elevator simulation. */
  private final Measurements defaultMeasurements;

  /**
   * creates a BasicSimElevator instance with the provided elevator simulation and name.
   *
   * @param elevator the elevator simulation to use
   * @param name the name of the elevator
   * @param gains the controller gains for the elevator
   */
  public BasicSimElevator(ElevatorSim elevator, String name, ControllerGains gains) {
    super(name, gains);
    this.elevator = elevator;

    defaultMeasurements = new ElevatorSimEncoder(elevator);
  }

  /**
   * creates a BasicSimElevator instance with the provided configuration.
   *
   * @param config the configuration for the elevator motor
   */
  public BasicSimElevator(BasicMotorConfig config) {
    super(config);

    this.elevator = createElevatorSim(config);

    defaultMeasurements = new ElevatorSimEncoder(elevator);
  }

  @Override
  protected void setInputVoltage(double voltage) {
    elevator.setInputVoltage(voltage);
  }

  /**
   * Creates an ElevatorSim based on the provided configuration.
   *
   * @param config the configuration for the elevator motor
   * @return a new ElevatorSim instance
   */
  private static ElevatorSim createElevatorSim(BasicMotorConfig config) {
    double minHeight;
    double maxHeight;
    if (config.constraintsConfig.constraintType == ControllerConstrains.ConstraintType.LIMITED) {
      minHeight = config.constraintsConfig.minValue;
      maxHeight = config.constraintsConfig.maxValue;
    } else {
      minHeight = 0;
      maxHeight = Double.POSITIVE_INFINITY;
    }

    var simConfig = config.simulationConfig;

    if (simConfig.kV == 0 && simConfig.kA == 0) {
      return new ElevatorSim(
          config.motorConfig.motorType,
          config.motorConfig.gearRatio,
          simConfig.elevatorSimConfig.massKG,
          simConfig.elevatorSimConfig.pulleyRadiusMeters,
          minHeight,
          maxHeight,
          simConfig.elevatorSimConfig.enableGravitySimulation,
          minHeight,
          simConfig.positionStandardDeviation,
          simConfig.velocityStandardDeviation);
    } else {
      return new ElevatorSim(
          simConfig.kV,
          simConfig.kA,
          config.motorConfig.motorType,
          minHeight,
          maxHeight,
          simConfig.elevatorSimConfig.enableGravitySimulation,
          minHeight,
          simConfig.positionStandardDeviation,
          simConfig.velocityStandardDeviation);
    }
  }

  @Override
  protected double getCurrentDraw() {
    return elevator.getCurrentDrawAmps();
  }

  @Override
  protected Measurements getDefaultMeasurements() {
    return defaultMeasurements;
  }

  @Override
  protected void setMotorPosition(double position) {
    elevator.setState(position, elevator.getVelocityMetersPerSecond());
  }
}
