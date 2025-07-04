package com.basicMotor.motors.simulation;

import com.basicMotor.configuration.BasicMotorConfig;
import com.basicMotor.gains.ControllerGains;
import com.basicMotor.measurements.Measurements;
import com.basicMotor.measurements.simulationEncoder.MotorSimEncoder;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * a class that simulates a DC motor using the DCMotorSim class. it is in the basic sim motor system
 * and has all the functionality of a basic sim system. use this when you want to simulate a motor
 * in your robot code. this can be anything from a shooter, arm, drivetrain, etc. you will want to
 * use a specific simulation class if available units are in rotations.
 */
public class BasicSimMotor extends BasicSimSystem {
  /** The DCMotorSim instance used by this BasicSimMotor. */
  private final DCMotorSim motor;
  /** The default measurements for the motor simulation. */
  private final Measurements defaultMeasurements;

  /**
   * Creates a BasicSimMotor instance with the provided DCMotorSim and name.
   *
   * @param motor the DCMotorSim to use
   * @param name the name of the motor
   * @param gains the controller gains for the motor
   * @param unitConversion the conversion factor for the motor's position units
   */
  public BasicSimMotor(
      DCMotorSim motor, String name, ControllerGains gains, double unitConversion) {
    super(name, gains);
    this.motor = motor;

    defaultMeasurements = new MotorSimEncoder(motor, unitConversion);
  }

  /**
   * Creates a BasicSimMotor instance with the provided configuration.
   *
   * @param config the configuration for the motor
   */
  public BasicSimMotor(BasicMotorConfig config) {
    super(config);

    this.motor = createSimMotor(config);

    this.defaultMeasurements = new MotorSimEncoder(motor, config.motorConfig.unitConversion);
  }

  @Override
  protected void setInputVoltage(double voltage) {
    motor.setInputVoltage(voltage);
  }

  /**
   * Creates a DCMotorSim based on the provided configuration.
   *
   * @param config the configuration for the motor
   * @return a new DCMotorSim instance
   */
  private static DCMotorSim createSimMotor(BasicMotorConfig config) {
    var simConfig = config.simulationConfig;

    if (simConfig.momentOfInertia == 0 && simConfig.kV == 0 && simConfig.kA == 0)
      throw new IllegalArgumentException(
          "you must provide either a moment of inertia or kV and kA for the simulation motor");

    var plant =
        simConfig.momentOfInertia == 0
            ? LinearSystemId.createDCMotorSystem(simConfig.kV, simConfig.kA)
            : LinearSystemId.createDCMotorSystem(
                config.motorConfig.motorType,
                simConfig.momentOfInertia,
                config.motorConfig.gearRatio);

    return new DCMotorSim(
        plant,
        config.motorConfig.motorType.withReduction(config.motorConfig.gearRatio),
        simConfig.positionStandardDeviation,
        simConfig.velocityStandardDeviation);
  }

  @Override
  protected double getCurrentDraw() {
    return motor.getCurrentDrawAmps();
  }

  @Override
  protected Measurements getDefaultMeasurements() {
    return defaultMeasurements;
  }

  @Override
  protected void setMotorPosition(double position) {
    motor.setAngle(Units.rotationsToRadians(position));
  }
}
