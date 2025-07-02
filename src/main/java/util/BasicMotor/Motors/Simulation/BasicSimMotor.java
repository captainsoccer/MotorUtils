package util.BasicMotor.Motors.Simulation;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import util.BasicMotor.Configuration.BasicMotorConfig;
import util.BasicMotor.Gains.ControllerGains;
import util.BasicMotor.Measurements.Measurements;
import util.BasicMotor.Measurements.SimulationEncoder.MotorSimEncoder;

public class BasicSimMotor extends BasicSimSystem {
  private final DCMotorSim motor;
  private final Measurements defaultMeasurements;

  public BasicSimMotor(
      DCMotorSim motor, String name, ControllerGains gains, double unitConversion) {
    super(motor, name, gains);
    this.motor = motor;

    defaultMeasurements = new MotorSimEncoder(motor, unitConversion);
  }

  public BasicSimMotor(BasicMotorConfig config) {
    this(
        createSimMotor(config),
        config.motorConfig.name,
        config.getControllerGains(),
        config.motorConfig.unitConversion);
  }

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
