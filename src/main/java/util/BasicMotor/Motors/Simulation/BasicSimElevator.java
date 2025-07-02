package util.BasicMotor.Motors.Simulation;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import util.BasicMotor.Gains.ControllerGains;
import util.BasicMotor.Measurements.Measurements;
import util.BasicMotor.Measurements.SimulationEncoder.ElevatorSimEncoder;

public class BasicSimElevator extends BasicSimSystem {
  private final ElevatorSim elevator;

  private final Measurements defaultMeasurements;

  public BasicSimElevator(ElevatorSim elevator, String name, ControllerGains gains) {
    super(elevator, name, gains);
    this.elevator = elevator;

    defaultMeasurements = new ElevatorSimEncoder(elevator);
  }

  // public BasicSimElevator() {

  // }

  // private static ElevatorSim createElevatorSim(DCMotor motor, double gearing, double wheelRadius,
  // double mass) {
  //     var plant = LinearSystemId.createElevatorSystem(motor, mass, wheelRadius, gearing);

  //     return new ElevatorSim()
  // }

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
