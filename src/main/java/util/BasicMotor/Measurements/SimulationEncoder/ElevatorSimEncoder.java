package util.BasicMotor.Measurements.SimulationEncoder;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import util.BasicMotor.Measurements.Measurements;

public class ElevatorSimEncoder extends Measurements {
  private final ElevatorSim elevator;

  private double currentVelocity = 0.0;
  private double previousVelocity = 0.0;

  private double acceleration = 0.0;

  /**
   * Creates a new ElevatorSimEncoder for the given ElevatorSim.
   *
   * @param elevator the ElevatorSim to use for simulation
   */
  public ElevatorSimEncoder(ElevatorSim elevator) {
    this.elevator = elevator;
  }

  @Override
  public Measurement update(double dt) {
    elevator.update(dt);

    currentVelocity = elevator.getVelocityMetersPerSecond();

    // Calculate acceleration as the change in velocity over time
    acceleration = (currentVelocity - previousVelocity) / dt;

    previousVelocity = currentVelocity;

    return super.update(dt);
  }

  @Override
  protected double getUpdatedPosition() {
    return elevator.getPositionMeters();
  }

  @Override
  protected double getUpdatedVelocity() {
    return currentVelocity;
  }

  @Override
  protected double getUpdatedAcceleration() {
    return acceleration;
  }
}
