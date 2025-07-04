package com.basicMotor.measurements.simulationEncoder;

import com.basicMotor.measurements.Measurements;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlyWheelSimEncoder extends Measurements {
  /** The FlywheelSim instance used by this FlyWheelSimEncoder. */
  private final FlywheelSim flywheelSim;

  private double position = 0.0;

  private double velocity = 0.0;

  /**
   * Creates a FlyWheelSimEncoder instance with the provided FlywheelSim.
   *
   * @param flywheelSim the FlywheelSim to use
   */
  public FlyWheelSimEncoder(FlywheelSim flywheelSim) {
    this.flywheelSim = flywheelSim;
  }

  @Override
  public Measurement update(double dt) {
    flywheelSim.update(dt);

    velocity = flywheelSim.getAngularVelocity().in(Units.RotationsPerSecond);

    position += velocity * dt; // Update position based on velocity and time step

    return super.update(dt);
  }

  @Override
  protected double getUpdatedPosition() {
    return position;
  }

  @Override
  protected double getUpdatedVelocity() {
    return velocity;
  }

  @Override
  protected double getUpdatedAcceleration() {
    return flywheelSim.getAngularAcceleration().in(Units.RotationsPerSecondPerSecond);
  }
}
