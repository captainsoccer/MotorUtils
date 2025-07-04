package com.basicMotor.measurements.simulationEncoder;

import com.basicMotor.measurements.Measurements;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmSimEncoder extends Measurements {
  private final SingleJointedArmSim armSim;

  double currentVelocity = 0.0;
  double previousVelocity = 0.0;

  double acceleration = 0.0;

  public ArmSimEncoder(SingleJointedArmSim armSim) {
    this.armSim = armSim;
  }

  @Override
  public Measurement update(double dt) {
    armSim.update(dt);

    var angle = edu.wpi.first.units.Units.RadiansPerSecond.of(armSim.getVelocityRadPerSec());
    currentVelocity = angle.in(edu.wpi.first.units.Units.RotationsPerSecond);

    // Calculate acceleration as the change in velocity over time
    acceleration = (currentVelocity - previousVelocity) / dt;

    previousVelocity = currentVelocity;

    return super.update(dt);
  }

  @Override
  protected double getUpdatedPosition() {
    return Units.radiansToRotations(armSim.getAngleRads());
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
