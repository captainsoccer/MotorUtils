package util.BasicMotor.Measurements;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

public class MeasurementsCTRE extends Measurements {

  private final double timeout;

  StatusSignal<Angle> motorPosition;
  StatusSignal<AngularVelocity> motorVelocity;
  StatusSignal<AngularAcceleration> motorAcceleration;

  double positionLatencyCompensatedValue = 0;
  double velocityLatencyCompensatedValue = 0;

  public MeasurementsCTRE(
      StatusSignal<Angle> positionSignal,
      StatusSignal<AngularVelocity> velocitySignal,
      StatusSignal<AngularAcceleration> accelerationSignal,
      double refreshHZ) {
    this(positionSignal, velocitySignal, accelerationSignal, refreshHZ, 1.0);
  }

  public MeasurementsCTRE(
      StatusSignal<Angle> positionSignal,
      StatusSignal<AngularVelocity> velocitySignal,
      StatusSignal<AngularAcceleration> accelerationSignal,
      double refreshHZ,
      double gearRatio) {
    super(gearRatio);

    motorPosition = positionSignal;
    motorVelocity = velocitySignal;
    motorAcceleration = accelerationSignal;

    positionSignal.setUpdateFrequency(refreshHZ);
    velocitySignal.setUpdateFrequency(refreshHZ);
    accelerationSignal.setUpdateFrequency(refreshHZ);

    timeout = 1 / (refreshHZ * 4);
  }

  @Override
  public Measurement update(double dt) {
    BaseStatusSignal.waitForAll(timeout, motorPosition, motorVelocity, motorAcceleration);

    var position = BaseStatusSignal.getLatencyCompensatedValue(motorPosition, motorVelocity);
    positionLatencyCompensatedValue = position.in(Units.Rotations);

    var velocity = BaseStatusSignal.getLatencyCompensatedValue(motorVelocity, motorAcceleration);
    velocityLatencyCompensatedValue = velocity.in(Units.RotationsPerSecond);

    return super.update(dt);
  }

  @Override
  protected double getUpdatedPosition() {
    return positionLatencyCompensatedValue;
  }

  @Override
  protected double getUpdatedVelocity() {
    return velocityLatencyCompensatedValue;
  }

  @Override
  protected double getUpdatedAcceleration() {
    return motorAcceleration.getValueAsDouble();
  }
}
