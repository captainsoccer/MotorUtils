package util.BasicMotor.Measurements;

import com.revrobotics.RelativeEncoder;

public class MeasurementsREVRelative extends Measurements {
  private final RelativeEncoder encoder;

  private double lastVelocity = 0;
  private double currentVelocity = 0;
  private double acceleration = 0;

  public MeasurementsREVRelative(RelativeEncoder encoder, double gearRatio) {
    super(gearRatio);
    this.encoder = encoder;
  }

  @Override
  public Measurement update(double dt) {
    currentVelocity = encoder.getVelocity();

    acceleration = (currentVelocity - lastVelocity) / dt;

    lastVelocity = currentVelocity;

    return super.update(dt);
  }

  @Override
  protected double getUpdatedPosition() {
    return encoder.getPosition();
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
