package util.BasicMotor.Measurements.RevEncoders;

import com.revrobotics.RelativeEncoder;
import util.BasicMotor.Measurements.Measurements;

/**
 * This class is used to get the measurements from the REV motor controller it handles updating the
 * measurements and getting the latency compensated values
 */
public class MeasurementsREVRelative extends Measurements {
  /** the encoder used to get the measurements from the motor controller */
  private final RelativeEncoder encoder;

  /** the last velocity of the motor controller this is used to calculate the acceleration */
  private double lastVelocity = 0;
  /** the current velocity of the motor controller this is used to calculate the acceleration */
  private double currentVelocity = 0;
  /** the acceleration of the motor controller this is used to calculate the acceleration */
  private double acceleration = 0;

  /**
   * Creates a new measurements object with the given encoder and gear ratio
   *
   * @param encoder the encoder used to get the measurements from the motor controller
   * @param gearRatio the gear ratio of the motor (the measurements are divided by this)
   */
  public MeasurementsREVRelative(RelativeEncoder encoder, double gearRatio) {
    super(gearRatio);
    this.encoder = encoder;
  }

  @Override
  public Measurement update(double dt) {
    currentVelocity = encoder.getVelocity() / 60; // Convert to rotations per second

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

  public void setEncoderPosition(double position) {
    encoder.setPosition(position);
  }
}
