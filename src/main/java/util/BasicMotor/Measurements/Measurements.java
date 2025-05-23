package util.BasicMotor.Measurements;

import util.BasicMotor.BasicMotor;

/**
 * This class is used to get the measurements of the motor. It is used to get the position,
 * velocity, and acceleration of the motor. It is used to get the measurements of the motor.
 * this class is used by {@link BasicMotor} to get the measurements of the motor
 */
public abstract class Measurements {
  /**
   * The measurements of the motor
   *
   * @param position
   * @param velocity
   * @param acceleration
   */
  public record Measurement(double position, double velocity, double acceleration) {}

  /**
   * The gear ratio of the motor This is used to convert the measurements from the motor to the
   * actual measurements
   */
  private double gearRatio;

  /** The last measurement of the motor This is used to store the last measurement of the motor */
  private Measurement lastMeasurement = new Measurement(0, 0, 0);

  /**
   * Creates a new measurements object with the given gear ratio
   *
   * @param gearRatio the gear ratio of the motor
   */
  public Measurements(double gearRatio) {
    this.gearRatio = gearRatio;
  }

  /** Creates a new measurements object with the default gear ratio of 1 */
  public Measurements() {
    this(1.0);
  }

  /**
   * Updates the measurements of the motor
   *
   * @param dt the time since the last update
   * @return the new measurements of the motor
   */
  public Measurement update(double dt) {
    // update the measurements
    double position = getUpdatedPosition();
    double velocity = getUpdatedVelocity();
    double acceleration = getUpdatedAcceleration();

    // calculate the new measurements
    position /= gearRatio;
    velocity /= gearRatio;
    acceleration /= gearRatio;

    lastMeasurement = new Measurement(position, velocity, acceleration);

    return lastMeasurement;
  }

  /**
   * Sets the gear ratio of the motor
   *
   * @param gearRatio the gear ratio of the motor
   */
  public void setGearRatio(double gearRatio) {
    this.gearRatio = gearRatio;
  }

  /**
   * Gets the gear ratio of the motor
   *
   * @return the gear ratio of the motor
   */
  public double getGearRatio() {
    return gearRatio;
  }

  /**
   * Gets the last measurement of the motor
   *
   * @return the last measurement of the motor
   */
  public Measurement getMeasurement() {
    return lastMeasurement;
  }

  /**
   * Gets the last position of the motor
   *
   * @return the last position of the motor
   */
  public double getPosition() {
    return lastMeasurement.position;
  }

  /**
   * Gets the last velocity of the motor
   *
   * @return the last velocity of the motor
   */
  public double getVelocity() {
    return lastMeasurement.velocity;
  }

  /**
   * Gets the last acceleration of the motor
   *
   * @return the last acceleration of the motor
   */
  public double getAcceleration() {
    return lastMeasurement.acceleration;
  }

  /**
   * updates the position of the motor
   *
   * @return the new position of the motor
   */
  protected abstract double getUpdatedPosition();

  /**
   * updates the velocity of the motor
   *
   * @return the new velocity of the motor
   */
  protected abstract double getUpdatedVelocity();

  /**
   * updates the acceleration of the motor
   *
   * @return the new acceleration of the motor
   */
  protected abstract double getUpdatedAcceleration();
}
