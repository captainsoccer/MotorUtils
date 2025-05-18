package util.BasicMotor.Measurements;

public abstract class Measurements {
  public record Measurement(double position, double velocity, double acceleration) {}
  ;

  private double gearRatio;

  private Measurement lastMeasurement = new Measurement(0, 0, 0);

  public Measurements(double gearRatio) {
    this.gearRatio = gearRatio;
  }

  public Measurements() {
    this(1.0);
  }

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

  public void setGearRatio(double gearRatio) {
    this.gearRatio = gearRatio;
  }

  public double getGearRatio() {
    return gearRatio;
  }

  public Measurement getMeasurement() {
    return lastMeasurement;
  }

  public double getPosition() {
    return lastMeasurement.position;
  }

  public double getVelocity() {
    return lastMeasurement.velocity;
  }

  public double getAcceleration() {
    return lastMeasurement.acceleration;
  }

  protected abstract double getUpdatedPosition();

  protected abstract double getUpdatedVelocity();

  protected abstract double getUpdatedAcceleration();
}
