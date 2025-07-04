package com.basicMotor.gains;

import static com.basicMotor.MotorManager.motorIdleVoltage;

import com.basicMotor.MotorManager;

/**
 * This class is used to set the PID gains of the motor controller. It is used to set the PID gains
 * of the motor controller.
 *
 * <p>it contains the following values: k_P: the proportional gain (>= 0) k_I: the integral gain (>=
 * 0) k_D: the derivative gain (>= 0) i_Zone: the integrator zone (>= 0) this is the zone where the
 * integrator is active i_maxAccum: the maximum accumulation of the integrator (>= 0) this is the
 * maximum value that the
 */
public class PIDGains {

  /** what to change when updating the PID gains */
  public enum ChangeType {
    K_P,
    K_I,
    K_D,
    I_ZONE,
    I_MAX_ACCUM,
    TOLERANCE
  }

  /** the PID elements of the gains */
  private double k_P, k_I, K_D;

  /** the integrator modifies */
  private double i_Zone, i_maxAccum;

  /**
   * the tolerance of the PID controller (if the error is less than this value, the controller will
   * stop)
   */
  private double tolerance;

  /**
   * creates a PID gains object with the given values
   *
   * @param k_P the proportional gain (>= 0)
   * @param k_I the integral gain (>= 0)
   * @param k_D the derivative gain (>= 0)
   * @param i_Zone the integrator zone (>= 0) this is the zone where the integrator is active
   * @param i_maxAccum the maximum accumulation of the integrator (>= 0) this is the maximum value
   *     that the integrator can accumulate
   */
  public PIDGains(
      double k_P, double k_I, double k_D, double i_Zone, double i_maxAccum, double tolerance) {
    if (k_P < 0) throw new IllegalArgumentException("k_P must be greater than zero");
    this.k_P = k_P;

    if (k_I < 0) throw new IllegalArgumentException("k_I must be greater than zero");
    this.k_I = k_I;

    if (k_D < 0) throw new IllegalArgumentException("k_D must be greater than zero");
    this.K_D = k_D;

    if (i_Zone < 0) throw new IllegalArgumentException("i_Zone must be greater than zero");
    this.i_Zone = i_Zone;

    if (i_maxAccum < 0) throw new IllegalArgumentException("i_maxAccum must be greater than zero");
    this.i_maxAccum = i_maxAccum;

    if (tolerance < 0) throw new IllegalArgumentException("tolerance must be greater than zero");
    this.tolerance = tolerance;
  }

  /**
   * creates a PID gains object with the given values
   *
   * @param k_P the proportional gain (>= 0)
   * @param k_I the integral gain (>= 0)
   * @param k_D the derivative gain (>= 0)
   */
  public PIDGains(double k_P, double k_I, double k_D) {
    this(k_P, k_I, k_D, Double.POSITIVE_INFINITY, MotorManager.defaultMaxMotorOutput, 0);
  }

  /**
   * creates a PID gains object with the given values
   *
   * @param k_P the proportional gain (>= 0)
   * @param k_I the integral gain (>= 0)
   * @param k_D the derivative gain (>= 0)
   */
  public PIDGains(double k_P, double k_I, double k_D, double tolerance) {
    this(k_P, k_I, k_D, Double.POSITIVE_INFINITY, MotorManager.defaultMaxMotorOutput, tolerance);
  }

  /** creates an empty PID gains object (no k_P, no k_I, no k_D) */
  public PIDGains() {
    this(0, 0, 0);
  }

  /**
   * gets the proportional gain of the PID controller units are volts per unit of measurement
   *
   * @return the proportional gain of the PID controller
   */
  public double getK_P() {
    return k_P;
  }

  /**
   * gets the integral gain of the PID controller units are volt second per unit of measurement
   *
   * @return the integral gain of the PID controller
   */
  public double getK_I() {
    return k_I;
  }

  /**
   * gets the derivative gain of the PID controller units are volts per unit of measurement per
   * second
   *
   * @return the derivative gain of the PID controller
   */
  public double getK_D() {
    return K_D;
  }

  /**
   * gets the integrator zone of the PID controller units are unit of measurement
   *
   * @return the integrator zone of the PID controller
   */
  public double getI_Zone() {
    return i_Zone;
  }

  /**
   * gets the maximum accumulation of the integrator of the PID controller units are unit of
   * measurement seconds
   *
   * @return the maximum accumulation of the integrator of the PID controller
   */
  public double getI_MaxAccum() {
    return i_maxAccum;
  }

  /**
   * gets the tolerance of the PID controller units are unit of measurement
   *
   * @return the tolerance of the PID controller
   */
  public double getTolerance() {
    return tolerance;
  }

  /**
   * converts the PID gains to motor gains this is used to convert the PID gains to motor gains it
   * is necessary to offset the gear ratio of the motor (that the output stays the same)
   *
   * @param gearRatio the gear ratio of the motor
   * @return the motor gains
   */
  public PIDGains convertToMotorGains(double gearRatio) {
    return new PIDGains(
        k_P / gearRatio,
        k_I / gearRatio,
        K_D / gearRatio,
        i_Zone * gearRatio,
        i_maxAccum,
        tolerance * gearRatio);
  }

  /**
   * converts the PID gains to duty cycle gains this is used to convert the PID gains to duty cycle
   * gains it is necessary if the motor controller uses a duty cycle instead of a voltage
   *
   * @return the duty cycle gains
   */
  public PIDGains convertToDutyCycle() {
    return new PIDGains(
        k_P / motorIdleVoltage,
        k_I / motorIdleVoltage,
        K_D / motorIdleVoltage,
        i_Zone,
        i_maxAccum / motorIdleVoltage,
        tolerance);
  }

  /**
   * updates the PID gains from the dashboard
   *
   * @param value the value to set the gain to
   * @param changeType which gain to change
   * @return true if the value has changed, false if it has not (used for setting the built-in
   *     controller values)
   */
  public boolean updatePIDGains(double value, ChangeType changeType) {
    if (value < 0) throw new IllegalArgumentException("value must be greater than zero");
    double oldValue = getValue(changeType);

    if (oldValue == value) {
      return false; // no change
    }

    setValue(value, changeType);
    return true; // value has changed
  }

  /**
   * sets the value of the PID gain based on the change type
   *
   * @param value the value to set the gain to
   * @param changeType which gain to change
   */
  private void setValue(double value, ChangeType changeType) {
    switch (changeType) {
      case K_P -> k_P = value;
      case K_I -> k_I = value;
      case K_D -> K_D = value;
      case I_ZONE -> i_Zone = value;
      case I_MAX_ACCUM -> i_maxAccum = value;
      case TOLERANCE -> tolerance = value;
    }
  }

  /**
   * gets the value of the PID gain based on the change type
   *
   * @param changeType which gain to get
   * @return the value of the PID gain
   */
  private double getValue(ChangeType changeType) {
    return switch (changeType) {
      case K_P -> k_P;
      case K_I -> k_I;
      case K_D -> K_D;
      case I_ZONE -> i_Zone;
      case I_MAX_ACCUM -> i_maxAccum;
      case TOLERANCE -> tolerance;
    };
  }
}
