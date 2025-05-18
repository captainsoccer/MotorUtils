package util.BasicMotor.Gains;

public class PIDGains {

  private static final double maxMotorOutput = 12.0;
  /** what to change when updating the PID gains */
  public enum ChangeType {
    K_P,
    K_I,
    K_D,
    I_ZONE,
    I_MAX_ACCUM,
    TOLERANCE,
    MAX_OUTPUT,
    MIN_OUTPUT
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

  /** the maximum and minimum output of the PID controller (in volts) */
  private double maxOutput, minOutput;

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
      double k_P,
      double k_I,
      double k_D,
      double i_Zone,
      double i_maxAccum,
      double tolerance,
      double maxOutput,
      double minOutput)
      throws IllegalArgumentException {
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

    if (maxOutput <= minOutput)
      throw new IllegalArgumentException("maxOutput must be greater than minOutput");

    if (maxOutput < -maxMotorOutput)
      throw new IllegalArgumentException("maxOutput must be greater than " + -maxMotorOutput);
    if (maxOutput > maxMotorOutput)
      throw new IllegalArgumentException("maxOutput must be less than " + maxMotorOutput);
    this.maxOutput = maxOutput;

    if (minOutput < -maxMotorOutput)
      throw new IllegalArgumentException("minOutput must be greater than " + -maxMotorOutput);
    this.minOutput = minOutput;
  }

  /**
   * creates a PID gains object with the given values
   *
   * @param k_P the proportional gain (>= 0)
   * @param k_I the integral gain (>= 0)
   * @param k_D the derivative gain (>= 0)
   */
  public PIDGains(double k_P, double k_I, double k_D) throws IllegalArgumentException {
    this(
        k_P,
        k_I,
        k_D,
        Double.POSITIVE_INFINITY,
        Double.POSITIVE_INFINITY,
        0,
        maxMotorOutput,
        -maxMotorOutput);
  }

  /**
   * creates a PID gains object with the given values
   *
   * @param k_P the proportional gain (>= 0)
   * @param k_I the integral gain (>= 0)
   * @param k_D the derivative gain (>= 0)
   */
  public PIDGains(double k_P, double k_I, double k_D, double tolerance)
      throws IllegalArgumentException {
    this(
        k_P,
        k_I,
        k_D,
        Double.POSITIVE_INFINITY,
        Double.POSITIVE_INFINITY,
        tolerance,
        maxMotorOutput,
        -maxMotorOutput);
  }

  /** creates an empty PID gains object (no k_P, no k_I, no k_D) */
  public PIDGains() {
    this(0, 0, 0);
  }

  public double getK_P() {
    return k_P;
  }

  public double getK_I() {
    return k_I;
  }

  public double getK_D() {
    return K_D;
  }

  public double getI_Zone() {
    return i_Zone;
  }

  public double getI_MaxAccum() {
    return i_maxAccum;
  }

  public double getTolerance() {
    return tolerance;
  }

  public double getMaxOutput() {
    return maxOutput;
  }

  public double getMinOutput() {
    return minOutput;
  }

  private Double[] getValues() {
    return new Double[] {k_P, k_I, K_D, i_Zone, i_maxAccum, tolerance, maxOutput, minOutput};
  }

  private void setValues(Double[] values) {
    if (values.length != 8) throw new IllegalArgumentException("values must be of length 5");

    this.k_P = values[0];
    this.k_I = values[1];
    this.K_D = values[2];
    this.i_Zone = values[3];
    this.i_maxAccum = values[4];
    this.tolerance = values[5];
    this.maxOutput = values[6];
    this.minOutput = values[7];
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

    Double[] values = getValues();

    double oldValue = values[changeType.ordinal()];

    values[changeType.ordinal()] = value;

    setValues(values);

    return oldValue != value;
  }
}
