package com.basicMotor.gains.currentLimits;

/**
 * this class takes the current limits class and makes it fully used for REV motor controllers the
 * rev motor controllers ignore the supply current limit and supply lower time if you don't need to
 * use the free speed RPM or stall current limit, you can use the normal CurrentLimits class
 */
public class CurrentLimitsREV extends CurrentLimits {
  /** the maximum current output of the motor controller (in amps) while in stall */
  private final int stallCurrentLimit;

  /**
   * the secondary current limit if the current output of the motor controller exceeds this limit,
   * the motor controller will stop for a short time
   */
  private final int secondaryCurrentLimit;

  /**
   * the free speed of the motor controller (in RPS) if below this speed the motor is considered in
   * stall and the stall current limit is applied
   */
  private final double freeSpeedRPS;

  /**
   * creates a current limit with the given values
   *
   * @param freeSpeedCurrentLimit the maximum current output of the motor controller (in amps) when
   *     not in stall
   * @param stallCurrentLimit the maximum current output of the motor controller (in amps) while in
   *     stall
   * @param freeSpeedRPS the free speed of the motor controller (in RPS (revolutions per second)) if
   *     below this speed the motor is considered in stall and the stall current limit is applied,
   *     the gear ratio is automatically applied to the free speed so no need to divide by it
   * @param secondaryCurrentLimit a secondary current limit that is not used by REV controllers
   */
  public CurrentLimitsREV(
      int freeSpeedCurrentLimit,
      int stallCurrentLimit,
      double freeSpeedRPS,
      int secondaryCurrentLimit) {
    super(freeSpeedCurrentLimit, 0, 0, 0);
    this.stallCurrentLimit = Math.abs(stallCurrentLimit);
    this.freeSpeedRPS = Math.abs(freeSpeedRPS);
    this.secondaryCurrentLimit = Math.abs(secondaryCurrentLimit);
  }

  /**
   * gets the stall current limit of the motor controller
   *
   * @return the stall current limit of the motor controller (in amps)
   */
  public int getStallCurrentLimit() {
    return stallCurrentLimit;
  }

  /**
   * gets the free speed of the motor controller
   *
   * @return the free speed of the motor controller (in RPS (revolutions per second))
   */
  public double getFreeSpeedRPS() {
    return freeSpeedRPS;
  }

  /**
   * gets the secondary current limit of the motor controller
   *
   * @return the secondary current limit of the motor controller (in amps)
   */
  public int getSecondaryCurrentLimit() {
    return secondaryCurrentLimit;
  }
}
