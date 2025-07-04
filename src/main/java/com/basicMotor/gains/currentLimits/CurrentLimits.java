package com.basicMotor.gains.currentLimits;

/**
 * This class is used to set the current limits of the motor controller. It is used to limit the
 * current draw of the motor to prevent overheating and damage to the motor. this is modeled after
 * the TalonFX current limits so other motor controllers may not have the same limits
 */
public class CurrentLimits {
  /**
   * the maximum current output of the motor controller (in amps) this is different from
   * supplyCurrentLimit
   */
  private final int statorCurrentLimit;
  /**
   * the maximum current draw of the motor controller (in amps) this is different from
   * statorCurrentLimit
   */
  private final int supplyCurrentLimit;
  /**
   * the time (in seconds) that the motor can stay at supply current limit before it lowers to
   * supply lower limit this is useful for bursts of current
   */
  private final double supplyLowerTime;
  /**
   * the current the motor drops to after the supply current limit is reached for supplyLowerTime
   * this is useful for bursts of current
   */
  private final int supplyLowerLimit;

  /**
   * creates a current limit with the given values
   *
   * @param statorCurrentLimit the maximum current output of the motor controller (in amps)
   * @param supplyCurrentLimit the maximum current draw of the motor controller (in amps)
   * @param supplyLowerTime the time (in seconds) that the motor can stay at supply current limit
   * @param supplyLowerLimit the current the motor drops to after the supply current limit is
   *     reached for
   */
  public CurrentLimits(
      int statorCurrentLimit,
      int supplyCurrentLimit,
      double supplyLowerTime,
      int supplyLowerLimit) {
    this.statorCurrentLimit = Math.abs(statorCurrentLimit);
    this.supplyCurrentLimit = Math.abs(supplyCurrentLimit);
    this.supplyLowerTime = Math.abs(supplyLowerTime);
    this.supplyLowerLimit = Math.abs(supplyLowerLimit);
  }

  /**
   * creates a current limit with the given values
   *
   * @param statorCurrentLimit the maximum current output of the motor controller (in amps)
   * @param supplyCurrentLimit the maximum current draw of the motor controller (in amps)
   */
  public CurrentLimits(int statorCurrentLimit, int supplyCurrentLimit) {
    this(statorCurrentLimit, supplyCurrentLimit, 0, 0);
  }

  /**
   * gets the maximum current output of the motor controller (in amps)
   *
   * @return the maximum current output of the motor controller (in amps)
   */
  public int getStatorCurrentLimit() {
    return statorCurrentLimit;
  }

  /**
   * gets the maximum current draw of the motor controller (in amps)
   *
   * @return the maximum current draw of the motor controller (in amps)
   */
  public int getSupplyCurrentLimit() {
    return supplyCurrentLimit;
  }

  /**
   * gets the time (in seconds) that the motor can stay at supply current limit before it lowers to
   * supply lower limit this is useful for bursts of current
   *
   * @return the time (in seconds) that the motor can stay at supply current limit
   */
  public double getSupplyLowerTime() {
    return supplyLowerTime;
  }

  /**
   * gets the current the motor drops to after the supply current limit is reached for
   * supplyLowerTime this is useful for bursts of current
   *
   * @return the current the motor drops to after the supply current limit is reached for
   *     supplyLowerTime
   */
  public int getSupplyLowerLimit() {
    return supplyLowerLimit;
  }
}
