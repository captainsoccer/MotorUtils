package com.basicMotor.configuration;

import com.basicMotor.gains.currentLimits.CurrentLimits;

/**
 * This class represents the configuration for a basic TalonFX motor controller. It extends the
 * BasicMotorConfig class and provides specific configurations for TalonFX motors.
 */
public class BasicTalonFXConfig extends BasicMotorConfig {
  /**
   * the current limit configuration of the motor controller
   *
   * <p>this is used to set the current limits of the motor controller
   */
  public final CurrentLimitConfig currentLimitConfig = new CurrentLimitConfig();

  /**
   * the CAN bus name to use for the motor controller the default is "rio" which is the name of the
   * CAN bus on the roboRIO do not change this unless you know what you are doing
   */
  public String canBusName = "rio";

  /**
   * the config for the current limit of the talonFX motor controller
   */
  public static class CurrentLimitConfig {
    /**
     * the maximum current output of the motor controller (in amps)
     *
     * <p>this is the max current the motor windings will allow before stopping, this will usually
     * be higher than the supply current limit by a wide margin.
     */
    public int statorCurrentLimit = 0;
    /**
     * the maximum current draw of the motor controller (in amps)
     *
     * <p>this is the max current the motor controller will draw from the battery before stopping or
     * lowering to the {@link #lowerCurrentLimit} after the {@link #lowerLimitTime} has passed. this
     * is useful if the robot is experiencing brownouts or breakers are tripping.
     */
    public int supplyCurrentLimit = 0;
    /**
     * the time (in seconds) that the motor can stay at supply current limit before it lowers to
     * {@link #lowerCurrentLimit} this is useful for bursts of current needed for a short time
     */
    public double lowerLimitTime = 0;
    /**
     * the current the motor drops to after the supply current limit is reached for {@link
     * #lowerLimitTime} this is useful for bursts of current
     *
     * <p>this is usually lower than the supply current limit and is used to prevent overheating
     */
    public int lowerCurrentLimit = 0;

    /**
     * creates the current limit configuration with the given values
     *
     * @return the current limits of the motor controller
     */
    public CurrentLimits getCurrentLimits() {
      return new CurrentLimits(
          statorCurrentLimit, supplyCurrentLimit, lowerLimitTime, lowerCurrentLimit);
    }
  }
}
