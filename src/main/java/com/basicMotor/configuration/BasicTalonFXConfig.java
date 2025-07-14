package com.basicMotor.configuration;

import com.basicMotor.gains.currentLimits.CurrentLimits;

/**
 * This class represents the configuration for a basic TalonFX motor controller.
 * It extends the BasicMotorConfig class and provides specific configurations for TalonFX motors.
 * Use this class when using a TalonFX motor controller.
 * (Falcon 500, Kraken X60, Kraken X44).
 * See the <a href="wiki link">wiki</a> //TODO: add wiki link
 * for more information on how to use this class.
 */
public class BasicTalonFXConfig extends BasicMotorConfig {
  /**
   * The current limits configuration for the TalonFX motor controller.
   * Use this to protect the motor from overheating and drawing too much current.
   */
  public final CurrentLimitConfig currentLimitConfig = new CurrentLimitConfig();

  /**
   * The name of the CAN bus that the TalonFX motor controller is connected to.
   * This is only useful when using a canivore and the motor is connected to a canivore.
   * Otherwise, do not change this value.
   */
  public String canBusName = "rio";

  /**
   * This enables Phoenix Pro.
   * Use this only if your device is licensed for Phoenix Pro.
   * This enables multiple features such as: FOC, synchronized canBus communication, fused canCoder, and more.
   */
  public boolean enablePro = false;

  /**
   * Handles the configuration for the current limits.
   */
  public static class CurrentLimitConfig {
    /**
     * The maximum current output of the motor controller (in amps).
     * This is different from the supply current limit, and will usually be higher.
     * This can be used to limit the force output of the motor, to prevent damaging the mechanism.
     */
    public int statorCurrentLimit = 0;
    /**
     * The maximum current draw of the motor controller (in amps).
     * This is the current that the motor controller will draw from the battery.
     * If the motor draws this amount of current for more then {@link #lowerLimitTime} seconds,
     * it will lower to {@link #lowerCurrentLimit}.
     * Use this if there are brownouts or breakers tripping.
     * Otherwise, it is recommended to use {@link #statorCurrentLimit} instead.
     */
    public int supplyCurrentLimit = 0;
    /**
     * The time (in seconds) that the motor controller will stay in the supply current limit before lowering to the lower current limit.
     * This is useful to prevent the motor from overheating and tripping breakers.
     */
    public double lowerLimitTime = 0;
    /**
     * The current the motor drops to after the supply current limit is reached for {@link #lowerLimitTime}.
     * Works only if {@link #supplyCurrentLimit} and {@link #lowerLimitTime} is set.
     */
    public int lowerCurrentLimit = 0;

    /**
     * Creates the current limit configuration with the given values
     *
     * @return The current limits of the motor controller
     */
    public CurrentLimits getCurrentLimits() {
      return new CurrentLimits(
          statorCurrentLimit, supplyCurrentLimit, lowerLimitTime, lowerCurrentLimit);
    }
  }
}
