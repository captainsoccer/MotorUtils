package com.basicMotor.configuration;

import com.basicMotor.gains.currentLimits.CurrentLimitsREV;
import com.basicMotor.motors.sparkBase.BasicSparkBase;

/**
 * This class represents the configuration for a basic spark base motor controller. It extends the
 * BasicMotorConfig class and provides specific configurations for spark base motors.
 */
public class BasicSparkBaseConfig extends BasicMotorConfig {

  /** the current limit configuration for the spark base motor controller */
  public final CurrentLimitConfig currentLimitConfig = new CurrentLimitConfig();

  /**
   * the external encoder configuration for the spark base motor controller this is used to
   * configure the external encoder connected directly to the motor controller
   */
  public final ExternalEncoderConfig externalEncoderConfig = new ExternalEncoderConfig();

  /**
   * the absolute encoder configuration for the spark base motor controller this is used to
   * configure the absolute encoder connected directly to the motor controller
   */
  public final AbsoluteEncoderConfig absoluteEncoderConfig = new AbsoluteEncoderConfig();

  @Override
  public boolean usingExternalEncoder() {
    return externalEncoderConfig.useExternalEncoder || absoluteEncoderConfig.useAbsoluteEncoder;
  }

  /** the current limit configuration for a spark base motor controller */
  public static class CurrentLimitConfig {
    /**
     * the maximum current output of the motor controller while in free speed (in amps) free speed
     * can be defined in {@link #freeSpeedRPS}
     *
     * <p>this is the max current the motor windings will allow before stopping, this will usually
     * be higher than the supply current limit by a wide margin.
     */
    public int freeSpeedCurrentLimit = 0;
    /**
     * the maximum current draw of the motor controller while in stall (in amps) anything below the
     * free speed is considered stall, free speed can be defined in {@link #freeSpeedRPS}
     */
    public int stallCurrentLimit = 0;
    /**
     * the free speed of the motor controller (in RPS (revolutions per second)) any speed above this
     * speed is considered free speed and the free speed current limit will be applied, if below
     * this speed the motor is considered in stall and the stall current limit is applied. gear rato
     * is automatically applied to the free speed so no need to divide by it
     */
    public double freeSpeedRPS = 0;
    /**
     * the max current output of the motor controller (in amps) if the current output of the motor
     * controller exceeds this limit, the motor controller will stop for a short time
     */
    public int secondaryCurrentLimit = 0;

    /**
     * creates the current limit configuration with the given values
     *
     * @return the current limits of the motor controller
     */
    public CurrentLimitsREV getCurrentLimits() {
      return new CurrentLimitsREV(
          freeSpeedCurrentLimit, stallCurrentLimit, freeSpeedRPS, secondaryCurrentLimit);
    }
  }

  /** the config of a spark base motor controller external encoder */
  public static class ExternalEncoderConfig {
    /** should the motor use an external encoder (connected directly to the motor controller)? */
    public boolean useExternalEncoder = false;

    /** is the external encoder inverted (does it count in the opposite direction of the motor)? */
    public boolean inverted = false;

    /**
     * the number that the reading of the external encoder should be multiplied by to get the motor
     * output a value bigger than 1.0 is a reduction in the motor output. (the motor spins more than
     * the sensor)
     */
    public double sensorToMotorRatio = 1.0;

    /**
     * the number that the reading of the external encoder should be divided by to get the mechanism
     * output a value bigger than 1.0 is a reduction in the mechanism output. (the sensor spins more
     * than the mechanism)
     */
    public double mechanismToSensorRatio = 1.0;
  }

  /** the config of a spark base motor controller absolute encoder */
  public static class AbsoluteEncoderConfig {
    /** should the motor use an absolute encoder (connected directly to the motor controller)? */
    public boolean useAbsoluteEncoder = false;

    /** is the absolute encoder inverted (does it count in the opposite direction of the motor)? */
    public boolean inverted = false;

    /**
     * the zero offset of the absolute encoder (in rotations) the reading of the absolute encoder
     * with this value will be considered zero
     */
    public double zeroOffset = 0.0;

    /**
     * the number that the reading of the absolute encoder should be multiplied by to get the motor
     * output a value bigger than 1.0 is a reduction in the motor output. (the motor spins more than
     * the sensor)
     */
    public double sensorToMotorRatio = 1.0;

    /**
     * the number that the reading of the absolute encoder should be divided by to get the mechanism
     * output a value bigger than 1.0 is a reduction in the mechanism output. (the sensor spins more
     * than the mechanism)
     */
    public double mechanismToSensorRatio = 1.0;

    /**
     * the range of the absolute encoder this is used to determine the range of the absolute encoder
     * it can be set to ZERO_TO_ONE or HALF_ROTATION.
     */
    public BasicSparkBase.AbsoluteEncoderRange absoluteEncoderRange =
        BasicSparkBase.AbsoluteEncoderRange.ZERO_TO_ONE;
  }
}
