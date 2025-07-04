package com.basicMotor.motors.sparkBase;

import com.basicMotor.configuration.BasicMotorConfig;
import com.basicMotor.configuration.BasicSparkBaseConfig;
import com.basicMotor.gains.ControllerGains;
import com.basicMotor.MotorManager;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;

/**
 * This class represents a basic Spark Flex motor controller. it extends the BasicSparkBase class
 * and provides specific implementations
 */
public class BasicSparkFlex extends BasicSparkBase {
  /**
   * creates a basic spark flex motor controller with the given gains and id
   *
   * @param gains the gains of the motor controller
   * @param id the id of the motor controller
   * @param name the name of the motor controller
   * @param gearRatio the gear ratio of the motor controller
   * @param unitConversion the value that will be multiplied by to convert the measurements to the
   *     desired units
   * @param location the location of the motor controller (RIO or MOTOR)
   */
  public BasicSparkFlex(
      ControllerGains gains,
      int id,
      String name,
      double gearRatio,
      double unitConversion,
      MotorManager.ControllerLocation location) {

    super(
        new SparkFlex(id, SparkLowLevel.MotorType.kBrushless),
        new SparkFlexConfig(),
        gains,
        name,
        gearRatio,
        unitConversion,
        location);
  }

  /**
   * creates a basic spark flex motor controller with the given gains and id
   *
   * @param gains the gains of the motor controller
   * @param id the id of the motor controller
   * @param name the name of the motor controller
   * @param gearRatio the gear ratio of the motor controller
   * @param location the location of the motor controller (RIO or MOTOR)
   */
  public BasicSparkFlex(
      ControllerGains gains,
      int id,
      String name,
      double gearRatio,
      MotorManager.ControllerLocation location) {

    this(gains, id, name, gearRatio, 1, location);
  }

  /**
   * creates a basic spark flex motor controller with the given configuration
   *
   * @param config the configuration of the motor controller
   */
  public BasicSparkFlex(BasicMotorConfig config) {
    super(
        new SparkFlex(config.motorConfig.id, SparkLowLevel.MotorType.kBrushless),
        new SparkFlexConfig(),
        config);

    if (config instanceof BasicSparkBaseConfig sparkBaseConfig) {
      if (sparkBaseConfig.externalEncoderConfig.useExternalEncoder
          && sparkBaseConfig.absoluteEncoderConfig.useAbsoluteEncoder) {
        var absoluteEncoderConfig = sparkBaseConfig.absoluteEncoderConfig;

        useExternalEncoderWithAbsoluteEncoder(
            absoluteEncoderConfig.inverted,
            absoluteEncoderConfig.zeroOffset,
            absoluteEncoderConfig.sensorToMotorRatio,
            absoluteEncoderConfig.mechanismToSensorRatio,
            absoluteEncoderConfig.absoluteEncoderRange);
      }
    }
  }

  @Override
  protected void configExternalEncoder(
      boolean inverted, double sensorToMotorRatio, double mechanismToSensorRatio) {
    assert config instanceof SparkFlexConfig;

    var config = (SparkFlexConfig) this.config;

    // sets whether the absolute encoder is inverted or not
    config.externalEncoder.inverted(inverted);
    // sets the conversion factor for the absolute encoder position and velocity
    config.externalEncoder.positionConversionFactor(sensorToMotorRatio);
    config.externalEncoder.velocityConversionFactor(sensorToMotorRatio);
  }

  @Override
  protected RelativeEncoder getExternalEncoder() {
    assert motor instanceof SparkFlex;

    var sparkMax = (SparkFlex) motor;

    return sparkMax.getExternalEncoder();
  }

  /**
   * use this when you want to use an external encoder which doubles as an absolute encoder but you
   * want to use the relative encoder for the pid controller (not bound to a range and more
   * accurate)
   *
   * @param inverted whether the external encoder is inverted (counts in the opposite direction of
   *     the motor)
   * @param sensorToMotorRatio the number that the reading of the external encoder should be
   *     multiplied by to get the motor output
   * @param mechanismToSensorRatio the number that the reading of the external encoder should be
   *     divided by to get the mechanism output
   * @param zeroOffset the zero offset of the absolute encoder (in rotations)
   * @param absoluteEncoderRange the range of the absolute encoder
   */
  public void useExternalEncoderWithAbsoluteEncoder(
      boolean inverted,
      double sensorToMotorRatio,
      double mechanismToSensorRatio,
      double zeroOffset,
      AbsoluteEncoderRange absoluteEncoderRange) {

    // configures the absolute encoder
    setAbsoluteEncoderConfig(inverted, zeroOffset, sensorToMotorRatio, absoluteEncoderRange);

    // sets the external encoder configuration
    useExternalEncoder(inverted, sensorToMotorRatio, mechanismToSensorRatio);

    assert motor instanceof SparkFlex;

    var sparkFlex = (SparkFlex) motor;

    sparkFlex.getExternalEncoder().setPosition(sparkFlex.getAbsoluteEncoder().getPosition());
  }
}
