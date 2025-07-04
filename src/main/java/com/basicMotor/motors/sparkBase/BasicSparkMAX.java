package com.basicMotor.motors.sparkBase;

import com.basicMotor.configuration.BasicMotorConfig;
import com.basicMotor.configuration.BasicSparkBaseConfig;
import com.basicMotor.gains.ControllerGains;
import com.basicMotor.MotorManager;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * This class represents a basic Spark Max motor controller.
 * It extends the BasicSparkBase class
 * and provides specific implementations for the Spark Max motor controller.
 */
public class BasicSparkMAX extends BasicSparkBase {
  /**
   * creates a basic spark max motor controller with the given gains and id
   *
   * @param gains the gains of the motor controller
   * @param id the id of the motor controller
   * @param name the name of the motor controller
   * @param gearRatio the gear ratio of the motor controller
   * @param unitConversion the value that will be multiplied by to convert the measurements to the
   *     desired units
   * @param location the location of the motor controller (RIO or MOTOR)
   */
  public BasicSparkMAX(
      ControllerGains gains,
      int id,
      String name,
      double gearRatio,
      double unitConversion,
      MotorManager.ControllerLocation location) {

    super(
        new SparkMax(id, SparkLowLevel.MotorType.kBrushless),
        new SparkMaxConfig(),
        gains,
        name,
        gearRatio,
        unitConversion,
        location);
  }

  /**
   * creates a basic spark max motor controller with the given gains and id
   *
   * @param gains the gains of the motor controller
   * @param id the id of the motor controller
   * @param name the name of the motor controller
   * @param gearRatio the gear ratio of the motor controller
   * @param location the location of the motor controller (RIO or MOTOR)
   */
  public BasicSparkMAX(
      ControllerGains gains,
      int id,
      String name,
      double gearRatio,
      MotorManager.ControllerLocation location) {

    this(gains, id, name, gearRatio, 1, location);
  }

  /**
   * creates a basic spark max motor controller with the given configuration
   *
   * @param config the configuration of the motor controller
   */
  public BasicSparkMAX(BasicMotorConfig config) {
    super(
        new SparkMax(config.motorConfig.id, SparkLowLevel.MotorType.kBrushless),
        new SparkMaxConfig(),
        config);

    if (config instanceof BasicSparkBaseConfig sparkBaseConfig) {
      if (sparkBaseConfig.externalEncoderConfig.useExternalEncoder
          && sparkBaseConfig.absoluteEncoderConfig.useAbsoluteEncoder) {
        throw new IllegalArgumentException(
            "motor: "
                + config.motorConfig.name
                + " cannot use both absolute and external encoders at the same time");
      }
    }
  }

  @Override
  public void useAbsoluteEncoder(
      boolean inverted,
      double zeroOffset,
      double sensorToMotorRatio,
      double mechanismToSensorRatio,
      AbsoluteEncoderRange absoluteEncoderRange) {
    // sets the absolute encoder configuration
    config.absoluteEncoder.setSparkMaxDataPortConfig();

    super.useAbsoluteEncoder(
        inverted, zeroOffset, sensorToMotorRatio, mechanismToSensorRatio, absoluteEncoderRange);
  }

  @Override
  public void useExternalEncoder(
      boolean inverted, double sensorToMotorRatio, double mechanismToSensorRatio) {
    config.absoluteEncoder.setSparkMaxDataPortConfig();

    super.useExternalEncoder(inverted, sensorToMotorRatio, mechanismToSensorRatio);
  }

  @Override
  protected void configExternalEncoder(
      boolean inverted, double sensorToMotorRatio, double mechanismToSensorRatio) {
    assert config instanceof SparkMaxConfig;

    var config = (SparkMaxConfig) this.config;

    // sets the absolute encoder configuration
    config.alternateEncoder.setSparkMaxDataPortConfig();
    // sets whether the absolute encoder is inverted or not
    config.alternateEncoder.inverted(inverted);
    // sets the conversion factor for the absolute encoder position and velocity
    config.alternateEncoder.positionConversionFactor(sensorToMotorRatio);
    config.alternateEncoder.velocityConversionFactor(sensorToMotorRatio);
  }

  @Override
  protected RelativeEncoder getExternalEncoder() {
    assert motor instanceof SparkMax;

    var sparkMax = (SparkMax) motor;

    return sparkMax.getAlternateEncoder();
  }
}
