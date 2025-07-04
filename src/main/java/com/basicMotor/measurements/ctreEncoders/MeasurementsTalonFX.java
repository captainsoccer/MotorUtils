package com.basicMotor.measurements.ctreEncoders;

import com.basicMotor.measurements.Measurements;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * This class is used to get the measurements from the CTRE motor controller it handles updating the
 * measurements and getting the latency compensated values
 */
public class MeasurementsTalonFX extends Measurements {

  /**
   * the timeout for the motor controller to wait for the signals to update if it does not update in
   * this time, it will use the old value
   */
  private final double timeout;

  /** the motor position signal */
  StatusSignal<Angle> motorPosition;
  /** the motor velocity signal */
  StatusSignal<AngularVelocity> motorVelocity;
  /** the motor acceleration signal */
  StatusSignal<AngularAcceleration> motorAcceleration;

  /** the latency compensated the value of the motor position */
  double positionLatencyCompensatedValue = 0;
  /** the latency compensated the value of the motor velocity */
  double velocityLatencyCompensatedValue = 0;

  /**
   * Creates a new measurements object with the given signals
   *
   * @param positionSignal the position signal of the motor
   * @param velocitySignal the velocity signal of the motor
   * @param accelerationSignal the acceleration signal of the motor
   * @param refreshHZ the refresh rate of the signals (how often to update the signals)
   * @param gearRatio the gear ratio of the motor (the measurements are divided by this)
   * @param unitConversion the value that will be multiplied by to convert the measurements to the desired units
   */
  public MeasurementsTalonFX(
      StatusSignal<Angle> positionSignal,
      StatusSignal<AngularVelocity> velocitySignal,
      StatusSignal<AngularAcceleration> accelerationSignal,
      double refreshHZ,
      double gearRatio,
      double unitConversion) {
    super(gearRatio, unitConversion);

    motorPosition = positionSignal;
    motorVelocity = velocitySignal;
    motorAcceleration = accelerationSignal;

    positionSignal.setUpdateFrequency(refreshHZ);
    velocitySignal.setUpdateFrequency(refreshHZ);
    accelerationSignal.setUpdateFrequency(refreshHZ);

    timeout = 1 / (refreshHZ * 4);
  }

  /**
   * sets the update frequency of the signals
   *
   * @param refreshHZ the refresh rate of the signals (how often to update the signals)
   */
  public void setUpdateFrequency(double refreshHZ) {
    motorPosition.setUpdateFrequency(refreshHZ);
    motorVelocity.setUpdateFrequency(refreshHZ);
    motorAcceleration.setUpdateFrequency(refreshHZ);
  }

  @Override
  public Measurement update(double dt) {
    BaseStatusSignal.waitForAll(timeout, motorPosition, motorVelocity, motorAcceleration);

    var position = BaseStatusSignal.getLatencyCompensatedValue(motorPosition, motorVelocity);
    positionLatencyCompensatedValue = position.in(Units.Rotations);

    var velocity = BaseStatusSignal.getLatencyCompensatedValue(motorVelocity, motorAcceleration);
    velocityLatencyCompensatedValue = velocity.in(Units.RotationsPerSecond);

    return super.update(dt);
  }

  @Override
  protected double getUpdatedPosition() {
    return positionLatencyCompensatedValue;
  }

  @Override
  protected double getUpdatedVelocity() {
    return velocityLatencyCompensatedValue;
  }

  @Override
  protected double getUpdatedAcceleration() {
    return motorAcceleration.getValueAsDouble();
  }
}
