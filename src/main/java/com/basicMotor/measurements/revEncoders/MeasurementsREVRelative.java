package com.basicMotor.measurements.revEncoders;

import com.basicMotor.measurements.Measurements;
import com.revrobotics.RelativeEncoder;

/**
 * This class is used to get the measurements from the REV motor controller it handles updating the
 * measurements and getting the latency compensated values
 */
public class MeasurementsREVRelative extends Measurements {
    /**
     * the encoder used to get the measurements from the motor controller
     */
    private final RelativeEncoder encoder;

    /**
     * the last velocity of the motor controller this is used to calculate the acceleration
     */
    private double lastVelocity = 0;
    /**
     * the current velocity of the motor controller this is used to calculate the acceleration
     */
    private double currentVelocity = 0;
    /**
     * the acceleration of the motor controller this is used to calculate the acceleration
     */
    private double acceleration = 0;

    /**
     * Creates a new measurements object with the given encoder and gear ratio
     *
     * @param encoder        the encoder used to get the measurements from the motor controller
     * @param gearRatio      the gear ratio of the motor (the measurements are divided by this)
     * @param unitConversion the value that will be multiplied by to convert the measurements to the
     *                       desired units
     */
    public MeasurementsREVRelative(RelativeEncoder encoder, double gearRatio, double unitConversion) {
        super(gearRatio, unitConversion);
        this.encoder = encoder;
    }

    @Override
    public Measurement update(double dt) {
        currentVelocity = encoder.getVelocity() / 60; // Convert to rotations per second

        acceleration = (currentVelocity - lastVelocity) / dt;

        lastVelocity = currentVelocity;

        return super.update(dt);
    }

    @Override
    protected double getUpdatedPosition() {
        return encoder.getPosition();
    }

    @Override
    protected double getUpdatedVelocity() {
        return currentVelocity;
    }

    @Override
    protected double getUpdatedAcceleration() {
        return acceleration;
    }

    /**
     * Sets the position of the encoder. This is useful for resetting the encoder position.
     *
     * @param position the new position to set for the encoder
     */
    public void setEncoderPosition(double position) {
        encoder.setPosition(position);
    }
}
