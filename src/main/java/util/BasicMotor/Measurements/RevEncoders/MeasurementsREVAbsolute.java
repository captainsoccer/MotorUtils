package util.BasicMotor.Measurements.RevEncoders;

import com.revrobotics.AbsoluteEncoder;
import util.BasicMotor.Measurements.Measurements;

public class MeasurementsREVAbsolute extends Measurements {
    /**
     * the absolute encoder used to get the measurements
     */
    private final AbsoluteEncoder encoder;

    /**
     * the current velocity of the motor in rotations per second
     */
    private double currentVelocity;
    /**
     * the previous velocity of the motor in rotations per second
     */
    private double previousVelocity;
    /**
     * the acceleration of the motor in rotations per second squared
     */
    private double acceleration;

    /**
     * Creates a new measurements object with the given encoder and mechanism to a sensor ratio
     *
     * @param encoder the absolute encoder used to get the measurements
     * @param mechanismToSensorRatio the ratio of the mechanism to the sensor (how many rotations of the mechanism are one rotation of the sensor)
     */
    public MeasurementsREVAbsolute(AbsoluteEncoder encoder, double mechanismToSensorRatio) {
        super(mechanismToSensorRatio);
        this.encoder = encoder;
    }

    @Override
    public Measurement update(double dt) {
        //converts rpm to rps
        currentVelocity = encoder.getVelocity() / 60;

        acceleration = (currentVelocity - previousVelocity) / dt;

        previousVelocity = currentVelocity;

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
}
