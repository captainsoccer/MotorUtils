package util.BasicMotor.Measurements;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class MeasurementsCANCoder extends Measurements{
    /**
     * the timeout for the motor controller to wait for the signals to update if it does not update in
     * this time, it will use the old value
     */
    private final double timeout;

    /** the motor position signal */
    StatusSignal<Angle> motorPosition;
    /** the motor velocity signal */
    StatusSignal<AngularVelocity> motorVelocity;

    /** the latency compensated the value of the motor position */
    double positionLatencyCompensatedValue = 0;

    private double currentVelocity = 0;
    private double lastVelocity = 0;
    private double acceleration = 0;

    /**
     * Creates a new measurements object with the given signals
     *
     * @param positionSignal the position signal of the motor
     * @param velocitySignal the velocity signal of the motor
     * @param refreshHZ the refresh rate of the signals (how often to update the signals)
     * @param gearRatio the gear ratio of the motor (the measurements are divided by this)
     */
    public MeasurementsCANCoder(
            StatusSignal<Angle> positionSignal,
            StatusSignal<AngularVelocity> velocitySignal,
            double refreshHZ,
            double gearRatio) {
        super(gearRatio);

        motorPosition = positionSignal;
        motorVelocity = velocitySignal;

        positionSignal.setUpdateFrequency(refreshHZ);
        velocitySignal.setUpdateFrequency(refreshHZ);

        timeout = 1 / (refreshHZ * 4);
    }

    public void setUpdateFrequency(double refreshHZ) {
        motorPosition.setUpdateFrequency(refreshHZ);
        motorVelocity.setUpdateFrequency(refreshHZ);
    }

    @Override
    public Measurement update(double dt) {
        BaseStatusSignal.waitForAll(timeout, motorPosition, motorVelocity);

        var position = BaseStatusSignal.getLatencyCompensatedValue(motorPosition, motorVelocity);
        positionLatencyCompensatedValue = position.in(Units.Rotations);

        currentVelocity = motorVelocity.getValueAsDouble();

        acceleration = (currentVelocity - lastVelocity) / dt;

        lastVelocity = currentVelocity;

        return super.update(dt);
    }

    @Override
    protected double getUpdatedPosition() {
        return positionLatencyCompensatedValue;
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
