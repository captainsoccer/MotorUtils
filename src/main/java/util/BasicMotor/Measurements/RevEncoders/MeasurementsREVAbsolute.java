package util.BasicMotor.Measurements.RevEncoders;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.DriverStation;

public class MeasurementsREVAbsolute extends  MeasurementsREV {
    private final AbsoluteEncoder encoder;

    private double currentVelocity;
    private double previousVelocity;
    private double acceleration;

    public MeasurementsREVAbsolute(AbsoluteEncoder encoder, double mechanismToSensorRatio) {
        super(mechanismToSensorRatio);
        this.encoder = encoder;
    }

    @Override
    public Measurement update(double dt) {
        currentVelocity = encoder.getVelocity() / 60;

        acceleration = (currentVelocity - previousVelocity) / dt;

        previousVelocity = currentVelocity;

        return super.update(dt);
    }

    @Override
    public void setEncoderPosition(double position) {
        DriverStation.reportWarning("cannot set position of an absolute encoder", true);
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
