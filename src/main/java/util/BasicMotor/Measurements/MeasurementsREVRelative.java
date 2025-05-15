package util.BasicMotor.Measurements;


import com.revrobotics.RelativeEncoder;

public class MeasurementsREVRelative extends Measurements {
    private final RelativeEncoder encoder;

    private double lastVelocity = 0;
    private double currentVelocity = 0;
    private double acceleration = 0;

    public MeasurementsREVRelative(RelativeEncoder encoder, double gearRatio) {
        super(gearRatio);
        this.encoder = encoder;
    }

    public MeasurementsREVRelative(RelativeEncoder encoder) {
        this(encoder, 1.0);
    }

    @Override
    public Measurement update(double dt) {
        currentVelocity = encoder.getVelocity();

        acceleration = (currentVelocity - lastVelocity) / dt;

        lastVelocity = currentVelocity;

        return super.update(dt);
    }

    @Override
    protected double getPosition() {
        return encoder.getPosition();
    }

    @Override
    protected double getVelocity() {
        return currentVelocity;
    }

    @Override
    protected double getAcceleration() {
        return acceleration;
    }
}
