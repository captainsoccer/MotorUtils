package util.BasicMotor.Measurements.RevEncoders;

import util.BasicMotor.Measurements.Measurements;

public abstract class MeasurementsREV extends Measurements {
    public abstract void setEncoderPosition(double position);

    public MeasurementsREV(double gearRatio) {
        super(gearRatio);
    }
}
