package util.PIDController;

import java.util.function.DoubleSupplier;

public class Measurements {
    private double position;
    private double velocity;
    private double acceleration;

    private final DoubleSupplier positionSupplier;
    private final DoubleSupplier velocitySupplier;
    private final DoubleSupplier accelerationSupplier;


}
