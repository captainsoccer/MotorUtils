package util.BasicMotor.Measurements.SimulationEncoder;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import util.BasicMotor.Measurements.Measurements;

public class MotorSimEncoder extends Measurements {
    private final DCMotorSim motor;

    public MotorSimEncoder(DCMotorSim motor, double gearRatio, double unitConversion) {
        super(gearRatio, unitConversion);
        this.motor = motor;
    }

    public MotorSimEncoder(DCMotorSim motor, double gearRatio) {
        this(motor, gearRatio, 1);
    }

    @Override
    public Measurement update(double dt){
        motor.update(dt);

        return super.update(dt);
    }

    @Override
    protected double getUpdatedPosition() {
        return motor.getAngularPosition().in(Units.Rotations);
    }

    @Override
    protected double getUpdatedVelocity() {
        return motor.getAngularVelocity().in(Units.RotationsPerSecond);
    }

    @Override
    protected double getUpdatedAcceleration() {
        return motor.getAngularAcceleration().in(Units.RotationsPerSecondPerSecond);
    }
}
