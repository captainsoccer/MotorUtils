package util.BasicMotor.Motors.Simulation;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import util.BasicMotor.Gains.ControllerGains;
import util.BasicMotor.Measurements.Measurements;
import util.BasicMotor.Measurements.SimulationEncoder.MotorSimEncoder;

public class BasicSimMotor extends BasicSimSystem{
    private final DCMotorSim motor;
    private final Measurements defaultMeasurements;

    public BasicSimMotor(DCMotorSim motor, String name, ControllerGains gains, double gearRatio, double unitConversion) {
        super(motor, name, gains);
        this.motor = motor;

        defaultMeasurements = new MotorSimEncoder(motor, gearRatio, unitConversion);
    }

    @Override
    protected double getCurrentDraw() {
        return motor.getCurrentDrawAmps();
    }

    @Override
    protected Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }
}
