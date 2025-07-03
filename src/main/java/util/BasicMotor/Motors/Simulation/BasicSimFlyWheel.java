package util.BasicMotor.Motors.Simulation;


import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import util.BasicMotor.Gains.ControllerGains;
import util.BasicMotor.Measurements.Measurements;
import util.BasicMotor.Measurements.SimulationEncoder.FlyWheelSimEncoder;

/**
 * A class that simulates a flywheel system using the FlywheelSim class.
 * It is in the basic sim motor system and has all the functionality of a basic sim system.
 * Use this when you want to simulate a flywheel in your robot code.
 */
public class BasicSimFlyWheel extends BasicSimSystem {
    /**
     * The FlywheelSim instance used by this BasicSimFlyWheel.
     */
    private final FlywheelSim flywheelSim;

    /**
     * The default measurements for the flywheel simulation.
     */
    private final Measurements defaultMeasurements;

    /**
     * Creates a BasicSimFlyWheel instance with the provided FlywheelSim and name.
     *
     * @param flywheelSim the FlywheelSim to use
     * @param name        the name of the flywheel
     * @param gains       the controller gains for the flywheel
     */
    public  BasicSimFlyWheel(FlywheelSim flywheelSim, String name, ControllerGains gains) {
        super(name, gains);
        this.flywheelSim = flywheelSim;

        defaultMeasurements = new FlyWheelSimEncoder(flywheelSim);
    }

    @Override
    protected void setInputVoltage(double voltage) {
        flywheelSim.setInputVoltage(voltage);
    }

    @Override
    protected double getCurrentDraw() {
        return flywheelSim.getCurrentDrawAmps();
    }

    @Override
    protected Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }

    @Override
    protected void setMotorPosition(double position) {
        //nothing to do here, position doesn't matter for flywheels
    }
}
