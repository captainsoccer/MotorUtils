package util.BasicMotor.Motors.Simulation;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import util.BasicMotor.Configuration.BasicMotorConfig;
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

    public BasicSimMotor(BasicMotorConfig config) {
        this(createSimMotor(config), config.motorConfig.name, config.getControllerGains(),
                config.motorConfig.gearRatio, config.motorConfig.unitConversion);
    }

    private static DCMotorSim createSimMotor(BasicMotorConfig config){
        var simConfig = config.simulationConfig;

        if(simConfig.momentOfInertia == 0 && simConfig.kV == 0 && simConfig.kA == 0)
            throw new IllegalArgumentException("you must provide either a moment of inertia or kV and kA for the simulation motor");

        var plant = simConfig.momentOfInertia == 0
                ? LinearSystemId.createDCMotorSystem(simConfig.kV, simConfig.kA)
                : LinearSystemId.createDCMotorSystem(config.motorConfig.motorType, simConfig.momentOfInertia, config.motorConfig.gearRatio);

        return new DCMotorSim(plant, config.motorConfig.motorType, simConfig.positionStandardDeviation, simConfig.velocityStandardDeviation);
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
