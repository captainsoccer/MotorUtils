package util.BasicMotor.Motors.Simulation;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import util.BasicMotor.Configuration.BasicMotorConfig;
import util.BasicMotor.Gains.ControllerConstrains;
import util.BasicMotor.Gains.ControllerGains;
import util.BasicMotor.Measurements.Measurements;
import util.BasicMotor.Measurements.SimulationEncoder.ArmSimEncoder;

public class BasicSimArm extends BasicSimSystem {
    private final SingleJointedArmSim armSim;

    private final Measurements defaultMeasurements;

    public BasicSimArm(SingleJointedArmSim armSim, String name, ControllerGains gains) {
        super(name, gains);
        this.armSim = armSim;

        defaultMeasurements = new ArmSimEncoder(armSim);
    }

    public BasicSimArm(BasicMotorConfig config) {
        super(config);

        this.armSim = createArmSim(config);

        defaultMeasurements = new ArmSimEncoder(armSim);
    }

    private static SingleJointedArmSim createArmSim(BasicMotorConfig config) {
        var plant = LinearSystemId.createSingleJointedArmSystem(config.motorConfig.motorType,
                config.simulationConfig.momentOfInertia, config.motorConfig.gearRatio);

        double minAngle;
        double maxAngle;
        if(config.constraintsConfig.constraintType == ControllerConstrains.ConstraintType.LIMITED){
            minAngle = Units.rotationsToRadians(config.constraintsConfig.minValue);
            maxAngle = Units.rotationsToRadians(config.constraintsConfig.maxValue);
        }
        else{
            minAngle = Double.NEGATIVE_INFINITY;
            maxAngle = Double.POSITIVE_INFINITY;
        }

        var simConfig = config.simulationConfig;

        double positionSTD = Units.rotationsToRadians(simConfig.positionStandardDeviation);

        return new SingleJointedArmSim(plant, config.motorConfig.motorType, config.motorConfig.gearRatio,
                simConfig.armSimConfig.armlengthMeters,
                minAngle,
                maxAngle,
                simConfig.armSimConfig.simulateGravity,
                simConfig.armSimConfig.startingAngle,
                positionSTD);
    }

    @Override
    protected void setInputVoltage(double voltage) {
        armSim.setInputVoltage(voltage);
    }

    @Override
    protected double getCurrentDraw() {
        return armSim.getCurrentDrawAmps();
    }

    @Override
    protected Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }

    @Override
    protected void setMotorPosition(double position) {
        double angle = Units.rotationsToRadians(position);
        armSim.setState(angle, armSim.getVelocityRadPerSec());
    }
}
