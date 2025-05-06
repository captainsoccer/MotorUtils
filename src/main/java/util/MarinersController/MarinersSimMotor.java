package util.MarinersController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import java.util.function.Supplier;

/**
 * A class to control a simulated motor
 * used in conjunction with the {@link MarinersController} class
 * can be used to simulate any motor
 */
public class MarinersSimMotor extends MarinersController {
    /**
     * the motor object
     */
    private final DCMotorSim motor;

    /**
     * the voltage of the master motor (if this motor is a follower) (null if this motor is not a follower)
     */
    private Supplier<Double> motorMasterVoltage;

    /**
     * the output of the motor in volts
     */
    private double motorOutput = 0;

    private MarinersMeasurements createMeasurement(double unitConversion) {
        return new MarinersMeasurements(
                () -> {
                    motor.update(1 / RUN_HZ);
                    return motor.getAngularPositionRotations();
                },
                () -> motor.getAngularVelocity().in(Units.RotationsPerSecond),
                // () -> motor.getAngularAcceleration().in(Units.RotationsPerSecondPerSecond),
                unitConversion
        );
    }

    /**
     * creates the controller
     *
     * @param name            the name of the controller (for logging)
     * @param motorType       the type of motor to simulate (do not include the reduction)
     * @param motorReduction  the ratio between motor and the mechanism (motor turns / mechanism turns) (no unit conversion)
     * @param unitConversion  if using any unit conversion (for example, if the mechanism is in meters, the unit conversion would be the meters per rotation)
     * @param momentOfInertia the moment of inertia of the system (in kg m^2) (after motor reduction (in mechanism units))
     */
    public MarinersSimMotor(String name, DCMotor motorType, double motorReduction, double unitConversion, double momentOfInertia) {
        super(name, ControllerLocation.RIO);

        motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorType, momentOfInertia, motorReduction),
                motorType.withReduction(motorReduction));

        super.setMeasurements(createMeasurement(unitConversion));
    }

    /**
     * creates the controller
     * using a gear ratio of 1
     *
     * @param name            the name of the controller (for logging)
     * @param motorType       the type of motor to simulate (do not include the reduction)
     * @param motorReduction  the ratio between motor and the mechanism (motor turns / mechanism turns) (no unit conversion)
     * @param momentOfInertia the moment of inertia of the system (in kg m^2) (after motor reduction (in mechanism units))
     */
    public MarinersSimMotor(String name, DCMotor motorType, double motorReduction, double momentOfInertia) {
        this(name, motorType, motorReduction, 1, momentOfInertia);
    }

    /**
     * creates the controller
     *
     * @param name      the name of the motor (for logging)
     * @param motorType the type of motor to simulate (do not include the reduction)
     * @param kV        the kv of the motor (in volts / w (radians per sec)) (in mechanism units)
     * @param kA        the ka of the motor (int volts / w / s (radians per sec^2)) (in mechanism units)
     * @param motorReduction  the ratio between motor and the mechanism (motor turns / mechanism turns) (no unit conversion)
     * @param unitConversion  if using any unit conversion (for example, if the mechanism is in meters, the unit conversion would be the meters per rotation)
     */
    public MarinersSimMotor(String name, DCMotor motorType, double kV, double kA, double motorReduction, double unitConversion) {
        super(name, ControllerLocation.RIO);

        motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(kV, kA),
                motorType.withReduction(motorReduction));

        super.setMeasurements(createMeasurement(unitConversion));
    }

    /**
     * creates the controller
     *
     * @param name      the name of the motor (for logging)
     * @param motorType the type of motor to simulate (do not include the reduction)
     * @param kV        the kv of the motor (in mechanism units)
     * @param kA        the ka of the motor (in mechanism units)
     * @param motorReduction  the ratio between motor and the mechanism (motor turns / mechanism turns) (no unit conversion)
     * @param unitConversion  if using any unit conversion (for example, if the mechanism is in meters, the unit conversion would be the meters per rotation)
     */
    public MarinersSimMotor(String name, DCMotor motorType,
                            Per<VoltageUnit, AngularVelocityUnit> kV,
                            Per<VoltageUnit, AngularAccelerationUnit> kA,
                            double motorReduction,
                            double unitConversion) {
        super(name, ControllerLocation.RIO);

        double kv = kV.in(Units.VoltsPerRadianPerSecond);
        double ka = kA.in(Units.VoltsPerRadianPerSecondSquared);

        motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(kv, ka),
                motorType.withReduction(motorReduction));

        super.setMeasurements(createMeasurement(unitConversion));
    }

    /**
     * creates the controller
     *
     * @param name  the name of the motor
     * @param motor the sim motor object don't forget to set the motor reduction (not unit conversion)
     * @param unitConversion the unit conversion for the mechanism (for example, if the mechanism is in meters, the unit conversion would be the meters per rotation)
     */
    public MarinersSimMotor(String name, DCMotorSim motor, double unitConversion) {
        super(name, ControllerLocation.RIO);

        this.motor = motor;

        super.setMeasurements(createMeasurement(unitConversion));
    }

    /**
     * creates the controller
     *
     * @param name  the name of the motor
     * @param motor the sim motor object don't forget to set the motor reduction (not unit conversion)
     */
    public MarinersSimMotor(String name, DCMotorSim motor) {
        this(name, motor, 1);
    }

    @Override
    protected void setOutput(double output, ControlMode controlMode, double feedForward) {
        switch (controlMode) {
            case Position, ProfiledPosition, Velocity, ProfiledVelocity ->
                    throw new UnsupportedOperationException("can't use motor controller in simulation mode for position or velocity control");
            case Stopped, Follower -> motorOutput = 0;
            case DutyCycle -> motorOutput = output * 12;
            case Voltage -> motorOutput = output;
        }

        if (motorMasterVoltage != null) {
            motorOutput = motorMasterVoltage.get();
        }

        motor.setInputVoltage(motorOutput);
    }

    @Override
    protected void updateInputs(MotorInputs inputs) {
        inputs.currentDraw = motor.getCurrentDrawAmps();
        inputs.voltageInput = 12;
        inputs.powerDraw = inputs.currentDraw * 12;
        inputs.voltageOutput = motorOutput;
        inputs.powerOutput = inputs.powerDraw;
        inputs.currentOutput = inputs.powerOutput / inputs.voltageOutput;
        inputs.temperature = 0;
        inputs.dutyCycle = motorOutput / 12;
    }

    @Override
    protected void setMotorFollower(MarinersController master, boolean invert) {
        assert master instanceof MarinersSimMotor;

        motorMasterVoltage = invert ?
                () -> -((MarinersSimMotor) master).motorOutput :
                () -> ((MarinersSimMotor) master).motorOutput;
    }

    @Override
    public void setMotorIdleMode(boolean brake) {
        //nothing to do here
    }

    @Override
    protected void stopMotorOutput() {
        motorOutput = 0;

        motor.setInputVoltage(0);
    }

    @Override
    protected void setPIDFMotor(PIDFGains gains) {
        //nothing to do here
    }

    @Override
    public void setCurrentLimits(int currentLimit, int currentThreshold) {
        //nothing to do here
    }

    @Override
    protected void setMaxMinOutputMotor(double max, double min) {
        //nothing to do here
    }

    @Override
    protected void setMotorDeadBandDutyCycleMotor(double deadBand) {
        //nothing to do here
    }

    @Override
    public void setMotorInverted(boolean inverted) {
        //nothing to do here
    }

    @Override
    public void resetMotorEncoder() {
        motor.setState(0, 0);
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        motor.setState(position, motor.getAngularVelocityRPM());
    }
}
