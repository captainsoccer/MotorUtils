package util.MarinersController;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import java.util.LinkedList;

/**
 * A class to control a TalonFX motor controller
 * @see MarinersController
 * @see TalonFX
 */
public class MarinersTalonFX extends MarinersController {

    private static final int DEVICE_TEMP_UPDATE_HZ = 10;
    private static final int SUPPLY_CURRENT_UPDATE_HZ = 10;
    private static final int STATOR_CURRENT_UPDATE_HZ = 10;
    private static final int SUPPLY_VOLTAGE_UPDATE_HZ = 10;
    private static final int MOTOR_VOLTAGE_UPDATE_HZ = 50;
    private static final int DUTY_CYCLE_UPDATE_HZ = 100;

    /**
     * the TalonFX motor controller
     */
    private final TalonFX motor;

    /**
     * the configuration for the motor output
     * (needed that info is not lost when changing the motor output)
     */
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final StatusSignal<Temperature> deviceTemp;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Voltage> supplyVoltage;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Double> dutyCycle;

    private final BaseStatusSignal[] signals;

    /**
     * create a new measurement object for the motor
     * using the built-in position, velocity, and acceleration
     * this creates a measurement object that waits for all the signals to update before returning the value
     * (that means that it is a blocking call)
     * @param gearRatio the gear ratio of the motor
     * @return the new measurement object
     */
    private MarinersMeasurements createMeasurement(double gearRatio){
        return new MarinersMeasurementsCTRE(
                motor.getPosition(),
                motor.getVelocity(),
                motor.getAcceleration(),
                gearRatio
        );
    }

    public MarinersTalonFX(String name, ControllerLocation location, int id, double gearRatio){
        super(name, location);

        this.motor = createMotor(id);

        this.deviceTemp = motor.getDeviceTemp();
        this.supplyCurrent = motor.getSupplyCurrent();
        this.statorCurrent = motor.getStatorCurrent();
        this.supplyVoltage = motor.getSupplyVoltage();
        this.motorVoltage = motor.getMotorVoltage();
        this.dutyCycle = motor.getDutyCycle();

        LinkedList<BaseStatusSignal> signals = new LinkedList<>();

        signals.add(deviceTemp);
        signals.add(supplyCurrent);
        signals.add(statorCurrent);
        signals.add(supplyVoltage);
        signals.add(motorVoltage);
        signals.add(dutyCycle);

        this.signals = new BaseStatusSignal[signals.size()];

        signals.toArray(this.signals);

        super.setMeasurements(createMeasurement(gearRatio));
    }

    /**
     * creates the controller
     * @param name the name of the controller (for logging)
     * @param location the location of the controller (RIO or MOTOR)
     */
    public MarinersTalonFX(String name, ControllerLocation location, int id){
        this(name, location, id, 1);
    }

    /**
     * creates the controller
     * @param name the name of the controller (for logging)
     * @param location the location of the controller (RIO or MOTOR)
     * @param gains the PIDF gains for the controller (the units are voltage to measurements units)
     */
    public MarinersTalonFX(String name, ControllerLocation location, int id, PIDFGains gains){
        this(name, location, id, gains, 1);
    }

    /**
     * creates the controller
     * @param name the name of the controller (for logging)
     * @param location the location of the controller (RIO or MOTOR)
     * @param gains the PIDF gains for the controller
     * @param gearRatio the gear ratio of the motor
     */
    public MarinersTalonFX(String name, ControllerLocation location, int id, PIDFGains gains, double gearRatio){
        this(name, location, id, gearRatio);

        setPIDF(gains);
    }

    /**
     * @return the TalonFX motor controller
     */
    public TalonFX getMotor(){
        return motor;
    }

    /**
     * creates a new TalonFX motor controller
     * @param id the id of the motor controller
     * @return the new motor controller
     */
    private TalonFX createMotor(int id){
        TalonFX talonFX = new TalonFX(id);

        talonFX.getConfigurator().apply(new TalonFXConfiguration());

        StatusCode error;

        // set the update frequency for position signal to the run frequency
        error = talonFX.getPosition().setUpdateFrequency(RUN_HZ);
        reportError("Error setting position update frequency", error);

        // set the update frequency for velocity signal to the run frequency
        error = talonFX.getVelocity().setUpdateFrequency(RUN_HZ);
        reportError("Error setting velocity update frequency", error);

        // set the update frequency for acceleration signal to the run frequency
        error = talonFX.getAcceleration().setUpdateFrequency(RUN_HZ);
        reportError("Error setting acceleration update frequency", error);

        error = talonFX.getDeviceTemp().setUpdateFrequency(DEVICE_TEMP_UPDATE_HZ);
        reportError("Error setting temperature update frequency", error);

        error = talonFX.getSupplyCurrent().setUpdateFrequency(SUPPLY_CURRENT_UPDATE_HZ);
        reportError("Error setting supply current update frequency", error);

        error = talonFX.getStatorCurrent().setUpdateFrequency(STATOR_CURRENT_UPDATE_HZ);
        reportError("Error setting stator current update frequency", error);

        error = talonFX.getSupplyVoltage().setUpdateFrequency(SUPPLY_VOLTAGE_UPDATE_HZ);
        reportError("Error setting supply voltage update frequency", error);

        error = talonFX.getMotorVoltage().setUpdateFrequency(MOTOR_VOLTAGE_UPDATE_HZ);
        reportError("Error setting motor voltage update frequency", error);

        error = talonFX.getDutyCycle().setUpdateFrequency(DUTY_CYCLE_UPDATE_HZ);
        reportError("Error setting duty cycle update frequency", error);

        error = talonFX.optimizeBusUtilization();
        reportError("Error optimizing bus utilization", error);

        return talonFX;
    }


    @Override
    protected void setPIDFMotor(PIDFGains gains) {
        Slot0Configs slot0 = new Slot0Configs();

        slot0.kP = gains.getP() / measurements.getGearRatio() / 12;

        slot0.kI = gains.getI() / measurements.getGearRatio() / 12;

        slot0.kD = gains.getD() / measurements.getGearRatio() / 12;

        StatusCode error = motor.getConfigurator().apply(slot0);
        reportError("Error setting PIDF gains", error);
    }

    @Override
    public void setCurrentLimits(int currentLimit, int currentThreshold) {

        CurrentLimitsConfigs limit = config.CurrentLimits;

        limit.StatorCurrentLimit = currentLimit;

        limit.SupplyCurrentLimit = currentThreshold;

        limit.SupplyCurrentLimitEnable = true;
        limit.StatorCurrentLimitEnable = true;

        StatusCode error = motor.getConfigurator().apply(limit);
        reportError("Error setting current limits", error);
    }

    @Override
    protected void setMaxMinOutputMotor(double max, double min) {

        MotorOutputConfigs motorOutputConfig = config.MotorOutput;

        motorOutputConfig.PeakForwardDutyCycle = max / 12;

        motorOutputConfig.PeakReverseDutyCycle = -Math.abs(min / 12);

        StatusCode error = motor.getConfigurator().apply(motorOutputConfig);
        reportError("Error setting max and min output", error);
    }

    @Override
    public void setMotorInverted(boolean inverted) {
        MotorOutputConfigs motorOutputConfig = config.MotorOutput;

        motorOutputConfig.Inverted =
                inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

        StatusCode error = motor.getConfigurator().apply(motorOutputConfig);
        reportError("Error setting motor inverted", error);
    }

    @Override
    public void resetMotorEncoder() {
        motor.setPosition(0);
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        motor.setPosition(position * measurements.getGearRatio());
    }

    @Override
    protected void setMotorDeadBandDutyCycleMotor(double deadBand) {
        MotorOutputConfigs motorOutputConfig = config.MotorOutput;

        motorOutputConfig.DutyCycleNeutralDeadband = Math.abs(deadBand);

        StatusCode error = motor.getConfigurator().apply(motorOutputConfig);
        reportError("Error setting deadband", error);
    }


    @Override
    public void setMotorIdleMode(boolean brake){
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        config.MotorOutput.NeutralMode = mode;
        motor.setNeutralMode(mode);
    }

    @Override
    protected void updateInputs(MotorInputs inputs) {
        BaseStatusSignal.refreshAll(signals);

        inputs.currentDraw = supplyCurrent.getValueAsDouble();
        inputs.currentOutput = statorCurrent.getValueAsDouble();
        inputs.voltageOutput = motorVoltage.getValueAsDouble();
        inputs.voltageInput = supplyVoltage.getValueAsDouble();
        inputs.powerDraw = inputs.currentDraw * inputs.voltageInput;
        inputs.powerOutput = inputs.currentOutput * inputs.voltageOutput;
        inputs.temperature = deviceTemp.getValueAsDouble();
        inputs.dutyCycle = dutyCycle.getValueAsDouble();
    }

    @Override
    protected void setMotorFollower(MarinersController master, boolean invert) {
        MarinersTalonFX base = (MarinersTalonFX)master;

        Follower follower = new Follower(base.getMotor().getDeviceID(), invert);

        StatusCode error = motor.setControl(follower);
        reportError("Error setting follower", error);
    }

    @Override
    protected void stopMotorOutput() {
        motor.stopMotor();
    }

    /**
     * reports an error if the error code is not OK
     * @param message the message to report
     * @param error the error code
     */
    private void reportError(String message, StatusCode error) {
        if(error != StatusCode.OK){
            super.reportError(message, error.name());
        }
    }

    @Override
    protected void setOutput(double motorOutput, ControlMode controlMode, double feedForward) {
         ControlRequest request = switch (controlMode){
                case Position, ProfiledPosition -> new PositionVoltage(motorOutput)
                        .withFeedForward(feedForward);

                case Velocity, ProfiledVelocity -> new VelocityVoltage(motorOutput)
                        .withFeedForward(feedForward);

                case Voltage -> new VoltageOut(motorOutput);

                case DutyCycle -> new DutyCycleOut(motorOutput);

                default -> config.MotorOutput.NeutralMode == NeutralModeValue.Brake ?
                        new StaticBrake() :
                        new CoastOut();
            };

            StatusCode error = motor.setControl(request);
            reportError("Error setting motor output", error);

    }
}
