package util.MarinersController;

import com.revrobotics.*;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ExternalEncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import java.util.function.Function;

/**
 * a class for spark motor controllers (for both spark max and spark flex)
 * this class is used for controlling spark motor controllers
 * this class is a subclass of {@link MarinersController}
 * this does automatic error reporting to the driver station
 * and also built in logging in advantage kit
 * has support for basic output control and PIDF control and profiled control
 */
public class MarinersSparkBase extends MarinersController {

    private static final int APPLIED_OUTPUT_PERIOD_MS = 10;
    private static final int PRIMARY_ENCODER_POSITION_PERIOD_MS = 20;
    private static final int PRIMARY_ENCODER_VELOCITY_PERIOD_MS = 20;
    private static final int BUS_VOLTAGE_PERIOD_MS = 20;
    private static final int FAULTS_PERIOD_MS = 20;
    private static final int MOTOR_TEMPERATURE_PERIOD_MS = 20;
    private static final int ABSOLUTE_ENCODER_POSITION_PERIOD_MS = 20;
    private static final int ABSOLUTE_ENCODER_VELOCITY_PERIOD_MS = 20;

    /**
     * the type of motor controller
     */
    public enum MotorType {
        SPARK_MAX,
        SPARK_FLEX
    }

    /**
     * the motor controller object
     */
    private final SparkBase motor;

    /**
     * the configuration for the motor controller
     * used for setting the motor controller's settings
     */
    private final SparkBaseConfig config;

    /**
     * the type of motor controller
     * used for when setting an external encoder
     */
    private final MotorType type;

    public static final double ENERGY_LOSS_WATTS = 1;

    /**
     * sets the measurements for the motor controller using the built-in encoder
     * @param gearRatio the gear ratio of the motor controller (the value that the measurements will be divided by)
     */
    private void setMeasurements(double gearRatio) {
        RelativeEncoder encoder = motor.getEncoder();

        super.setMeasurements(
                new MarinersMeasurements(
                        encoder::getPosition,
                        () -> encoder.getVelocity() / 60,
                        gearRatio
                )
        );
    }

    /**
     * configures the spark base
     * @param sparkBase the spark base to configure
     */
    private void configSparkBase(SparkBase sparkBase) {
        int period = (int) (1000 / RUN_HZ);

        sparkBase.setControlFramePeriodMs(period);

        SignalsConfig signalsConfig = config.signals;

        signalsConfig.appliedOutputPeriodMs(APPLIED_OUTPUT_PERIOD_MS).
                primaryEncoderPositionPeriodMs(PRIMARY_ENCODER_POSITION_PERIOD_MS).
                primaryEncoderVelocityPeriodMs(PRIMARY_ENCODER_VELOCITY_PERIOD_MS).
                busVoltagePeriodMs(BUS_VOLTAGE_PERIOD_MS).
                faultsPeriodMs(FAULTS_PERIOD_MS).
                motorTemperaturePeriodMs(MOTOR_TEMPERATURE_PERIOD_MS);

        config.voltageCompensation(12);

        REVLibError error = sparkBase.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        reportError("Error configuring motor", error);
    }

        /**
     * creates a new spark motor controller
     * this will create a motor that can only be controlled by duty cycle or voltage
     * until {@link #setPIDF(PIDFGains)} or {@link #setPIDF(PIDFGains, Function)} is called
     * @param id the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type the type of motor controller
     * @param name the name of the motor controller
     */
    public MarinersSparkBase(String name, ControllerLocation location, int id, boolean isBrushless, MotorType type) {
        super(name, location);

        this.type = type;

        config = switch(type){
            case SPARK_MAX -> new SparkMaxConfig();
            case SPARK_FLEX -> new SparkFlexConfig();
        };

        SparkLowLevel.MotorType motorType = isBrushless ? SparkLowLevel.MotorType.kBrushless : SparkLowLevel.MotorType.kBrushed;

        motor = switch (type) {
            case SPARK_MAX ->
                    new SparkMax(id, motorType);
            case SPARK_FLEX ->
                    new SparkFlex(id, motorType);
        };

        configSparkBase(motor);

        setMeasurements(1);
    }

    /**
     * creates a new spark motor controller
     * @param id the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type the type of motor controller
     * @param gains the PIDF gains for the motor controller
     * @param gearRatio the gear ratio of the motor controller (the value that the measurements will be divided by)
     * @param name the name of the motor controller
     */
    public MarinersSparkBase(String name, ControllerLocation location, int id, boolean isBrushless, MotorType type, PIDFGains gains, double gearRatio) {
        this(name, location, id, isBrushless, type);

        super.setPIDF(gains);

        setMeasurements(gearRatio);
    }

    /**
     * creates a new spark motor controller
     * @param name the name of the motor controller
     * @param location the location of the motor controller
     * @param id the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type the type of motor controller
     * @param gains the PIDF gains for the motor controller
     */
    public MarinersSparkBase(String name, ControllerLocation location, int id, boolean isBrushless, MotorType type, PIDFGains gains) {
        this(name, location, id, isBrushless, type, gains, 1);
    }

    /**
     * creates a new spark motor controller
     * @param id the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type the type of motor controller
     * @param gains the PIDF gains for the motor controller
     * @param profile the trapezoid profile for the motor controller (used for motion profiling)
     * @param gearRatio the gear ratio of the motor controller (the value that the measurements will be divided by)
     * @param name the name of the motor controller
     */
    public MarinersSparkBase(String name, ControllerLocation location, int id, boolean isBrushless, MotorType type, PIDFGains gains, TrapezoidProfile profile, double gearRatio) {
        this(name, location, id, isBrushless, type);

        super.setPIDF(gains);

        super.setProfile(profile);

        setMeasurements(gearRatio);
    }

    /**
     * creates a new spark motor controller
     * @param id the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type the type of motor controller
     * @param gains the PIDF gains for the motor controller
     * @param profile the trapezoid profile for the motor controller (used for motion profiling)
     * @param name the name of the motor controller
     */
    public MarinersSparkBase(String name, ControllerLocation location, int id, boolean isBrushless, MotorType type, PIDFGains gains, TrapezoidProfile profile) {
        this(name, location, id, isBrushless, type, gains, profile, 1);
    }

    /**
     * creates a new spark motor controller
     * @param id the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type the type of motor controller
     * @param name the name of the motor controller
     * @param gains the PIDF gains for the motor controller
     * @param gearRatio the gear ratio of the motor controller (the value that the measurements will be divided by)
     * @param firstDerivativeLimit the first derivative limit for the motor controller (used for motion profiling)
     * @param secondDerivativeLimit the second derivative limit for the motor controller (used for motion profiling)
     */
    public MarinersSparkBase(String name, ControllerLocation location, int id, boolean isBrushless, MotorType type, PIDFGains gains, double gearRatio, double firstDerivativeLimit, double secondDerivativeLimit) {
        this(name, location, id, isBrushless, type);
        
        super.setProfile(firstDerivativeLimit, secondDerivativeLimit);

        super.setPIDF(gains);

        setMeasurements(gearRatio);
    }

    /**
     * if using a thorough bore encoder use this function instead of {@link #useExternalAbsoluteEncoder(boolean, double, double)}
     * this will have more precise measurements and better overall
     * use this only if you have a thorough bore encoder connected to a spark flex
     * @param inverted if the encoder is inverted (use to set the same direction as the motor)
     * @param zeroOffset the zero offset for the encoder (a value between 0 and 1) that will be removed from the absolute encoder's position
     * @param gearRatio the gear ratio of the motor controller (the value that the measurements will be divided by) (need to add this if using a gear ratio between the encoder and the motor)
     */
    public void useExternalAbsoluteEncoderUsingRelative(boolean inverted, double zeroOffset, double gearRatio){

        if(type != MotorType.SPARK_FLEX){
            throw new IllegalArgumentException("This function is only for spark flex controllers");
        }

        SparkFlex sparkFlex = (SparkFlex)motor;

        RelativeEncoder encoder = sparkFlex.getExternalEncoder();

        AbsoluteEncoder absoluteEncoder = getAbsoluteEncoder(inverted, zeroOffset, gearRatio);

        REVLibError error;

        SparkFlexConfig config = (SparkFlexConfig) this.config;

        ExternalEncoderConfig externalEncoderConfig = config.externalEncoder;

        externalEncoderConfig.inverted(inverted).
        positionConversionFactor(gearRatio).
        velocityConversionFactor(gearRatio);

        config.signals.externalOrAltEncoderPosition((int) (1000 / RUN_HZ)).
        externalOrAltEncoderVelocity((int) (1000 / RUN_HZ)).
        absoluteEncoderPositionPeriodMs(ABSOLUTE_ENCODER_POSITION_PERIOD_MS).
        absoluteEncoderVelocityPeriodMs(ABSOLUTE_ENCODER_VELOCITY_PERIOD_MS);

        config.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);

        error = motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        reportError("Error configuring motor with external encoder", error);

        encoder.setPosition(absoluteEncoder.getPosition());
        
        super.setMeasurements(
                new MarinersMeasurements(
                        encoder::getPosition,
                        () -> encoder.getVelocity() / 60,
                        gearRatio
                )
        );
    }

    /**
     * if using a thorough bore encoder use this function instead of {@link #useExternalAbsoluteEncoder(boolean, double)}
     * this will have more precise measurements and better overall
     * use this only if you have a thorough bore encoder connected to a spark flex
     * @param inverted if the encoder is inverted (use to set the same direction as the motor)
     * @param zeroOffset the zero offset for the encoder (a value between 0 and 1) that will be removed from the absolute encoder's position
     */
    public void useExternalAbsoluteEncoderUsingRelative(boolean inverted, double zeroOffset){
        useExternalAbsoluteEncoderUsingRelative(inverted, zeroOffset, 1);
    }

    /**
     * sets the measurements for the motor controller using an external encoder
     * also works for using the motor's built-in controller
     * if using a spark flex controller and a thorough bore encoder use {@link #useExternalAbsoluteEncoderUsingRelative(boolean, double, double)}
     * @param inverted if the encoder is inverted (use to set the same direction as the motor)
     * @param zeroOffset the zero offset for the encoder (a value between 0 and 1) that will be removed from the absolute encoder's position
     * @param gearRatio the gear ratio of the motor controller (the value that the measurements will be divided by) (need to add this if using a gear ratio between the encoder and the motor)
     */
    public void useExternalAbsoluteEncoder(boolean inverted, double zeroOffset, double gearRatio){
        AbsoluteEncoder encoder = getAbsoluteEncoder(inverted, zeroOffset, gearRatio);

        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        REVLibError error = motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        reportError("Error configuring motor with absolute encoder", error);        

        super.setMeasurements(
                new MarinersMeasurements(
                        encoder::getPosition,
                        gearRatio
                )
        );
    }

    /**
     * sets the measurements for the motor controller using an external encoder
     * also works for using the motor's built-in controller
     * if using a spark flex controller and a thorough bore encoder use {@link #useExternalAbsoluteEncoderUsingRelative(boolean, double)}
     * @param inverted if the encoder is inverted (use to set the same direction as the motor)
     * @param zeroOffset the zero offset for the encoder (a value between 0 and 1) that will be removed from the absolute encoder's position
     */
    public void useExternalAbsoluteEncoder(boolean inverted, double zeroOffset){
        useExternalAbsoluteEncoder(inverted, zeroOffset, 1);
    }

    /**
     * gets the absolute encoder for the motor controller
     * @param inverted if the encoder is inverted
     * @param zeroOffset the zero offset for the encoder
     * @param gearRatio the gear ratio of the motor controller
     * @return the absolute encoder for the motor controller
     */
    private AbsoluteEncoder getAbsoluteEncoder(boolean inverted, double zeroOffset, double gearRatio){
        AbsoluteEncoder encoder = motor.getAbsoluteEncoder();

        AbsoluteEncoderConfig encoderConfig = config.absoluteEncoder;

        encoderConfig.inverted(inverted).
        zeroOffset(zeroOffset).
        positionConversionFactor(gearRatio).
        velocityConversionFactor(gearRatio);

        config.signals.absoluteEncoderPositionPeriodMs((int) (1000 / RUN_HZ))
        .absoluteEncoderVelocityPeriodMs((int) (1000 / RUN_HZ));

        return encoder;
    }

    /**
     * sets the current limits for the motor
     * @param stallCurrentLimit the current limit for the motor when stalled
     * @param freeSpeedCurrentLimit the current limit for the motor when at free speed
     * @param thresholdRPM the rpm that below it the motor will be considered stalled (after that value the current limit will scale until the limit set until free speed)
     * @param thresholdCurrentLimit the current threshold for the motor (above this value the motor will be disabled for a short period of time)
     */
    public void setCurrentLimits(int stallCurrentLimit, int freeSpeedCurrentLimit, int thresholdRPM, int thresholdCurrentLimit) {

        config.smartCurrentLimit(stallCurrentLimit, freeSpeedCurrentLimit, thresholdRPM);

        config.secondaryCurrentLimit(thresholdCurrentLimit);

        REVLibError error = motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        reportError("Error configuring motor current limits", error);
        
    }

    /**
     * sets the current limits for the motor
     * @param currentLimit the current limit for the motor (the motor will try to stay below this value)
     * @param currentThreshold the current threshold for the motor (above this value the motor will be disabled for a short period of time)
     */
    @Override
    public void setCurrentLimits(int currentLimit, int currentThreshold){

        config.smartCurrentLimit(currentLimit);

        config.secondaryCurrentLimit(currentThreshold);

        REVLibError error = motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        reportError("Error configuring motor current limits", error);
        
    }


    private void reportError(String message, REVLibError error){
        if(error != REVLibError.kOk){
            super.reportError(message, error.name());
        }
    }

    /**
     * gets the motor object
     * @return the motor object
     */
    public SparkBase getMotor(){
        return motor;
    }

    @Override
    protected void updateInputs(MotorInputs inputs) {
        inputs.currentOutput = motor.getOutputCurrent();
        inputs.dutyCycle = motor.getAppliedOutput();
        inputs.voltageInput = motor.getBusVoltage();
        inputs.voltageOutput = inputs.dutyCycle * inputs.voltageInput;
        inputs.temperature = motor.getMotorTemperature();
        inputs.currentDraw = ((inputs.currentOutput * inputs.voltageInput + ENERGY_LOSS_WATTS) / inputs.voltageInput);
        inputs.powerOutput = inputs.voltageOutput * inputs.currentOutput;
        inputs.powerDraw = inputs.voltageInput * inputs.currentDraw;

        inputs.currentFaults = motor.getLastError().name();
    }

    @Override
    protected void setMotorFollower(MarinersController master, boolean invert) {
        if(master.getClass() != MarinersSparkBase.class){
            throw new IllegalArgumentException("cannot set a motor as follower to a different kind of motor");
        }

        MarinersSparkBase base = (MarinersSparkBase)master;

        config.follow(base.getMotor(), invert);

        REVLibError error = motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        reportError("Error configuring motor follower", error);
        
    }


    @Override
    public void setMotorIdleMode(boolean brake){
        IdleMode mode = brake ? IdleMode.kBrake : IdleMode.kCoast;

        config.idleMode(mode);

        REVLibError error = motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        reportError("Error configuring motor idle mode", error);
        
    }


    @Override
    protected void stopMotorOutput(){
        motor.stopMotor();
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        motor.getEncoder().setPosition(position * measurements.getGearRatio());
    }

    @Override
    protected void setPIDFMotor(PIDFGains gains) {
        
        ClosedLoopConfig controller = config.closedLoop;


        controller.p(gains.getP() / measurements.getGearRatio() / 12);
        controller.i(gains.getI() / measurements.getGearRatio() / 12);
        controller.d(gains.getD() / measurements.getGearRatio() / 12);
        controller.iZone(gains.getIZone() / measurements.getGearRatio() / 12);
        
        REVLibError error = motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        reportError("Error configuring motor PID", error);
        
    }

    @Override
    protected void setMaxMinOutputMotor(double max, double min) {

        ClosedLoopConfig controller = config.closedLoop;

        controller.outputRange(-Math.abs(min) / 12, max / 12);

        REVLibError error = motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        reportError("Error configuring motor output range", error);
        
    }

    @Override
    protected void setMotorDeadBandDutyCycleMotor(double deadBand) {
        reportWarning("Dead band for spark controllers is only available for controllers running on rio");
    }

    @Override
    public void setMotorInverted(boolean inverted) {
        config.inverted(inverted);

        REVLibError error = motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        reportError("Error configuring motor inverted", error);
    }

    @Override
    public void resetMotorEncoder() {
        motor.getEncoder().setPosition(0);
    }

    @Override
    protected void setOutput(double output, ControlMode controlMode, double feedForward) {
        SparkBase.ControlType controlType = switch (controlMode){
            case Voltage -> SparkBase.ControlType.kVoltage;
            case Position, ProfiledPosition -> SparkBase.ControlType.kPosition;
            case Velocity, ProfiledVelocity -> SparkBase.ControlType.kVelocity;
            default -> SparkBase.ControlType.kDutyCycle;
        };

        // REVLibError error = motor.getClosedLoopController().setReference(output, controlType, 0, feedForward);
        REVLibError error = motor.getClosedLoopController().setReference(output, controlType, ClosedLoopSlot.kSlot0, feedForward);
        
        reportError("Error setting motor output", error);
    }
}
