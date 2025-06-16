package util.BasicMotor.Motors.SparkMAX;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import util.BasicMotor.BasicMotor;
import util.BasicMotor.Controllers.Controller;
import util.BasicMotor.Gains.*;
import util.BasicMotor.LogFrame;
import util.BasicMotor.Measurements.Measurements;
import util.BasicMotor.Measurements.RevEncoders.MeasurementsREV;
import util.BasicMotor.Measurements.RevEncoders.MeasurementsREVRelative;
import util.BasicMotor.MotorManager;

public class BasicSparkMAX extends BasicMotor {
    private final SparkMax motor;
    private final SparkMaxConfig config;

    private final Measurements defaultMeasurements;

    // the idle power draw of the Spark MAX in watts (according to ChatGPT)
    private static final double sparkMaxIdlePowerDraw = 0.72;

    public BasicSparkMAX(
            ControllerGains gains,
            int id,
            String name,
            double gearRatio,
            MotorManager.ControllerLocation location) {

        super(gains, name, location);

        this.motor = new SparkMax(id, SparkLowLevel.MotorType.kBrushless);

        this.config = new SparkMaxConfig();
        config.voltageCompensation(
                MotorManager.motorIdleVoltage); // set the voltage compensation to the idle voltage
        // all configs should be stored in code and not on motor
        applyConfig();

        defaultMeasurements = new MeasurementsREVRelative(motor.getEncoder(), gearRatio);
        setMeasurements(defaultMeasurements);
    }

    @Override
    protected void updatePIDGainsToMotor(PIDGains pidGains) {
        // sets the PID gains for the closed loop controller
        config.closedLoop.pid(pidGains.getK_P(), pidGains.getK_I(), pidGains.getK_D());
        config.closedLoop.iZone(pidGains.getI_Zone());
        config.closedLoop.iMaxAccum(pidGains.getI_MaxAccum());

        applyConfig();
    }

    @Override
    protected void updateConstraints(ControllerConstrains constraints) {
        // sets the max voltage to the max motor output
        config.closedLoop.maxOutput(constraints.getMaxMotorOutput() / MotorManager.motorIdleVoltage);
        config.closedLoop.minOutput(constraints.getMinMotorOutput() / MotorManager.motorIdleVoltage);

        if (constraints.getConstraintType() == ControllerConstrains.ConstraintType.LIMITED) {
            //sets the soft limits to the max and min values
            config.softLimit.forwardSoftLimit(constraints.getMaxValue());
            config.softLimit.reverseSoftLimit(constraints.getMinValue());
            // enables the soft limits
            config.softLimit.forwardSoftLimitEnabled(true);
            config.softLimit.reverseSoftLimitEnabled(true);
        } else {
            // disables the soft limits
            config.softLimit.forwardSoftLimitEnabled(false);
            config.softLimit.reverseSoftLimitEnabled(false);
        }

        applyConfig();
    }

    @Override
    protected Measurements getDefaultMeasurements() {
        return defaultMeasurements;
    }

    @Override
    protected void setMotorOutput(double setpoint, double feedForward, Controller.RequestType mode) {
        switch (mode) {
            // stop the motor output
            case STOP -> stopMotorOutput();
            // set the motor output to a voltage
            case VOLTAGE -> motor.setVoltage(setpoint);
            // set the motor output to a duty cycle
            case PRECENT_OUTPUT -> motor.set(setpoint);
            // set the motor output to a position
            case POSITION, PROFILED_POSITION -> setClosedLoopOutput(
                    setpoint, feedForward, SparkBase.ControlType.kPosition);
            // set the motor output to a velocity
            case VELOCITY, PROFILED_VELOCITY -> setClosedLoopOutput(
                    setpoint, feedForward, SparkBase.ControlType.kVelocity);
        }
    }

    /**
     * sets the closed loop output of the motor
     *
     * @param setpoint    the setpoint of the closed loop output
     * @param feedForward the feed forward of the closed loop output
     * @param mode        the control type of the closed loop output
     */
    private void setClosedLoopOutput(double setpoint, double feedForward, SparkBase.ControlType mode) {
        var okSignal = motor.getClosedLoopController().setReference(setpoint, mode, ClosedLoopSlot.kSlot0, feedForward);

        if (okSignal != REVLibError.kOk) {
            DriverStation.reportError(
                    "Failed to set closed loop output for Spark MAX motor: " + name + ". Error: " + okSignal.name(),
                    false);
        }
    }

    @Override
    protected void stopMotorOutput() {
        motor.stopMotor();
    }

    @Override
    protected LogFrame.SensorData getSensorData() {
        double voltage = motor.getBusVoltage();
        double current = motor.getOutputCurrent();
        double temperature = motor.getMotorTemperature();
        double dutyCycle = motor.getAppliedOutput();

        double outputVoltage = motor.getAppliedOutput() * MotorManager.motorIdleVoltage;
        double powerOutput = outputVoltage * current;

        double powerDraw = sparkMaxIdlePowerDraw + powerOutput; // idle power draw + output power
        double currentDraw = powerDraw / voltage; // current draw is power draw / voltage

        String faults = motor.getLastError().name(); // get the faults of the motor

        return new LogFrame.SensorData(
                temperature,
                currentDraw,
                current,
                outputVoltage,
                voltage,
                powerDraw,
                powerOutput,
                dutyCycle,
                faults
        );
    }

    @Override
    protected LogFrame.PIDOutput getPIDLatestOutput() {
        //spark max does not support getting the PID output directly
        return LogFrame.PIDOutput.EMPTY;
    }

    @Override
    public void setCurrentLimits(CurrentLimits currentLimits) {

        //TODO: decide what to do with this shit
        config.secondaryCurrentLimit(currentLimits.getSupplyLowerLimit()); // it is stator not supply current limit

        if (currentLimits instanceof CurrentLimitsREV limits) {
            int rpm = (limits.getFreeSpeedRPS() * 60) * (int) getMeasurements().getGearRatio(); // convert RPS to RPM
            config.smartCurrentLimit(limits.getStallCurrentLimit(), limits.getStatorCurrentLimit(), rpm);
        } else {
            // if the current limits are not REV specific, use the normal current limits
            config.smartCurrentLimit(currentLimits.getStatorCurrentLimit());
        }

        applyConfig();
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        SparkBaseConfig.IdleMode value =
                switch (mode) {
                    case BRAKE -> SparkBaseConfig.IdleMode.kBrake;
                    case COAST -> SparkBaseConfig.IdleMode.kCoast;
                };

        config.idleMode(value);
        applyConfig();
    }

    @Override
    protected void setMotorPosition(double position) {
        if (!(getMeasurements() instanceof MeasurementsREV encoder)) {
            DriverStation.reportWarning("motor: " + name + " does not use anymore an encoder of the motor controller, so the position cannot be set", false);
            return;
        }

        encoder.setEncoderPosition(position);

        motor.getClosedLoopController().setIAccum(0);
    }

    @Override
    protected void stopRecordingMeasurements() {
        //TODO: remember rev timings
        config.signals
    }

    @Override
    protected void startRecordingMeasurements(double HZ) {
        //TODO: remember rev timings
    }

    @Override
    protected void setMotorFollow(BasicMotor master, boolean inverted) {
        var motor = (BasicSparkMAX) master;

        config.follow(motor.motor, inverted);
        applyConfig();

        this.motor.resumeFollowerMode();
    }

    @Override
    protected void stopMotorFollow() {
        motor.pauseFollowerMode();
    }

    /**
     * applies the current configuration to the motor
     */
    private void applyConfig() {
        var okSignal = motor.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);

        if(okSignal != REVLibError.kOk) {
            DriverStation.reportError(
                    "Failed to apply configuration to Spark MAX motor: " + name + ". Error: " + okSignal.name(),
                    false);
        }
    }

    private void configPeriodicFrames(double mainLoopHZ){

    }

    // TODO: add support for other rev encoders switching (like absolute encoders) directly
}
