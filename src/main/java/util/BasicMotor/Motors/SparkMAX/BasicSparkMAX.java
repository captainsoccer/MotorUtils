package util.BasicMotor.Motors.SparkMAX;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import util.BasicMotor.BasicMotor;
import util.BasicMotor.Controllers.Controller;
import util.BasicMotor.Gains.ControllerConstrains;
import util.BasicMotor.Gains.ControllerGains;
import util.BasicMotor.Gains.CurrentLimits;
import util.BasicMotor.Gains.PIDGains;
import util.BasicMotor.LogFrame;
import util.BasicMotor.Measurements.RevEncoders.MeasurementsREV;
import util.BasicMotor.Measurements.RevEncoders.MeasurementsREVRelative;
import util.BasicMotor.MotorManager;

public class BasicSparkMAX extends BasicMotor {
    private final SparkMax motor;
    private final SparkMaxConfig config;

    public BasicSparkMAX(
            ControllerGains gains,
            int id,
            boolean isBrushless,
            String name,
            double gearRatio,
            MotorManager.ControllerLocation location) {

        super(gains, name, location);

        this.motor =
                new SparkMax(
                        id,
                        isBrushless ? SparkLowLevel.MotorType.kBrushless : SparkLowLevel.MotorType.kBrushed);

        this.config = new SparkMaxConfig();
        config.voltageCompensation(
                MotorManager.motorIdleVoltage); // set the voltage compensation to the idle voltage
        // all configs should be stored in code and not on motor
        applyConfig();

        setMeasurements(new MeasurementsREVRelative(motor.getEncoder(), gearRatio));
    }

    @Override
    protected void updatePIDGainsToMotor(PIDGains pidGains) {
        config.closedLoop.pid(pidGains.getK_P(), pidGains.getK_I(), pidGains.getK_D());
        config.closedLoop.iZone(pidGains.getI_Zone());
        config.closedLoop.iMaxAccum(pidGains.getI_MaxAccum());

        applyConfig();
    }

    @Override
    protected void updateConstraints(ControllerConstrains constraints) {
        config.closedLoop.maxOutput(constraints.getMaxMotorOutput() / MotorManager.motorIdleVoltage);
        config.closedLoop.minOutput(constraints.getMinMotorOutput() / MotorManager.motorIdleVoltage);

        if (constraints.getConstraintType() == ControllerConstrains.ConstraintType.LIMITED) {
            config.softLimit.forwardSoftLimit(constraints.getMaxValue());
            config.softLimit.reverseSoftLimit(constraints.getMinValue());
            config.softLimit.forwardSoftLimitEnabled(true);
            config.softLimit.reverseSoftLimitEnabled(true);
        } else {
            config.softLimit.forwardSoftLimitEnabled(false);
            config.softLimit.reverseSoftLimitEnabled(false);
        }

        applyConfig();
    }

    @Override
    protected void setMotorOutput(double setpoint, double feedForward, Controller.RequestType mode) {
        // TODO: add error code handling for the motor controller
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
        motor.getClosedLoopController().setReference(setpoint, mode, ClosedLoopSlot.kSlot0, feedForward);
    }

    @Override
    protected void stopMotorOutput() {
        motor.stopMotor();
    }

    @Override
    protected LogFrame.SensorData getSensorData() {
        return null;
    }

    @Override
    protected LogFrame.PIDOutput getPIDLatestOutput() {
        return LogFrame.PIDOutput.EMPTY;
    }

    @Override
    public void setCurrentLimits(CurrentLimits currentLimits) {
        // TODO: create a rev current limits class
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
            // TODO: add alert to say the encoder is not a rev encoder anymore
            return;
        }

        encoder.setEncoderPosition(position);

        motor.getClosedLoopController().setIAccum(0);
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
        // TODO: add error code handling for setting the configuration
        motor.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters);
    }

    // TODO: add support for other rev encoders switching (like absolute encoders) directly
}
