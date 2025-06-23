package util.BasicMotor.Motors.SparkBase;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj.DriverStation;
import util.BasicMotor.BasicMotor;
import util.BasicMotor.Controllers.Controller;
import util.BasicMotor.Gains.*;
import util.BasicMotor.LogFrame;
import util.BasicMotor.Measurements.Measurements;
import util.BasicMotor.Measurements.RevEncoders.MeasurementsREVAbsolute;
import util.BasicMotor.Measurements.RevEncoders.MeasurementsREVRelative;
import util.BasicMotor.MotorManager;

abstract class BasicSparkBase extends BasicMotor {
    protected final SparkBase motor;
    protected final SparkBaseConfig config;

    private final Measurements defaultMeasurements;

    private double kI = 0; // the integral gain, used for logging the I output

    // the idle power draw of the Spark MAX in watts (according to ChatGPT)
    private static final double sparkMaxIdlePowerDraw = 0.72;

    public BasicSparkBase(
            SparkBase motor,
            SparkBaseConfig config,
            ControllerGains gains,
            String name,
            double gearRatio,
            MotorManager.ControllerLocation location) {

        super(gains, name, location);

        this.motor = motor;

        this.config = config;
        config.voltageCompensation(
                MotorManager.motorIdleVoltage); // set the voltage compensation to the idle voltage
        // all configs should be stored in code and not on motor
        applyConfig();

        configurePeriodicFrames(location.HZ);

        defaultMeasurements = new MeasurementsREVRelative(motor.getEncoder(), gearRatio);
    }

    /**
     * gets the Spark MAX motor controller (useful if needed direct access to the motor controller)
     *
     * @return the Spark MAX motor controller
     */
    public SparkBase getMotor() {
        return motor;
    }

    @Override
    protected Measurements getDefaultMeasurements() {
        return defaultMeasurements;
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
    protected LogFrame.PIDOutput getPIDLatestOutput() {
        //spark max supports only integral accumulation, so we will return the I accumulator value
        double iAccum = motor.getClosedLoopController().getIAccum();

        return new LogFrame.PIDOutput(0, iAccum * kI, 0, 0);
    }

    @Override
    protected void updatePIDGainsToMotor(PIDGains pidGains) {
        // sets the PID gains for the closed loop controller
        config.closedLoop.pid(pidGains.getK_P(), pidGains.getK_I(), pidGains.getK_D());
        config.closedLoop.iZone(pidGains.getI_Zone());
        config.closedLoop.iMaxAccum(pidGains.getI_MaxAccum());

        if (pidGains.getTolerance() != 0) {
            DriverStation.reportWarning(
                    "Spark MAX does not use tolerance in the PID controller (works on rio PID Controller), so it is ignored: " + name,
                    false);
        }

        kI = pidGains.getK_I(); // store the integral gain for later use

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

        if (constraints.getVoltageDeadband() != 0) {
            DriverStation.reportWarning(
                    "Spark MAX does not use voltage deadband (works on RIO PID controller), so it is ignored: " + name,
                    false);
        }

        applyConfig();
    }

    @Override
    public void setCurrentLimits(CurrentLimits currentLimits) {

        if (currentLimits instanceof CurrentLimitsREV limits) {
            int rpm = (limits.getFreeSpeedRPS() * 60) * (int) getMeasurements().getGearRatio(); // convert RPS to RPM
            config.smartCurrentLimit(limits.getStallCurrentLimit(), limits.getStatorCurrentLimit(), rpm);

            config.secondaryCurrentLimit(limits.getSecondaryCurrentLimit());
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
    public void setMotorInverted(boolean inverted) {
        config.inverted(inverted);

        applyConfig();
    }

    @Override
    protected void setMotorPosition(double position) {
        if (!(getMeasurements() instanceof MeasurementsREVRelative encoder)) {
            DriverStation.reportWarning(
                    "motor: " + name + " does not use anymore an encoder that supports setting position,so the position cannot be set", false);
            return;
        }

        encoder.setEncoderPosition(position);

        motor.getClosedLoopController().setIAccum(0);
    }

    /**
     * configures the periodic frames for the Spark MAX motor controller
     * This is used to set the frequency of the periodic frames
     * some might not affect the Spark MAX, but are included for completeness
     * check the chart at <a href="https://docs.revrobotics.com/brushless/spark-max/control-interfaces#periodic-status-frames">...</a>
     *
     * @param mainLoopHZ the frequency of the main loop in Hz
     */
    private void configurePeriodicFrames(double mainLoopHZ) {
        var signals = config.signals;
        int sensorLoopPeriodMs = (int) ((1 / MotorManager.SENSOR_LOOP_HZ) * 1000); // convert to milliseconds
        int mainLoopPeriodMs = (int) ((1 / mainLoopHZ) * 1000); // convert to milliseconds

        signals.busVoltagePeriodMs(sensorLoopPeriodMs); //currently does nothing
        signals.motorTemperaturePeriodMs(sensorLoopPeriodMs); //currently does nothing
        signals.iAccumulationPeriodMs(sensorLoopPeriodMs); //currently unknown if it does anything
        signals.outputCurrentPeriodMs(sensorLoopPeriodMs); //currently does nothing

        signals.appliedOutputPeriodMs(mainLoopPeriodMs); // currently does something
        signals.primaryEncoderPositionAlwaysOn(true);
        signals.primaryEncoderVelocityAlwaysOn(true);
        signals.primaryEncoderPositionPeriodMs(mainLoopPeriodMs); // currently does something
        signals.primaryEncoderVelocityPeriodMs(mainLoopPeriodMs); // currently does something

        applyConfig();
    }

    @Override
    protected void stopRecordingMeasurements() {
        var signals = config.signals;

        int maxPeriodMs = 32767; // maximum period in milliseconds for the Spark MAX according to the documentation
        // check the docs at https://docs.revrobotics.com/brushless/spark-max/control-interfaces#periodic-status-frames

        signals.primaryEncoderPositionAlwaysOn(false);
        signals.primaryEncoderVelocityAlwaysOn(false);
        signals.primaryEncoderPositionPeriodMs(maxPeriodMs);
        signals.primaryEncoderVelocityPeriodMs(maxPeriodMs);

        applyConfig();
    }

    @Override
    protected void startRecordingMeasurements(double HZ) {
        var signals = config.signals;

        int periodMs = (int) ((1 / HZ) * 1000); // convert to milliseconds

        // set the encoder position and velocity to be always on
        signals.primaryEncoderPositionAlwaysOn(true);
        signals.primaryEncoderVelocityAlwaysOn(true);
        signals.primaryEncoderPositionPeriodMs(periodMs);
        signals.primaryEncoderVelocityPeriodMs(periodMs);

        applyConfig();
    }

    @Override
    protected void setMotorFollow(BasicMotor master, boolean inverted) {
        var motor = (BasicSparkBase) master;

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

        if (okSignal != REVLibError.kOk) {
            DriverStation.reportError(
                    "Failed to apply configuration to Spark MAX motor: " + name + ". Error: " + okSignal.name(),
                    false);
        }
    }

    /**
     * an enum that represents the range of the absolute encoder
     * it can be either 0 to 1 or -0.5 to 0.5
     */
    public enum AbsoluteEncoderRange {
        /**
         * 0 to 1 of range
         */
        ZERO_TO_ONE,
        /**
         * -0.5 to 0.5 of range
         */
        HALF_REVOLUTION;

        public boolean zeroCentered() {
            return this == HALF_REVOLUTION;
        }
    }

    /**
     * restores the Spark MAX to use the default encoder (the primary encoder)
     */
    public void useDefaultEncoder() {
        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);

        // sets the primary encoder as the feedback sensor for the closed loop controller (also apply the config)
        setDefaultMeasurements();
    }

    /**
     * configures the Spark MAX to use an absolute encoder for the pid loop (if it runs on the Spark MAX)
     * it uses the absolute encoder connected to the Spark MAX data port
     *
     * @param inverted               if the absolute encoder is inverted (the position is reversed)
     * @param zeroOffset             the position the encoder reports that should be considered zero
     * @param sensorToMotorRatio     this value divides the ratio between the sensor and the motor
     *                               (the value reported by the sensor to get the motor position)
     * @param mechanismToSensorRatio this value divides the ratio between the mechanism and the sensor
     *                               (the value reported by the sensor to get the mechanism position)
     * @param absoluteEncoderRange   if the encoder should report a range of 0 to 1 or -0.5 to 0.5
     */
    public void useAbsoluteEncoder(boolean inverted, double zeroOffset, double sensorToMotorRatio,
                                   double mechanismToSensorRatio, AbsoluteEncoderRange absoluteEncoderRange) {
        //sets whether the absolute encoder is inverted or not
        config.absoluteEncoder.inverted(inverted);
        //sets the conversion factor for the absolute encoder position and velocity
        config.absoluteEncoder.positionConversionFactor(1 / sensorToMotorRatio);
        config.absoluteEncoder.velocityConversionFactor(1 / sensorToMotorRatio);
        //sets the zero centered range of the absolute encoder
        config.absoluteEncoder.zeroCentered(absoluteEncoderRange.zeroCentered());
        //sets the zero offset of the absolute encoder
        config.absoluteEncoder.zeroOffset(zeroOffset);
        //sets the feedback sensor for the closed loop controller
        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);

        int periodMs = (int) ((1 / controllerLocation.HZ) * 1000); // convert to milliseconds
        // sets the period for the absolute encoder position and velocity
        // (the default encoder period is automatically disabled)
        config.signals.absoluteEncoderPositionPeriodMs(periodMs);
        config.signals.absoluteEncoderVelocityPeriodMs(periodMs);

        //apply the configuration to the motor
        applyConfig();

        // set the measurements to the absolute encoder measurements
        setMeasurements(new MeasurementsREVAbsolute(motor.getAbsoluteEncoder(), mechanismToSensorRatio));
    }

    /**
     * configures the Spark MAX to use an absolute encoder for the pid loop (if it runs on the Spark MAX)
     * it uses the absolute encoder connected to the Spark MAX data port
     *
     * @param inverted             if the absolute encoder is inverted (the position is reversed)
     * @param zeroOffset           the position the encoder reports that should be considered zero
     * @param sensorToMotorRatio   this value divides the ratio between the sensor and the motor
     *                             (the value reported by the sensor to get the motor position)
     * @param absoluteEncoderRange if the encoder should report a range of 0 to 1 or -0.5 to 0.5
     */
    public void useAbsoluteEncoder(boolean inverted, double zeroOffset, double sensorToMotorRatio, AbsoluteEncoderRange absoluteEncoderRange) {
        useAbsoluteEncoder(inverted, zeroOffset, sensorToMotorRatio, 1, absoluteEncoderRange);
    }

    /**
     * configures the Spark MAX to use an absolute encoder for the pid loop (if it runs on the Spark MAX)
     * it uses the absolute encoder connected to the Spark MAX data port
     *
     * @param inverted           if the absolute encoder is inverted (the position is reversed)
     * @param zeroOffset         the position the encoder reports that should be considered zero
     * @param sensorToMotorRatio this value divides the ratio between the sensor and the motor
     *                           (the value reported by the sensor to get the motor position)
     */
    public void useAbsoluteEncoder(boolean inverted, double zeroOffset, double sensorToMotorRatio) {
        useAbsoluteEncoder(inverted, zeroOffset, sensorToMotorRatio, AbsoluteEncoderRange.ZERO_TO_ONE);
    }

    /**
     * configures the Spark MAX to use an external encoder for the pid loop (if it runs on the Spark MAX)
     * it uses the external encoder connected to the Spark MAX data port
     *
     * @param inverted               if the external encoder is inverted (the position is reversed)
     * @param sensorToMotorRatio     this value divides the ratio between the sensor and the motor
     *                               (the value reported by the sensor to get the motor position)
     * @param mechanismToSensorRatio this value divides the ratio between the mechanism and the sensor
     *                               (the value reported by the sensor to get the mechanism position)
     */
    public void useExternalEncoder(boolean inverted, double sensorToMotorRatio, double mechanismToSensorRatio) {
        //sets the feedback sensor for the closed loop controller
        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAlternateOrExternalEncoder);

        configExternalEncoder(inverted, sensorToMotorRatio, mechanismToSensorRatio);

        int periodMs = (int) ((1 / controllerLocation.HZ) * 1000); // convert to milliseconds
        // sets the period for the absolute encoder position and velocity
        // (the default encoder period is automatically disabled)
        config.signals.externalOrAltEncoderPosition(periodMs);
        config.signals.externalOrAltEncoderVelocity(periodMs);

        //apply the configuration to the motor
        applyConfig();

        // set the measurements to the absolute encoder measurements
        setMeasurements(new MeasurementsREVRelative(getExternalEncoder(), mechanismToSensorRatio));
    }

    /**
     * configures the Spark MAX to use an external encoder for the pid loop (if it runs on the Spark MAX)
     * it uses the external encoder connected to the Spark MAX data port
     *
     * @param inverted           if the external encoder is inverted (the position is reversed)
     * @param sensorToMotorRatio this value divides the ratio between the sensor and the motor
     *                           (the value reported by the sensor to get the motor position)
     */
    public void useExternalEncoder(boolean inverted, double sensorToMotorRatio) {
        useExternalEncoder(inverted, sensorToMotorRatio, 1);
    }

    abstract protected void configExternalEncoder(boolean inverted, double sensorToMotorRatio, double mechanismToSensorRatio);

    abstract protected RelativeEncoder getExternalEncoder();
}
