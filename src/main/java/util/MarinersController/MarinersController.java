package util.MarinersController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import java.util.Objects;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Function;

/**
 * A class to control a motor controller
 * this holds the pid controller, feed forward, and profile of the controller
 * this class is used to interface with the motor controller
 * you can run a pid controller on the roborio or the motor controller
 * you can also add a motion profile to the controller
 * also you can add a feed forward to the controller that is based on the measurement (or static)
 * this class is abstract and is meant to be extended for specific motor controllers
 *
 * @see MarinersMeasurements
 * @see PIDController
 * @see TrapezoidProfile
 */
public abstract class MarinersController {

    /**
     * available control modes for the controller
     */
    public enum ControlMode {
        /**
         * the controller is stopped (that means no output is sent to the motor) (depends on the idle mode of the motor)
         */
        Stopped,

        /**
         * the motor will follow every output another motor does
         */
        Follower,

        /**
         * the controller is running in duty cycle control mode
         * the output is between -1 and 1 (where -1 is full reverse and 1 is full forward)
         */
        DutyCycle,

        /**
         * the controller is running in voltage control mode
         * the output is in volts
         * the max and min is dependent on the battery voltage
         */
        Voltage,

        /**
         * the controller is running in position control mode
         * the output is in position units (default is rotation)
         */
        Position,

        /**
         * the controller is running in velocity control mode
         * the output is in velocity units (default is rotation per second)
         */
        Velocity,

        /**
         * the controller is running in profiled position control mode
         * that means there is a motion profile set for the controller
         * the output is in position units (default is rotation)
         */
        ProfiledPosition,

        /**
         * the controller is running in profiled velocity control mode
         * that means there is a motion profile set for the controller
         * the output is in velocity units (default is rotation per second)
         */
        ProfiledVelocity;

        public boolean needPID() {
            return this != DutyCycle && this != Voltage && this != Stopped && this != Follower;
        }

        public boolean needMotionProfile() {
            return this == ProfiledPosition || this == ProfiledVelocity;
        }

        public boolean isPositionControl() {
            return this == Position || this == ProfiledPosition;
        }

        public boolean isVelocityControl() {
            return this == Velocity || this == ProfiledVelocity;
        }
    }

    /**
     * the location of the controller where it is running
     * this effects the speed of the thread of the motor
     * and also can effect the amount of can bus traffic
     */
    public enum ControllerLocation {
        /**
         * the controller is running on the roborio
         * that means the pid is calculated on the roborio and the output is sent to the motor controller
         * (more can bus traffic)
         */
        RIO,

        /**
         * the controller is running on the motor controller
         * that means the pid is calculated on the motor controller, and the output is sent to the motor,
         * but also there is a motion profile on the roborio that can be used
         * (less can bus traffic)
         */
        MOTOR
    }

    @AutoLog
    public static class BaseControllerInputs {
        public double position = 0;
        public double velocity = 0;
        public double acceleration = 0;
        public String controlMode = "Stopped";
        public double setpoint = 0;
        public double goal = 0;
        public double temperature = 0;
        public double currentDraw = 0;
        public double currentOutput = 0;
        public double voltageOutput = 0;
        public double voltageInput = 0;
        public double powerDraw = 0;
        public double powerOutput = 0;
        public double dutyCycle = 0;
        public String currentFaults = "";
    }

    protected static class MotorInputs {
        protected double temperature = 0;
        protected double currentDraw = 0;
        protected double currentOutput = 0;
        protected double voltageOutput = 0;
        protected double voltageInput = 0;
        protected double powerDraw = 0;
        protected double powerOutput = 0;
        protected double dutyCycle = 0;
        protected String currentFaults = "";

        protected void fillInputs(BaseControllerInputs inputs) {
            inputs.temperature = temperature;
            inputs.currentDraw = currentDraw;
            inputs.currentOutput = currentOutput;
            inputs.voltageOutput = voltageOutput;
            inputs.voltageInput = voltageInput;
            inputs.powerDraw = powerDraw;
            inputs.powerOutput = powerOutput;
            inputs.dutyCycle = dutyCycle;
            inputs.currentFaults = currentFaults;
        }
    }

    /**
     * The measurements of the system (position, velocity, acceleration)
     */
    protected MarinersMeasurements measurements = new MarinersMeasurements(() -> 0.0, 1);

    /**
     * The frequency that the controller runs at
     */
    public final double RUN_HZ;

    /**
     * The PID controller used for the controller
     */
    private PIDController pidController;

    /**
     * The feed forward function used for the controller
     */
    protected Function<Double, Double> feedForward;

    /**
     * The profile used for the controller
     */
    private TrapezoidProfile profile;

    /**
     * The maximum and minimum output of the controller in volts
     */
    private double[] maxMinOutput = {12, -12};

    /**
     * The lock for the setpoint of the controller
     * this is used to prevent the setpoint from being changed while the controller is running
     * (locks when setpoint, goal, or control mode is being changed or used)
     */
    private final ReentrantLock setpointLock = new ReentrantLock();

    /**
     * The lock for the measurements of the controller
     * this is used to prevent the measurements from being changed while the controller is running
     * (locks when measurements are being updated or being used)
     */
    private final ReentrantLock measurementLock = new ReentrantLock();

    /**
     * The control mode of the controller
     */
    protected ControlMode controlMode = ControlMode.Stopped;

    /**
     * The setpoint of the controller
     */
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    /**
     * The goal of the controller
     */
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();

    /**
     * The inputs of the controller
     */
    private final BaseControllerInputsAutoLogged inputs = new BaseControllerInputsAutoLogged();

    /**
     * The inputs of the motor controller (filled by the motor extending this class) (fills the inputs with the controller)
     */
    private final MotorInputs motorInputs = new MotorInputs();

    /**
     * the position wrapping min max
     * if the controller is in position control mode and the position is outside of this range,
     * the controller will wrap the position to be within this range
     * the array should be the minimum value first then the maximum
     * units are the units of the measurements
     */
    private Double[] wrappingMinMax;

    /**
     * the soft limits of the controller,
     * the first value is the minimum value and the second value is the maximum value
     * units are the units of the measurements
     */
    private Double[] softLimits;

    /**
     * The deadband of the motor in voltage
     * if the motor output is less than this value, the motor will not put out any power
     */
    private double motorVoltageDeadBand = 0.0;

    /**
     * motor name
     */
    public final String name;

    /**
     * the location of the controller where it is running
     */
    protected final ControllerLocation location;

    /**
     * if the pid tuning is running
     */
    private boolean isRunningPIDTuning = false;

    /**
     * the current pid gains (used for pid tuning)
     */
    private PIDFGains currentGains;

    /**
     * the arbitrary feed forward used for using a feed forward that is set by the reference
     */
    private double arbitraryFeedForward = 0;

    /**
     * the voltage needed to overcome static friction
     */
    private double motorKs = 0;

    /**
     * runs the controller
     */
    public void runController() {
        double dt = 1 / RUN_HZ;

        try {
            measurementLock.lock();
            measurements.update(dt);
        } finally {
            measurementLock.unlock();
        }

        if (isRunningPIDTuning && location == ControllerLocation.MOTOR) {
            PIDFGains newGains = PIDFGains.fromController(pidController);
            if (!newGains.equals(currentGains)) {
                setPIDFMotor(newGains);
                currentGains = newGains;
            }
        }

        double output;
        ControlMode controlMode;

        try {
            setpointLock.lock();

            controlMode = this.controlMode;

            if (controlMode == ControlMode.Follower || controlMode == ControlMode.Stopped) {
                return;
            }

            if(!controlMode.isPositionControl() && softLimits != null){
                if(measurements.getPosition() >= softLimits[1] && setpoint.position > 0){
                    setpoint.position = 0;
                }
                else if(measurements.getPosition() <= softLimits[0] && setpoint.position < 0){
                    setpoint.position = 0;

                }
            }
            else{
                setpoint.position = checkSoftLimitForPosition(setpoint.position);

                if(controlMode == ControlMode.ProfiledPosition){
                    goal.position = checkSoftLimitForPosition(goal.position);
                }
            }

            if (!controlMode.needPID()) {
                setOutput(setpoint.position, controlMode, 0);
                return;
            }

            double measurement = switch (controlMode) {
                case Position, ProfiledPosition -> measurements.getPosition();
                case Velocity, ProfiledVelocity -> measurements.getVelocity();
                default -> 0;
            };

            //check if needed to wrap the position
            if (wrappingMinMax != null && controlMode.isPositionControl()) {
                setpoint.position = calculatePositionWrapping(measurement, setpoint.position);

                if (controlMode == ControlMode.ProfiledPosition)
                    goal.position = calculatePositionWrapping(measurement, goal.position);
            }

            //calculate the motion profile
            if (controlMode.needMotionProfile()) setpoint = profile.calculate(1 / RUN_HZ, setpoint, goal);

            //calculate the feed forward
            double feedForward = arbitraryFeedForward + this.feedForward.apply(measurement) * setpoint.position;

            //if using a built-in controller, sends the output to the motor controller
            if (location == ControllerLocation.MOTOR) {
                setOutput(setpoint.position * measurements.getGearRatio(), controlMode, feedForward);
                return;
            }

            // calculate the output of the pid controller
            output = pidController.calculate(measurement, setpoint.position);

            // if the pid controller is at the setpoint, set the output to the feed forward
            if (pidController.atSetpoint()) {
                output = feedForward;
            } else {
                double staticForce = switch (controlMode) {
                    case Position, ProfiledPosition -> Math.signum(setpoint.position - measurement) * motorKs;
                    case Velocity, ProfiledVelocity -> Math.signum(setpoint.position) * motorKs;
                    default -> 0;
                };

                output += feedForward + staticForce;
            }

            if (Math.abs(output) <= motorVoltageDeadBand) {
                output = 0;
            }
        } finally {
            setpointLock.unlock();
        }

        //sends the motor output to the motor
        setOutput(MathUtil.clamp(output, maxMinOutput[1], maxMinOutput[0]), ControlMode.Voltage, 0);
    }

    private double calculatePositionWrapping(double measurement, double setpoint) {
        double errorBound = (wrappingMinMax[1] - wrappingMinMax[0]) / 2.0;

        return MathUtil.inputModulus(setpoint - measurement, -errorBound, errorBound) + measurement;
    }

    private double checkSoftLimitForPosition(double setPoint){
        if(softLimits == null) return setPoint;
        return MathUtil.clamp(setPoint, softLimits[0], softLimits[1]);
    }

    protected abstract void setOutput(double output, ControlMode controlMode, double feedForward);

    /**
     * updates the inputs of the controller
     * this will be used for logging
     */
    public void update() {
        //locks and updates the setPoints of the controller (setPoint, goal and controlMode)
        try {
            setpointLock.lock();

            if (RobotState.isDisabled()) {
                controlMode = ControlMode.Stopped;
                stopMotorOutput();
            }

            inputs.controlMode = controlMode.name();
            inputs.setpoint = setpoint.position;
            TrapezoidProfile.State goal = this.goal;

            inputs.goal = goal == null ? 0 : goal.position;
        } finally {
            setpointLock.unlock();
        }

        //locks and updates the measurements of the controller (position, velocity, acceleration)
        try {
            measurementLock.lock();

            inputs.position = measurements.getPosition();
            inputs.velocity = measurements.getVelocity();
            inputs.acceleration = measurements.getAcceleration();
        } finally {
            measurementLock.unlock();
        }

        updateInputs(motorInputs);

        motorInputs.fillInputs(inputs);

        Logger.processInputs("Motors/" + name, inputs);
    }

    /**
     * updates the inputs of the controller (only for motor specif logging)
     *
     * @param inputs the inputs of the controller
     */
    protected abstract void updateInputs(MotorInputs inputs);

    /**
     * gets the current control mode of the controller
     *
     * @return the current control mode of the controller
     */
    public ControlMode getControlMode() {
        try {
            setpointLock.lock();
            return controlMode;
        } finally {
            setpointLock.unlock();
        }
    }

    /**
     * gets the current position of the motor after gear ratio (default is rotations)
     *
     * @return the current position of the motor after gear ratio
     */
    public double getPosition() {
        try {
            measurementLock.lock();
            return measurements.getPosition();
        } finally {
            measurementLock.unlock();
        }
    }

    /**
     * gets the current velocity of the motor after gear ratio (default is rotations per second)
     *
     * @return the current velocity of the motor after gear ratio
     */
    public double getVelocity() {
        try {
            measurementLock.lock();
            return measurements.getVelocity();
        } finally {
            measurementLock.unlock();
        }
    }

    /**
     * gets the current acceleration of the motor after gear ratio (default is rotations per second squared)
     *
     * @return the current acceleration of the motor after gear ratio
     */
    public double getAcceleration() {
        try {
            measurementLock.lock();
            return measurements.getAcceleration();
        } finally {
            measurementLock.unlock();
        }
    }

    /**
     * gets the current setpoint of the controller (if using profiled control mode, this will be the next setpoint (not the goal always)
     *
     * @return the current setpoint of the controller
     */
    public double getSetpoint() {
        try {
            setpointLock.lock();
            return setpoint.position;
        } finally {
            setpointLock.unlock();
        }
    }

    /**
     * if using profiled control mode, this will return the goal of the controller (the end state)
     * if not using profiled control mode, this will return the setpoint of the controller
     *
     * @return the current goal of the controller
     */
    public TrapezoidProfile.State getGoal() {

        try {
            setpointLock.lock();
            return controlMode.needMotionProfile() ? goal : setpoint;
        } finally {
            setpointLock.unlock();
        }
    }

    /**
     * gets the current PID gains of the controller (will only include P, I, D, maybe F)
     *
     * @return the current PID gains of the controller
     */
    public PIDFGains getPIDF() {
        return currentGains;
    }

    /**
     * @return true if the pid controller is at the setpoint (within the tolerance)
     */
    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    /**
     * sets the motor as a follower to another motor,
     * this will make the motor follow the output of the master motor
     * (this will make the motor ignore any reference set to it)
     * can only be undone by restarting the code
     *
     * @param master the primary motor controller (the one that this motor will follow)
     * @param invert true, if the motor should follow the master in reverse
     *               (if the master spins clockwise, this motor will spin counter-clockwise)
     */
    public void setMotorAsFollower(MarinersController master, boolean invert) {
        if (master.getClass() != this.getClass())
            throw new IllegalArgumentException("cannot set a motor as follower to a different kind of motor");

        try {
            setpointLock.lock();
            controlMode = ControlMode.Follower;
        } finally {
            setpointLock.unlock();
        }

        setMotorFollower(master, invert);
    }

    protected abstract void setMotorFollower(MarinersController master, boolean invert);

    /**
     * sets the reference of the controller
     *
     * @param setpoint             the setpoint of the controller (needs to be appropriately set for the control mode)
     * @param controlMode          the control mode of the controller
     * @param arbitraryFeedForward the feed forward of the controller that is set by the reference
     */
    public void setReference(double setpoint, ControlMode controlMode, double arbitraryFeedForward) {

        Objects.requireNonNull(controlMode, "Control mode cannot be null");

        if (controlMode.needPID()) Objects.requireNonNull(pidController, "PID control on mode requires pid gains");

        if (controlMode.needMotionProfile())
            Objects.requireNonNull(profile, "Profiled control mode requires a profile");

        try {
            setpointLock.lock();

            if (RobotState.isDisabled()) {
                if (this.controlMode != ControlMode.Stopped) {
                    stopMotorOutput();
                    this.controlMode = ControlMode.Stopped;
                }
                return;
            }

            if (controlMode == ControlMode.Follower) {
                DriverStation.reportError("cannot set reference to a follower motor", true);
                return;
            }

            this.controlMode = controlMode;
            this.arbitraryFeedForward = arbitraryFeedForward;

            switch (controlMode) {
                case Voltage -> this.setpoint.position = MathUtil.clamp(setpoint, maxMinOutput[1], maxMinOutput[0]);
                case DutyCycle ->
                        this.setpoint.position = MathUtil.clamp(setpoint, maxMinOutput[1] / 12, maxMinOutput[0] / 12);
                case Position, Velocity -> this.setpoint.position = setpoint;
                case ProfiledPosition, ProfiledVelocity -> this.goal.position = setpoint;
                default -> stopMotorOutput();
            }

        } finally {
            setpointLock.unlock();
        }
    }

    /**
     * sets the reference of the controller
     *
     * @param setpoint    the setpoint of the controller (needs to be appropriately set for the control mode)
     * @param controlMode the control mode of the controller
     */
    public void setReference(double setpoint, ControlMode controlMode) {

        Objects.requireNonNull(controlMode, "Control mode cannot be null");

        if (controlMode.needPID()) Objects.requireNonNull(pidController, "PID control on mode requires pid gains");

        if (controlMode.needMotionProfile())
            Objects.requireNonNull(profile, "Profiled control mode requires a profile");

        try {
            setpointLock.lock();

            if (RobotState.isDisabled()) {
                if (this.controlMode != ControlMode.Stopped) {
                    stopMotorOutput();
                    this.controlMode = ControlMode.Stopped;

                }
                return;
            }

            if (controlMode == ControlMode.Follower) {
                DriverStation.reportError("cannot set reference to a follower motor", true);
                return;
            }

            this.controlMode = controlMode;
            this.arbitraryFeedForward = 0;

            switch (controlMode) {
                case Voltage -> this.setpoint.position = MathUtil.clamp(setpoint, maxMinOutput[1], maxMinOutput[0]);
                case DutyCycle ->
                        this.setpoint.position = MathUtil.clamp(setpoint, maxMinOutput[1] / 12, maxMinOutput[0] / 12);
                case Position, Velocity -> this.setpoint.position = setpoint;
                case ProfiledPosition, ProfiledVelocity -> this.goal.position = setpoint;
                default -> stopMotorOutput();
            }

        } finally {
            setpointLock.unlock();
        }
    }

    /**
     * sets the reference of the controller
     *
     * @param goal        the goal of the controller (needs to be appropriately set for the control mode)
     *                    when using profiled position control mode, this would be the position and velocity of the controlled value
     *                    when using profiled velocity control mode, this would be the velocity and acceleration of the controlled value
     *                    this will be used when the end state first derivative is not zero
     * @param controlMode the control mode of the controller (needs to be ProfiledPosition or ProfiledVelocity)
     */
    public void setReference(TrapezoidProfile.State goal, ControlMode controlMode) {

        Objects.requireNonNull(goal, "Goal cannot be null");

        Objects.requireNonNull(controlMode, "Control mode cannot be null");

        if (!controlMode.needMotionProfile()) {
            throw new IllegalArgumentException("Goal is only valid for Profiled control modes");
        }

        Objects.requireNonNull(pidController, "PID control on mode requires pid gains");

        Objects.requireNonNull(profile, "Profiled control mode requires a profile");

        try {
            setpointLock.lock();

            if (RobotState.isDisabled()) {
                if (this.controlMode != ControlMode.Stopped) {
                    stopMotorOutput();
                    this.controlMode = ControlMode.Stopped;

                }
                return;
            }

            if (controlMode == ControlMode.Follower) {
                DriverStation.reportError("cannot set reference to a follower motor", true);
                return;
            }

            this.controlMode = controlMode;
            this.goal = goal;
        } finally {
            setpointLock.unlock();
        }
    }

    /**
     * stops the motor (stops the pid controller and motor output) until a new reference is set
     */
    public void stopMotor() {
        try {
            setpointLock.lock();
            controlMode = ControlMode.Stopped;
        } finally {
            setpointLock.unlock();
        }
        stopMotorOutput();
    }

    /**
     * sets the motor idle mode
     *
     * @param brake true if the motor should brake when idle, false if the motor should coast when idle
     */
    public abstract void setMotorIdleMode(boolean brake);

    /**
     * actually sets the motor output to 0
     */
    protected abstract void stopMotorOutput();

    /**
     * sets the voltage output of the controller
     * equivalent to {@link #setReference(double, ControlMode)} with ControlMode.Voltage
     *
     * @param voltage the voltage output of the controller
     */
    public void setVoltage(double voltage) {
        setReference(voltage, ControlMode.Voltage);
    }

    /**
     * sets the voltage output of the controller
     * equivalent to {@link #setReference(double, ControlMode)} with ControlMode.Voltage
     *
     * @param voltage the voltage output of the controller
     */
    public void setVoltage(edu.wpi.first.units.measure.Voltage voltage) {
        setVoltage(voltage.baseUnitMagnitude());
    }

    /**
     * sets the duty cycle output of the controller
     * equivalent to {@link #setReference(double, ControlMode)} with ControlMode.DutyCycle
     *
     * @param dutyCycle the duty cycle output of the controller (from -1 to 1)
     */
    public void setDutyCycle(double dutyCycle) {
        setReference(dutyCycle, ControlMode.DutyCycle);
    }

    /**
     * sets the pid gains of the controller
     *
     * @param gains the pid gains of the controller
     * @throws IllegalArgumentException if the gains are null
     */
    public void setPIDF(PIDFGains gains) {
        Objects.requireNonNull(gains, "Gains cannot be null");

        this.currentGains = gains;

        pidController = gains.createPIDController();
        setPIDFMotor(gains);

        feedForward = (measurement) -> gains.getF();
    }

    protected abstract void setPIDFMotor(PIDFGains gains);

    /**
     * sets the current limit and current threshold for the motor
     * @param currentLimit the output current soft limit (will try to avoid going above)
     * @param currentThreshold the current draw the motor will shut off after drawing (can be lower than limit)
     */
    public abstract void setCurrentLimits(int currentLimit, int currentThreshold);

    /**
     * Enables position wrapping for the controller.
     * This method is used to wrap the position to be within a specified range.
     * For example, if the minimum is 0 and the maximum is 1,
     * and the position is 1.5, the position will be wrapped to 0.5.
     * This is useful for systems that have a continuous range of motion.
     * (like a swerve steer)
     * DO NOT USE THIS FOR SYSTEMS THAT HAVE A LIMITED RANGE OF MOTION
     * (like an elevator) OR CAN BE DAMAGED BY WRAPPING
     * (like a turret)
     *
     * @param minMax An array containing the minimum and maximum values for position wrapping.
     *               The array should have exactly two elements: the minimum value first, then the maximum value.
     *               If null, position wrapping will be disabled.
     * @throws IllegalArgumentException if the array length is not 2 or null.
     */
    public void enablePositionWrapping(Double[] minMax) {
        if (minMax != null && minMax.length != 2) {
            throw new IllegalArgumentException("minMax must have exactly 2 elements");
        }

        wrappingMinMax = minMax;
    }

    /**
     * reports an error to the driver station and the smart dashboard
     *
     * @param message the message to report (for example, "error setting current limits")
     * @param error   the error code (for example, "current limit too low")
     */
    protected void reportError(String message, String error) {

        String send = message + " for " + name + " motor with errorCode: " + error;

        DriverStation.reportError(send, false);

        try (Alert alert = new Alert(send, Alert.AlertType.kError)) {
            alert.set(true);
        }
    }

    /**
     * reports a warning to the driver station and the smart dashboard
     *
     * @param message the message to report (for example, "warning setting current limits")
     */
    protected void reportWarning(@SuppressWarnings("SameParameterValue") String message) {

        String send = message + " for " + name;

        DriverStation.reportWarning(send, false);

        try (Alert alert = new Alert(send, Alert.AlertType.kWarning)) {
            alert.set(true);
        }
    }

    /**
     * sets the static feed forward of the controller
     * (the voltage needed to overcome static friction)
     *
     * @param feedForward the voltage needed to overcome static friction (positive value)
     */
    public void setStaticFeedForward(double feedForward) {
        this.motorKs = feedForward;
    }

    /**
     * Enables position wrapping for the controller.
     * This method is used to wrap the position to be within a specified range.
     * For example, if the minimum is 0 and the maximum is 1,
     * and the position is 1.5, the position will be wrapped to 0.5.
     * This is useful for systems that have a continuous range of motion.
     * (like a swerve steer)
     * DO NOT USE THIS FOR SYSTEMS THAT HAVE A LIMITED RANGE OF MOTION
     * (like an elevator) OR CAN BE DAMAGED BY WRAPPING
     * (like a turret)
     *
     * @param minimum the minimum value
     * @param maximum the maximum value
     */
    public void enablePositionWrapping(double minimum, double maximum) {
        enablePositionWrapping(new Double[]{minimum, maximum});
    }

    /**
     * Disables position wrapping for the controller.
     */
    public void disablePositionWrapping() {
        enablePositionWrapping(null);
    }

    /**
     * Checks if position wrapping is enabled for the controller.
     *
     * @return true if position wrapping is enabled, false otherwise.
     */
    public boolean isPositionWrappingEnabled() {
        return wrappingMinMax != null;
    }

    public void enableSoftLimits(Double[] limits) {
        if (limits != null && limits.length != 2) {
            throw new IllegalArgumentException("limits must have exactly 2 elements");
        }

        if (limits != null && limits[0] > limits[1]) {
            throw new IllegalArgumentException("min limit must be less than max limit");
        }

        softLimits = limits;
    }

    public void enableSoftLimits(double min, double max) {
        enableSoftLimits(new Double[]{min, max});
    }

    public void disableSoftLimits() {
        enableSoftLimits(null);
    }

    public boolean isSoftLimitsEnabled() {
        return softLimits != null;
    }

    /**
     * sets the pid gains and feed forward of the controller
     *
     * @param gains       the pid gains of the controller (F is ignored)
     * @param feedForward the function that calculates the feed forward of the controller based on the measurement
     */
    public void setPIDF(PIDFGains gains, Function<Double, Double> feedForward) {

        Objects.requireNonNull(gains, "Gains cannot be null");

        Objects.requireNonNull(feedForward, "Feed forward cannot be null");

        this.currentGains = gains;

        pidController = gains.createPIDController();

        setPIDFMotor(gains);

        this.feedForward = feedForward;
    }

    /**
     * sets a feedForward (different from the static feed forward)
     * is multiplied by the setpoint
     *
     * @param kF units should be Volts / reference units
     */
    public void setFeedForward(double kF) {
        if (kF < 0) throw new IllegalArgumentException("feed forward should be greater then zero");

        feedForward = (measurement) -> kF;
    }

    /**
     * sets the feedforward to a function
     *
     * @param feedForward should return the feedForward value (units are Volts / reference Units)
     */
    public void setFeedForward(Function<Double, Double> feedForward) {
        this.feedForward = feedForward;
    }

    /**
     * sends the pid to the smart dashboard
     * that way you can change the pid values on the fly
     * WARNING this will break the feed forward if it is not static (base on the measurement)
     * if using a controller that is on the rio, you can't stop the values from being updated on the board
     * and also feedforward will be updated
     * (both of them until you restart the code)
     */
    public void startPIDTuning() {
        SmartDashboard.putData(name + "FeedForward", builder ->
                builder.addDoubleProperty("feedForward", () -> feedForward.apply(0.0),
                        (value) -> feedForward = (measurement) -> value));

        isRunningPIDTuning = true;

        SmartDashboard.putData(name + "PID", pidController);
    }

    /**
     * stops the pid tuning
     * only does something if started the tuning and using the motor controller on the motor
     */
    public void stopPIDTuning() {
        isRunningPIDTuning = false;
    }

    /**
     * checks if the pid tuning is running
     *
     * @return true if the pid tuning is running
     */
    public boolean isRunningPIDTuning() {
        return isRunningPIDTuning;
    }


    /**
     * sets the measurements of the controller
     *
     * @param measurements the measurements of the controller
     *                     these are the position, velocity, and acceleration of the controlled value
     */
    public void setMeasurements(MarinersMeasurements measurements) {

        Objects.requireNonNull(measurements, "Measurements cannot be null");

        this.measurements = measurements;
    }

    public MarinersMeasurements getMeasurements() {
        return measurements;
    }

    /**
     * sets the max and min output of the controller in volts
     *
     * @param maxMinOutput the max and min output of the controller in volts
     */
    public void setMaxMinOutput(double[] maxMinOutput) {

        Objects.requireNonNull(maxMinOutput, "Max min output cannot be null");

        setMaxMinOutput(maxMinOutput[0], maxMinOutput[1]);
    }

    /**
     * sets the max and min output of the controller in volts
     *
     * @param max the max output of the controller in volts
     * @param min the min output of the controller in volts
     */
    public void setMaxMinOutput(double max, double min) {
        this.maxMinOutput = new double[]{max, -Math.abs(min)};
        setMaxMinOutputMotor(max, min);
    }

    protected abstract void setMaxMinOutputMotor(double max, double min);

    /**
     * sets the profile of the controller
     *
     * @param profile the profile of the controller
     *                (needs to be appropriately set for the control mode)
     *                normally, this would be the max velocity and acceleration of the controlled value,
     *                but if used profiled velocity control, this would be the max acceleration and jerk
     */
    public void setProfile(TrapezoidProfile profile) {
        this.profile = profile;
    }

    /**
     * sets the profile of the controller
     *
     * @param constraints the constraints of the profile (needs to be appropriately set for the control mode)
     *                    normally, this would be the max velocity and acceleration of the controlled value,
     *                    but if used profiled velocity control, this would be the max acceleration and jerk
     */
    public void setProfile(TrapezoidProfile.Constraints constraints) {

        Objects.requireNonNull(constraints, "Constraints cannot be null");

        profile = new TrapezoidProfile(constraints);
    }

    /**
     * sets the profile of the controller
     *
     * @param first_derivative  the max value of the first derivative of the controlled value (for example, if using position control, this would be velocity)
     * @param second_derivative the max value of the second derivative of the controlled value (for example, if using position control, this would be acceleration)
     */
    public void setProfile(double first_derivative, double second_derivative) {
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(first_derivative, second_derivative));
    }

    /**
     * sets the deadband of the motor in duty cycle
     * if the motor output is less than this value, the motor will not put out any power
     *
     * @param deadBand the deadband of the motor in duty cycle
     */
    public void setMotorDeadBandDutyCycle(double deadBand) {
        motorVoltageDeadBand = deadBand;
        setMotorDeadBandDutyCycleMotor(deadBand);
    }

    /**
     * sends the deadband to the motor
     *
     * @param deadBand the deadband of the motor in duty cycle
     */
    protected abstract void setMotorDeadBandDutyCycleMotor(double deadBand);

    /**
     * sets the deadband of the motor in voltage
     * if the motor output is less than this value, the motor will not put out any power
     * motors work in duty cycle, so this will convert the voltage to duty cycle (12 volts is 1)
     *
     * @param deadBand the deadband of the motor in voltage
     */
    public void setMotorDeadBandVoltage(double deadBand) {
        setMotorDeadBandDutyCycle(deadBand / 12);
    }

    /**
     * is the motor inverted
     * true if counterclockwise is positive, false if clockwise is positive
     *
     * @param inverted true if the motor is inverted, false if the motor is not inverted
     */
    public abstract void setMotorInverted(boolean inverted);

    /**
     * resets the motor encoder
     * (only works if the measurements are based on the encoder)
     */
    public abstract void resetMotorEncoder();

    /**
     * sets the motor encoder position
     * (only works if the measurements are based on the encoder)
     * the value is multiplied by the gear ratio
     *
     * @param position the position of the motor encoder (default is rotations)
     */
    public abstract void setMotorEncoderPosition(double position);


    /**
     * creates the controller without any pid control or feed forward
     *
     * @param name     the name of the motor
     * @param location the location of the controller where it is running
     */
    protected MarinersController(String name, ControllerLocation location) {

        Objects.requireNonNull(name, "Name cannot be null");

        if (name.isBlank()) {
            throw new IllegalArgumentException("Name cannot be blank");
        }

        Objects.requireNonNull(location, "Location cannot be null");

        this.name = name;
        this.location = location;

        this.RUN_HZ = ControllerMaster.getInstance().addController(this, location);
    }
}
