package com.basicMotor;

import com.basicMotor.configuration.BasicMotorConfig;
import com.basicMotor.controllers.Controller;
import com.basicMotor.gains.ControllerConstrains;
import com.basicMotor.gains.ControllerGains;
import com.basicMotor.gains.currentLimits.CurrentLimits;
import com.basicMotor.gains.PIDGains;
import com.basicMotor.measurements.Measurements;
import com.basicMotor.MotorManager.ControllerLocation;
import edu.wpi.first.wpilibj.RobotState;

import java.util.Objects;

/**
 * this is a basic motor class, it is used to simplify the process of creating a motor and also
 * making it cross-platform for different motor controllers it handles logging, running pid, and
 * updating the measurements
 */
public abstract class BasicMotor {
    /**
     * what state is the motor in
     */
    public enum MotorState {
        /**
         * the motor is stopped
         */
        STOPPED,
        /**
         * the motor is running
         */
        RUNNING,
        /**
         * the motor is following another motor
         */
        FOLLOWING,

        /**
         * motor state during the disabled period (practically the same as stopped)
         */
        DISABLED;
    }

    /**
     * the mode of the motor when it is not running this is used to set the motor to brake or
     */
    public enum IdleMode {
        /**
         * the motor is in brake mode
         */
        BRAKE,
        /**
         * the motor is in coast mode
         */
        COAST
    }

    /**
     * the controller of the motor
     */
    private final Controller controller;

    /**
     * the measurements of the motor
     */
    private Measurements measurements;

    /**
     * the log frame of the motor
     */
    protected final LogFrame.LogFrameAutoLogged logFrame = new LogFrame.LogFrameAutoLogged();

    /**
     * the location of the controller (RIO or motor controller)
     */
    protected final ControllerLocation controllerLocation;

    /**
     * if the motor is running or not
     */
    private MotorState motorState;

    /**
     * if the PID gains have changed (then it updates the motor controller on the slower thread)
     */
    private boolean hasPIDGainsChanged = false;

    /**
     * updates the PID gains to the motor controller this is used to update the PID gains of the motor
     * controller when the PID gains change
     *
     * @param pidGains the new PID gains to set
     */
    protected abstract void updatePIDGainsToMotor(PIDGains pidGains);

    /**
     * if the constraints have changed (then it updates the motor controller on the slower thread)
     */
    private boolean hasConstraintsChanged = false;

    /**
     * updates the constraints of the motor controller this is used to update the constraints of the
     * motor controller when the constraints change
     *
     * @param constraints the new constraints to set
     */
    protected abstract void updateConstraints(ControllerConstrains constraints);

    /**
     * the name of the motor (used for logging)
     */
    protected final String name;

    /**
     * the configuration of the motor controller used for initialization
     */
    private final BasicMotorConfig config;

    /**
     * if the motor has been initialized or not
     */
    private boolean initialized = false;

    /**
     * creates the motor.
     *
     * @param controllerGains    the gains of the controller
     * @param name               the name of the motor (used for logging)
     * @param controllerLocation the location of the controller (RIO or motor controller)
     */
    public BasicMotor(
            ControllerGains controllerGains, String name, ControllerLocation controllerLocation) {
        this(controllerGains, name, controllerLocation, null);
    }

    /**
     * creates the motor.
     *
     * @param config the configuration of the motor controller
     */
    public BasicMotor(BasicMotorConfig config) {
        this(config.getControllerGains(), config.motorConfig.name, config.motorConfig.location, config);
    }

    /**
     * creates the motor.
     *
     * @param controllerGains    the gains of the controller
     * @param name               the name of the motor (used for logging)
     * @param controllerLocation the location of the controller (RIO or motor controller)
     * @param config             the configuration of the motor controller (saved for initialization)
     */
    private BasicMotor(
            ControllerGains controllerGains,
            String name,
            ControllerLocation controllerLocation,
            BasicMotorConfig config) {
        Objects.requireNonNull(controllerGains);
        Objects.requireNonNull(name);
        Objects.requireNonNull(controllerLocation);

        this.controllerLocation = controllerLocation;

        Runnable setHasPIDGainsChanged = () -> hasPIDGainsChanged = true;
        Runnable setHasConstraintsChanged = () -> hasConstraintsChanged = true;
        controller = new Controller(controllerGains, setHasPIDGainsChanged, setHasConstraintsChanged);

        this.name = name;

        this.config = config;

        MotorManager.getInstance()
                .registerMotor(
                        name, controllerLocation, this::run, this::updateSensorData, this::getLatestFrame);
    }

    /**
     * initializes the motor with the given configuration
     *
     * @return the default measurements of the motor
     */
    public Measurements initializeMotor() {
        if (initialized) return getDefaultMeasurements();

        if (getDefaultMeasurements() == null) return null;

        double gearRatio = getDefaultMeasurements().getGearRatio();

        updatePIDGainsToMotor(
                controller.getControllerGains().getPidGains().convertToMotorGains(gearRatio));
        updateConstraints(
                controller
                        .getControllerGains()
                        .getControllerConstrains()
                        .convertToMotorConstrains(gearRatio));

        if (config == null) {
            initialized = true;
            return getDefaultMeasurements();
        }

        setIdleMode(config.motorConfig.idleMode);
        setMotorInverted(config.motorConfig.inverted);

        initialized = true;

        return config.usingExternalEncoder() ? getMeasurements() : getDefaultMeasurements();
    }

    // getters and setters

    /**
     * sets the measurements of the motor this is used when you want to switch a source of the
     * measurements this will stop the recording of the measurements from the motor controller if you
     * want to start recording the measurements again, you can call {@link #setDefaultMeasurements()}
     *
     * @param measurements the new measurements
     */
    public void setMeasurements(Measurements measurements) {
        stopRecordingMeasurements();
        this.measurements = measurements;
    }

    /**
     * sets the default measurements of the motor this is used to set the default measurements of the
     * motor this will start recording the measurements from the motor controller
     */
    public void setDefaultMeasurements() {
        this.measurements = getDefaultMeasurements();
        startRecordingMeasurements(controllerLocation.getHZ());
    }

    /**
     * gets the default measurements of the motor this is used
     *
     * @return the default measurements of the motor
     */
    protected abstract Measurements getDefaultMeasurements();

    /**
     * gets the measurements of the motor
     *
     * @return the measurements of the motor
     */
    public Measurements getMeasurements() {
        return measurements;
    }

    /**
     * gets the latest measurement of the motor
     *
     * @return the latest measurement of the motor
     */
    public Measurements.Measurement getMeasurement() {
        return measurements.getMeasurement();
    }

    /**
     * gets the controller of the motor
     *
     * @return the controller of the motor
     */
    public Controller getController() {
        return controller;
    }

    /**
     * gets the latest log frame this is used for logging
     *
     * @return the latest log frame
     */
    private LogFrame.LogFrameAutoLogged getLatestFrame() {
        return logFrame;
    }

    /**
     * stops the motor
     */
    public void stopMotor() {
        controller.setReference(new Controller.ControllerRequest());
    }

    /**
     * sets the current limits of the motor some current limits are not supported on all motors
     *
     * @param currentLimits the current limits of the motor
     */
    public abstract void setCurrentLimits(CurrentLimits currentLimits);

    /**
     * sets the idle mode of the motor this is used to set the motor to brake or coast mode
     *
     * @param mode the idle mode of the motor
     */
    public abstract void setIdleMode(IdleMode mode);

    /**
     * sets if the motor is inverted or not, this is used to set the motor to be inverted or not
     * default is counter-clockwise positive
     *
     * @param inverted if the motor should be inverted (true for clockwise positive, false for
     *                 counter-clockwise positive)
     */
    public abstract void setMotorInverted(boolean inverted);

    /**
     * sets the motor encoder to a new positon
     *
     * @param position the position
     */
    protected abstract void setMotorPosition(double position);

    /**
     * used when there is no more need to record measurements this is used to stop the measurement
     * recording (to free up canbus) measurements can be re-enabled by calling {@link
     * #startRecordingMeasurements(double HZ)}
     */
    protected abstract void stopRecordingMeasurements();

    /**
     * starts recording measurements this is used to start the measurement recording (used when
     * accidentally stopped the measurements)
     *
     * @param HZ the frequency to record the measurements at (in Hz)
     */
    protected abstract void startRecordingMeasurements(double HZ);

    /**
     * start following another motor this only works if the other motor is the same type as this motor
     * this will disable pid control on this motor and measurements logging
     *
     * @param master   the motor to follow
     * @param inverted if the motor should be opposite of the master
     */
    public void followMotor(BasicMotor master, boolean inverted) {
        if (master == this) {
            return;
        }

        if (this.getClass() != master.getClass()) {
            throw new IllegalArgumentException("Cannot follow a motor of a different type");
        }

        motorState = MotorState.FOLLOWING;
        setMotorFollow(master, inverted);
        stopRecordingMeasurements();
    }

    /**
     * stops following the motor this will re-enable pid control on this motor and measurements
     * logging
     */
    public void stopFollowing() {
        motorState = MotorState.STOPPED;
        stopMotorFollow();
        stopMotorOutput();
        startRecordingMeasurements(controllerLocation.getHZ());
    }

    /**
     * sets the motor to follow the output of another motor
     *
     * @param master   the motor to follow
     * @param inverted if the motor should be opposite of the master
     */
    protected abstract void setMotorFollow(BasicMotor master, boolean inverted);

    /**
     * stops the motor from following another motor this is used to stop the motor from following
     * another motor
     */
    protected abstract void stopMotorFollow();

    // forwarded functions (for ease of use)

    /**
     * sets the reference of the motor this is used to set the setpoint of the motor
     *
     * @param request the request to set the reference to
     */
    public void setReference(Controller.ControllerRequest request) {
        controller.setReference(request);
    }

    /**
     * sets the reference of the motor this is used to set the setpoint of the motor
     *
     * @param setpoint the setpoint of the motor (units depending on the mode)
     * @param mode     the mode of the motor (position, velocity, voltage, percent output)
     */
    public void setReference(double setpoint, Controller.RequestType mode) {
        controller.setReference(setpoint, mode);
    }

    /**
     * sets the reference of the motor this is used to set the setpoint of the motor
     *
     * @param setpoint             the setpoint of the motor (units depending on the mode)
     * @param mode                 the mode of the motor (position, velocity, voltage, percent output)
     * @param arbitraryFeedForward a voltage applied on top of the pid and other feedForwards
     */
    public void setReference(
            double setpoint, Controller.RequestType mode, double arbitraryFeedForward) {
        controller.setReference(new Controller.ControllerRequest(setpoint, mode, arbitraryFeedForward));
    }

    /**
     * sets the motor output to voltage.
     *
     * @param volts the voltage to set the motor to (in volts) (-12 to 12)
     */
    public void setVoltage(double volts){
        controller.setReference(volts, Controller.RequestType.VOLTAGE);
    }

    /**
     * sets the motor output to percent output.
     *
     * <p>this is used to set the motor to a percentage of the maximum output (-1 to 1)
     *
     * @param percentOutput the percentage of the maximum output (-1 to 1)
     */
    public void setPrecentOutput(double percentOutput) {
        controller.setReference(percentOutput, Controller.RequestType.PRECENT_OUTPUT);
    }

    /**
     * gets the current position of the motor
     *
     * <p>default units are rotations but can be changed in the measurements
     *
     * @return the current position of the motor
     */
    public double getPosition() {
        return measurements.getMeasurement().position();
    }

    /**
     * gets the current velocity of the motor
     *
     * <p>default units are rotations per second, but can be changed in the measurements
     *
     * @return the current velocity of the motor
     */
    public double getVelocity() {
        return measurements.getMeasurement().velocity();
    }

    /**
     * if the motor is at the setpoint, this is used to check if the motor is at the setpoint
     *
     * @return true if the motor is at the setpoint, false otherwise
     */
    public boolean atSetpoint() {
        return logFrame.atSetpoint;
    }

    /**
     * if the motor is at the goal, this is used to check if the motor is at the goal
     *
     * @return true if the motor is at the goal, false otherwise
     */
    public boolean atGoal() {
        return logFrame.atGoal;
    }

    /**
     * resets the motor encoder to a specif position
     *
     * @param newPosition the new position of the motor (gear ratio is applied later)
     */
    public void resetEncoder(double newPosition) {
        if(!controller.getRequestType().isVelocityControl()) controller.reset(newPosition, 0);
        else controller.reset(0, 0);

        setMotorPosition(newPosition * measurements.getGearRatio());
    }

    /**
     * resets the motor encoder to a specif position
     *
     * @param newPosition the new position of the motor (gear ratio is applied later)
     * @param newVelocity the new velocity of the motor (gear ratio is applied later)
     */
    public void resetEncoder(double newPosition, double newVelocity) {
        if(!controller.getRequestType().isVelocityControl()) controller.reset(newPosition, newVelocity);
        else controller.reset(newVelocity, 0);

        setMotorPosition(newPosition * measurements.getGearRatio());
    }

    /**
     * resets the controllers (essentially makes the motor as if it was just created, but in the
     * latest position)
     */
    public void reset() {
        resetEncoder(measurements.getPosition());
    }

    // periodic functions

    /**
     * this runs the main loop for the controller runs pid if needed and updates the measurements this
     * is called on a separate thread
     */
    private void run() {
        if (!initialized) {
            measurements = initializeMotor();
            if (measurements == null) return;
        }

        // doesn't need to do anything if the motor is following another motor
        if (motorState == MotorState.FOLLOWING) {
            return;
        }

        // updates the measurements
        var measurement = updateMeasurements();
        logFrame.measurement = measurement;

        // checks if the motor is stopped or needs to be stopped
        boolean shouldRun = shouldRunLoop(controller.getRequestType(), measurement);
        if (shouldRun) motorState = MotorState.RUNNING;
        else return;

        var motorOutput =
                runController(measurement, 1 / controllerLocation.getHZ(), controller.getRequest());

        double tolerance = controller.getControllerGains().getPidGains().getTolerance();

        boolean atSetpoint = Math.abs(motorOutput.error()) <= tolerance;
        logFrame.atSetpoint = atSetpoint;

        if (motorOutput.mode().isProfiled()) {
            logFrame.atGoal =
                    (Math.abs(motorOutput.goal() - motorOutput.setpoint()) < tolerance) && atSetpoint;
        } else logFrame.atGoal = atSetpoint;

        // updates the log frame with the motor output
        logFrame.controllerFrame = motorOutput;
        // sets the motor output
        if (controllerLocation == ControllerLocation.RIO)
            setMotorOutput(motorOutput.totalOutput(), 0, Controller.RequestType.VOLTAGE);
        else
            setMotorOutput(
                    motorOutput.setpoint(),
                    motorOutput.feedForwardOutput().totalOutput(),
                    motorOutput.mode());
    }

    /**
     * takes the latest measurements and checks if needed to use the default measurements (usually
     * used for the first run)
     *
     * @return the latest measurement of the motor
     */
    private Measurements.Measurement updateMeasurements() {
        if (measurements == null) {
            measurements = getDefaultMeasurements();
        }

        return measurements.update(1 / controllerLocation.getHZ());
    }

    /**
     * should the main loop continue running or should stop, it would stop if the motor is disabled or
     * if the request type is stop
     *
     * @param requestType the request type of the controller
     * @return true if the loop should continue running, false otherwise
     */
    private boolean shouldRunLoop(
            Controller.RequestType requestType, Measurements.Measurement measurement) {
        // if the robot is disabled, then we need to stop the motor
        if (RobotState.isDisabled()) {
            if (motorState == MotorState.DISABLED) {
                return false;
            }
            motorState = MotorState.DISABLED;
            return false;
        }

        if (motorState == MotorState.DISABLED && requestType != Controller.RequestType.STOP) {
            // if the motor was disabled, then we need to reset the controller to the current measurement
            if (requestType.isVelocityControl())
                controller.reset(measurement.velocity(), measurement.acceleration());
            else controller.reset(measurement.position(), measurement.velocity());
            // continue running the loop
            return true;
        }

        // if the request type is stop, then we need to stop the motor
        if (requestType == Controller.RequestType.STOP) {
            if (motorState != MotorState.STOPPED) {
                motorState = MotorState.STOPPED;
                stopMotorOutput();
                logFrame.controllerFrame = LogFrame.ControllerFrame.EMPTY;
                logFrame.pidOutput = LogFrame.PIDOutput.EMPTY;
            }
            return false;
        }

        return true;
    }

    /**
     * runs the controller with the given measurement and dt this is used to calculate the output of
     * the controller this functions handles constraints, feedforward, motion profiles, and PID
     * control
     *
     * @param measurement       the latest measurement of the motor
     * @param dt                the time since the last update (in seconds)
     * @param controllerRequest the request of the controller
     * @return the controller frame with the output of the controller
     */
    private LogFrame.ControllerFrame runController(
            Measurements.Measurement measurement,
            double dt,
            Controller.ControllerRequest controllerRequest) {
        // cheks if motor is within the constraints
        controller.calculateConstraints(measurement, controllerRequest);

        // if the controller is not using PID, we can just set the output directly
        if (!controllerRequest.requestType().requiresPID()) {
            double output =
                    controllerRequest.requestType() == Controller.RequestType.VOLTAGE
                            ? controllerRequest.goal().position
                            : controllerRequest.goal().position * logFrame.sensorData.voltageInput();

            return new LogFrame.ControllerFrame(
                    output,
                    controllerRequest.goal().position,
                    measurement.position(),
                    controllerRequest.requestType());
        }

        // if using a motion profile, then calculate the profile
        if (controllerRequest.requestType().isProfiled()) controller.calculateProfile(dt);
        else controller.setSetpointToGoal();

        // calculate the direction of travel
        double directionOfTravel =
                Math.signum(
                        controllerRequest.requestType().isPositionControl()
                                ? controller.getSetpoint().position - measurement.position()
                                : controller.getSetpoint().position);

        // calculate the feedforward output
        var FFOutput = controller.calculateFeedForward(directionOfTravel);

        // calculates the error of the controller
        double referenceMeasurement =
                controllerRequest.requestType().isVelocityControl()
                        ? measurement.velocity()
                        : measurement.position();
        double error = controller.getSetpoint().position - referenceMeasurement;

        // if the controller is on the motor, then we can just return the feedforward output
        if (controllerLocation == ControllerLocation.MOTOR) {
            return new LogFrame.ControllerFrame(
                    FFOutput.totalOutput(),
                    FFOutput,
                    controller.getSetpoint().position,
                    referenceMeasurement,
                    error,
                    controller.getRequest().goal().position,
                    controllerRequest.requestType());
        }

        // calculate the PID output
        var pidOutput = controller.calculatePID(referenceMeasurement, dt);

        // sums the feedforward and pid output
        double totalOutput = FFOutput.totalOutput() + pidOutput.totalOutput();
        // updates the log frame with the pid output
        logFrame.pidOutput = pidOutput;

        // returns the combined controller frame and cheks the motor output
        return new LogFrame.ControllerFrame(
                controller.checkMotorOutput(totalOutput),
                FFOutput,
                controller.getSetpoint().position,
                referenceMeasurement,
                error,
                controller.getRequest().goal().position,
                controllerRequest.requestType());
    }

    /**
     * sets the motor output
     *
     * @param setpoint    the setpoint of the motor (units depending on the mode)
     * @param feedForward the feedforward of the motor (in volts) (only used if the controller is on
     *                    the motor)
     * @param mode        the mode of the motor (position, velocity, voltage, percent output)
     */
    protected abstract void setMotorOutput(
            double setpoint, double feedForward, Controller.RequestType mode);

    /**
     * stops the motor output
     */
    protected abstract void stopMotorOutput();

    /**
     * gets all the sensor data from the motor (this runs on a separate slower thread)
     */
    private void updateSensorData() {
        if (!initialized) return;
        logFrame.sensorData = getSensorData();

        if (controllerLocation == ControllerLocation.MOTOR) logFrame.pidOutput = getPIDLatestOutput();

        if (config != null)
            logFrame.appliedTorque =
                    config.motorConfig.motorType.getTorque(logFrame.sensorData.currentOutput())
                            * config.motorConfig.gearRatio;

        // if the pid has changed, then update the built-in motor pid
        if (hasPIDGainsChanged) {
            hasPIDGainsChanged = false;

            var convertedGains =
                    controller
                            .getControllerGains()
                            .getPidGains()
                            .convertToMotorGains(measurements.getGearRatio());

            updatePIDGainsToMotor(convertedGains);
        }

        // if the constraints have changed, then update the built-in motor pid
        if (hasConstraintsChanged) {
            hasConstraintsChanged = false;

            var convertedConstraints =
                    controller
                            .getControllerGains()
                            .getControllerConstrains()
                            .convertToMotorConstrains(measurements.getGearRatio());

            updateConstraints(convertedConstraints);
        }
    }

    /**
     * gets the latest sensor data from the motor
     *
     * @return the latest sensor data
     */
    protected abstract LogFrame.SensorData getSensorData();

    /**
     * gets the pid output of the built-in controller used only when the pid is on the motor
     * controller this returns the stored pid output that should be updated in {@link
     * #getSensorData()} function
     *
     * @return the pid output
     */
    protected abstract LogFrame.PIDOutput getPIDLatestOutput();
}
