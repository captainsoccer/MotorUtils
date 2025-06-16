package util.BasicMotor;

import edu.wpi.first.wpilibj.RobotState;
import util.BasicMotor.Controllers.Controller;
import util.BasicMotor.Gains.ControllerConstrains;
import util.BasicMotor.Gains.ControllerGains;
import util.BasicMotor.Gains.CurrentLimits;
import util.BasicMotor.Gains.PIDGains;
import util.BasicMotor.Measurements.Measurements;
import util.BasicMotor.MotorManager.ControllerLocation;

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

        public boolean stopped() {
            return this == STOPPED || this == DISABLED;
        }
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
    protected final LogFrameAutoLogged logFrame = new LogFrameAutoLogged();

    /**
     * the location of the controller (RIO or motor controller)
     */
    protected final MotorManager.ControllerLocation controllerLocation;

    /**
     * if the motor is running or not
     */
    private MotorState motorState;

    /**
     * if the PID gains have changed (then it updates the motor controller on the slower thread)
     */
    private boolean hasPIDGainsChanged = false;

    /**
     * a function to set the PID gains changed flag
     */
    private void setHasPIDGainsChanged() {
        hasPIDGainsChanged = true;
    }

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
     * a function to set the constraints changed flag
     */
    private void setHasConstraintsChanged() {
        hasConstraintsChanged = true;
    }

    /**
     * the name of the motor (used for logging)
     */
    protected final String name;

    /**
     * updates the constraints of the motor controller this is used to update the constraints of the
     * motor controller when the constraints change
     *
     * @param constraints the new constraints to set
     */
    protected abstract void updateConstraints(ControllerConstrains constraints);

    // constructors

    /**
     * creates the motor. the motor child needs to register the measurements
     *
     * @param controllerGains    the gains of the controller
     * @param name               the name of the motor (used for logging)
     * @param controllerLocation the location of the controller (RIO or motor controller)
     */
    public BasicMotor(
            ControllerGains controllerGains, String name, ControllerLocation controllerLocation) {
        this.controllerLocation = controllerLocation;
        controller =
                new Controller(
                        controllerGains, this::setHasPIDGainsChanged, this::setHasConstraintsChanged);

        this.name = name;

        MotorManager.getInstance()
                .registerMotor(
                        name, controllerLocation, this::run, this::updateSensorData, this::getLatestFrame);
    }

    // getters and setters

    /**
     * sets the measurements of the motor this is used when you want to switch a source of the
     * measurements
     * this will stop the recording of the measurements from the motor controller
     * if you want to start recording the measurements again, you can call {@link #setDefaultMeasurements()}
     *
     * @param measurements the new measurements
     */
    public void setMeasurements(Measurements measurements) {
        stopRecordingMeasurements();
        this.measurements = measurements;
    }

    /**
     * sets the default measurements of the motor this is used to set the default measurements of the
     * motor
     * this will start recording the measurements from the motor controller
     */
    public void setDefaultMeasurements() {
        this.measurements = getDefaultMeasurements();
        startRecordingMeasurements(controllerLocation.HZ);
    }

    /**
     * gets the default measurements of the motor this is used
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
    private LogFrameAutoLogged getLatestFrame() {
        return logFrame;
    }

    /**
     * this runs the main loop for the controller runs pid if needed and updates the measurements this
     * is called on a separate thread
     */
    private void run() {
        // doesn't need to do anything if the motor is following another motor
        if (motorState == MotorState.FOLLOWING) {
            return;
        }

        // updates the measurements
        var measurement = measurements.update(1 / controllerLocation.HZ);
        logFrame.measurement = measurement;

        // checks if the motor is stopped or needs to be stopped
        if (motorState.stopped() && controller.getRequestType() == Controller.RequestType.STOP) {
            return;
        } else if (controller.getRequestType() == Controller.RequestType.STOP) {
            motorState = MotorState.STOPPED;
            stopMotorOutput();
            logFrame.controllerFrame = LogFrame.ControllerFrame.EMPTY;
            return;
        }
        // mode
        if (RobotState.isDisabled()) {
            motorState = MotorState.DISABLED;
            stopMotorOutput();
            return;
        }

        // if the motor was disabled before, resets the controller to the current measurement
        if (motorState == MotorState.DISABLED) {
            double value =
                    controller.getRequestType().isVelocityControl()
                            ? measurement.velocity()
                            : measurement.position();

            controller.reset(value);
        }

        motorState = MotorState.RUNNING;

        // the controller frame (for logging)
        LogFrame.ControllerFrame motorOutput;
        // the reference to give to the motor (can be any mode)
        double reference;
        // the feedforward (this is used if the controller is on the motor and there is any feedforward
        // calculation)
        double feedForward;
        // the output mode (this is used to set the motor output)
        Controller.RequestType outputMode;

        // if the controller is not using PID, we can just set the reference to the setpoint
        if (!controller.getRequestType().requiresPID()) {
            var request = controller.getRequestType();
            reference = controller.getRequest().goal().position;
            feedForward = 0;

            // if the controller is using voltage control, we need to set the reference to the setpoint
            if (request == Controller.RequestType.VOLTAGE) {
                motorOutput =
                        new LogFrame.ControllerFrame(reference, reference, measurement.position(), request);
                outputMode = Controller.RequestType.VOLTAGE;
            }
            // if using precent output, then the total output needs to be multiplied by the input voltage
            // to be in volts
            else {
                // multiplies the setpoint (duty cycle) by the input voltage to get the output voltage
                motorOutput =
                        new LogFrame.ControllerFrame(
                                reference * logFrame.sensorData.voltageInput(),
                                reference,
                                measurement.position(),
                                request);

                outputMode = Controller.RequestType.PRECENT_OUTPUT;
            }
        }
        // if the controller is using PID, we need to calculate the output
        else {
            // if calculating on the rio, then calculate the output in volts and set it
            if (controllerLocation == ControllerLocation.RIO) {
                motorOutput = controller.calculate(measurement, 1 / controllerLocation.HZ);

                reference = motorOutput.totalOutput();
                feedForward = 0;
                outputMode = Controller.RequestType.VOLTAGE;
            }
            // if calculating on the motor controller, then calculate the feedforward and the setpoint and
            // give to the motor
            else {
                var feedForwardOutput =
                        controller.calculateWithOutPID(measurement, 1 / controllerLocation.HZ);

                // adds the pid output from the motor controller (could be outdated by a cycle or two)
                motorOutput = new LogFrame.ControllerFrame(feedForwardOutput, getPIDLatestOutput());

                reference = motorOutput.setpoint();
                feedForward = motorOutput.feedForwardOutput().totalOutput();
                outputMode = motorOutput.mode();
            }
        }
        double tolerance = controller.getControllerGains().getPidGains().getTolerance();

        boolean atSetpoint = Math.abs(motorOutput.error()) < tolerance;
        logFrame.atSetpoint = atSetpoint;

        if (motorOutput.mode().isProfiled()) {
            logFrame.atGoal =
                    (Math.abs(motorOutput.goal() - motorOutput.setpoint()) < tolerance) && atSetpoint;
        }

        // updates the log frame with the motor output
        logFrame.controllerFrame = motorOutput;
        // sets the motor output
        setMotorOutput(reference, feedForward, outputMode);
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
     * stops the motor
     */
    public void stopMotor() {
        controller.setReference(new Controller.ControllerRequest());
    }

    /**
     * gets all the sensor data from the motor (this runs on a separate slower thread)
     */
    private void updateSensorData() {
        logFrame.sensorData = getSensorData();

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
     * @param inverted if the motor should be inverted (true for clockwise positive, false for counter-clockwise positive)
     */
    public abstract void setMotorInverted(boolean inverted);

    /**
     * sets the motor encoder to a new positon
     *
     * @param position the position
     */
    protected abstract void setMotorPosition(double position);

    /**
     * used when there is no more need to record measurements this is used to stop the
     * measurement recording (to free up canbus)
     * measurements can be re-enabled by calling {@link #startRecordingMeasurements(double HZ)}
     */
    protected abstract void stopRecordingMeasurements();

    /**
     * starts recording measurements this is used to start the measurement recording
     * (used when accidentally stopped the measurements)
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

    protected abstract void setMotorFollow(BasicMotor master, boolean inverted);

    protected abstract void stopMotorFollow();

    /**
     * stops following the motor this will re-enable pid control on this motor and measurements
     * logging
     */
    public void stopFollowing() {
        motorState = MotorState.STOPPED;
        stopMotorFollow();
        stopMotorOutput();
        stopRecordingMeasurements();
    }

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
     * gets the current position of the motor
     * <p>
     * default units are rotations but can be changed in the measurements
     * @return the current position of the motor
     */
    public double getPosition() {
        return measurements.getMeasurement().position();
    }

    /**
     * gets the current velocity of the motor
     * <p>
     * default units are rotations per second, but can be changed in the measurements
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
    public boolean isAtSetpoint() {
        return logFrame.atSetpoint;
    }

    /**
     * if the motor is at the goal this is used to check if the motor is at the goal
     *
     * @return true if the motor is at the goal, false otherwise
     */
    public boolean isAtGoal() {
        return logFrame.atGoal;
    }

    /**
     * resets the motor encoder to a specif position
     *
     * @param newPosition the new position of the motor (gear ratio is applied later)
     */
    public void resetEncoder(double newPosition) {
        controller.reset(newPosition);
        setMotorPosition(newPosition * measurements.getGearRatio());
    }

    /**
     * resets the controllers (essentially makes the motor as if it was just created, but in the
     * latest position)
     */
    public void reset() {
        resetEncoder(measurements.getGearRatio());
    }
}
