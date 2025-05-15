package util.BasicMotor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import util.BasicMotor.Gains.ControllerGains;

public class Controller implements Sendable {
    private final ControllerGains controllerGains;

    private final PIDController pidController;

    private ControllerRequest request;
    private TrapezoidProfile.State setpoint;

    public Controller(ControllerGains controllerGains, Runnable hasPIDGainsChangedConsumer) {
        this.controllerGains = controllerGains;
        this.controllerGains.setHasPIDGainsChanged(hasPIDGainsChangedConsumer);

        this.pidController = new PIDController(
            controllerGains.getPidGains().getK_P(),
            controllerGains.getPidGains().getK_I(),
            controllerGains.getPidGains().getK_D()
        );
        updatePIDGains();
    }

    public Controller(Runnable hasPIDGainsChangedConsumer) {
        this(new ControllerGains(), hasPIDGainsChangedConsumer);
    }

    public ControllerGains getControllerGains() {
        return controllerGains;
    }

    /**
     * sets the reference of the controller
     * @param request the request of the controller
     */
    public void setReference(ControllerRequest request) {
       this.request = request;
    }

    /**
     * sets the reference of the controller
     * @param setpoint the setpoint of the controller
     * @param requestType the request type of the controller
     */
    public void setReference(double setpoint, RequestType requestType) {
        setReference(new ControllerRequest(setpoint, requestType));
    }

    /**
     * sets the reference of the controller
     * @param setpoint the setpoint of the controller
     * @param setpointVelocity the setpoint velocity of the controller (used for profiled position and velocity)
     * @param requestType the request type of the controller
     */
    public void setReference(double setpoint, double setpointVelocity, RequestType requestType) {
        setReference(new ControllerRequest(setpoint, setpointVelocity, requestType));
    }

    /**
     * sets the reference of the controller
     * @param setpoint the setpoint of the controller
     * @param requestType the request type of the controller
     */
    public void setReference(TrapezoidProfile.State setpoint, RequestType requestType) {
        setReference(new ControllerRequest(setpoint, requestType));
    }

    /**
     * calculates the output of the controller without the PID controller
     * this is used when the pid of the motor is running on the motor controller
     * @param measurement the measurement of the controller (depending on the request type)
     * @param dt the time since the last calculation
     * @return the output of the controller in volts
     */
    public double calculateWithOutPID(double measurement, double dt) {
        if(request.requestType == RequestType.STOP) return 0;

        calculateConstraints(measurement);

        setpoint = calculateProfile(dt);

        double directionOfTravel = switch (request.requestType){
            case POSITION, PROFILED_POSITION -> setpoint.position > measurement ? 1 : -1;
            default -> Math.signum(setpoint.position);
        };

        return calculateFeedForward(directionOfTravel);
    }

    /**
     * calculates the output of the controller
     * @param measurement the measurement of the controller (depending on the request type)
     * @param dt the time since the last calculation
     * @return the output of the controller in volts
     */
    public double calculate(double measurement, double dt) {
        double value = calculateWithOutPID(measurement, dt);

        return value + calculatePID(measurement);
    }

    /**
     * calculates the PID of the controller
     * @param measurement the measurement of the controller (depending on the request type)
     * @return the output PID of the controller in volts
     */
    private double calculatePID(double measurement) {
        double value = this.pidController.calculate(measurement, this.setpoint.position);

        if(this.pidController.atSetpoint()) value = 0;

        var pidGains = this.controllerGains.getPidGains();

        return Math.max(pidGains.getMinOutput(), Math.min(pidGains.getMaxOutput(), value));
    }

    /**
     * calculates the feed forward of the controller
     * @param directionOfTravel the direction of travel of the motor (used to calculate the friction feed forward)
     * @return the feed forward of the controller in volts
     */
    private double calculateFeedForward(double directionOfTravel) {
        var feedForwards = this.controllerGains.getControllerFeedForwards();

        return feedForwards.getSimpleFeedForward() +
                feedForwards.getFrictionFeedForward() * directionOfTravel +
                feedForwards.getK_V() * setpoint.position +
                feedForwards.getCalculatedFeedForward(setpoint.position);
    }

    /**
     * calculates the profile of movement
     * @param dt the time since the last calculation
     * @return the new setpoint of the controller
     */
    private TrapezoidProfile.State calculateProfile(double dt){
        if(!request.requestType.isProfiled())
            return new TrapezoidProfile.State(request.goal.position, request.goal.velocity);

        var profile = this.controllerGains.getControllerProfile();

        return profile.calculate(dt, setpoint, request.goal);
    }

    /**
     * calculates the constraints of the controller based on the measurement and the last request
     * @param measurement the measurement of the controller (depending on the request type)
     */
    private void calculateConstraints(double measurement) {
        this.controllerGains.getControllerConstrains().calculate(measurement, request);
    }

    /**
     * resets the integral sum and the previous error of the PID controller
     */
    public void reset() {
        this.pidController.reset();
        this.setpoint = new TrapezoidProfile.State();
    }

    /**
     * gets the current setpoint of the controller
     * @return the current setpoint of the controller
     */
    public TrapezoidProfile.State getSetpoint() {
        return setpoint;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        controllerGains.initSendable(builder);

        builder.addDoubleProperty("goal", () -> request.goal.position,
                (value) -> setReference(value, request.requestType));

        builder.addDoubleProperty("setpoint", () -> setpoint.position,
                (value) -> setReference(value, request.requestType));
    }

    /**
     * updates the PID gains of the PID controller
     */
    public void updatePIDGains(){
        this.pidController.setPID(
            controllerGains.getPidGains().getK_P(),
            controllerGains.getPidGains().getK_I(),
            controllerGains.getPidGains().getK_D()
        );

        this.pidController.setTolerance(controllerGains.getPidGains().getTolerance());

        this.pidController.setIntegratorRange(
            -controllerGains.getPidGains().getI_MaxAccum(),
            controllerGains.getPidGains().getI_MaxAccum()
        );

        this.pidController.setIZone(controllerGains.getPidGains().getI_Zone());
    }


    public enum RequestType  {
        STOP,
        VOLTAGE,
        PRECENT_OUTPUT,
        POSITION,
        PROFILED_POSITION,
        VELOCITY,
        PROFILED_VELOCITY;

        public boolean isPositionControl() {
            return this == POSITION || this == PROFILED_POSITION;
        }

        public boolean isVelocityControl() {
            return this == VELOCITY || this == PROFILED_VELOCITY;
        }

        public boolean isProfiled() {
            return this == PROFILED_POSITION || this == PROFILED_VELOCITY;
        }

        public boolean requiresPID() {
            return this != VOLTAGE && this != PRECENT_OUTPUT;
        }
    }

    public record ControllerRequest(TrapezoidProfile.State goal, RequestType requestType){
        public ControllerRequest(double setpoint, RequestType requestType) {
            this(new TrapezoidProfile.State(setpoint, 0), requestType);
        }

        public ControllerRequest(double setpoint, double setpointVelocity, RequestType requestType) {
            this(new TrapezoidProfile.State(setpoint, setpointVelocity), requestType);
        }

        public ControllerRequest() {
            this(new TrapezoidProfile.State(), RequestType.STOP);
        }
    }
}
