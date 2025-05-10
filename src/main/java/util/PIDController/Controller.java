package util.PIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.function.BooleanConsumer;
import util.PIDController.Gains.ControllerGains;

public class Controller {


    private final ControllerGains controllerGains;

    private final PIDController pidController;

    private ControllerRequest request;
    private TrapezoidProfile.State setpoint;

    public Controller(ControllerGains controllerGains, BooleanConsumer hasPIDGainsChangedConsumer) {
        this.controllerGains = controllerGains;
        this.controllerGains.setHasPIDGainsChanged(hasPIDGainsChangedConsumer);

        this.pidController = new PIDController(
            controllerGains.getPidGains().getK_P(),
            controllerGains.getPidGains().getK_I(),
            controllerGains.getPidGains().getK_D()
        );
        updatePIDGains();
    }

    public ControllerGains getControllerGains() {
        return controllerGains;
    }

    public void setReference(ControllerRequest request) {
       this.request = request;
    }

    public void setReference(double setpoint, RequestType requestType) {
        setReference(new ControllerRequest(setpoint, requestType));
    }

    public void setReference(double setpoint, double setpointVelocity, RequestType requestType) {
        setReference(new ControllerRequest(setpoint, setpointVelocity, requestType));
    }

    public void setReference(TrapezoidProfile.State setpoint, RequestType requestType) {
        setReference(new ControllerRequest(setpoint, requestType));
    }

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

    private double calculate(double measurement, double dt) {
        double value = calculateWithOutPID(measurement, dt);

        return value + calculatePID(measurement);
    }

    private double calculatePID(double measurement) {
        double value = this.pidController.calculate(measurement, this.setpoint.position);

        if(this.pidController.atSetpoint()) value = 0;

        var pidGains = this.controllerGains.getPidGains();

        return Math.max(pidGains.getMinOutput(), Math.min(pidGains.getMaxOutput(), value));
    }

    private double calculateFeedForward(double directionOfTravel) {
        var feedForwards = this.controllerGains.getControllerFeedForwards();

        return feedForwards.getSimpleFeedForward() +
                feedForwards.getFrictionFeedForward() * directionOfTravel +
                feedForwards.getK_V() * setpoint.position +
                feedForwards.getCalculatedFeedForward(setpoint.position);
    }

    private TrapezoidProfile.State calculateProfile(double dt){
        if(!request.requestType.isProfiled())
            return new TrapezoidProfile.State(request.goal.position, request.goal.velocity);

        var profile = this.controllerGains.getControllerProfile();

        return profile.calculate(dt, setpoint, request.goal);
    }

    private void calculateConstraints(double measurement) {
        this.controllerGains.getControllerConstrains().calculate(measurement, request);
    }

    /**
     * resets the integral sum and the previous error of the PID controller
     */
    public void reset() {
        this.pidController.reset();
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
