package util.BasicMotor;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import util.BasicMotor.Gains.ControllerGains;
import util.BasicMotor.LogFrame.FeedForwardOutput;
import util.BasicMotor.Measurements.Measurements;

public class Controller implements Sendable {
  /**
   * the gains of the controller
   * pid, feedforward, constraints and profile
   */
  private final ControllerGains controllerGains;

    /**
     * the controller of the controller
     * this is used to calculate the output of the controller
     * pid, feedforward, constraints and profile
     */
  private final BasicPIDController pidController;

  /**
   * the latest request of the controller
   * this contains the control mode and the goal
   */
  private ControllerRequest request;
  /**
   * the setpoint of the controller
   * this is used when using profiled position and velocity
   */
  private TrapezoidProfile.State setpoint;

    /**
     * creates a controller with the given gains
     *
     * @param controllerGains the gains of the controller
     * @param hasPIDGainsChangeRunnable the runnable that is called when the PID gains are changed
     */
  public Controller(ControllerGains controllerGains, Runnable hasPIDGainsChangeRunnable) {
    this.controllerGains = controllerGains;
    this.controllerGains.setHasPIDGainsChanged(hasPIDGainsChangeRunnable);

    this.pidController = new BasicPIDController(controllerGains.getPidGains());
    updatePIDGains();
  }

  /**
   * creates a controller with empty gains (no pid, no feedforward, no constraints, no profile)
   * @param hasPIDGainsChangedConsumer the runnable that is called when the PID gains are changed
   */
  public Controller(Runnable hasPIDGainsChangedConsumer) {
    this(new ControllerGains(), hasPIDGainsChangedConsumer);
  }

    /**
     * gets the controller gains of the controller
     *
     * @return the controller gains of the controller
     */
  public ControllerGains getControllerGains() {
    return controllerGains;
  }

  /**
   * sets the reference of the controller
   *
   * @param request the request of the controller
   */
  public void setReference(ControllerRequest request) {
    this.request = request;
  }

  /**
   * sets the reference of the controller
   *
   * @param setpoint the setpoint of the controller
   * @param requestType the request type of the controller
   */
  public void setReference(double setpoint, RequestType requestType) {
    setReference(new ControllerRequest(setpoint, requestType));
  }

  /**
   * sets the reference of the controller
   *
   * @param setpoint the setpoint of the controller
   * @param setpointVelocity the setpoint velocity of the controller (used for profiled position and
   *     velocity)
   * @param requestType the request type of the controller
   */
  public void setReference(double setpoint, double setpointVelocity, RequestType requestType) {
    setReference(new ControllerRequest(setpoint, setpointVelocity, requestType));
  }

  /**
   * sets the reference of the controller
   *
   * @param goal the setpoint of the controller
   * @param requestType the request type of the controller
   */
  public void setReference(TrapezoidProfile.State goal, RequestType requestType) {
    setReference(new ControllerRequest(goal, requestType));
  }

  /**
   * calculates the output of the controller without the PID controller this is used when the pid of
   * the motor is running on the motor controller
   *
   * @param measurements the measurement of the controller (depending on the request type)
   * @param dt the time since the last calculation
   * @return the output of the controller in volts
   */
  public LogFrame.ControllerFrame calculateWithOutPID(Measurements.Measurement measurements, double dt) {
    //gets the measurement of the controller
    double measurement = switch(request.requestType) {
      case VELOCITY, PROFILED_VELOCITY -> measurements.velocity();
      default -> measurements.position();
    };

    calculateConstraints(measurements);

    setpoint = calculateProfile(dt);

    double directionOfTravel =
        switch (request.requestType) {
          case POSITION, PROFILED_POSITION -> setpoint.position > measurement ? 1 : -1;
          default -> Math.signum(setpoint.position);
        };

    var feedForward = calculateFeedForward(directionOfTravel);

    double error = request.requestType.requiresPID() ? setpoint.position - measurement : 0;

    return new LogFrame.ControllerFrame(
            feedForward.totalOutput(),
            new LogFrame.PIDOutput(),
            feedForward,
            setpoint.position,
            measurement,
            error,
            request.goal.position,
            request.requestType
    );
  }

  /**
   * gets the current setpoint of the controller
   *
   * @return the current setpoint of the controller
   */
  public TrapezoidProfile.State getSetpoint() {
    return setpoint;
  }

  public RequestType getRequestType() {
    return request.requestType;
  }

  public ControllerRequest getRequest() {
    return request;
  }

  /** updates the PID gains of the PID controller */
  public void updatePIDGains() {
    this.pidController.setGains(controllerGains.getPidGains());
  }

  /** resets the integral sum and the previous error of the PID controller */
  public void reset() {
    this.pidController.reset();
    this.setpoint = new TrapezoidProfile.State();
  }

  //calculations
  /**
   * calculates the output of the controller
   *
   * @param measurement the measurement of the controller (depending on the request type)
   * @param dt the time since the last calculation
   * @return the output of the controller in volts
   */
  public LogFrame.ControllerFrame calculate(Measurements.Measurement measurement, double dt) {

    var feedForwards = calculateWithOutPID(measurement, dt);

    var pidOutput = calculatePID(measurement, dt);

    return new LogFrame.ControllerFrame(feedForwards, pidOutput);
  }

  /**
   * calculates the PID of the controller
   *
   * @param measurements the measurement of the controller (depending on the request type)
   * @return the output PID of the controller in volts
   */
  private LogFrame.PIDOutput calculatePID(Measurements.Measurement measurements, double dt) {
    double measurement = switch (request.requestType){
      case VELOCITY, PROFILED_VELOCITY -> measurements.velocity();
      default -> measurements.position();
    };

    return this.pidController.calculate(measurement, this.setpoint.position, dt);
  }

  /**
   * calculates the feed forward of the controller
   *
   * @param directionOfTravel the direction of travel of the motor (used to calculate the friction
   *     feed forward)
   * @return the feed forward of the controller in volts
   */
  private FeedForwardOutput calculateFeedForward(double directionOfTravel) {
    var feedForwards = this.controllerGains.getControllerFeedForwards();

    return new FeedForwardOutput(
            feedForwards.getSimpleFeedForward(),
            feedForwards.getFrictionFeedForward() * directionOfTravel,
            feedForwards.getK_V() * setpoint.position,
            feedForwards.getCalculatedFeedForward(setpoint.position)
    );
  }

  /**
   * calculates the profile of movement
   *
   * @param dt the time since the last calculation
   * @return the new setpoint of the controller
   */
  private TrapezoidProfile.State calculateProfile(double dt) {
    if (!request.requestType.isProfiled())
      return new TrapezoidProfile.State(request.goal.position, request.goal.velocity);

    var profile = this.controllerGains.getControllerProfile();

    return profile.calculate(dt, setpoint, request.goal);
  }

  /**
   * calculates the constraints of the controller based on the measurement and the last request
   *
   * @param measurement the measurement of the controller (depending on the request type)
   */
  private void calculateConstraints(Measurements.Measurement measurement) {
    this.controllerGains.getControllerConstrains().calculateConstraints(measurement, request);
  }

  // maintenance things
  @Override
  public void initSendable(SendableBuilder builder) {
    controllerGains.initSendable(builder);

    builder.addDoubleProperty(
        "goal", () -> request.goal.position, (value) -> setReference(value, request.requestType));

    builder.addDoubleProperty(
        "setpoint", () -> setpoint.position, (value) -> setReference(value, request.requestType));
  }

  public enum RequestType {
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

  public record ControllerRequest(TrapezoidProfile.State goal, RequestType requestType) {
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
