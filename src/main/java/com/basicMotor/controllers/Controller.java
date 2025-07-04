package com.basicMotor.controllers;

import com.basicMotor.BasicMotor;
import com.basicMotor.gains.ControllerGains;
import com.basicMotor.LogFrame;
import com.basicMotor.LogFrame.FeedForwardOutput;
import com.basicMotor.measurements.Measurements;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import java.util.Objects;

/**
 * this is the controller used to control the basic motor class {@link BasicMotor} this handles pid
 * (if needed), feedforward, constraints and profile of the motor
 */
public class Controller implements Sendable {
  private static int instances = 0;

  /** the gains of the controller pid, feedforward, constraints and profile */
  private final ControllerGains controllerGains;

  /**
   * the controller of the controller this is used to calculate the output of the controller pid,
   * feedforward, constraints and profile
   */
  private final BasicPIDController pidController;

  /** the latest request of the controller this contains the control mode and the goal */
  private ControllerRequest request = new ControllerRequest();
  /** the setpoint of the controller this is used when using profiled position and velocity */
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  /**
   * creates a controller with the given gains
   *
   * @param controllerGains the gains of the controller
   * @param hasPIDGainsChangeRunnable the runnable that is called when the PID gains are changed
   */
  public Controller(
      ControllerGains controllerGains,
      Runnable hasPIDGainsChangeRunnable,
      Runnable hasConstraintsChangeRunnable) {
    this.controllerGains = controllerGains;
    this.controllerGains.setHasPIDGainsChanged(hasPIDGainsChangeRunnable);
    this.controllerGains.setHasConstraintsChanged(hasConstraintsChangeRunnable);

    this.pidController = new BasicPIDController(controllerGains.getPidGains());

    instances++;

    SendableRegistry.add(this, "MotorController", instances);
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
    Objects.requireNonNull(request);
    Objects.requireNonNull(request.requestType);
    Objects.requireNonNull(request.goal);

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
   * gets the current setpoint of the controller
   *
   * @return the current setpoint of the controller
   */
  public TrapezoidProfile.State getSetpoint() {
    return setpoint;
  }

  /**
   * gets the current request type of the controller
   *
   * @return the current request type of the controller
   */
  public RequestType getRequestType() {
    return request.requestType;
  }

  /**
   * gets the current request of the controller
   *
   * @return the current request of the controller
   */
  public ControllerRequest getRequest() {
    return request;
  }

  /**
   * resets the integral sum and the previous error of the PID controller and sets the setpoint to
   * the current position for profiling purposes (will be set to correct in the next loop)
   */
  public void reset(double measurement, double measurementVelocity) {
    this.pidController.reset();
    this.setpoint = new TrapezoidProfile.State(measurement, measurementVelocity);
  }

  // calculations
  /**
   * calculates the PID of the controller
   *
   * @param measurement the measurement of the controller (depending on the request type)
   * @return the output PID of the controller in volts
   */
  public LogFrame.PIDOutput calculatePID(double measurement, double dt) {
    return this.pidController.calculate(this.setpoint.position, measurement, dt);
  }

  /** sets the setpoint to the goal used when not using a profiled control */
  public void setSetpointToGoal() {
    this.setpoint = request.goal;
  }

  /**
   * calculates the feed forward of the controller
   *
   * @param directionOfTravel the direction of travel of the motor (used to calculate the friction
   *     feed forward)
   * @return the feed forward of the controller in volts
   */
  public FeedForwardOutput calculateFeedForward(double directionOfTravel) {
    return controllerGains
        .getControllerFeedForwards()
        .calculateFeedForwardOutput(
            this.setpoint.position, directionOfTravel, request.arbFeedForward);
  }

  /**
   * calculates the profile of movement and sets the setpoint to the calculated profile
   *
   * @param dt the time since the last calculation
   */
  public void calculateProfile(double dt) {
    var profile = this.controllerGains.getControllerProfile();

    setpoint = profile.calculate(dt, setpoint, request.goal);
  }

  /**
   * checks the motor output of the controller (if it is above or below the maximum output of the
   * motor or is below the minimum output of the motor)
   *
   * @param output the output of the controller in volts
   * @return the checked output of the controller in volts
   */
  public double checkMotorOutput(double output) {
    return controllerGains.getControllerConstrains().checkMotorOutput(output);
  }

  /**
   * calculates the constraints of the controller based on the measurement and the last request
   *
   * @param measurement the measurement of the controller (depending on the request type)
   */
  public void calculateConstraints(
      Measurements.Measurement measurement, ControllerRequest request) {
    this.controllerGains.getControllerConstrains().calculateConstraints(measurement, request);
  }

  // maintenance things
  @Override
  public void initSendable(SendableBuilder builder) {
    controllerGains.initSendable(builder);

    builder.addDoubleProperty(
        "setpoint", () -> setpoint.position, (value) -> setReference(value, request.requestType));
  }

  /** the type of the mode to control the motor */
  public enum RequestType {
    /** stops the motor */
    STOP,
    /** controls the motor with a voltage output */
    VOLTAGE,
    /** controls the motor with a percent output (duty cycle) (-1 to 1) */
    PRECENT_OUTPUT,
    /** controls the motor with a position output (needs a pid controller) */
    POSITION,
    /** controls the motor with a profiled position output (needs a pid controller and a profile) */
    PROFILED_POSITION,
    /**
     * controls the motor with a velocity output (needs a pid controller and recommended to use
     * feedforward)
     */
    VELOCITY,
    /** controls the motor with a profiled velocity output (needs a pid controller and a profile) */
    PROFILED_VELOCITY;

    /**
     * checks if the request type is a position control
     *
     * @return true if the request type is a position control
     */
    public boolean isPositionControl() {
      return this == POSITION || this == PROFILED_POSITION;
    }

    /**
     * checks if the request type is a velocity control
     *
     * @return true if the request type is a velocity control
     */
    public boolean isVelocityControl() {
      return this == VELOCITY || this == PROFILED_VELOCITY;
    }

    /**
     * checks if the request type is a profiled control
     *
     * @return true if the request type is a profiled control
     */
    public boolean isProfiled() {
      return this == PROFILED_POSITION || this == PROFILED_VELOCITY;
    }

    /**
     * checks if the request type requires a pid controller
     *
     * @return true if the request type requires a pid controller
     */
    public boolean requiresPID() {
      return this != VOLTAGE && this != PRECENT_OUTPUT && this != STOP;
    }
  }

  /**
   * the request of the controller this contains the control mode and the goal
   *
   * @param goal the goal of the controller
   * @param requestType the request type of the controller
   * @param arbFeedForward a voltage feedforward given by the user that will be added to the motor
   *     output
   */
  public record ControllerRequest(
      TrapezoidProfile.State goal, RequestType requestType, double arbFeedForward) {
    /**
     * creates a controller request with a goal and a mode
     *
     * @param goal the goal
     * @param requestType the mode
     */
    public ControllerRequest(double goal, RequestType requestType) {
      this(new TrapezoidProfile.State(goal, 0), requestType, 0);
    }

    public ControllerRequest(TrapezoidProfile.State goal, RequestType requestType) {
      this(goal, requestType, 0);
    }

    public ControllerRequest(double goal, double setpointVelocity, RequestType requestType) {
      this(new TrapezoidProfile.State(goal, setpointVelocity), requestType, 0);
    }

    public ControllerRequest() {
      this(new TrapezoidProfile.State(), RequestType.STOP, 0);
    }

    public ControllerRequest(double goal, RequestType requestType, double arbFeedForward) {
      this(new TrapezoidProfile.State(goal, 0), requestType, arbFeedForward);
    }
  }
}
