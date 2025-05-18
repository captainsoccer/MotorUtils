package util.BasicMotor.Gains;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;

public class ControllerGains {

  private PIDGains pidGains = new PIDGains();
  private ControllerConstrains controllerConstrains = new ControllerConstrains();
  private ControllerFeedForwards controllerFeedForwards = new ControllerFeedForwards();

  private TrapezoidProfile.Constraints profileConstraints =
      new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

  private TrapezoidProfile controllerProfile = new TrapezoidProfile(profileConstraints);

  private Runnable setHasPIDGainsChanged;

  public void setHasPIDGainsChanged(Runnable hasPIDGainsChanged) {
    if (this.setHasPIDGainsChanged != null) return;
    this.setHasPIDGainsChanged = hasPIDGainsChanged;
  }

  public PIDGains getPidGains() {
    return pidGains;
  }

  public ControllerConstrains getControllerConstrains() {
    return controllerConstrains;
  }

  public ControllerFeedForwards getControllerFeedForwards() {
    return controllerFeedForwards;
  }

  public TrapezoidProfile getControllerProfile() {
    return controllerProfile;
  }

  public boolean isProfiled() {
    return profileConstraints.maxAcceleration != Double.POSITIVE_INFINITY
        && profileConstraints.maxVelocity != Double.POSITIVE_INFINITY;
  }

  public void setPidGains(double k_P, double k_I, double k_D) {
    this.pidGains = new PIDGains(k_P, k_I, k_D);
  }

  public void setPidGains(PIDGains pidGains) {
    this.pidGains = pidGains;
  }

  public void setControllerConstrains(ControllerConstrains controllerConstrains) {
    this.controllerConstrains = controllerConstrains;
  }

  public void setControllerFeedForwards(ControllerFeedForwards controllerFeedForwards) {
    this.controllerFeedForwards = controllerFeedForwards;
  }

  public void initSendable(SendableBuilder builder) {
    if (isProfiled()) {
      builder.setSmartDashboardType("ProfiledPIDController");

      builder.addDoubleProperty(
          "maxVelocity",
          () -> profileConstraints.maxVelocity,
          (x) -> {
            profileConstraints =
                new TrapezoidProfile.Constraints(x, profileConstraints.maxAcceleration);
            controllerProfile = new TrapezoidProfile(profileConstraints);
          });

      builder.addDoubleProperty(
          "maxAcceleration",
          () -> profileConstraints.maxAcceleration,
          (x) -> {
            profileConstraints =
                new TrapezoidProfile.Constraints(profileConstraints.maxVelocity, x);

            controllerProfile = new TrapezoidProfile(profileConstraints);
          });
    } else builder.setSmartDashboardType("PIDController");

    buildPIDSendable(builder);
    buildFeedForwardSendable(builder);
  }

  private void buildFeedForwardSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "simpleFeedForward",
        controllerFeedForwards::getSimpleFeedForward,
        (value) ->
            controllerFeedForwards.updateFeedForwards(
                value, ControllerFeedForwards.ChangeType.SIMPLE_FEED_FORWARD));

    builder.addDoubleProperty(
        "frictionFeedForward",
        controllerFeedForwards::getFrictionFeedForward,
        (value) ->
            controllerFeedForwards.updateFeedForwards(
                value, ControllerFeedForwards.ChangeType.FRICTION_FEED_FORWARD));

    builder.addDoubleProperty(
        "k_V",
        controllerFeedForwards::getK_V,
        (value) ->
            controllerFeedForwards.updateFeedForwards(
                value, ControllerFeedForwards.ChangeType.K_V));
  }

  /**
   * builds the sendable for the PID gains (k_P, k_I, k_D, i_Zone, i_MaxAccum, tolerance, maxOutput,
   * minOutput)
   *
   * @param builder the sendable builder
   */
  private void buildPIDSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "p", pidGains::getK_P, (value) -> updatePIDGains(value, PIDGains.ChangeType.K_P));

    builder.addDoubleProperty(
        "i", pidGains::getK_I, (value) -> updatePIDGains(value, PIDGains.ChangeType.K_I));

    builder.addDoubleProperty(
        "d", pidGains::getK_D, (value) -> updatePIDGains(value, PIDGains.ChangeType.K_D));

    builder.addDoubleProperty(
        "izone", pidGains::getI_Zone, (value) -> updatePIDGains(value, PIDGains.ChangeType.I_ZONE));

    builder.addDoubleProperty(
        "iMaxAccum",
        pidGains::getI_MaxAccum,
        (value) -> updatePIDGains(value, PIDGains.ChangeType.I_MAX_ACCUM));

    builder.addDoubleProperty(
        "tolerance",
        pidGains::getTolerance,
        (value) -> updatePIDGains(value, PIDGains.ChangeType.TOLERANCE));

    builder.addDoubleProperty(
        "maxOutput",
        pidGains::getMaxOutput,
        (value) -> updatePIDGains(value, PIDGains.ChangeType.MAX_OUTPUT));

    builder.addDoubleProperty(
        "minOutput",
        pidGains::getMinOutput,
        (value) -> updatePIDGains(value, PIDGains.ChangeType.MIN_OUTPUT));
  }

  private void updatePIDGains(double value, PIDGains.ChangeType changeType) {
    if (pidGains.updatePIDGains(value, changeType)) {
      setHasPIDGainsChanged.run();
    }
  }

  /** creates an empty controller gains object (no PID gains, no feed forwards, no constraints) */
  public ControllerGains() {}

  /**
   * creates a controller gains object with the given PID gains
   *
   * @param k_P the proportional gain
   * @param k_I the integral gain
   * @param k_D the derivative gain
   */
  public ControllerGains(double k_P, double k_I, double k_D) {
    pidGains = new PIDGains(k_P, k_I, k_D);
  }

  /**
   * creates a controller gains object with the given PID gains
   *
   * @param pidGains the PID gains
   */
  public ControllerGains(PIDGains pidGains) {
    this.pidGains = pidGains;
  }

  /**
   * creates a controller gains object with the given feed forwards
   *
   * @param controllerFeedForwards the feed forwards
   */
  public ControllerGains(ControllerFeedForwards controllerFeedForwards) {
    this.controllerFeedForwards = controllerFeedForwards;
  }

  /**
   * creates a controller gains object with the given pid gains and constraints
   *
   * @param pidGains the PID gains
   * @param controllerConstrains the constraints
   */
  public ControllerGains(PIDGains pidGains, ControllerConstrains controllerConstrains) {
    this.pidGains = pidGains;
    this.controllerConstrains = controllerConstrains;
  }

  /**
   * creates a controller gains object with the given pid gains and feed forwards
   *
   * @param pidGains the PID gains
   * @param controllerFeedForwards the feed forwards
   */
  public ControllerGains(PIDGains pidGains, ControllerFeedForwards controllerFeedForwards) {
    this.pidGains = pidGains;
    this.controllerFeedForwards = controllerFeedForwards;
  }

  /**
   * creates a controller gains object with the given pid gains, constraints and feed forwards
   *
   * @param pidGains the PID gains
   * @param controllerConstrains the constraints
   * @param controllerFeedForwards the feed forwards
   */
  public ControllerGains(
      PIDGains pidGains,
      ControllerConstrains controllerConstrains,
      ControllerFeedForwards controllerFeedForwards) {
    this.pidGains = pidGains;
    this.controllerConstrains = controllerConstrains;
    this.controllerFeedForwards = controllerFeedForwards;
  }
}
