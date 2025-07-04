package com.basicMotor.gains;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * this class is used to store the gains of the controller it is used to set the PID gains, feed
 * forwards, and constraints
 */
public class ControllerGains {
    /**
     * the gains of the PID controller this is used to set the PID gains of the motor controller
     */
    private PIDGains pidGains = new PIDGains();
    /**
     * the constraints of the controller this is used to set the constraints of the motor controller
     */
    private ControllerConstrains controllerConstrains = new ControllerConstrains();
    /**
     * the feed forwards of the controller this is used to set the feed forwards of the motor
     * controller
     */
    private ControllerFeedForwards controllerFeedForwards = new ControllerFeedForwards();

    /**
     * the constraints of the profile this is used to set the constraints of the motor controller
     */
    private TrapezoidProfile.Constraints profileConstraints =
            new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    /**
     * the profile of the controller (used for motion profiling)
     */
    private TrapezoidProfile controllerProfile = new TrapezoidProfile(profileConstraints);

    /**
     * the function that is called when the PID gains are changed this is used to update the motor
     * controller on the slower thread
     */
    private Runnable setHasPIDGainsChanged;

    private Runnable setHasConstraintsChanged;

    /**
     * the function that is called when the PID gains are changed this is used to update the motor
     * controller on the slower thread
     * @param hasPIDGainsChanged  the function that is called when the PID gains are changed
     */
    public void setHasPIDGainsChanged(Runnable hasPIDGainsChanged) {
        if (this.setHasPIDGainsChanged != null) return;
        this.setHasPIDGainsChanged = hasPIDGainsChanged;
    }

    /**
     * the function that is called when the constraints are changed this is used to update the motor
     * controller on the slower thread
     * @param hasConstraintsChanged the function that is called when the constraints are changed
     */
    public void setHasConstraintsChanged(Runnable hasConstraintsChanged) {
        if (this.setHasConstraintsChanged != null) return;
        this.setHasConstraintsChanged = hasConstraintsChanged;
    }

    /**
     * gets the PID gains of the controller
     *
     * @return the PID gains of the controller
     */
    public PIDGains getPidGains() {
        return pidGains;
    }

    /**
     * gets the constraints of the controller
     *
     * @return the constraints of the controller
     */
    public ControllerConstrains getControllerConstrains() {
        return controllerConstrains;
    }

    /**
     * gets the feed forwards of the controller
     *
     * @return the feed forwards of the controller
     */
    public ControllerFeedForwards getControllerFeedForwards() {
        return controllerFeedForwards;
    }

    /**
     * gets the constraints of the profile
     *
     * @return the constraints of the profile
     */
    public TrapezoidProfile getControllerProfile() {
        return controllerProfile;
    }

    /**
     * checks if the controller is profiled
     *
     * @return true if the controller is profiled, false if it is not
     */
    public boolean isProfiled() {
        return profileConstraints.maxVelocity != Double.POSITIVE_INFINITY
                && profileConstraints.maxAcceleration != Double.POSITIVE_INFINITY;
    }

    /**
     * sets the PID gains of the controller
     *
     * @param k_P the proportional gain
     * @param k_I the integral gain
     * @param k_D the derivative gain
     */
    public void setPidGains(double k_P, double k_I, double k_D) {
        this.pidGains = new PIDGains(k_P, k_I, k_D);
        setHasPIDGainsChanged.run();
    }

    /**
     * sets the PID gains of the controller
     *
     * @param pidGains the PID gains
     */
    public void setPidGains(PIDGains pidGains) {
        this.pidGains = pidGains;
        setHasPIDGainsChanged.run();
    }

    /**
     * sets the constraints of the controller
     *
     * @param controllerConstrains the constraints of the controller
     */
    public void setControllerConstrains(ControllerConstrains controllerConstrains) {
        this.controllerConstrains = controllerConstrains;
        setHasConstraintsChanged.run();
    }

    /**
     * sets the feed forwards of the controller
     *
     * @param controllerFeedForwards the feed forwards of the controller
     */
    public void setControllerFeedForwards(ControllerFeedForwards controllerFeedForwards) {
        this.controllerFeedForwards = controllerFeedForwards;
    }

    /**
     * sets the constraints of the profile
     *
     * @param profileConstraints the constraints of the profile
     */
    public void setControllerProfile(TrapezoidProfile.Constraints profileConstraints) {
        this.profileConstraints = profileConstraints;
        controllerProfile = new TrapezoidProfile(profileConstraints);
    }

    /**
     * this function is used to initialize the sendable builder for the controller gains. it adds the
     * PID gains, feed forwards, and motion profile constraints to the sendable builder this is used
     * when the controller gains are added to the SmartDashboard
     *
     * @param builder the sendable builder
     */
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

    /**
     * builds the sendable for the feed forwards (simpleFeedForward, frictionFeedForward,
     * setpointFeedForward)
     *
     * @param builder the sendable builder
     */
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
                "setpointFeedForward",
                controllerFeedForwards::getSetpointFeedForward,
                (value) ->
                        controllerFeedForwards.updateFeedForwards(
                                value, ControllerFeedForwards.ChangeType.SETPOINT_FEED_FORWARD));
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
    }

    private void updatePIDGains(double value, PIDGains.ChangeType changeType) {
        // cheks if the pid gains have changed
        if (pidGains.updatePIDGains(value, changeType)) {
            setHasPIDGainsChanged.run();
        }
    }

    /**
     * creates an empty controller gains object (no PID gains, no feed forwards, no constraints)
     */
    public ControllerGains() {
    }

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
     * @param pidGains             the PID gains
     * @param controllerConstrains the constraints
     */
    public ControllerGains(PIDGains pidGains, ControllerConstrains controllerConstrains) {
        this.pidGains = pidGains;
        this.controllerConstrains = controllerConstrains;
    }

    /**
     * creates a controller gains object with the given pid gains and feed forwards
     *
     * @param pidGains               the PID gains
     * @param controllerFeedForwards the feed forwards
     */
    public ControllerGains(PIDGains pidGains, ControllerFeedForwards controllerFeedForwards) {
        this.pidGains = pidGains;
        this.controllerFeedForwards = controllerFeedForwards;
    }

    /**
     * creates a controller gains object with the given pid gains, constraints and feed forwards
     *
     * @param pidGains               the PID gains
     * @param controllerConstrains   the constraints
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

    /**
     * creates a controller gains object with the given pid gains, constraints and feed forwards
     *
     * @param pidGains               the PID gains
     * @param controllerConstrains   the constraints
     * @param controllerFeedForwards the feed forwards
     * @param profileConstraints     the constraints of the profile
     */
    public ControllerGains(
            PIDGains pidGains,
            ControllerConstrains controllerConstrains,
            ControllerFeedForwards controllerFeedForwards,
            TrapezoidProfile.Constraints profileConstraints) {
        this.pidGains = pidGains;
        this.controllerConstrains = controllerConstrains;
        this.controllerFeedForwards = controllerFeedForwards;
        this.profileConstraints = profileConstraints;

        this.controllerProfile = new TrapezoidProfile(profileConstraints);
    }
}
