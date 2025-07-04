package com.basicMotor.gains;

import com.basicMotor.BasicMotor;
import com.basicMotor.LogFrame;
import java.util.function.Function;

/**
 * this class is used to set the feed forward gains of the motor controller. it is used to calculate
 * the feed forward output of the motor controller all the feedforwards of the {@link BasicMotor}
 * are calculated in this class on the rio
 */
public class ControllerFeedForwards {

  /**
   * the type of change that is being made to the feed forward gains this is used to determine which
   * gain to change from the dashboard
   */
  public enum ChangeType {
    SIMPLE_FEED_FORWARD,
    FRICTION_FEED_FORWARD,
    SETPOINT_FEED_FORWARD
  }

  /** a simple feed forward gain (adds constant voltage to the output) can also be used as k_g */
  private double simpleFeedForward;

  /**
   * a feed forward gain for the friction (adds constant voltage to the output) based on the
   * direction of the motor
   */
  private double frictionFeedForward;

  /** a feed forward gain that is multiplied by the setpoint of the controller */
  private double setpointFeedForward;

  /**
   * a feed forward that is given the setpoint and returns a value - for example, it can be used as
   * a k_g for an arm
   */
  private final Function<Double, Double> feedForwardFunction;

  /**
   * creates a feed forward gain with the given values
   *
   * @param simpleFeedForward the simple feed forward gain
   * @param frictionFeedForward the friction feed forward gain
   * @param setpointFeedForward the setpoint feed forward gain
   * @param feedForwardFunction the feed forward function
   */
  public ControllerFeedForwards(
      double simpleFeedForward,
      double frictionFeedForward,
      double setpointFeedForward,
      Function<Double, Double> feedForwardFunction) {

    if (simpleFeedForward < 0)
      throw new IllegalArgumentException("simpleFeedForward must be greater than or equal to zero");
    this.simpleFeedForward = simpleFeedForward;

    if (frictionFeedForward < 0)
      throw new IllegalArgumentException(
          "frictionFeedForward must be greater than or equal to zero");
    this.frictionFeedForward = frictionFeedForward;

    if (setpointFeedForward < 0)
      throw new IllegalArgumentException(
          "setpointFeedForward must be greater than or equal to zero");
    this.setpointFeedForward = setpointFeedForward;

    this.feedForwardFunction = feedForwardFunction;
  }

  /**
   * creates a feed forward that is only a setpoint feed forward gain used, for example, in a
   * flywheel controller
   *
   * @param setpointFeedForward the setpoint feed forward gain
   */
  public ControllerFeedForwards(double setpointFeedForward) {
    this(0, 0, setpointFeedForward, (x) -> 0.0);
  }

  /**
   * creates a feed forward that is only a setpoint feed forward gain and a friction feed forward
   * gain
   *
   * @param setpointFeedForward the setpoint feed forward gain
   * @param frictionFeedForward the friction feed forward gain
   */
  public ControllerFeedForwards(double setpointFeedForward, double frictionFeedForward) {
    this(0, frictionFeedForward, setpointFeedForward, (x) -> 0.0);
  }

  /**
   * creates a feed forward that does not use a feed forward function
   *
   * @param setpointFeedForward the setpoint feed forward gain
   * @param simpleFeedForward the simple feed forward gain
   * @param frictionFeedForward the friction feed forward gain
   */
  public ControllerFeedForwards(
      double setpointFeedForward, double simpleFeedForward, double frictionFeedForward) {
    this(frictionFeedForward, simpleFeedForward, setpointFeedForward, (x) -> 0.0);
  }

  /**
   * creates an empty feed forward gain (no simple feed forward, no friction feed forward, no
   * setpoint feed forward)
   */
  public ControllerFeedForwards() {
    this(0, 0, 0, (x) -> 0.0);
  }

  /**
   * gets the simple feed forward gain units are volts
   *
   * @return the simple feed forward gain
   */
  public double getSimpleFeedForward() {
    return simpleFeedForward;
  }

  /**
   * gets the friction feed forward gain units are volts
   *
   * @return the friction feed forward gain
   */
  public double getFrictionFeedForward() {
    return frictionFeedForward;
  }

  /**
   * gets the setpoint feed forward gain units are volts per unit of measurement
   *
   * @return the setpoint feed forward gain
   */
  public double getSetpointFeedForward() {
    return setpointFeedForward;
  }

  /**
   * gets the feed forward function units are volts per unit of measurement
   *
   * @return the feed forward function
   */
  public Function<Double, Double> getFeedForwardFunction() {
    return feedForwardFunction;
  }

  /**
   * calculates the feed forward gain based on the setpoint
   *
   * @param setpoint the setpoint of the controller
   * @return the feed forward gain
   */
  public double getCalculatedFeedForward(double setpoint) {
    return feedForwardFunction.apply(setpoint);
  }

  /**
   * calculates the feed forward output of the controller based on the setpoint, direction of
   * travel, and arbitrary feed forward
   *
   * @param setpoint the setpoint of the controller
   * @param directionOfTravel the direction of travel of the motor (1 for forward, -1 for reverse)
   * @param arbitraryFeedForward the arbitrary feed forward value to add to the output
   * @return the feed forward output of the controller
   */
  public LogFrame.FeedForwardOutput calculateFeedForwardOutput(
      double setpoint, double directionOfTravel, double arbitraryFeedForward) {
    return new LogFrame.FeedForwardOutput(
        simpleFeedForward,
        frictionFeedForward * directionOfTravel,
        setpointFeedForward * setpoint,
        getCalculatedFeedForward(setpoint),
        arbitraryFeedForward);
  }

  public void updateFeedForwards(double value, ChangeType type) {
    switch (type) {
      case SIMPLE_FEED_FORWARD:
        simpleFeedForward = value;
        break;
      case FRICTION_FEED_FORWARD:
        frictionFeedForward = value;
        break;
      case SETPOINT_FEED_FORWARD:
        setpointFeedForward = value;
        break;
    }
  }
}
