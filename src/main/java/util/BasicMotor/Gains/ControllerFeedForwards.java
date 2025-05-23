package util.BasicMotor.Gains;

import java.util.function.Function;

/**
 * this class is used to set the feed forward gains of the motor controller.
 * it is used to calculate the feed forward output of the motor controller
 * all the feedforwards of the {@link util.BasicMotor.BasicMotor} are calculated in this class on the rio
 */
public class ControllerFeedForwards {

    /**
     * the type of change that is being made to the feed forward gains
     * this is used to determine which gain to change from the dashboard
     */
  public enum ChangeType {
    SIMPLE_FEED_FORWARD,
    FRICTION_FEED_FORWARD,
    K_V
  }

  /** \ a simple feed forward gain (adds constant voltage to the output) can also be used as k_g */
  private double simpleFeedForward;

  /**
   * a feed forward gain for the friction (adds constant voltage to the output) based on the
   * direction of the motor
   */
  private double frictionFeedForward;

  /** a feed forward gain that is multiplied by the setpoint of the controller */
  private double k_V;

  /** a feed forward that is given the setpoint and returns a value can be used as k_g for an arm */
  private final Function<Double, Double> feedForwardFunction;

  /**
   * creates a feed forward gain with the given values
   *
   * @param simpleFeedForward the simple feed forward gain
   * @param frictionFeedForward the friction feed forward gain
   * @param k_V the k_V gain
   * @param feedForwardFunction the feed forward function
   */
  public ControllerFeedForwards(
      double simpleFeedForward,
      double frictionFeedForward,
      double k_V,
      Function<Double, Double> feedForwardFunction) {
    this.simpleFeedForward = simpleFeedForward;
    this.frictionFeedForward = frictionFeedForward;
    this.k_V = k_V;
    this.feedForwardFunction = feedForwardFunction;
  }

  /**
   * creates a feed forward that is only a simple feed forward gain
   *
   * @param k_V the simple feed forward gain
   */
  public ControllerFeedForwards(double k_V) {
    this(0, 0, k_V, (x) -> 0.0);
  }

    /**
     * creates a feed forward that is only a simple feed forward gain and a friction feed forward gain
     *
     * @param k_V the simple feed forward gain
     * @param frictionFeedForward the friction feed forward gain
     */
  public ControllerFeedForwards(double k_V, double frictionFeedForward) {
    this(0, frictionFeedForward, k_V, (x) -> 0.0);
  }

    /**
     * creates a feed forward that is only a simple feed forward gain and a friction feed forward gain
     *
     * @param k_V the simple feed forward gain
     * @param simpleFeedForward the simple feed forward gain
     * @param frictionFeedForward the friction feed forward gain
     */
  public ControllerFeedForwards(double k_V, double simpleFeedForward, double frictionFeedForward) {
    this(frictionFeedForward, simpleFeedForward, k_V, (x) -> 0.0);
  }

  /**
   * creates an empty feed forward gain (no simple feed forward, no friction feed forward, no k_FF)
   */
  public ControllerFeedForwards() {
    this(0, 0, 0, (x) -> 0.0);
  }

  /**
   * gets the simple feed forward gain
   * units are volts
   * @return the simple feed forward gain
   */
  public double getSimpleFeedForward() {
    return simpleFeedForward;
  }

    /**
     * gets the friction feed forward gain
     * units are volts
     * @return the friction feed forward gain
     */
  public double getFrictionFeedForward() {
    return frictionFeedForward;
  }

    /**
     * gets the k_V gain
     * units are volts per unit of measurement
     * @return the k_V gain
     */
  public double getK_V() {
    return k_V;
  }

    /**
     * gets the feed forward function
     * units are volts per unit of measurement
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

  public void updateFeedForwards(double value, ChangeType type) {
    switch (type) {
      case SIMPLE_FEED_FORWARD:
        simpleFeedForward = value;
        break;
      case FRICTION_FEED_FORWARD:
        frictionFeedForward = value;
        break;
      case K_V:
        k_V = value;
        break;
    }
  }
}
