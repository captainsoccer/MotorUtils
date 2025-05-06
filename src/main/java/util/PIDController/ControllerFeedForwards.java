package util.PIDController;


import java.util.function.Function;

public class ControllerFeedForwards {
    /**\
     * a simple feed forward gain (adds constant voltage to the output)
     * can also be used as k_g
     */
    private final double simpleFeedForward;

    /**
     * a feed forward gain for the friction (adds constant voltage to the output) based on the direction of the motor
     */
    private final double frictionFeedForward;

    /**
     * a feed forward gain that is multiplied by the setpoint of the controller
     */
    private final double k_V;

    /**
     * a feed forward that is given the setpoint and returns a value
     * can be used as k_g for an arm
     */
    private final Function<Double, Double> feedForwardFunction;

    /**
     * creates a feed forward gain with the given values
     * @param simpleFeedForward the simple feed forward gain
     * @param frictionFeedForward the friction feed forward gain
     * @param k_V the k_V gain
     * @param feedForwardFunction the feed forward function
     */
    public ControllerFeedForwards(double simpleFeedForward, double frictionFeedForward, double k_V, Function<Double, Double> feedForwardFunction) {
        this.simpleFeedForward = simpleFeedForward;
        this.frictionFeedForward = frictionFeedForward;
        this.k_V = k_V;
        this.feedForwardFunction = feedForwardFunction;
    }

    /**
     * creates a feed forward that is only a simple feed forward gain
     * @param k_V the simple feed forward gain
     */
    public ControllerFeedForwards(double k_V) {
        this(0, 0, k_V, (x) -> 0.0);
    }

    public ControllerFeedForwards(double k_V, double frictionFeedForward) {
        this(0, frictionFeedForward, k_V, (x) -> 0.0);
    }

    public ControllerFeedForwards(double k_V, double simpleFeedForward, double frictionFeedForward) {
        this(frictionFeedForward, simpleFeedForward, k_V, (x) -> 0.0);
    }

    /**
     * creates an empty feed forward gain (no simple feed forward, no friction feed forward, no k_FF)
     */
    public ControllerFeedForwards(){
        this(0, 0, 0, (x) -> 0.0);
    }

    public double getSimpleFeedForward() {
        return simpleFeedForward;
    }

    public double getFrictionFeedForward() {
        return frictionFeedForward;
    }

    public double getK_V() {
        return k_V;
    }

    public Function<Double, Double> getFeedForwardFunction() {
        return feedForwardFunction;
    }

    /**
     * calculates the feed forward gain based on the setpoint
     * @param setpoint the setpoint of the controller
     * @return the feed forward gain
     */
    public double getCalculatedFeedForward(double setpoint) {
        return feedForwardFunction.apply(setpoint);
    }
}
