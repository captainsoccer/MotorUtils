package util.PIDController;

public class ControllerConstrains {
    public enum ConstraintType {
        CONTINUOUS,
        LIMITED,
        NONE
    }

    private final ConstraintType constraintType;

    private final double minValue;
    private final double maxValue;

    /**
     * creates a constrains object with the given type and limits
     * @param type type of the constrains (continuous, limited, none)
     * @param minValue the minimum value of the constrains (if continuous is the minimum value to round to,
     *                 if limited is the minimum value of the limits)
     * @param maxValue the maximum value of the constrains (if continuous is the maximum value to round to,
     *                 if limited is the maximum value of the limits)
     */
    public ControllerConstrains(ConstraintType type, double minValue, double maxValue) {
        this.constraintType = type;
        this.minValue = minValue;
        this.maxValue = maxValue;
    }

    /**
     * creates an empty constrains object (no limits and no continuity)
     */
    public ControllerConstrains() {
        this(ConstraintType.NONE, 0, 0);
    }

    public ConstraintType getConstraintType() {
        return constraintType;
    }

    public double getMinValue() {
        return minValue;
    }

    public double getMaxValue() {
        return maxValue;
    }
}
