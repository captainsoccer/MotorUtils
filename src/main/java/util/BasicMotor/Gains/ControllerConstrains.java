package util.BasicMotor.Gains;

import edu.wpi.first.math.MathUtil;
import util.BasicMotor.Controllers.Controller;
import util.BasicMotor.Measurements.Measurements;

public class ControllerConstrains {
    /**
     * which type of constraints to use
     */
    public enum ConstraintType {
        /**
         * continuous constraints
         * this means that the controller will wrap around the limits,
         * for example, turn on a swerve module
         */
        CONTINUOUS,
        /**
         * limited constraints
         * this means that the controller will limit the output to the limits,
         * for example, an elevator with an extension limit
         */
        LIMITED,
        /**
         * no constraints
         * this means that the controller will not limit the output,
         * for example, a drive motor or a flywheel
         */
        NONE
    }

    /**
     * the type of constraints to use
     */
    private final ConstraintType constraintType;

    /**
     * the minimum value of the constraints
     * this is used for limited and continuous constraints
     */
    private final double minValue;

    /**
     * the maximum value of the constraints
     * this is used for limited and continuous constraints
     */
    private final double maxValue;

    /**
     * creates a constrains object with the given type and limits
     *
     * @param type     type of the constrains (continuous, limited, none)
     * @param minValue the minimum value of the constrains (if continuous is the minimum value to
     *                 round to, if limited is the minimum value of the limits)
     * @param maxValue the maximum value of the constrains (if continuous is the maximum value to
     *                 round to, if limited is the maximum value of the limits)
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

    /**
     * calculates the constraints of the controller
     * checks if the request is in the limits of the motor or needs to be wrapped
     * @param measurement the measurement of the motor
     * @param request the request of the controller
     */
    public void calculateConstraints(Measurements.Measurement measurement, Controller.ControllerRequest request) {
        if (constraintType == ConstraintType.NONE) return;

        if (constraintType == ConstraintType.LIMITED) {
            if (request.requestType().isPositionControl()) {
                if (request.goal().position >= maxValue) {
                    request.goal().position = maxValue;
                    request.goal().velocity = 0;
                }
                if (request.goal().position <= minValue) {
                    request.goal().position = minValue;
                    request.goal().velocity = 0;
                }

                return;
            }

            if (measurement.position() <= minValue && request.goal().position < 0) {
                request.goal().position = 0;
                request.goal().velocity = 0;
            }
            if (measurement.position() >= maxValue && request.goal().position > 0) {
                request.goal().position = 0;
                request.goal().velocity = 0;
            }
        } else {
            if (!request.requestType().isPositionControl()) return;

            double errorBound = (maxValue - minValue) / 2.0;

            request.goal().position =
                    MathUtil.inputModulus(request.goal().position - measurement.position(), -errorBound, errorBound)
                            + measurement.position();
        }
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
