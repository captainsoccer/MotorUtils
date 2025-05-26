package util.BasicMotor.Configuration;

import util.BasicMotor.Gains.*;
import util.BasicMotor.MotorManager;

import java.util.function.Function;

/**
 * this class stores all the data on the motor controller. it is used to configure the motor
 * it is a way to pack all the data into one class and make it easier to pass around
 * this class is modeled after the TalonFX configuration
 * <p>
 * for any other motor controller, use the appropriate configuration class
 */
public class BasicMotorConfig {

    /**
     * the basic motor configuration (minimum configuration required to run the motor)
     */
    public MotorConfig motorConfig = new MotorConfig();
    /**
     * the PID configuration of the motor controller
     * <p>
     * this is used to set the PID gains of the motor controller
     */
    public PIDConfig pidConfig = new PIDConfig();
    /**
     * the feed forward configuration of the motor controller
     * <p>
     * this is used to set the feed forward gains of the motor controller
     */
    public FeedForwardConfig feedForwardConfig = new FeedForwardConfig();
    /**
     * the constraints configuration of the motor controller
     * <p>
     * this is used to set the constraints of the motor controller
     */
    public ConstraintsConfig constraintsConfig = new ConstraintsConfig();
    /**
     * the current limit configuration of the motor controller
     * <p>
     * this is used to set the current limits of the motor controller
     */
    public CurrentLimitConfig currentLimitConfig = new CurrentLimitConfig();

    /**
     * bunches up all the configurations into one class
     * @return the controller gains which include PID gains, constraints, and feed forwards
     */
    public ControllerGains getControllerGains() {
        return new ControllerGains(pidConfig.getGains(), constraintsConfig.getConstraints(), feedForwardConfig.getFeedForwards());
    }

    /**
     * the basic motor configuration
     */
    public static class MotorConfig {
        /**
         * the id of the motor controller
         * <p>
         * this is used to identify the motor controller
         */
        public int id = 0;
        /**
         * the name of the motor controller
         * <p>
         * this is used to identify the motor controller in the logger
         */
        public String name = "";
        /**
         * the gear ratio of the motor controller
         * <p>
         * this is used to convert the motor output to the desired output
         */
        public double gearRatio = 1;
        /**
         * the location of the motor controller
         * <p>
         * this decides if the pid controller is on the motor controller or on the rio
         * if it's on the motor controller, it will result in lower can bus usage and faster response
         * but if it's on the rio, all the features will be available and can use any remote sensor directly for feedback
         */
        public MotorManager.ControllerLocation location = MotorManager.ControllerLocation.MOTOR;
    }

    /**
     * the PID configuration of the motor controller
     * <p>
     * this is used to set the PID gains of the motor controller
     */
    public static class PIDConfig {
        /**
         * the kp gain of the PID controller units are: (voltage / unit of measurement)
         */
        public double kP = 0;
        /**
         * the ki gain of the PID controller units are: (voltage * second / unit of measurement)
         */
        public double kI = 0;
        /**
         * the kd gain of the PID controller units are: (voltage / unit of measurement * second)
         */
        public double kD = 0;
        /**
         * the iZone of the PID controller units are: (unit of measurement)
         * <p>
         * this is the zone in which the integral term is applied, outside of this zone the integral term will be zeroed
         */
        public double iZone = 0;
        /**
         * the maximum accumulation of the integral term of the PID controller units are: (voltage * second / unit of measurement)
         * <p>
         * this is the maximum value that the integral term can accumulate to, if it exceeds this value, it will be clamped
         */
        public double iMaxAccum = 0;
        /**
         * the tolerance of the PID controller units are: (unit of measurement)
         * <p>
         * this is the tolerance of the PID controller, if the error is within this tolerance,
         * the controller will consider it done and not apply any output (not including feed forward)
         */
        public double tolerance = 0;

        /**
         * gets the PID gains of the controller
         * <p>
         * this is used to get the PID gains of the controller
         *
         * @return the PID gains of the controller
         */
        public PIDGains getGains() {
            return new PIDGains(kP, kI, kD, iZone, iMaxAccum, tolerance);
        }
    }

    /**
     * the feed forward configuration of the motor controller
     * <p>
     * this is used to set the feed forward gains of the motor controller
     */
    public static class FeedForwardConfig {
        /**
         * the friction feed forward gain of the motor controller units are: (voltage)
         * <p>
         * this is used to compensate for the friction in the motor,
         * it will be applied to the output in the direction of travel
         */
        public double frictionFF = 0;
        /**
         * the k_V feed forward gain of the motor controller units are: (voltage / unit of measurement)
         * <p>
         * this is usually used in velocity control, but it is just a feed forward gain that is multiplied by the setpoint
         */
        public double k_V = 0;
        /**
         * the simple feed forward gain of the motor controller units are: (voltage)
         * <p>
         * it's just a constant voltage applied to the output used, for example, in elevators
         */
        public double simpleFF = 0;
        /**
         * a custom feed forward function that takes the setpoint and returns a value
         * <p>
         * this can be used for more complex feed forwards, like an arm with gravity compensation
         */
        public Function<Double, Double> customFF = (setpoint) -> 0.0;

        /**
         * gets the feed forwards of the controller
         * <p>
         * this is used to get the feed forwards of the controller
         *
         * @return the feed forwards of the controller
         */
        public ControllerFeedForwards getFeedForwards() {
            return new ControllerFeedForwards(
                    simpleFF, frictionFF, k_V, customFF);
        }
    }

    /**
     * the constraints configuration of the motor controller
     * <p>
     * this is used to set the constraints of the motor controller
     */
    public static class ConstraintsConfig {
        /**
         * the type of constraint to apply to the controller
         * <p>
         * this can be NONE, LIMITED, or CONTINUOUS <p>
         * * NONE means no constraints are applied <p>
         * * LIMITED means the output is clamped to the max and min values,
         * for example, an arm that can only move between 0 and 90 degrees <p>
         *
         * * CONTINUOUS means the output is continuous and wraps around,
         * for example, a wheel that can spin indefinitely or a swerve module <p>
         */
        public ControllerConstrains.ConstraintType constraintType = ControllerConstrains.ConstraintType.NONE;
        /**
         * the maximum value of the constraint units are: (unit of measurement)
         * <p>
         * this represents the maximum value of the constraint: <p>
         *  * for a limited constraint, this is the maximum value the motor position can reach <p>
         *  * for a continuous constraint,
         *  this is the maximum value the position can reach before wrapping around (for example, 360) <p>
         */
        public double maxValue = 0;
        /**
         * the minimum value of the constraint units are: (unit of measurement)
         * <p>
         * this represents the minimum value of the constraint: <p>
         *  * for a limited constraint, this is the minimum value the motor position can reach <p>
         *  * for a continuous constraint,
         *  this is the minimum value the position can reach before wrapping around (for example, 0) <p>
         */
        public double minValue = 0;
        /**
         * the maximum output of the constraint units are: (voltage)
         * <p>
         * this is the maximum output of the motor controller in the forward direction
         */
        public double maxOutput = MotorManager.defaultMaxMotorOutput;
        /**
         * the minimum output of the constraint units are: (voltage)
         * <p>
         * this is the minimum output of the motor controller in the reverse direction
         */
        public double minOutput = -MotorManager.defaultMaxMotorOutput;
        /**
         * the voltage deadband of the constraint units are: (voltage)
         * <p>
         * this is the deadband of the motor controller, if the output is within this range,
         * the motor will not apply any output to prevent jittering and damage to the motor
         */
        public double voltageDeadband = 0;

        /**
         * gets the constraints of the controller
         * <p>
         * this is used to get the constraints of the controller
         *
         * @return the constraints of the controller
         */
        public ControllerConstrains getConstraints() {
            return new ControllerConstrains(
                    constraintType, maxValue, minValue, maxOutput, minOutput, voltageDeadband);
        }
    }

    public static class CurrentLimitConfig {
        /**
         * the maximum current output of the motor controller (in amps)
         * <p>
         * this is the max current the motor windings will allow before stopping,
         * this will usually be higher than the supply current limit by a wide margin.
         */
        public int statorCurrentLimit = 0;
        /**
         * the maximum current draw of the motor controller (in amps)
         * <p>
         * this is the max current the motor controller will draw from the battery before stopping or
         * lowering to the {@link #lowerCurrentLimit} after the {@link #lowerLimitTime} has passed.
         * this is useful if the robot is experiencing brownouts or breakers are tripping.
         */
        public int supplyCurrentLimit = 0;
        /**
         * the time (in seconds) that the motor can stay at supply current limit before it lowers to
         * {@link #lowerCurrentLimit} this is useful for bursts of current needed for a short time
         */
        public double lowerLimitTime = 0;
        /**
         * the current the motor drops to after the supply current limit is reached for
         * {@link #lowerLimitTime} this is useful for bursts of current
         * <p>
         * this is usually lower than the supply current limit and is used to prevent overheating
         */
        public int lowerCurrentLimit = 0;

        /**
         * creates the current limit configuration with the given values
         * @return the current limits of the motor controller
         */
        public CurrentLimits getCurrentLimits() {
            return new CurrentLimits(
                    statorCurrentLimit, supplyCurrentLimit, lowerLimitTime, lowerCurrentLimit);
        }
    }
}
