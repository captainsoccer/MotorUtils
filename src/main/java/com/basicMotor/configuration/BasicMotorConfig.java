package com.basicMotor.configuration;

import com.basicMotor.BasicMotor;
import com.basicMotor.gains.ControllerConstrains;
import com.basicMotor.gains.ControllerFeedForwards;
import com.basicMotor.gains.ControllerGains;
import com.basicMotor.gains.PIDGains;
import com.basicMotor.MotorManager;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.function.Function;

/**
 * this class stores all the data on the motor controller. it is used to configure the motor it is a
 * way to pack all the data into one class and make it easier to pass around. if you want to control
 * current limits or other advanced features, use the motor controller's specific configuration
 * class
 *
 * <p>
 */
public class BasicMotorConfig {

  /** the basic motor configuration (minimum configuration required to run the motor) */
  public MotorConfig motorConfig = new MotorConfig();
  /**
   * the PID configuration of the motor controller
   *
   * <p>this is used to set the PID gains of the motor controller
   */
  public PIDConfig pidConfig = new PIDConfig();
  /**
   * the feed forward configuration of the motor controller
   *
   * <p>this is used to set the feed forward gains of the motor controller
   */
  public FeedForwardConfig feedForwardConfig = new FeedForwardConfig();
  /**
   * the constraints configuration of the motor controller
   *
   * <p>this is used to set the constraints of the motor controller
   */
  public ConstraintsConfig constraintsConfig = new ConstraintsConfig();

  /**
   * the profile configuration of the motor controller
   *
   * <p>this is used to set the motion profile of the motor controller
   */
  public ProfileConfig profileConfig = new ProfileConfig();

  /**
   * the simulation configuration of the motor controller
   *
   * <p>this is used to set the simulation gains of the motor controller
   */
  public SimulationConfig simulationConfig = new SimulationConfig();

  /**
   * bunches up all the configurations into one class
   *
   * @return the controller gains which include PID gains, constraints, and feed forwards
   */
  public ControllerGains getControllerGains() {
    return new ControllerGains(
        pidConfig.getGains(),
        constraintsConfig.getConstraints(),
        feedForwardConfig.getFeedForwards(),
        profileConfig.getProfileConstraints());
  }

  public boolean usingExternalEncoder() {
    return false;
  }

  /** the basic motor configuration */
  public static class MotorConfig {
    /**
     * the id of the motor controller
     *
     * <p>this is used to identify the motor controller
     */
    public int id = 0;
    /**
     * the name of the motor controller
     *
     * <p>this is used to identify the motor controller in the logger
     */
    public String name = "motor";
    /**
     * the gear ratio of the motor controller
     *
     * <p>this is used to convert the motor output to the desired output a number greater than 1
     * means the motor is geared down (a mechanism spins slower than the motor)
     */
    public double gearRatio = 1;

    /**
     * the value that will be multiplied by to convert the measurements to the desired units. the
     * default units of the motor are rotations. (rotations, rotations per second, rotations per
     * second squared)
     *
     * <p>you can convert to the following units easily:
     *
     * <p>* - meters: multiply by the circumference of the wheel (2 * pi * radius)
     *
     * <p>* - degrees: multiply by 360 (1 rotation = 360 degrees)
     *
     * <p>* - radians: multiply by 2 * pi (1 rotation = 2 * pi radians)
     */
    public double unitConversion = 1;

    /**
     * if the motor controller is inverted, true means clockwise is positive false means
     * counter-clockwise is positive
     */
    public boolean inverted = false;

    /**
     * the idle mode of the motor controller COAST means the motor will not try to hold its position
     * when not powered BRAKE means the motor will try to hold its position when not powered
     */
    public BasicMotor.IdleMode idleMode = BasicMotor.IdleMode.COAST;
    /**
     * the location of the motor controller
     *
     * <p>this decides if the pid controller is on the motor controller or on the rio if it's on the
     * motor controller, it will result in lower can bus usage and faster response but if it's on
     * the rio, all the features will be available and can use any remote sensor directly for
     * feedback
     */
    public MotorManager.ControllerLocation location = MotorManager.ControllerLocation.MOTOR;

    /**
     * the type of motor that the controller is connected to. this is used to calculate the motor
     * torque. also used for simulation purposes.
     */
    public DCMotor motorType = DCMotor.getNEO(1);
  }

  /**
   * the PID configuration of the motor controller
   *
   * <p>this is used to set the PID gains of the motor controller
   */
  public static class PIDConfig {
    /** the kp gain of the PID controller units are: (voltage / unit of measurement) */
    public double kP = 0;
    /** the ki gain of the PID controller units are: (voltage * second / unit of measurement) */
    public double kI = 0;
    /** the kd gain of the PID controller units are: (voltage / unit of measurement * second) */
    public double kD = 0;
    /**
     * the iZone of the PID controller units are: (unit of measurement)
     *
     * <p>this is the zone in which the integral term is applied, outside of this zone the integral
     * term will be zeroed
     */
    public double iZone = Double.POSITIVE_INFINITY;
    /**
     * the maximum accumulation and contribution of the integral term units are: (voltage)
     *
     * <p>this is the maximum value that the integral term can accumulate to, if it exceeds this
     * value, it will be clamped
     */
    public double iMaxAccum = MotorManager.defaultMaxMotorOutput;
    /**
     * the tolerance of the PID controller units are: (unit of measurement)
     *
     * <p>this is the tolerance of the PID controller, if the error is within this tolerance, the
     * controller will consider it done and not apply any output (not including feed forward)
     */
    public double tolerance = 0;

    /**
     * gets the PID gains of the controller
     *
     * <p>this is used to get the PID gains of the controller
     *
     * @return the PID gains of the controller
     */
    public PIDGains getGains() {
      return new PIDGains(kP, kI, kD, iZone, iMaxAccum, tolerance);
    }
  }

  /**
   * the feed forward configuration of the motor controller
   *
   * <p>this is used to set the feed forward gains of the motor controller
   */
  public static class FeedForwardConfig {
    /**
     * the friction feed forward gain of the motor controller units are: (voltage)
     *
     * <p>this is used to compensate for the friction in the motor, it will be applied to the output
     * in the direction of travel
     */
    public double frictionFeedForward = 0;
    /**
     * the k_V feed forward gain of the motor controller units are: (voltage / unit of measurement)
     *
     * <p>this is usually used in velocity control, but it is just a feed forward gain that is
     * multiplied by the setpoint
     */
    public double setpointFeedForward = 0;
    /**
     * the simple feed forward gain of the motor controller units are: (voltage)
     *
     * <p>it's just a constant voltage applied to the output used, for example, in elevators
     */
    public double simpleFeedForward = 0;
    /**
     * a custom feed forward function that takes the setpoint and returns a value
     *
     * <p>this can be used for more complex feed forwards, like an arm with gravity compensation
     */
    public Function<Double, Double> customFeedForward = (setpoint) -> 0.0;

    /**
     * gets the feed forwards of the controller
     *
     * <p>this is used to get the feed forwards of the controller
     *
     * @return the feed forwards of the controller
     */
    public ControllerFeedForwards getFeedForwards() {
      return new ControllerFeedForwards(
          simpleFeedForward, frictionFeedForward, setpointFeedForward, customFeedForward);
    }
  }

  /**
   * the constraints configuration of the motor controller
   *
   * <p>this is used to set the constraints of the motor controller
   */
  public static class ConstraintsConfig {
    /**
     * the type of constraint to apply to the controller
     *
     * <p>this can be NONE, LIMITED, or CONTINUOUS
     *
     * <p>* NONE means no constraints are applied
     *
     * <p>* LIMITED means the output is clamped to the max and min values, for example, an arm that
     * can only move between 0 and 90 degrees
     *
     * <p>* CONTINUOUS means the output is continuous and wraps around, for example, a wheel that
     * can spin indefinitely or a swerve module
     *
     * <p>
     */
    public ControllerConstrains.ConstraintType constraintType =
        ControllerConstrains.ConstraintType.NONE;
    /**
     * the maximum value of the constraint units are: (unit of measurement)
     *
     * <p>this represents the maximum value of the constraint:
     *
     * <p>* for a limited constraint, this is the maximum value the motor position can reach
     *
     * <p>* for a continuous constraint, this is the maximum value the position can reach before
     * wrapping around (for example, 360)
     *
     * <p>
     */
    public double maxValue = 0;
    /**
     * the minimum value of the constraint units are: (unit of measurement)
     *
     * <p>this represents the minimum value of the constraint:
     *
     * <p>* for a limited constraint, this is the minimum value the motor position can reach
     *
     * <p>* for a continuous constraint, this is the minimum value the position can reach before
     * wrapping around (for example, 0)
     *
     * <p>
     */
    public double minValue = 0;
    /**
     * the maximum output of the constraint units are: (voltage)
     *
     * <p>this is the maximum output of the motor controller in the forward direction
     */
    public double maxOutput = MotorManager.defaultMaxMotorOutput;
    /**
     * the minimum output of the constraint units are: (voltage)
     *
     * <p>this is the minimum output of the motor controller in the reverse direction
     */
    public double minOutput = -MotorManager.defaultMaxMotorOutput;
    /**
     * the voltage deadband of the constraint units are: (voltage)
     *
     * <p>this is the deadband of the motor controller, if the output is within this range, the
     * motor will not apply any output to prevent jittering and damage to the motor
     */
    public double voltageDeadband = 0;

    /**
     * gets the constraints of the controller
     *
     * <p>this is used to get the constraints of the controller
     *
     * @return the constraints of the controller
     */
    public ControllerConstrains getConstraints() {
      return new ControllerConstrains(
          constraintType, minValue, maxValue, maxOutput, minOutput, voltageDeadband);
    }
  }

  /**
   * the profile configuration of the motor controller used for motion profiling and trajectory
   * generation
   */
  public static class ProfileConfig {
    /**
     * the maximum velocity of the motor controller units are: (unit of control per second) this is
     * used to create a motion profile for the motor controller if using a profiled position
     * control, this is the maximum velocity if using a profiled velocity control, this is the
     * maximum acceleration
     */
    public double maximumMeasurementVelocity = Double.POSITIVE_INFINITY;

    /**
     * the maximum acceleration of the motor controller units are: (unit of control per second
     * squared) this is used to create a motion profile for the motor controller if using a profiled
     * position control, this is the maximum acceleration if using a profiled velocity control, this
     * is the maximum jerk
     */
    public double maximumMeasurementAcceleration = Double.POSITIVE_INFINITY;

    public TrapezoidProfile.Constraints getProfileConstraints() {
      return new TrapezoidProfile.Constraints(
          maximumMeasurementVelocity, maximumMeasurementAcceleration);
    }
  }

  public static class SimulationConfig {
    /**
     * the kV gain of the motor controller units are: (voltage / unit of measurement per second) you
     * can use this and {@link #kA} to simulate the motor controller or use {@link #momentOfInertia}
     * the kv must be in SI units. for general use (angular mechanisms), it is in (voltage / radians
     * per second). for elevators, telescopic arms, and other linear mechanisms, it is in (voltage /
     * meters per second). but it is after gear ratio. for an arm it will be in (voltage / radians
     * per second). and the radians are the mechanisms, not the motor.
     */
    public double kV = 0;

    /**
     * the kA gain of the motor controller units are: (voltage / unit of measurement * per second
     * squared) the ka must be in SI units. for general use (angular mechanisms), it is in (voltage
     * / radians per second squared). for elevators, telescopic arms, and other linear mechanisms,
     * it is in (voltage / meters per second squared). but it is after gear ratio. for an arm it
     * will be in (voltage / radians per second squared). and the radians are the mechanisms, not
     * the motor.
     */
    public double kA = 0;

    /**
     * the moment of inertia of the motor controller units are: (kilogram * meter squared) this is
     * used to simulate the motor inertia in the simulation if this is 0, the kV and kA must be
     * provided
     */
    public double momentOfInertia = 0.0;

    /**
     * the standard deviation of the position measurement in the simulation this is used to simulate
     * the noise in the position measurement units are: (unit of measurement)
     */
    public double positionStandardDeviation = 0.0;

    /**
     * the standard deviation of the velocity measurement in the simulation this is used to simulate
     * the noise in the velocity measurement units are: (unit of measurement per second)
     */
    public double velocityStandardDeviation = 0.0;

    /**
     * the elevator simulation configuration used if the motor is an elevator or a similar
     * mechanism, and you want to simulate it when using the elevator, units are meters, other units
     * will break the simulation
     */
    public final ElevatorSimConfig elevatorSimConfig = new ElevatorSimConfig();

    /**
     * the arm simulation configuration used if the motor is an arm or a similar mechanism, and you
     * want to simulate it
     */
    public final ArmSimConfig armSimConfig = new ArmSimConfig();

    /** the elevator simulation configuration */
    public static class ElevatorSimConfig {
      /**
       * if the elevator simulation should enable gravity simulation if true, there will be a force
       * acting on the elevator due to gravity if false, the elevator will not be affected by
       * gravity
       */
      public boolean enableGravitySimulation = true;

      /**
       * the mass of the elevator in kilograms
       *
       * <p>this is used to simulate the elevator's mass in the simulation if the kv and ka are
       * provided, this will be ignored
       */
      public double massKG = 0;

      /**
       * the radius of the pulley in meters
       *
       * <p>this is used to simulate the pulley radius in the simulation if the kv and ka are
       * provided, this will be ignored
       */
      public double pulleyRadiusMeters = 0;
    }

    /**
     * the arm simulation configuration used if the motor is an arm or a similar mechanism, and you
     * want to simulate it
     */
    public static class ArmSimConfig {
      /** the length of the arm in meters taken from the pivot point to the end of the arm */
      public double armlengthMeters = 0.0;

      /**
       * should the arm simulate gravity?
       *
       * <p>if true, there will be a force acting on the arm due to gravity
       *
       * <p>if false, the arm will not be affected by gravity
       */
      public boolean simulateGravity = true;

      /**
       * the starting angle of the arm in rotations in simulation, the 0 angle is parralel to the
       * ground (full force of gravity is applied in 0 angle)
       */
      public double startingAngle = 0.0;
    }
  }
}
