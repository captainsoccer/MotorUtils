package com.basicMotor.gains;

import com.basicMotor.Manager.MotorManager;
import com.basicMotor.controllers.Controller;
import com.basicMotor.measurements.Measurements;
import edu.wpi.first.math.MathUtil;

/**
 * this class is used to set the constraints of the controller it handles the limits of the
 * controller and the continuity of the controller and max motor output and min motor output this is
 * modeled after the TalonFX constraints
 */
public class ControllerConstrains {
  /** which type of constraints to use */
  public enum ConstraintType {
    /**
     * continuous constraints this means that the controller will wrap around the limits, for
     * example, turn on a swerve module
     */
    CONTINUOUS,
    /**
     * limited constraints this means that the controller will limit the output to the limits, for
     * example, an elevator with an extension limit
     */
    LIMITED,
    /**
     * no constraints this means that the controller will not limit the output, for example, a drive
     * motor or a flywheel
     */
    NONE
  }

  /** the type of constraints to use */
  private final ConstraintType constraintType;

  /** the minimum value of the constraints this is used for limited and continuous constraints */
  private final double minValue;

  /** the maximum value of the constraints this is used for limited and continuous constraints */
  private final double maxValue;

  /** the maximum output of the motor (in volts) this is used for capping the output of the motor */
  private final double maxMotorOutput;

  /** the minimum output of the motor (in volts) this is used for capping the output of the motor */
  private final double minMotorOutput;

  /**
   * the minimum output voltage of the motor, any output below this value will be ignored (absolute
   * value)
   */
  private final double voltageDeadband;

  /**
   * creates a constrains object with the given type and limits
   *
   * @param type type of the constrains (continuous, limited, none)
   * @param minValue the minimum value of the constrains (if continuous is the minimum value to
   *     round to, if limited is the minimum value of the limits)
   * @param maxValue the maximum value of the constrains (if continuous is the maximum value to
   *     round to, if limited is the maximum value of the limits)
   * @param maxMotorOutput the maximum output of the motor (in volts) this is used for capping the
   *     output of the motor (default is 13.0)
   * @param minMotorOutput the minimum output of the motor (in volts) this is used for capping the
   *     output of the motor (default is -13.0)
   * @param deadband the minumum abosulte voltage the motor needs to apply (in volts), any value
   *     below this will be ignored and the motor will not move.
   */
  public ControllerConstrains(
      ConstraintType type,
      double minValue,
      double maxValue,
      double maxMotorOutput,
      double minMotorOutput,
      double deadband) {
    // if the type is null, set it to NONE
    if (type == null) type = ConstraintType.NONE;
    // check if the min and max values are valid
    if (minValue > maxValue) {
      throw new IllegalArgumentException(
          "the minimum value must be less than or equal to the maximum value");
    }

    this.constraintType = type;
    this.minValue = minValue;
    this.maxValue = maxValue;
    this.maxMotorOutput =
        MathUtil.clamp(maxMotorOutput, -MotorManager.config.defaultMaxMotorOutput, MotorManager.config.defaultMaxMotorOutput);
    if (minMotorOutput > 0) minMotorOutput = -minMotorOutput;
    this.minMotorOutput =
        MathUtil.clamp(minMotorOutput, -MotorManager.config.defaultMaxMotorOutput, MotorManager.config.defaultMaxMotorOutput);
    this.voltageDeadband = Math.abs(deadband);
  }

  /**
   * creates a constrains object with the given type and limits with the default max motor output
   *
   * @param type type of the constrains (continuous, limited, none)
   * @param minValue the minimum value of the constrains (if continuous is the minimum value to
   *     round to, if limited is the minimum value of the limits)
   * @param maxValue the maximum value of the constrains (if continuous is the maximum value to
   *     round to, if limited is the maximum value of the limits)
   */
  public ControllerConstrains(ConstraintType type, double minValue, double maxValue) {
    this(type, minValue, maxValue, MotorManager.config.defaultMaxMotorOutput, -MotorManager.config.defaultMaxMotorOutput, 0);
  }

  /**
   * creates a constrains object with the given type and limits with the default max motor output
   *
   * @param voltageDeadband the minimum output voltage of the motor, * any output below this value
   */
  public ControllerConstrains(double voltageDeadband) {
    this(ConstraintType.NONE, 0, 0, MotorManager.config.defaultMaxMotorOutput, -MotorManager.config.defaultMaxMotorOutput, voltageDeadband);
  }

  /**
   * creates a constrains object with the given type and limits with the default max motor output
   *
   * @param type type of the constrains (continuous, limited, none)
   * @param minValue the minimum value of the constrains (if continuous is the minimum value to
   *     round to, if limited is the minimum value of the limits)
   * @param maxValue the maximum value of the constrains (if continuous is the maximum value to
   *     round to, if limited is the maximum value of the limits)
   * @param voltageDeadband the minimum output voltage of the motor, * any output below this value
   *     will be ignored (absolute value)
   */
  public ControllerConstrains(
      ConstraintType type, double minValue, double maxValue, double voltageDeadband) {
    this(type, minValue, maxValue, MotorManager.config.defaultMaxMotorOutput, -MotorManager.config.defaultMaxMotorOutput, voltageDeadband);
  }

  /**
   * creates a constrains object with no limits and no continuity with set max and min motor output
   *
   * @param maxMotorOutput the maximum output of the motor (in volts) * this is used for capping the
   *     output of the motor * (default is 13.0)
   * @param minMotorOutput the minimum output of the motor (in volts) * this is used for capping the
   *     output of the motor * (default is -13.0)
   */
  public ControllerConstrains(double maxMotorOutput, double minMotorOutput) {
    this(ConstraintType.NONE, 0, 0, maxMotorOutput, minMotorOutput, 0);
  }

  /**
   * creates a constrains object with no limits and no continuity with set max and min motor output
   *
   * @param maxMotorOutput the maximum output of the motor (in volts) * this is used for capping the
   *     output of the motor * (default is 13.0)
   * @param minMotorOutput the minimum output of the motor (in volts) * this is used for capping the
   *     output of the motor * (default is -13.0)
   * @param voltageDeadband the minimum output voltage of the motor, * any output below this value
   *     will be ignored (absolute value)
   */
  public ControllerConstrains(
      double maxMotorOutput, double minMotorOutput, double voltageDeadband) {
    this(ConstraintType.NONE, 0, 0, maxMotorOutput, minMotorOutput, voltageDeadband);
  }

  /**
   * creates an empty constrains object (no limits and no continuity) and the default max motor
   * output
   */
  public ControllerConstrains() {
    this(ConstraintType.NONE, 0, 0, MotorManager.config.defaultMaxMotorOutput, -MotorManager.config.defaultMaxMotorOutput, 0);
  }

  /**
   * calculates the constraints of the controller checks if the request is in the limits of the
   * motor or needs to be wrapped
   *
   * @param measurement the measurement of the motor
   * @param request the request of the controller
   */
  public void calculateConstraints(
      Measurements.Measurement measurement, Controller.ControllerRequest request) {
    switch (constraintType) {
      case LIMITED -> calculateLimited(measurement, request);
      case CONTINUOUS -> calculateContinuous(measurement, request);
      default -> {
        // do nothing
      }
    }
  }

  /**
   * calculates the limits of the controller with the limited mode
   *
   * @param measurement the measurement of the motor
   * @param request the request of the controller
   */
  public void calculateLimited(
      Measurements.Measurement measurement, Controller.ControllerRequest request) {
    var controlMode = request.requestType();
    // check if the request is a position control (then apply the limits to the setpoint)
    if (controlMode.isPositionControl()) {
      // check if the request is in the limits of the motor
      if (request.goal().position >= maxValue) {
        request.goal().position = maxValue;
        request.goal().velocity = 0;
      }
      if (request.goal().position <= minValue) {
        request.goal().position = minValue;
        request.goal().velocity = 0;
      }
    }
    // if not position control,
    // then check if the measurement is in the limits of the motor
    // and make sure the direction is back to the zone
    else {
      if (measurement.position() <= minValue && request.goal().position < 0) {
        request.goal().position = 0;
        request.goal().velocity = 0;
      }
      if (measurement.position() >= maxValue && request.goal().position > 0) {
        request.goal().position = 0;
        request.goal().velocity = 0;
      }
    }
  }

  /**
   * calculates the limits of the controller with the continuous mode this means that the controller
   * will wrap around the limits, for example, turn on a swerve module
   *
   * @param measurement the measurement of the motor
   * @param request the request of the controller
   */
  public void calculateContinuous(
      Measurements.Measurement measurement, Controller.ControllerRequest request) {
    // check if the request is a position control (continuous constraints only work for position
    // control)
    if (!request.requestType().isPositionControl()) return;

    // calculate the error bound
    double errorBound = (maxValue - minValue) / 2.0;

    // store the original position
    double originalPosition = request.goal().position;

    // wrap the goal around the limits
    request.goal().position =
        MathUtil.inputModulus(
                request.goal().position - measurement.position(), -errorBound, errorBound)
            + measurement.position();

    // if the goal is in the opposite direction of the original position, reverse the velocity
    if (Math.signum(request.goal().position - measurement.position())
        != Math.signum(originalPosition - measurement.position())) {
      request.goal().velocity *= -1;
    }
  }

  /**
   * clamps the motor output to the limits of the motor
   *
   * @param output the output of the motor
   * @return the clamped output of the motor
   */
  public double clampMotorOutput(double output) {
    return MathUtil.clamp(output, minMotorOutput, maxMotorOutput);
  }

  /**
   * apply the deadband to the motor output
   *
   * @param output the output of the motor
   * @return the output of the motor with the deadband applied
   */
  public double deadbandMotorOutput(double output) {
    if (Math.abs(output) < voltageDeadband) {
      return 0;
    }
    return output;
  }

  /**
   * checks the motor output and applies the deadband and clamps the output to the limits of the
   * motor
   *
   * @param output the output of the motor
   * @return the checked output of the motor
   */
  public double checkMotorOutput(double output) {
    return deadbandMotorOutput(clampMotorOutput(output));
  }

  /**
   * gets the type of constraints
   *
   * @return the type of constraints
   */
  public ConstraintType getConstraintType() {
    return constraintType;
  }

  /**
   * gets the minimum value of the constraints
   *
   * @return the minimum value of the constraints
   */
  public double getMinValue() {
    return minValue;
  }

  /**
   * gets the maximum value of the constraints
   *
   * @return the maximum value of the constraints
   */
  public double getMaxValue() {
    return maxValue;
  }

  /**
   * gets the maximum output of the motor
   *
   * @return the maximum output of the motor
   */
  public double getMaxMotorOutput() {
    return maxMotorOutput;
  }

  /**
   * gets the minimum output of the motor
   *
   * @return the minimum output of the motor
   */
  public double getMinMotorOutput() {
    return minMotorOutput;
  }

  /**
   * gets the minimum output voltage of the motor
   *
   * @return the minimum output voltage of the motor
   */
  public double getVoltageDeadband() {
    return voltageDeadband;
  }

  /**
   * converts the constraints to motor constraints this is used for converting the constraints
   *
   * @param gearRatio the gear ratio of the motor (used to convert the constraints to motor
   *     constraints)
   * @return the motor constraints that are in the motor units
   */
  public ControllerConstrains convertToMotorConstrains(double gearRatio) {
    return new ControllerConstrains(
        constraintType,
        minValue * gearRatio,
        maxValue * gearRatio,
        maxMotorOutput,
        minMotorOutput,
        voltageDeadband);
  }
}
