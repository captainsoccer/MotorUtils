// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.swerveModule;

import java.util.ArrayList;

import com.basicMotor.BasicMotor;
import com.basicMotor.configuration.BasicMotorConfig;
import com.basicMotor.configuration.BasicSparkBaseConfig;
import com.basicMotor.configuration.BasicTalonFXConfig;
import com.basicMotor.configuration.BasicMotorConfig.FeedForwardConfig;
import com.basicMotor.configuration.BasicMotorConfig.PIDConfig;
import com.basicMotor.gains.ControllerConstraints;
import com.basicMotor.gains.ControllerFeedForwards;
import com.basicMotor.gains.PIDGains;
import com.basicMotor.motorManager.MotorManager;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Constants for the swerve modules.
 * This class contains the configurations for each swerve module.
 */
public enum SwerveModuleConstants {
    FRONT_LEFT(2, 0 ,
            // PID gains for the drive motor
            3, new PIDGains(5, 0, 0),
            // The feed forwards for the drive motor (the setpoint feed forward is the kV)
            new ControllerFeedForwards(2.7),
            0.011902948,

            // PID gains for the steer motor
            4, new PIDGains(20, 25, 0, 0.05, 4, 0.001),
            //The feed forwards for the steer motor
            new ControllerFeedForwards(0, 0.35464),
            //The motor constants used in the simulation
            0.1,0.2,
            new Translation2d(0.29, 0.29)), //The translation of the modules relative to the center of the robot

    FRONT_RIGHT(5, 0,
            // PID gains for the drive motor
            6, new PIDGains(5, 0, 0),
            // The feed forwards for the drive motor (the setpoint feed forward is the kV)
            new ControllerFeedForwards(2.7),
            0.011902948,

            // PID gains for the steer motor
            7, new PIDGains(20, 25, 0, 0.05, 4, 0.001),
            //The feed forwards for the steer motor
            new ControllerFeedForwards(0, 0.35464),
            //The motor constants used in the simulation
            0.1,0.2,
            new Translation2d(0.29, -0.29)),

    BACK_LEFT(8, 0,
            // PID gains for the drive motor
            9, new PIDGains(5, 0, 0),
            // The feed forwards for the drive motor (the setpoint feed forward is the kV)
            new ControllerFeedForwards(2.7),
            0.011902948,

            // PID gains for the steer motor
            10, new PIDGains(20, 25, 0, 0.05, 4, 0.001),
            //The feed forwards for the steer motor
            new ControllerFeedForwards(0, 0.35464),
            //The motor constants used in the simulation
            0.1,0.2,
            new Translation2d(-0.29, 0.29)),

    BACK_RIGHT(11, 0,
            // PID gains for the drive motor
            12, new PIDGains(5, 0, 0),
            // The feed forwards for the drive motor (the setpoint feed forward is the kV)
            new ControllerFeedForwards(2.7),
            0.011902948,

            // PID gains for the steer motor
            13, new PIDGains(20, 25, 0, 0.05, 4, 0.001),
            //The feed forwards for the steer motor
            new ControllerFeedForwards(0, 0.35464),
            //The motor constants used in the simulation
            0.1,0.2,
            new Translation2d(-0.29, -0.29));

    // Drive motor constants
    public static final double MAX_WHEEL_SPEED = 4; // Maximum wheel speed in meters per second (Not the free speed of the motor)
    public static final double WHEEL_RADIUS_METERS = 0.0508; // 2 inches in meters
    public static final double DRIVE_GEAR_RATIO = 6.75; // Adjust based on your gear ratio
    public static final DCMotor DRIVE_MOTOR_TYPE = DCMotor.getKrakenX60(1); // Adjust based on your motor type
    public static final int DRIVE_CURRENT_LIMIT = 101; // Adjust based on your motor's current limit

    // Common configurations for the drive motor
    private static BasicMotorConfig getDriveMotorCommonConfig() {
        var config = new BasicTalonFXConfig();

        config.motorConfig.gearRatio = DRIVE_GEAR_RATIO;
        config.motorConfig.unitConversion = 2 * Math.PI * WHEEL_RADIUS_METERS;
        config.motorConfig.idleMode = BasicMotor.IdleMode.BRAKE;
        config.motorConfig.motorType = DCMotor.getKrakenX60(1);

        config.currentLimitConfig.statorCurrentLimit = DRIVE_CURRENT_LIMIT;
        config.currentLimitConfig.supplyCurrentLimit = 60; // Adjust based on your motor's current limit

        return config;
    }

    // Common configurations for the steer motor
    private static BasicMotorConfig getSteerMotorCommonConfig() {
        var config = new BasicSparkBaseConfig();

        config.motorConfig.gearRatio = 12.75; // Adjust based on your gear ratio
        config.motorConfig.idleMode = BasicMotor.IdleMode.BRAKE;
        config.motorConfig.location = MotorManager.ControllerLocation.RIO; //For the cancoder to be used directly
        config.motorConfig.motorType = DCMotor.getNEO(1); // Adjust based on your motor type

        config.currentLimitConfig.freeSpeedCurrentLimit = 40; // Adjust based on your motor's current limit

        config.constraintsConfig.constraintType = ControllerConstraints.ConstraintType.CONTINUOUS;
        config.constraintsConfig.maxValue = 0.5; // Adjust based on your encoder's max value
        config.constraintsConfig.minValue = -0.5; // Adjust based on your encoder's min value

        return config;
    }

    //No need to change anything below this line!
    // Specific configurations for each swerve module
    public final String NAME;

    public final BasicMotorConfig DRIVE_MOTOR_CONFIG;
    public final BasicMotorConfig STEER_MOTOR_CONFIG;

    public final int CAN_CODER_ID;

    public final double ZERO_OFFSET;

    public final Translation2d location;

    SwerveModuleConstants(
            int canCoderID,
            double zeroOffset,
            int driveMotorID,
            PIDGains drivePIDGains,
            ControllerFeedForwards driveFeedForwards,
            double driveKA,
            int steerMotorID,
            PIDGains steerPIDGains,
            ControllerFeedForwards steerFeedForwards,
            double steerKV,
            double steerKA,
            Translation2d location) {

        this.NAME = this.name();

        this.CAN_CODER_ID = canCoderID;
        this.ZERO_OFFSET = zeroOffset;

        this.location = location;

        this.DRIVE_MOTOR_CONFIG = getDriveMotorCommonConfig();
        this.STEER_MOTOR_CONFIG = getSteerMotorCommonConfig();

        //apply specific configurations for the drive and steer motors
        DRIVE_MOTOR_CONFIG.motorConfig.name = NAME + " Drive Motor";
        DRIVE_MOTOR_CONFIG.motorConfig.id = driveMotorID;
        DRIVE_MOTOR_CONFIG.pidConfig = PIDConfig.fromGains(drivePIDGains);
        DRIVE_MOTOR_CONFIG.feedForwardConfig = FeedForwardConfig.fromFeedForwards(driveFeedForwards);
        DRIVE_MOTOR_CONFIG.simulationConfig.kV = driveFeedForwards.getSetpointFeedForward();
        DRIVE_MOTOR_CONFIG.simulationConfig.kA = driveKA;

        //apply specific configurations for the steer motor
        STEER_MOTOR_CONFIG.motorConfig.name = NAME + " Steer Motor";
        STEER_MOTOR_CONFIG.motorConfig.id = steerMotorID;
        STEER_MOTOR_CONFIG.pidConfig = PIDConfig.fromGains(steerPIDGains);
        STEER_MOTOR_CONFIG.feedForwardConfig = FeedForwardConfig.fromFeedForwards(steerFeedForwards);
        STEER_MOTOR_CONFIG.simulationConfig.kV = steerKV;
        STEER_MOTOR_CONFIG.simulationConfig.kA = steerKA;
    }

    /**
     * Gets the translations of all swerve modules.
     * The translations are relative to the center of the robot.
     * The order is: Front Left, Front Right, Back Left, Back Right.
     * @return An array of Translation2d representing the locations of the swerve modules.
     */
    public static Translation2d[] getTranslations(){
        ArrayList<Translation2d> translations = new ArrayList<>();

        for (SwerveModuleConstants constants : SwerveModuleConstants.values()) {
            translations.add(constants.location);
        }

        return translations.toArray(new Translation2d[4]);
    }
}
