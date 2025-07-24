// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drivetrain.swerveModule.SwerveModuleConstants;


/**
 * This class contains constants for PathPlanner integration.
 * It includes the PID controller configuration for the holonomic drive controller,
 * The fallback robot configuration,
 * and a method to determine if the PathPlanner path should be flipped based on the current alliance.
 */
public class PathplannerConstants {
    public static final PPHolonomicDriveController PATH_PLANNER_PID =
            new PPHolonomicDriveController(
                    new PIDConstants(0), //PID constants for the translation PID controller
                    new PIDConstants(0)); //PID constants for the rotation PID controller

    /**
     * Change this function according to your needs per season.
     * You may use the {@link DriverStation#getAlliance()} to check which alliance you are.
     * The default flipping is if the alliance is Red, then the pathplanner path should flip.
     * If no alliance is set, then the pathplanner path should not flip.
     *
     * @return Whether the pathplanner path should flip
     */
    public static boolean shouldFlipPath() {
        var currentAlliance = DriverStation.getAlliance();

        if (currentAlliance.isPresent()) {
            return currentAlliance.get() == DriverStation.Alliance.Red;
        }
        // If no alliance is set, do not flip the path.
        return false;
    }

    /**
     * The default robot configuration for PathPlanner.
     * This is used when the gui configuration is not set.
     */
    public static final RobotConfig FALLBACK_CONFIG =
            new RobotConfig(
                    20, //The total mass of the robot in kg. (this includes the bumpers, battery, etc.)
                    0, /*
                        * The robots moment of inertia.
                        * Calculate this using the method in the pathplanner documentation.
                        * https://pathplanner.dev/robot-config.html#robot-config-options
                        * In the Calculating MOI section.
                        */
                    new ModuleConfig(
                            SwerveModuleConstants.WHEEL_RADIUS_METERS,
                            SwerveModuleConstants.MAX_WHEEL_SPEED,
                            1, // This is the coefficient of friction for the wheel. Find it using testing.
                            SwerveModuleConstants.DRIVE_MOTOR_TYPE.withReduction(SwerveModuleConstants.DRIVE_GEAR_RATIO),
                            SwerveModuleConstants.DRIVE_CURRENT_LIMIT,
                            1),
                    SwerveModuleConstants.getTranslations());
}
