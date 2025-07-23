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


/** Add your docs here. */
public class PathplannerConstants {
    public static final PPHolonomicDriveController PATH_PLANNER_PID =
        new PPHolonomicDriveController(
            new PIDConstants(0), //PID constants for the translation PID cotnroller
            new PIDConstants(0)); //PID constants for the rotation PID controller

    /**
     * Change this function according to your needs per season.
     * You may use the {@link DriverStation#getAlliance()} to check which allaince you are.
     * @return Wheter the pathplanner path should flip
     */
    public static boolean shouldFlipPath(){
        return false;
    }

    public static final RobotConfig DEFAULT_CONFIG =
        new RobotConfig(
            0,
            0,
            new ModuleConfig(
                null,
                null,
                0,
                null,
                null,
                0),
            SwerveModuleConstants.getTranslations());
}
