// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.swerveModule;

import java.util.ArrayList;

import com.basicMotor.configuration.BasicSparkBaseConfig;
import com.basicMotor.configuration.BasicTalonFXConfig;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public enum SwerveModuleConstants {
    FRONT_LEFT(1, 0),
    FRONT_RIGHT(2, 0),
    BACK_LEFT(3, 0),
    BACK_RIGHT(4, 0);

    public final String NAME;

    //Change this based on the drive motor you are using
    public final BasicTalonFXConfig DRIVE_MOTOR_CONFIG = new BasicTalonFXConfig();
    //Change this based on the steer motor you are using
    public final BasicSparkBaseConfig STEER_MOTOR_CONFIG = new BasicSparkBaseConfig();

    public final int CAN_CODER_ID;

    public final double ZERO_OFFSET;

    public final Translation2d location;

    SwerveModuleConstants(int canCoderID, double zeroOffset){
        this.NAME = this.name();

        this.CAN_CODER_ID = canCoderID;
        this.ZERO_OFFSET = zeroOffset;
    }

    // the max speed of the drive wheel in meters per second
    public static final double MAX_WHEEL_SPEED = 4;

    public static Translation2d[] getTranslations(){
        ArrayList<Translation2d> translations = new ArrayList<>();

        for (SwerveModuleConstants constants : SwerveModuleConstants.values()) {
            translations.add(constants.location);
        }

        return translations.toArray(new Translation2d[4]);
    }
}
