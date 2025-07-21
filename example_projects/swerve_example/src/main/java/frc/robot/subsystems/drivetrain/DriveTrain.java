// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.swerveModule.SwerveModuleReal;
import frc.robot.subsystems.drivetrain.swerveModule.SwerveModuleConstants;
import frc.robot.subsystems.drivetrain.swerveModule.SwerveModuleIO;
import frc.robot.subsystems.drivetrain.swerveModule.SwerveModuleInputsAutoLogged;

public class DriveTrain extends SubsystemBase {

  @AutoLog
  public static class DriveTrainInputs{
    public SwerveModuleState[] currentStates = new SwerveModuleState[4];
    public Rotation2d angle = new Rotation2d();
  }
  private final DriveTrainInputsAutoLogged inputs = new DriveTrainInputsAutoLogged();

  private final SwerveModuleIO[] io = new SwerveModuleIO[4];
  private final SwerveModuleInputsAutoLogged[] moduleInputs = new SwerveModuleInputsAutoLogged[4];

  private SwerveModuleState[] targetStates = new SwerveModuleState[4];
  private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;
  

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    for(int i = 0; i < 4; i ++){
      io[i] = new SwerveModuleReal(SwerveModuleConstants.values()[i]);

      moduleInputs[i] = new SwerveModuleInputsAutoLogged();
    }

    kinematics = new SwerveDriveKinematics(SwerveModuleConstants.getTranslations());

    poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero, modulePositions, new Pose2d());
  }

  public void run(ChassisSpeeds targetSpeed, double dt){
    var desturatedSpeeds = ChassisSpeeds.discretize(targetSpeed, dt);

    targetStates = kinematics.toSwerveModuleStates(desturatedSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveModuleConstants.MAX_WHEEL_SPEED);

    for(int i = 0; i < 4; i ++){
      targetStates[i].optimize(moduleInputs[i].state.angle);

      io[i].setTarget(targetStates[i]);
    }

    Logger.recordOutput("DriveTrain/targetStates", targetStates);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for(int i = 0; i < 4; i++){
      io[i].update(moduleInputs[i]);

      inputs.currentStates[i] = moduleInputs[i].state;
      modulePositions[i] = moduleInputs[i].position;
      
      Logger.processInputs("SwerveModule/" + io[i].getName(), moduleInputs[i]);
    }

    poseEstimator.update(null, modulePositions)
  }
}
