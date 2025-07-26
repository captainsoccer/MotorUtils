// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.drivetrain.DriveTrain;

public class RobotContainer {
  private static CommandPS4Controller controller;

  private static DriveTrain driveTrain;

  private static LoggedDashboardChooser<Command> chooser;


  public RobotContainer() {
    driveTrain = new DriveTrain();
    controller = new CommandPS4Controller(0);
    configureBindings();
    
    var autoChooser = AutoBuilder.buildAutoChooser();

    chooser = new LoggedDashboardChooser<>("Auto chooser", autoChooser);
  }

  private void configureBindings() {
    // Configure the default command for the drive train
    Command driveCommand = new DriveCommand(
      driveTrain, 
      () -> -controller.getRawAxis(1),
      () -> -controller.getRawAxis(0),
      () -> -controller.getRawAxis(3),
      () -> 1 - (controller.getRawAxis(5) + 1) / 2);
      
    Command setDriveCommand = new StartEndCommand(
        () -> driveTrain.setDefaultCommand(driveCommand),
        () -> driveTrain.removeDefaultCommand()
    );

    new Trigger(RobotState::isEnabled).whileTrue(setDriveCommand);

    controller.options().onTrue(new InstantCommand(() -> driveTrain.resetGyro(Rotation2d.kZero)));
  }

  public Command getAutonomousCommand() {
    return chooser.get();
  }
}
