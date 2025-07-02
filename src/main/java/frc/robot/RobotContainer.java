// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import util.BasicMotor.BasicMotor;
import util.BasicMotor.Configuration.BasicMotorConfig;
import util.BasicMotor.Controllers.Controller.RequestType;
import util.BasicMotor.MotorManager;
import util.BasicMotor.Motors.Simulation.BasicSimMotor;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();

    MotorManager.getInstance();

    BasicMotorConfig config = new BasicMotorConfig();

    config.simulationConfig.momentOfInertia = 0.05;
    config.motorConfig.gearRatio = 5;
    config.motorConfig.name = "motor";

    BasicMotor motor = new BasicSimMotor(config);

    motor.setReference(5, RequestType.POSITION);

    SmartDashboard.putData(motor.getController());
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
