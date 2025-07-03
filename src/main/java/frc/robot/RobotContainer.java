// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import util.BasicMotor.BasicMotor;
import util.BasicMotor.Configuration.BasicMotorConfig;
import util.BasicMotor.Controllers.Controller.RequestType;
import util.BasicMotor.Motors.Simulation.BasicSimElevator;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();

    BasicMotorConfig config = new BasicMotorConfig();

    config.motorConfig.gearRatio = 5;
    config.motorConfig.name = "motor";
    config.motorConfig.motorType = DCMotor.getKrakenX60(2);
    config.simulationConfig.elevatorSimConfig.enableGravitySimulation = true;
    config.simulationConfig.elevatorSimConfig.massKG = 20;
    config.simulationConfig.elevatorSimConfig.pulleyRadiusMeters = 0.02;
    config.simulationConfig.momentOfInertia = 0.05;

    BasicMotor elevator = new BasicSimElevator(config);

    new Trigger(RobotState::isEnabled)
        .onTrue(new InstantCommand(() -> elevator.setReference(0, RequestType.VOLTAGE)));

    SmartDashboard.putData(elevator.getController());
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
