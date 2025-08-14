// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.HumanControls;
import frc.robot.Flywheel;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public Flywheel motor_1 = new Flywheel();
  public Flywheel motor_2 = new Flywheel();
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    swerveSubsystem.setDefaultCommand(
      new RunCommand(()->
      swerveSubsystem.drive(
        HumanControls.leftJoyX.getAsDouble(),
        HumanControls.leftJoyY.getAsDouble(),
        HumanControls.rightJoyX.getAsDouble(), 
        true)
      ,swerveSubsystem));

      HumanControls.controller.start().onTrue(
        new InstantCommand(() -> swerveSubsystem.resetHeading())
      );
  
      HumanControls.controller.back().onTrue(
        new InstantCommand(()->swerveSubsystem.stopall())
      );

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
