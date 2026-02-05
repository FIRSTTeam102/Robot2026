// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeFuel;
import frc.robot.subsystems.Intake;
import frc.robot.Robot;
import frc.robot.Constants;

public class RobotContainer {
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);

  private final Intake intake = new Intake();
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driverXbox.leftTrigger().whileTrue(new IntakeFuel(intake, () -> Robot.IntakeSpeedentry.getDouble(Constants.INTAKE_DEFAULT_SPEED)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
