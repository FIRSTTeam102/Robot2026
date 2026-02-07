// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.BasicShooter;
import frc.robot.commands.ChangeShooterAngle;
import frc.robot.commands.ExtendActuator;
import frc.robot.commands.RunShooter;
import frc.robot.Robot;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import java.io.File;
import frc.robot.commands.ChangeShooterAngle;

public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);

  private final Shooter shooter = new Shooter();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //operatorXbox.rightTrigger().whileTrue(new RunShooter(shooter));
    operatorXbox.rightBumper().whileTrue(new BasicShooter(shooter));
    operatorXbox.a().onTrue(new ChangeShooterAngle(shooter, ShooterConstants.HIGH_SHOOTER_ANGLE));
    operatorXbox.b().onTrue(new ChangeShooterAngle(shooter, ShooterConstants.PASSING_ANGLE));
    operatorXbox.x().onTrue(new ExtendActuator(shooter, () -> Robot.actuatorPositionEntry.getDouble(0.0)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}