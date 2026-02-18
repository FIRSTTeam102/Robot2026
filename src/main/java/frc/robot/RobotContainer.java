// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.BasicShooter;
import frc.robot.commands.ChangeShooterAngle;
import frc.robot.commands.ExtendActuator;
import frc.robot.commands.FullFuelCycle;
import frc.robot.commands.IntakeFuel;
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
import frc.robot.commands.RunIndexer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.Robot;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import java.io.File;
import frc.robot.commands.ChangeShooterAngle;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunClimber;

public class RobotContainer {
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  final CommandXboxController testerXbox = new CommandXboxController(5);

  private final Indexer indexer = new Indexer();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //running motors
    operatorXbox.leftTrigger().whileTrue(new RunIndexer(indexer));
    operatorXbox.leftBumper().whileTrue(new RunFeeder(indexer));
    operatorXbox.povLeft().whileTrue(new RunShooter(shooter));
    operatorXbox.rightBumper().whileTrue(new BasicShooter(shooter,ShooterConstants.FRONT_TRENCH));
    operatorXbox.povRight().whileTrue(new BasicShooter(shooter,ShooterConstants.FRONT_TOWER));


    //chnaging acuator 
    operatorXbox.a().onTrue(new ChangeShooterAngle(shooter, ShooterConstants.HIGH_SHOOTER_ANGLE));
    operatorXbox.b().onTrue(new ChangeShooterAngle(shooter, ShooterConstants.PASSING_ANGLE));
    operatorXbox.x().onTrue(new ExtendActuator(shooter, () -> Robot.actuatorPositionEntry.getDouble(1.0)));
   
    //combined subsystem
    operatorXbox.y().whileTrue(new FullFuelCycle(shooter, indexer, intake));
    operatorXbox.rightTrigger().whileTrue(new IntakeFuel(intake, () -> Robot.IntakeSpeed.getDouble(Constants.IntakeConstants.INTAKE_DEFAULT_SPEED)));
    operatorXbox.back().whileTrue(new SequentialCommandGroup(
                        new IntakeFuel(intake, () -> Robot.IntakeSpeed.getDouble(Constants.IntakeConstants.INTAKE_DEFAULT_SPEED)),
                        new RunIndexer(indexer)));

    operatorXbox.start().whileTrue(new SequentialCommandGroup(
      new RunFeeder(indexer),
      new RunShooter(shooter)));
    operatorXbox.povDown().onTrue(new RunClimber(climber));

    testerXbox.a().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    testerXbox.b().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    testerXbox.x().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    testerXbox.y().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));


  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}