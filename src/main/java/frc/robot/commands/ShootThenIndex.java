// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Indexer;

import frc.robot.Robot;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;


import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * 1) get pose from vision 
 * 2) calculate distance in inches 
 * 3) sends distance to Shooter 
 * 4) turns on indexer and feeder when shooter reaches speed
 */
public class ShootThenIndex extends Command {
  Shooter shooter; 
  SwerveSubsystem swerve;
  Indexer indexer;
  /** Creates a new RunShooter. */
  public ShootThenIndex(Shooter shooter, SwerveSubsystem swerve, Indexer indexer) {
    this.shooter = shooter;
    this.swerve = swerve;
    this.indexer = indexer;
    addRequirements(shooter, indexer);
  }

  @Override
  public void initialize() { }

  @Override
  public void execute() {
    if (MathUtil.isNear(shooter.setShooterSpeed(swerve.distanceToHub()), shooter.shooterRPM(), ShooterConstants.PIDRPMTOLERANCE)) {
      indexer.RunIndexer();
      indexer.runFeeder();
    }  

  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShooting();
    indexer.stopFeeder();
    indexer.stopIndexer();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}