// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import java.util.concurrent.TimeUnit;

import java.util.function.DoubleSupplier;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AllianceCheck extends Command {

    Shooter shooter;
    SwerveSubsystem swerve;
    Indexer indexer;

  public AllianceCheck(Shooter shooter,SwerveSubsystem swerve, Indexer indexer) 
   {
    
    this.swerve = swerve;
    this.shooter = shooter;
    this.indexer = indexer;

    addRequirements(shooter, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d robotpose = swerve.getPose();

    if (((DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) && (robotpose.getX()>5.625594)) || ((DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red) && (robotpose.getX()<10.915394))) {
      shooter.setShooterangle(ShooterConstants.PASSING_ANGLE);
         if (MathUtil.isNear(shooter.targetShooterPosition(ShooterConstants.PASSING_ANGLE), shooter.getShooterPosition(), .01)){
            shooter.startShooting(ShooterConstants.PASSING_VELOCITY);
            double expectedRPM = ShooterConstants.PASSING_VELOCITY*6784;
            if (MathUtil.isNear(expectedRPM, shooter.shooterRPM(), ShooterConstants.PIDRPMTOLERANCE)) {
              indexer.RunIndexer();
              indexer.runFeeder();
            }

        }  

        }
    else {

      shooter.setShooterangle(ShooterConstants.HIGH_SHOOTER_ANGLE);
        if (MathUtil.isNear(shooter.targetShooterPosition(ShooterConstants.HIGH_SHOOTER_ANGLE), shooter.getShooterPosition(), .01)){
            double expectedRPM = shooter.setShooterSpeed(swerve.distanceToHub());
            if (MathUtil.isNear(expectedRPM, shooter.shooterRPM(), ShooterConstants.PIDRPMTOLERANCE)) {
              indexer.RunIndexer();
              indexer.runFeeder();
            }
     }   
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stopIndexer();
    indexer.stopFeeder();
    shooter.stopShooting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
