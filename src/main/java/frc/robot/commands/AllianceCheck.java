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
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import java.util.concurrent.TimeUnit;

import java.util.function.DoubleSupplier;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AllianceCheck extends Command {

    Shooter shooter;
    SwerveSubsystem swerve;
    DoubleSupplier xSupplier;
    DoubleSupplier ySupplier;
    PIDController rotationPID;

  

  public AllianceCheck(Shooter shooter,SwerveSubsystem swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier) 
   {
    
    this.swerve = swerve;
    this.shooter = shooter;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    rotationPID = new PIDController(5.0, 0.0, 0.0);
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    rotationPID.setTolerance(Math.toRadians(Constants.AlignTolerance));

    addRequirements(swerve);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translation = new Translation2d (
          xSupplier.getAsDouble(),
          ySupplier.getAsDouble()
    );

    Pose2d robotpose = swerve.getPose();

    Rotation2d targetAngle = swerve.aimAtHub();
    if (((DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) && (robotpose.getX()>5.625594)) || ((DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red) && (robotpose.getX()<10.915394))) {
        shooter.setShooterangle(ShooterConstants.HIGH_SHOOTER_ANGLE);
        if (MathUtil.isNear(shooter.targetShooterPosition(ShooterConstants.HIGH_SHOOTER_ANGLE), shooter.getShooterPosition(), .01)){
            shooter.setShooterSpeed(swerve.distanceToHub());

        }
    else {
        shooter.setShooterangle(ShooterConstants.PASSING_ANGLE);
         if (MathUtil.isNear(shooter.targetShooterPosition(ShooterConstants.PASSING_ANGLE), shooter.getShooterPosition(), .01)){
            shooter.startShooting(ShooterConstants.PASSING_VELOCITY);
        }
     }   
    }

    double omega = rotationPID.calculate (
      swerve.getPose().getRotation().getRadians(),
      targetAngle.getRadians()
    );

    swerve.drive (
      translation,
      omega,
      true
    );   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0.0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
