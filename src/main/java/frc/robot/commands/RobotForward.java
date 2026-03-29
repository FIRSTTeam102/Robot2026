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
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RobotForward extends Command {
  /** Creates a new AimWhileMoving. */

  SwerveSubsystem swerve;
  PIDController rotationPID;
  DoubleSupplier xSupplier;
  DoubleSupplier ySupplier;
  static int orientation = 1;
  static Rotation2d targetAngle = new Rotation2d(0);
 


  public RobotForward(SwerveSubsystem swerve,DoubleSupplier xSupplier,DoubleSupplier ySupplier) 
   {
    
    this.swerve = swerve;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    rotationPID = new PIDController(4.0, 0.0, 0.0);
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    rotationPID.setTolerance(Math.toRadians(Constants.ALIGN_TOLERANCE));


    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
      orientation = -1;
      targetAngle = new Rotation2d(0);
    }
    else {
      orientation = 1;
      targetAngle = new Rotation2d(Math.PI);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translation = new Translation2d (
          xSupplier.getAsDouble()*orientation,
          ySupplier.getAsDouble()*orientation
    );

    
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
