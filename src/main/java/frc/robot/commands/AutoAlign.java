// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  SwerveSubsystem swerve;
  Pose2d targetPose;
  PIDController xPID;
  PIDController yPID;
  PIDController rPID;

  /** Creates a new AutoAlign. */
  public AutoAlign(SwerveSubsystem swerve, Pose2d targetPose) {
    this.swerve = swerve;
    this.targetPose = targetPose;

     xPID = new PIDController(5.0, 0, 2.5);
     yPID = new PIDController(5.0, 0, 2.5);
     rPID = new PIDController(5.0, 0, 2.5);

     xPID.setTolerance(Units.inchesToMeters(5.0)); 
     yPID.setTolerance(Units.inchesToMeters(5.0)); 
     rPID.setTolerance(Units.degreesToRadians(2.5));

     rPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = swerve.getPose();

    //clamps to limit max speeds, but not necesarry to function, deadband stops jittering
    double xTranslation = MathUtil.applyDeadband(MathUtil.clamp(xPID.calculate(robotPose.getX(),targetPose.getX()), -10.0, 10.0),0.03);
    double yTranslation = MathUtil.applyDeadband(MathUtil.clamp(yPID.calculate(robotPose.getY(),targetPose.getY()), -10.0, 10.0), 0.03);
    double rRotation = MathUtil.applyDeadband(MathUtil.clamp(rPID.calculate(swerve.getHeading().getRadians(),swerve.getHeading().getRadians()),-10.0,10.0), 0.03);    

    Translation2d translation = new Translation2d(xTranslation, yTranslation);

    swerve.drive(translation, rRotation, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0.0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xPID.atSetpoint() && yPID.atSetpoint() && rPID.atSetpoint());
  }
}
