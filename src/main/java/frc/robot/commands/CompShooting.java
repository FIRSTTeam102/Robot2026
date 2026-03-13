// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CompShooting extends Command {
  Shooter shooter;
  SwerveSubsystem swerve;
  Intake intake;
  Indexer indexer;
  static int counter = 0;

  public CompShooting(Shooter shooter, SwerveSubsystem swerve, Intake intake, Indexer indexer) {
  this.shooter = shooter;
  this.swerve = swerve;
  this.intake = intake;
  this.indexer = indexer;
  addRequirements(shooter, indexer, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d robotpose = swerve.getPose();
    double distance = Units.metersToInches(swerve.distanceToHub());

    if (((DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) && (robotpose.getX()>5.625594)) || ((DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red) && (robotpose.getX()<10.915394))) {
      shooter.setActuatorExtension(ShooterConstants.PASSING_EXTENSION);
      shooter.startShooting(ShooterConstants.PASSING_VELOCITY*2);
      if (shooter.shooterRPM()<=-6000) {
          counter ++;
              indexer.runFeeder();
               if (counter <= 60){
              indexer.RunIndexer();}
            else if (counter > 60){
              indexer.ReverseIndexer();
            }
            if (counter > 72){
                counter = 0;
            }
          intake.IntakeTheFuel(IntakeConstants.INTAKE_DEFAULT_SPEED);
      }
    }
    else if(distance > 120){
      shooter.setActuatorExtension(0.7);
      double expectedRPM = (-14.27526 * distance) - 1601.22854;
      shooter.startShooting(expectedRPM*2);
      System.out.println(expectedRPM);
      if (MathUtil.isNear(expectedRPM, shooter.shooterRPM(), ShooterConstants.RPMTOLERANCE)) {
        counter ++;     
        indexer.runFeeder();
              if (counter <= 60){
              indexer.RunIndexer();}
            else if (counter > 60){
              indexer.ReverseIndexer();
            }
            if (counter > 72){
                counter = 0;
            }
              intake.IntakeTheFuel(IntakeConstants.INTAKE_DEFAULT_SPEED);
      }
    }
    else if (distance <= 120){
        shooter.setActuatorExtension(0.3);
        double expectedRPM = (-0.00552478*Math.pow(distance, 3)) + (1.30763 * Math.pow(distance, 2)) - (118.31419 * distance) + 801.97076;
        shooter.startShooting(expectedRPM*2);
        System.out.println(expectedRPM);
       if (MathUtil.isNear(expectedRPM, shooter.shooterRPM(), ShooterConstants.RPMTOLERANCE)) {
          counter ++;      
          indexer.runFeeder();
                 if (counter <= 60){
              indexer.RunIndexer();}
            else if (counter > 60){
              indexer.ReverseIndexer();
            }
            if (counter > 72){
                counter = 0;
            }
              intake.IntakeTheFuel(IntakeConstants.INTAKE_DEFAULT_SPEED);
        } 
      }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooting();
    indexer.stopFeeder();
    indexer.stopIndexer();
    intake.IntakeTheFuel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
