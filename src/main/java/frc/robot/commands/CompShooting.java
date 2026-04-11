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
    double distance = swerve.distanceToHub();

    if (((DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) && (robotpose.getX()>5.625594)) || ((DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red) && (robotpose.getX()<10.915394))) {
      shooter.setActuatorExtension(ShooterConstants.PASSING_EXTENSION);
      shooter.setShooterRPM(ShooterConstants.PASSING_VELOCITY);
      if (shooter.shooterRPM()<=4500) {
        counter ++;
        indexer.jiggleIndexer(counter);
      }
    }
    else if(distance <= 68.01){// ZONE 1
      shooter.setActuatorExtension(0.2);
      double expectedRPM = ((0.0135566*Math.pow(distance, 3)) - (2.20607 * Math.pow(distance, 2)) + (133.78116 * distance));
      shooter.setShooterRPM(expectedRPM);
      System.out.println(expectedRPM);
      if (MathUtil.isNear(expectedRPM, shooter.shooterRPM(), ShooterConstants.RPMTOLERANCE)) {
        counter ++;     
        indexer.jiggleIndexer(counter);
      }
    }
    else if (distance <= 90.0){//ZONE 2
        shooter.setActuatorExtension(0.3);
        double expectedRPM = ((0.00814142*Math.pow(distance, 3)) - (1.61802 * Math.pow(distance, 2)) + (116.85856 * distance));
        shooter.setShooterRPM(expectedRPM);
        System.out.println(expectedRPM);
       if (MathUtil.isNear(expectedRPM, shooter.shooterRPM(), ShooterConstants.RPMTOLERANCE)) {
          counter ++;      
          indexer.jiggleIndexer(counter);
        } 
      }
    
     else if (distance <= 106.3){//ZONE 3
        shooter.setActuatorExtension(0.4);
        double expectedRPM = 11.41686*distance + 2053.40554;
        shooter.setShooterRPM(expectedRPM);
        System.out.println(expectedRPM);
       if (MathUtil.isNear(expectedRPM, shooter.shooterRPM(), ShooterConstants.RPMTOLERANCE)) {
          counter ++;      
          indexer.jiggleIndexer(counter);
        } 
      }
      else {//ZONE 4
        shooter.setActuatorExtension(0.5);
        double expectedRPM = (0.147374 * Math.pow(distance, 2)) + (7.14977 * distance) + 368.76315;
        shooter.setShooterRPM(expectedRPM);
        System.out.println(expectedRPM);
       if (MathUtil.isNear(expectedRPM, shooter.shooterRPM(), ShooterConstants.RPMTOLERANCE)) {
          counter ++;      
          indexer.jiggleIndexer(counter);
        } 
      }
      if (counter > 30){
        counter = 0;
      }
      intake.IntakeTheFuel(IntakeConstants.INTAKE_DEFAULT_SPEED);
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