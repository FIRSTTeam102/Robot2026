// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZoneShooting extends Command {
  Shooter shooter;
  SwerveSubsystem swerve;

  public ZoneShooting(Shooter shooter, SwerveSubsystem swerve) {
  this.shooter = shooter;
  this.swerve = swerve;
  addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double distance = swerve.distanceToHub();
    double distance = Robot.Distance.getDouble(ShooterConstants.TESTING_DISTANCE_DEFAULT);
    if(distance > 120){
      shooter.setActuatorExtension(0.7);
      shooter.startShooting((-14.27526 * distance) - 1601.22854); //equation
    }
     else if (distance <= 120){
        shooter.setActuatorExtension(0.3);
        shooter.startShooting((-0.0052478*Math.pow(distance, 3)) + (1.30763 * Math.pow(distance, 2)) - (118.31419 * distance) + 801.97076 ); 
       }
     
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
