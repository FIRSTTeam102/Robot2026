// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
    double distance = swerve.distanceToHub();
    if(distance > 144){
      shooter.setActuatorExtension(0.7);
      shooter.startShooting(-6700.0); //equation
    }
     else if (distance > 125.5 && distance <= 144){
        shooter.setActuatorExtension(0.6);
        shooter.startShooting((-40.50633* distance )+1043.03797); //TODO need to get more data for better equation
       }
     else {
       shooter.setActuatorExtension(0.3);
       shooter.startShooting(-0.0120833 * Math.pow(distance, 3) + 2.12207 * Math.pow(distance, 2)- (130.7185 * distance));
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
