// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FullClimbing extends Command {
Climber climber;
static boolean isUp;
  public FullClimbing(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     if (climber.getEncoderPosition() <= ClimberConstants.CLIMBER_ENCODER_EXTENSION){
      isUp = true;
      climber.SetClimberSpeed(ClimberConstants.CLIMBER_DEFAULT_SPEED);

    }
    else {
      isUp = false;
      climber.SetClimberSpeed(-ClimberConstants.CLIMBER_DEFAULT_SPEED);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.SetClimberSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((isUp && climber.getEncoderPosition() >= ClimberConstants.CLIMBER_ENCODER_MIN_EXTENSION) || (climber.getEncoderPosition() <= ClimberConstants.CLIMBER_ENCODER_EXTENSION && !isUp)){
      return true;
    }
    else {
      return false;
    }

  }
}
