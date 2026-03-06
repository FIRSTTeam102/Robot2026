// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleClimb extends Command {
Climber climber;
SwerveSubsystem swerve;
private boolean isUp = false;
private boolean climbed = true;
  public TeleClimb(Climber climber, SwerveSubsystem swerve) {
    this.climber = climber;
    this.swerve = swerve;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!MathUtil.isNear(0, climber.getEncoderPosition(), 10)) {
      climber.SetClimberSpeed(-0.35);
      climbed = true;
    }
    else {
      climbed = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotpose = swerve.getPose();

    if (climber.getEncoderPosition() <= -73.47900 && !isUp){
      climber.SetClimberSpeed(0); 
      isUp = true;
    }
    else if ((((DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) && (robotpose.getX()>2.0)) || ((DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red) && (robotpose.getX()<14.1))) && (isUp)) {
      climber.SetClimberSpeed(0.35);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.SetClimberSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((isUp && climber.getEncoderPosition()>=-0.25) || (!climbed)) {
      return true;
    }
    else {
      return false;
    }
  }
}
