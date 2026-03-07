// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;

import java.util.function.DoubleSupplier;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JoystickClimb extends Command {

  Climber climber;
  DoubleSupplier joystick;

  public JoystickClimb(Climber climber, DoubleSupplier joystick) {
    this.climber = climber;
    this.joystick = joystick; 

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        climber.SetClimberSpeed(MathUtil.applyDeadband(joystick.getAsDouble(), OperatorConstants.LEFT_Y_DEADBAND) * 0.25);

  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.SetClimberSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
 
}}