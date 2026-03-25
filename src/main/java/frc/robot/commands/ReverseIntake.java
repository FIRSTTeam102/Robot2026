// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import frc.robot.Robot;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;


import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * 1) get pose from vision 
 * 2) calculate distance in inches 
 * 3) sends distance to Shooter 
 */
public class ReverseIntake extends Command {
  Intake intake;
  /** Creates a new RunShooter. */
  public ReverseIntake(Intake intake) {
   this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.IntakeTheFuel(-1.0);
   }

  @Override
  public void execute() {
   

  }

  @Override
  public void end(boolean interrupted) {
    intake.IntakeTheFuel(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}