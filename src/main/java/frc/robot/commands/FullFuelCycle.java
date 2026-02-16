// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import java.util.function.DoubleSupplier;

import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FullFuelCycle extends Command {
  Shooter shooter;
  Indexer indexer; 
  Intake intake;

  /** Creates a new FullFuelCycle. */
  public FullFuelCycle(Shooter shooter, Indexer indexer, Intake intake) {
    this.shooter = shooter;
    this.indexer = indexer; 
    this.intake = intake;

    addRequirements(shooter, indexer, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterSpeed(Robot.Distance.getDouble(ShooterConstants.TESTING_DISTANCE_DEFAULT));
    indexer.RunIndexer();
    indexer.runFeeder();
    intake.IntakeTheFuel(Robot.IntakeSpeed.getDouble(IntakeConstants.INTAKE_DEFAULT_SPEED));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooting();
    indexer.stopIndexer();
    indexer.stopFeeder();
    intake.IntakeTheFuel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
