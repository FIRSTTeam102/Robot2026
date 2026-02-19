// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToHub extends Command {

  private final SwerveSubsystem swerve;
  private final PIDController rotationPID = new PIDController(5.0, 0.0, 0.0);

  /** Creates a new TurnToHub. */
  public TurnToHub(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);

    rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    rotationPID.setTolerance(Math.toRadians(Constants.AlignTolerance));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double current = swerve.getHeading().getRadians();
    double target = swerve.aimAtHub().getRadians();

    double omega = rotationPID.calculate(current, target);

    swerve.drive(
        new Translation2d(0, 0),
        omega,
        true
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(
      new Translation2d(0, 0),
      0.0,
      true
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotationPID.atSetpoint();
  }
}
