// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;


import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunShooter extends Command {
  Shooter shooter; 
  DoubleSupplier distanceSupplier;
  /** Creates a new RunShooter. */
  public RunShooter(Shooter shooter, DoubleSupplier distanceSupplier) {
    this.shooter = shooter;
    this.distanceSupplier = distanceSupplier;
    addRequirements(shooter);
  }

  @Override
  public void initialize() { }

  @Override
  public void execute() {
    // Pose2d robotpose = swerve.getPose();
    // double hubX = Constants.RedHubX;
    // if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
    //   hubX = Constants.BlueHubX;
    // }
    // double distance = Math.sqrt(Math.pow((hubX-robotpose.getX()),(2))+Math.pow((Constants.HubY-robotpose.getY()),(2)));
    // shooter.setShooterSpeed(ShooterConstants.TESTING_DISTANCE);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShooting();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}