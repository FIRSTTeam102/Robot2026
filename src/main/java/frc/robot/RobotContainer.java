// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.BasicShooter;
import frc.robot.commands.ChangeShooterAngle;
import frc.robot.commands.ExtendActuator;
import frc.robot.commands.FullFuelCycle;
import frc.robot.commands.IntakeFuel;
import frc.robot.commands.RunShooter;
import frc.robot.Robot;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TurnToHub;
import frc.robot.commands.AimWhileMoving;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import frc.robot.commands.RunIndexer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.ChangeShooterAngle;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunClimber;

import org.littletonrobotics.junction.LoggedRobot;

import swervelib.SwerveInputStream;

public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  final CommandXboxController testerXbox = new CommandXboxController(5);

  private final Indexer indexer = new Indexer();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();


  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
                                                                          
                                                                                
  private final SendableChooser<Command> autoChooser;
  
                                                                                /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(1.0)
                                                            .scaleRotation(.5)
                                                            .allianceRelativeControl(true)
                                                            .robotRelative(false);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(1)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));


  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                    () -> -driverXbox.getLeftY(),
                                                                    () -> -driverXbox.getLeftX())
                                                                  .robotRelative(false)
                                                                  .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                                  .deadband(OperatorConstants.DEADBAND)
                                                                  .scaleTranslation(DrivebaseConstants.DRIVE_FAST_SCALE)
                                                                  .allianceRelativeControl(true);
  
  Command driveRobotOrientAngularVelocity = drivebase.driveRobotOriented(driveRobotOriented);





  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
    intake.pistonFoward();
  }

  private void configureBindings() {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

        driverXbox.leftTrigger().onTrue(Commands.runOnce(
          ()->driveAngularVelocity.scaleTranslation(Constants.DrivebaseConstants.DRIVE_PRECISION_SCALE)
                                  .scaleRotation(0.3)
                                  ))
                      .onFalse(Commands.runOnce(
          ()->driveAngularVelocity.scaleTranslation(Constants.DrivebaseConstants.DRIVE_FAST_SCALE)
                                  .scaleRotation(0.5)));
                                  
        //Enable robotRelative driving if the right trigger is pressed.
        driverXbox.rightTrigger().onTrue(Commands.runOnce(
          ()->driveAngularVelocity.robotRelative(true)
                                  .allianceRelativeControl(false)
                                  ))
                        .onFalse(Commands.runOnce(
          ()->driveAngularVelocity.robotRelative(false)
                                  .allianceRelativeControl(true)
                        ));
        driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
        driverXbox.back().whileTrue(drivebase.centerModulesCommand());
        driverXbox.leftBumper().whileTrue(new AimWhileMoving(
          drivebase,
            () -> -driverXbox.getLeftY(),
            () -> -driverXbox.getLeftX()
          )
        );

        driverXbox.rightBumper().whileTrue(new TurnToHub(drivebase));
        
        
    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }
    //running motors
    operatorXbox.leftTrigger().whileTrue(new RunIndexer(indexer));
    operatorXbox.leftBumper().whileTrue(new RunFeeder(indexer));
    operatorXbox.povLeft().whileTrue(new RunShooter(shooter));
    operatorXbox.rightBumper().whileTrue(new BasicShooter(shooter,ShooterConstants.FRONT_TRENCH));
    operatorXbox.povRight().whileTrue(new BasicShooter(shooter,ShooterConstants.FRONT_TOWER));


    //chnaging acuator 
    operatorXbox.a().onTrue(new ChangeShooterAngle(shooter, ShooterConstants.HIGH_SHOOTER_ANGLE));
    operatorXbox.b().onTrue(new ChangeShooterAngle(shooter, ShooterConstants.PASSING_ANGLE));
    operatorXbox.x().onTrue(new ExtendActuator(shooter, () -> Robot.actuatorPositionEntry.getDouble(1.0)));
   
    //combined subsystem
    operatorXbox.y().whileTrue(new FullFuelCycle(shooter, indexer, intake));
    operatorXbox.rightTrigger().whileTrue(new IntakeFuel(intake, () -> Robot.IntakeSpeed.getDouble(Constants.IntakeConstants.INTAKE_DEFAULT_SPEED)));
   
    operatorXbox.povDown().onTrue(new RunClimber(climber));

    testerXbox.a().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    testerXbox.b().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    testerXbox.x().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    testerXbox.y().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));


  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void ZeroGyro(){
    drivebase.zeroGyroWithAlliance();
  }

  public void setDriveMode()
  {
    configureBindings();
  }
}