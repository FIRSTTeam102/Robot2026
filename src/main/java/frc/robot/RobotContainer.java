// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import java.io.File;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.BasicShooter;
import frc.robot.commands.ChangeShooterAngle;
import frc.robot.commands.Climbing;
import frc.robot.commands.CompShooting;
import frc.robot.commands.ExtendActuator;
import frc.robot.commands.FowardPiston;
import frc.robot.commands.FullClimbing;
import frc.robot.commands.IntakeFuel;
import frc.robot.commands.IntakeNoPneumatics;
import frc.robot.commands.JoystickClimb;
import frc.robot.commands.ResetEncoder;
import frc.robot.commands.ReverseClimb;
import frc.robot.commands.ReverseFeeder;
import frc.robot.commands.ReversePiston;
import frc.robot.commands.IndexerFeeder;
import frc.robot.commands.ReverseIntake;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoActuator;
import frc.robot.commands.AutoShooter;
import java.io.File;

import frc.robot.commands.RunIndexer;
import frc.robot.commands.ShooterPIDReset;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.ChangeShooterAngle;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.ExtendClimber;

import org.littletonrobotics.junction.LoggedRobot;


public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  final CommandXboxController testerXbox = new CommandXboxController(5);

  private final Indexer indexer = new Indexer();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  public final Climber climber = new Climber();


                                                                          
                                                                                
  private final SendableChooser<Command> autoChooser;






  public RobotContainer() {


    NamedCommands.registerCommand("Intake", new IntakeFuel(intake));
    NamedCommands.registerCommand("Climb Position", new FullClimbing(climber));
    NamedCommands.registerCommand("Extend Piston", new FowardPiston(intake));
    NamedCommands.registerCommand("Rev Shooter", new AutoShooter(shooter, 3500));
    NamedCommands.registerCommand("Indexer Feeder", new IndexerFeeder(indexer));
    NamedCommands.registerCommand("Zone 4 Angle", new AutoActuator(shooter, 0.5));
    NamedCommands.registerCommand("Zone 1 Angle", new AutoActuator(shooter, 0.3));


    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    

        
         

//Gavin's bindings BUBBLE TEAAA
    operatorXbox.leftTrigger().whileTrue( new IntakeFuel(intake));// USE IF ELASTIC () -> Robot.IntakeSpeed.getDouble(Constants.IntakeConstants.INTAKE_DEFAULT_SPEED
    operatorXbox.rightTrigger().whileTrue(new CompShooting(shooter, intake, indexer));
    operatorXbox.x().onTrue(new ExtendActuator(shooter, () -> Robot.actuatorPositionEntry.getDouble(0.0)));
    operatorXbox.a().onTrue(new AutoActuator(shooter, 0.7));
    operatorXbox.b().whileTrue(new ReverseIntake (intake));


    operatorXbox.rightBumper().whileTrue(new IndexerFeeder(indexer));

    operatorXbox.y().whileTrue(Commands.parallel(
      new BasicShooter(shooter,() -> Robot.ShooterSpeed.getDouble(Constants.ShooterConstants.BASIC_SHOOTER_SPEED_DEFAULT))
      ));
    

   // operatorXbox.povRight().whileTrue(new FowardPiston(intake));
   // operatorXbox.povDown().whileTrue(new ReversePiston(intake));


    //chnaging acuator 
    
    //combined subsystem
    //operatorXbox.y().whileTrue(new FullFuelCycle(shooter, indexer, intake));
    
    //operatorXbox.rightTrigger().whileFalse(new IdleIntake(intake));
    //operatorXbox.povUp().whileTrue(new AllianceCheck(shooter, drivebase, indexer));
    operatorXbox.start().whileTrue(new ReverseFeeder(indexer));
    //operatorXbox.povLeft().onTrue(new ShooterPIDReset(shooter)); //for tuning rev shooter pid







    testerXbox.a().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    testerXbox.b().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    testerXbox.x().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    testerXbox.y().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));


  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }



  public static boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) { //No alliance
      return false;
    }
  // Auto: both hubs enabled
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
  // No hub if we aren't in teleop
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
  // No game data, we're assuming that the hub is always enabled
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If the game data isn't right, we're going to default to enabled
        return true;
      }
    }

    // Shift 1 will be blue active if red won auto, and vice versa for red
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // TRANSITION SHIFT
        return true;
    } else if (matchTime > 105) {
      // SHIFT 1
        return shift1Active;
    } else if (matchTime > 80) {
      // SHIFT 2
        return !shift1Active;
    } else if (matchTime > 55) {
      // SHIFT 3
        return shift1Active;
    } else if (matchTime > 30) {
      // SHIFT 4
        return !shift1Active;
    } else {
      // Endgame (last 30s): both hubs active
        return true;
    }
  }

  public static boolean hasGameData() {
    if (DriverStation.getGameSpecificMessage().isEmpty()) {
      return false;
    }
    else {
      return true;
    }
  }

  public static boolean ourAllianceActiveShift1() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    String gameData = DriverStation.getGameSpecificMessage();
  // No game data, we're assuming that the hub is always enabled
    if (gameData.isEmpty()) {
      return true;
    }
    switch (gameData.charAt(0)) {
      case 'R': if (alliance.get() == Alliance.Red) {return false;} else {return true;}
      case 'B': if (alliance.get() == Alliance.Blue) {return false;} else {return true;}
      default: {
        // If the game data isn't right, we're going to default to enabled
        return true;
      }
    }

  }

  public static int timeLeftInShiftSeconds(double currentMatchTime) {
        if (currentMatchTime >= 140) {
            return (int) (currentMatchTime - 130);
        } else if (currentMatchTime >= 130 && currentMatchTime < 140) { //transition
            if (ourAllianceActiveShift1() && hasGameData()) { // Runs if we are active shift 1 & have game data
              return (int) (currentMatchTime - 105);
            }
            else { // Runs if we aren't shift 1, or if we don't have game data as a fallback (just show each shift individually)
              return (int) (currentMatchTime - 130);
            }
        } else if (currentMatchTime >= 105 && currentMatchTime < 130) { //s1
            return (int) (currentMatchTime - 105);
        } else if (currentMatchTime >= 80 && currentMatchTime < 105) { //s2
            return (int) (currentMatchTime - 80);
        } else if (currentMatchTime >= 55 && currentMatchTime < 80) { //s3
            return (int) (currentMatchTime - 55);
        } else if (currentMatchTime >= 30 && currentMatchTime < 55) { //s4
            if (ourAllianceActiveShift1()) {
              return (int) (currentMatchTime - 30);
            }
            else {
              return (int) currentMatchTime;
            }
        } else { //endgame
            return (int) currentMatchTime;
        }
    }

  public void setDriveMode()
  {
    configureBindings();
  }
}