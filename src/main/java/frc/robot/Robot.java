// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import frc.robot.Constants;
import frc.robot.commands.FowardPiston;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class Robot extends LoggedRobot {
  private NetworkTableEntry PIDinputentry;

  private Command m_autonomousCommand;
  public static NetworkTableEntry IndexerSpeed;
  public static NetworkTableEntry ShooterSpeed;
  public static NetworkTableEntry IntakeSpeed;
  public static NetworkTableEntry ClimberSpeed;
  public static NetworkTableEntry FeederSpeed;
  public static NetworkTableEntry Distance;
  public static NetworkTableEntry ShooterP;
  public static NetworkTableEntry ShooterI;
  public static NetworkTableEntry ShooterD;
  
  private RobotContainer m_robotContainer;
  public static NetworkTableEntry actuatorPositionEntry; 

  private static Robot   instance;
  Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  


  private Timer disabledTimer;


  
  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
    public void robotInit() {
      NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        actuatorPositionEntry = table.getEntry("Position of actuator");
        actuatorPositionEntry.setDouble(0.0);
                
        IndexerSpeed = table.getEntry("Indexer Speed");
        ShooterSpeed = table.getEntry("Shooter Basic Velocity");
        IntakeSpeed = table.getEntry("Intake Speed");
        ClimberSpeed = table.getEntry("Climber Speed");
        FeederSpeed = table.getEntry("Feeder Speed");
        Distance = table.getEntry("Manual Distance From Hub");
        ShooterP = table.getEntry("Shooter P Value");
        ShooterI = table.getEntry("Shooter I Value");
        ShooterD = table.getEntry("Shooter D Value");
        


        IndexerSpeed.setDouble(Constants.IndexerConstants.INDEXER_DEFAULT_SPEED);
        IntakeSpeed.setDouble(Constants.IntakeConstants.INTAKE_DEFAULT_SPEED);
        ShooterSpeed.setDouble(Constants.ShooterConstants.SHOOTINGVELOCITY_DEFAULT);
        ClimberSpeed.setDouble(Constants.ClimberConstants.CLIMBER_DEFAULT_SPEED);
        FeederSpeed.setDouble(Constants.IndexerConstants.FEEDER_DEFAULT_SPEED);
        Distance.setDouble(Constants.ShooterConstants.TESTING_DISTANCE_DEFAULT);
        ShooterP.setDouble(Constants.ShooterConstants.SHOOTER_P_DEFAULT);
        ShooterI.setDouble(Constants.ShooterConstants.SHOOTER_I_DEFAULT);
        ShooterD.setDouble(Constants.ShooterConstants.SHOOTER_D_DEFAULT);


      Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      Logger.start();

      //adds acutatuor positon as a chnagabel  value on elastic  
        actuatorPositionEntry = table.getEntry("Position of actuator");
        actuatorPositionEntry.setDouble(0.0);

      Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      Logger.start();

      
        
      
    }

  @Override
  public void robotPeriodic() {
    

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

  }

   

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    compressor.enableDigital();
  }

  @Override
  public void teleopPeriodic() {
    //taking out bc we just used actuatorPositionEntry.getDouble instead of using variable
    // double actuatorPosition = actuatorPositionEntry.getDouble(0);

        
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
