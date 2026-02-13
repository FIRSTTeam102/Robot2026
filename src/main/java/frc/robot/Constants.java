// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final double BlueHubX = 4.597;
  public static final double RedHubX = 11.938;
  public static final double HubY = 4.035;

  public static final double AlignTolerance = 1.0; //deg

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class ShooterConstants {
    public static final double END_HEIGHT = 56.4; //in inches- might change based on air resistance 
    public static final double STARTING_HEIGHT = 22.25; //TODO change based on final CAD model 
    public static final double GRAVITY = -386.4; //inches per seconds squared

    public static final double SHOOTER_ANGLE = 85; //in degrees might need to convert 
    public static final int SHOOTER_CAN_ID = 20;
    public static final double HIGH_SHOOTER_ANGLE = 78.0; //TODO find good angles for all constants

    public static final double PASSING_ANGLE = 67.0;
   // public static final double PASSING_VELOCITY = 300.766; //inches

    public static final int ACTUATOR_CAN_ID = 9; //TODO must change based on robot (not actually a can id probably)
    public static final double TESTING_DISTANCE = 5.0; //distance instead of pose to test shooter
    public static final double ACTUATOR_EXTENSION = 0.80;
  
}
  public static final class ElasticDefaults {
    public static final double INTAKE_DEFAULT_SPEED = 0.67;
  }

  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 41; //TODO change to correct ID

  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static double DriveFastScale = 1;
    public static double DrivePrecisionScale = 0.35;
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.3;
    public static final double TURN_CONSTANT    = 6;
  }
}
