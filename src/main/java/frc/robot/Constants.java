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

  public static final double BLUE_HUB_X = 4.597; //coordinates of hubs
  public static final double RED_HUB_X = 11.938;
  public static final double HUB_Y = 4.035;

  public static final double TOP_CORNER_Y = 7.55; //top in choreo with blue on left
  public static final double BOTTOM_CORNER_Y = 0.5; //coordinates of each corner to aim at while passing
  public static final double BLUE_CORNER_X = 0.5;
  public static final double RED_CORNER_X = 16.1;

  public static final double ALIGN_TOLERANCE = 1.0; //degrees, tolerance for robot angle alignment

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  
 


  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double DRIVE_FAST_SCALE = 1; //speed of robot
    public static final double DRIVE_PRECISION_SCALE = 0.35; //percent of max speed in precise mode
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1; //minimum input on the joystick that actually has an effect
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.3;
    public static final double TURN_CONSTANT    = 6;
  }
  public static final class ShooterConstants {
    public static final double END_HEIGHT = 56.4; //in inches- might change based on air resistance 
    public static final double STARTING_HEIGHT = 22.25; //TODO change based on final CAD model 
    public static final double GRAVITY = -386.4; //inches per seconds squared
    public static final double SHOOTER_ANGLE = 85.0 * (Math.PI/180); //in degrees might need to convert 
    public static final int SHOOTER_CAN_ID = 40;
    public static final int SERVO_CHANNEL = 2;
    public static final double HIGH_SHOOTER_ANGLE = 85.0; //TODO find good angles for all constants
    public static final double PASSING_ANGLE = 45.0;
    public static final double PASSING_VELOCITY = 0.5744;
    public static final double TESTING_DISTANCE_DEFAULT = 91.0; //distance instead of pose to test shooter
    public static final double ACTUATOR_EXTENSION = 0.80;
    public static final double SHOOTINGVELOCITY_DEFAULT = -1.0;

    public static final double SHOOTER_P_DEFAULT = 0.000152;
    public static final double SHOOTER_I_DEFAULT = 0;
    public static final double SHOOTER_D_DEFAULT = 0;

    //testing diatnces 
    public static final double FRONT_TRENCH = 0.777;
    public static final double FRONT_TOWER = 0.7184;
    
    public static final double kS = 0.079571; //found with sysid
    public static final double kV = 0.11073;

    public static final double PIDRPMTOLERANCE = 25; //may need to be changed

    }
  
  public static final class IndexerConstants {
    public static final int INDEXER_MOTOR_ID = 50; 
    public static final int FEEDER_CAN_ID = 51; 
    public static final double FEEDER_DEFAULT_SPEED = 0.8; //TODO chnage based on robot 
    public static final double INDEXER_DEFAULT_SPEED = -0.8; //TODO chnage based on robot 
  }

  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 30;
    public static final double INTAKE_DEFAULT_SPEED = 1.0; //change based on robot
    public static final int PISTON_ID = 2 ;
  }

  public static final class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 60; //final
    public static final int LIMIT_SWITCH_PORT = 6; //temp
    public static final double CLIMBER_DEFAULT_SPEED = 0.5; //worked in testing but may need to be increased
    public static final double CLIMBER_ENCODER_EXTENSION = -1350.0; //found on rev on test bed
    public static final double CLIMBER_ENCODER_MIN_EXTENSION = 0.0; 
    public static final double CLIMBER_ENCODER_TOLERANCE = 20.0; //seems like a reasonable tolerance but may need adjusting
  } 

  
}
