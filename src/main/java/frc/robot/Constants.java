// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
  public static final class ShooterConstants {
    public static final double END_HEIGHT = 56.4; //in inches- might change based on air resistance 
    public static final double STARTING_HEIGHT = 22.25; //TODO change based on final CAD model 
    public static final double GRAVITY = -386.4; //inches per seconds squared
    public static final double SHOOTER_ANGLE = 85; //in degrees might need to convert 
    public static final int SHOOTER_CAN_ID = 40;
    public static final int ACTUATOR_CAN_ID = 9; //TODO must change based on robot (not actually a can id probably)
    public static final double HIGH_SHOOTER_ANGLE = 75.0; //TODO find good angles for all constants
    public static final double PASSING_ANGLE = 55.0;
    public static final double TESTING_DISTANCE = 5.0; //distance instead of pose to test shooter
    public static final double ACTUATOR_EXTENSION = 0.80;
    }
  
  public static final class IndexerConstants {
    public static final int INDEXER_MOTOR_ID = 50; 
    public static final int FEEDER_CAN_ID = 51; 
    public static final double FEEDER_SPEED = -0.5; //TODO chnage based on robot 
    public static final double INDEXER_SPEED = -0.5; //TODO chnage based on robot 
  }

  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 30;
    public static final double INTAKE_SPEED = 0.8; //change based on robot
  }
}
