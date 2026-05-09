// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import frc.robot.Robot;
import frc.robot.Constants.IndexerConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import org.littletonrobotics.junction.AutoLogOutput;



import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  /** Creates a new Intake. */
  public Indexer() {}
    private SparkMax indexerMotor = new SparkMax(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
    private RelativeEncoder indexerEncoder = indexerMotor.getEncoder();
    private SparkMax feederMotor = new SparkMax(IndexerConstants.FEEDER_CAN_ID, MotorType.kBrushless); 
    private RelativeEncoder feederEncoder = feederMotor.getEncoder();

  


  public void RunIndexer(){
    indexerMotor.set(Robot.IndexerSpeed.getDouble(IndexerConstants.INDEXER_DEFAULT_SPEED));
      
  }

  public void ReverseIndexer(){
        indexerMotor.set(-(Robot.IndexerSpeed.getDouble(IndexerConstants.INDEXER_DEFAULT_SPEED)));

  }

  public void stopIndexer(){
    indexerMotor.stopMotor();
  }

  public void runFeeder(){
    feederMotor.set(Robot.FeederSpeed.getDouble(IndexerConstants.FEEDER_DEFAULT_SPEED));
  }

  public void reverseFeeder() {
    feederMotor.set(-(Robot.FeederSpeed.getDouble(IndexerConstants.FEEDER_DEFAULT_SPEED)));
  }

  public void stopFeeder() {
    feederMotor.stopMotor();
  }

  public void jiggleIndexer(int counter){
    runFeeder();
    if (counter<=28) {
      RunIndexer();
    }
    else {
      ReverseIndexer();
    }
  }

  public void shakeIndexer(int counter){
    if (counter>=25) {
      ReverseIndexer();
    }
    else {
      stopIndexer();
    }
  }

  @AutoLogOutput
  public double indexerRPM() {
    return indexerEncoder.getVelocity();
  }

  @AutoLogOutput
  public double feederRPM() {
    return feederEncoder.getVelocity();
  }

  
     @AutoLogOutput
  public double checkTempIndex(){
    return indexerMotor.getMotorTemperature();
  }

  @AutoLogOutput
  public double checkTempFeed(){
    return feederMotor.getMotorTemperature();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}