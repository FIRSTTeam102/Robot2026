// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  private SparkMax climberMotor = new SparkMax(ClimberConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
  private RelativeEncoder climberEncoder = climberMotor.getEncoder();
  private DigitalInput limitSwitch = new DigitalInput(ClimberConstants.LIMIT_SWITCH_PORT);


  public Climber() {}

  public void SetClimberSpeed(double speed) {
    climberMotor.set(speed);
  }

  public double getEncoderPosition() {
    return climberEncoder.getPosition();
  }

  public void zeroEncoder() {
    climberEncoder.setPosition(0.0);
  }

  public boolean limitSwitchPressed() {
    //if pressed returns true
    return !limitSwitch.get();
  }

  @Override
  public void periodic() {
    if (limitSwitchPressed()) {
      zeroEncoder();
    }
  }
}
