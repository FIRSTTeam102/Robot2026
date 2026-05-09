// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;



import org.littletonrobotics.junction.AutoLogOutput;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {}
        private SparkMax intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        private PneumaticHub hub = new PneumaticHub(2);
        private  Solenoid solenoid =  new Solenoid(2, PneumaticsModuleType.REVPH, IntakeConstants.PISTON_ID);
        private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();


        public void startCompressor(){
          hub.enableCompressorDigital();
        }

        
        public void IntakeTheFuel(double speed){
          intakeMotor.set(speed);
        }

        
        public void pistonFoward(){
          solenoid.set(true);
        //extends piston 
        
        }

         public void pistonReverse(){
          solenoid.set(false); //retracts piston 
        }

        @AutoLogOutput
        public boolean pistonExtended() {
          return solenoid.get();
        }

        @AutoLogOutput
        public boolean pressureFull() {
          return !hub.getPressureSwitch();
        }

        @AutoLogOutput
        public boolean compressorEnabled() {
          return hub.getCompressor();
        }

        @AutoLogOutput
       public double checkTempIntake(){
           return intakeMotor.getMotorTemperature();
        }

        @AutoLogOutput
        public double getIntakeRPM() {
          return intakeEncoder.getVelocity();
        }

  @Override
  public void periodic() {
    
  }
}