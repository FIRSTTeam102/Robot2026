
/*IMPORTANT!!! 
 *  USE GOOGLE IF YOU DON'T UNDERSTAND SOMETHING!!!
 *   IF YOU STILL DON'T UNDERSTAND CHECK LAST YEARS CODE!!!
 * (also I can't spell half these instructions are probubly spelld wrong :D)
*/
package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleConfigurator;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.encoders.CanAndMagSwerve;

public class Lights extends SubsystemBase {

  private CANdle candle;
  private int numberofLights = 60;

  public enum AnimationTypes {
    RedAlliance,
    BlueAlliance,
    Rainbow
  }

  private AnimationTypes previousAnimation = AnimationTypes.Rainbow;


  // TODO 5: Create enum of LED patterns (one for every action + more(ex: robot enabeld, rebot diabled, shooting)) (reserch Paterns :D)
 
  // TODO 6: Store previous animation state
    // Requirement: Prevent constantly restarting the same animation
    // TEST Confirm LEDs do not flicker when repeatedly setting same pattern
  
  public Lights() {
    candle = new CANdle(6);

    CANdleConfiguration config = new CANdleConfiguration();
      config.LED.BrightnessScalar = 0.5;
      config.LED.StripType = StripTypeValue.RGB;
    candle.getConfigurator().apply(config); 
  }

  public void setPattern(AnimationTypes animation) {
    var toAnimate = new RainbowAnimation(0, 67);
    candle.setControl(toAnimate);
  }

    // LED CONTROL METHOD
      // TODO 10: Create method: setPattern(AnimationType type)
        // Requirement: Robot code can change LED pattern based on state
        // TEST Call method from RobotContainer and verify LEDs change
     
        // TODO 11: Add switch (if-else if-else) statement inside setPattern()
        // Requirement: Each enum value produces a unique animation
        // TEST Verify each case produces visually different behavior
     
        // TODO 12: Add default case (Rainbow)
        // Requirement: LEDs should never be off unless intentional
   
    // ALLIANCE LOGIC
      //TODO 13: Create method: setForAllianceDefault()
        // Requirement: LEDs automatically show Red or Blue alliance
        // TEST Enable robot while connected to FMS or simulate alliance
          // Confirm LEDs match DriverStation alliance color
          // If no alliance detected, fallback to Rainbow

  @Override
  public void periodic() {
    System.out.println("hi");
    setPattern(AnimationTypes.Rainbow);
    // This method runs every scheduler cycle (~20ms)

    // TODO 14: Decide if periodic() should:
      // Automatically update alliance color
      // Check robot state (enabled/disabled)
      // Or remain empty
        // TEST If using periodic updates:
          //Confirm LEDs do NOT restart animation every 20ms
  }
}
