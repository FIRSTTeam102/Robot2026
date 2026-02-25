
/*IMPORTANT!!! 
 *  USE GOOGLE IF YOU DON'T UNDERSTAND SOMETHING!!!
 *   IF YOU STILL DON'T UNDERSTAND CHECK LAST YEARS CODE!!!
 * (also I can't spell half these instructions are probubly spelld wrong :D)
*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO 1: import CANdle/LED files
// TODO 2: import driverstation & Subsystembase

public class Lights extends SubsystemBase {
  // TODO 3: Create a CANdle object

  // TODO 4: Define total # of LEDs on robot

  // TODO 5: Create enum of LED patterns (one for every action + more(ex: robot enabeld, rebot diabled, shooting)) (reserch Paterns :D)
 
  // TODO 6: Store previous animation state
    // Requirement: Prevent constantly restarting the same animation
    // TEST Confirm LEDs do not flicker when repeatedly setting same pattern
  
  public Lights() {}
    // INITIALIZATION 
      // TODO 7: Initialize CANdle with correct CAN ID
        // TEST Confirm CANdle is detected on CAN bus (Phoenix Tuner)

      // TODO 8: Configure CANdle brightness
        // Requirement: LEDs should not be blinding in the pit
        // TEST Visually confirm brightness is reduced to safe level

      // TODO 9:  Configure LED strip type (GRB vs RGB)
        // Requirement: Colors must display correctly
        // TEST Set LEDs to solid red:
          // If they appear green or blue, strip type is wrong

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
    // This method runs every scheduler cycle (~20ms)

    // TODO 14: Decide if periodic() should:
      // Automatically update alliance color
      // Check robot state (enabled/disabled)
      // Or remain empty
        // TEST If using periodic updates:
          //Confirm LEDs do NOT restart animation every 20ms
  }
}
