// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IndexerFeeder extends Command {
  static int counter;
  Indexer indexer;    
  Shooter shooter;
  DoubleSupplier speedSupplier;
    /** Creates a new IntakeFuel. */
    public IndexerFeeder(Indexer indexer, Shooter shooter) {
      this.indexer =indexer;
      this.shooter = shooter;
    addRequirements(indexer);
    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    indexer.RunIndexer();
    if (shooter.shooterRPM()>1000) {
        indexer.runFeeder();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      counter ++;
               if (counter <= 20){
              indexer.RunIndexer();}
            else if (counter > 20){
              indexer.ReverseIndexer();
            }
            if (counter > 30){
                counter = 0;
            }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stopIndexer();
    indexer.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
