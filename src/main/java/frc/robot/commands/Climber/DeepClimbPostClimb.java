// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.DeepClimber;
import frc.robot.subsystem.DeepClimber.DeepClimberPhase;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeepClimbPostClimb extends Command {
   private final DeepClimber cmdDeepClimber;
   private final Current cmdAmps;
  
    /** Creates a new DeepClimbPreclimb. */

    // Use addRequirements() here to declare subsystem dependencies.
  
public DeepClimbPostClimb(DeepClimber deepClimber, Current amps) {
    addRequirements(deepClimber);
    cmdDeepClimber = deepClimber;
    cmdAmps = amps;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmdDeepClimber.setTorque(cmdAmps);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cmdDeepClimber.stop();
    cmdDeepClimber.PostClimbFlag = true;
  }

  // Returns true when the command should end. 
  @Override
  public boolean isFinished() {
    return cmdDeepClimber.hasPassedClimbThreshold(DeepClimberPhase.CLIMB2);
    
  }
}
