// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.DeepClimber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetDeepClimberPosition extends Command {
 
   private final DeepClimber cmdDeepClimber;

   public SetDeepClimberPosition(DeepClimber deepClimber){
      addRequirements(deepClimber);
      cmdDeepClimber = deepClimber;
   }
    // Use addRequirements() here to declare subsystem dependencies.
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmdDeepClimber.setTorque(Amps.of(20));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cmdDeepClimber.setTorque(Amps.of(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return
    cmdDeepClimber.getRotations()>=20;
  }
}
