
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.DeepClimber;
import frc.robot.subsystem.DeepClimber.DeepClimberPhase;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeepClimb extends Command {

  private final DeepClimber cmdDeepClimber;
  private final DeepClimberPhase cmdDeepClimberPhase;
  private final Current cmdFeedforward;

  public DeepClimb(DeepClimber deepClimber, DeepClimberPhase deepClimberPhase, Current feedforward) {
    addRequirements(deepClimber);
    cmdDeepClimber = deepClimber;
    cmdDeepClimberPhase = deepClimberPhase;
    cmdFeedforward = feedforward;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmdDeepClimber.setPosition(cmdDeepClimberPhase, cmdFeedforward);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cmdDeepClimber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmdDeepClimber.isNearPositionAndTolerance(cmdDeepClimberPhase.rotations, 1);
  }
}
