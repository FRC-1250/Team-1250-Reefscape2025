// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Climber;

public class ShallowClimb extends Command {
  private final Climber cmdClimber;
  private final Current cmdCurrent;

  public ShallowClimb(Climber climber, Current current) {
    addRequirements(climber);
    cmdClimber = climber;
    cmdCurrent = current;
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    cmdClimber.setTorque(cmdCurrent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cmdClimber.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmdClimber.hasPassedClimbThreshold();
  }
}
