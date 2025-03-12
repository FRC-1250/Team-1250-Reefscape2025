// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.Elevator.ElevatorPosition;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorPosition extends Command {
  private final Elevator cmdElevator;
  private final ElevatorPosition cmdPosition;

  public SetElevatorPosition(Elevator elevator, ElevatorPosition position) {
    cmdElevator = elevator;
    cmdPosition = position;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmdElevator.setPosition(cmdPosition.rotations);
  }

  @Override
  public void end(boolean interrupted) {
    cmdElevator.previousElevatorPosition = cmdElevator.elevatorPosition;
    cmdElevator.elevatorPosition = cmdPosition;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmdElevator.isNearPositionAndTolerance(cmdPosition.rotations, 0.25);
  }
}
