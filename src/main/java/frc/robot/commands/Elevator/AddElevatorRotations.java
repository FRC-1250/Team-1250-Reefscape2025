// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Elevator;

public class AddElevatorRotations extends Command {
  private final Elevator cmdElevator;
  private final double cmdRotations;

  public AddElevatorRotations(Elevator elevator, double rotations) {
    cmdElevator = elevator;
    cmdRotations = rotations;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmdElevator.setPosition(cmdElevator.getLeftMotorPosition() + cmdRotations);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmdElevator.isNearPositionAndTolerance(cmdElevator.getLeftMotorPosition() + cmdRotations, 0.25);
  }
}
