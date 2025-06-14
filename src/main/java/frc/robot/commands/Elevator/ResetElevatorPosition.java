// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.Elevator;

public class ResetElevatorPosition extends InstantCommand {
    private final Elevator cmdElevator;

    public ResetElevatorPosition(Elevator elevator) {
        cmdElevator = elevator;
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        cmdElevator.resetMotorPositionToPosition(0);
    }
}
