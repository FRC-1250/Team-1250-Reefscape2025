// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import static edu.wpi.first.units.Units.Amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Elevator;

public class HomeElevatorBasedOnAmps extends Command {
    private final Elevator cmdElevator;

    public HomeElevatorBasedOnAmps(Elevator elevator) {
        cmdElevator = elevator;
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        cmdElevator.setSpeed(-0.1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        cmdElevator.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return cmdElevator.isBeyondAmpLimit(Amp.of(10));
    }
}
