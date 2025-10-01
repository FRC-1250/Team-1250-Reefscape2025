// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeEndEffector;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.Intake;

public class HoldIntakePosition extends InstantCommand {

    private final Intake cmdIntake;

    public HoldIntakePosition(Intake intake) {
        addRequirements(intake);
        cmdIntake = intake;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        cmdIntake.holdPosition();
    }
}
