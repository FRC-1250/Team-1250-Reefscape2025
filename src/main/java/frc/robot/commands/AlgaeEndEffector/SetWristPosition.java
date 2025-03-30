// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeEndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Wrist;
import frc.robot.subsystem.Wrist.WristPosition;

public class SetWristPosition extends Command {

    private final Wrist cmdWrist;
    private final WristPosition cmdWristPosition;

    public SetWristPosition(Wrist Wrist, WristPosition wristPosition) {
        addRequirements(Wrist);
        cmdWrist = Wrist;
        cmdWristPosition = wristPosition;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        cmdWrist.setWristPosition(cmdWristPosition);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return cmdWrist.isWristNearPosition(cmdWristPosition);
    }
}
