// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.DeepClimber;

public class SetDeepClimbTorque extends Command {
    private final DeepClimber cmdDeepClimber;
    private final Current cmdAmps;

    public SetDeepClimbTorque(DeepClimber deepClimber, Current amps) {
        addRequirements(deepClimber);
        cmdDeepClimber = deepClimber;
        cmdAmps = amps;
    }

    // Called when the command is initially scheduled.
    @Override
    public void execute() {
        cmdDeepClimber.setTorque(cmdAmps);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        cmdDeepClimber.stop();
    }

}
