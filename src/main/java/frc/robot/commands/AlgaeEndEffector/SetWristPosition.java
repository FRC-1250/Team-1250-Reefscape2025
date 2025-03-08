// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeEndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.AlgaeEndEffector;
import frc.robot.subsystem.AlgaeEndEffector.WristPosition;

public class SetWristPosition extends Command {

  private final AlgaeEndEffector cmdAlgaeEndEffector;
  private final WristPosition cmdWristPosition;

  public SetWristPosition(AlgaeEndEffector algaeEndEffector, WristPosition wristPosition) {
    addRequirements(algaeEndEffector);
    cmdAlgaeEndEffector = algaeEndEffector;
    cmdWristPosition = wristPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmdAlgaeEndEffector.setWristPosition(cmdWristPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmdAlgaeEndEffector.isWristNearPosition(cmdWristPosition);
  }
}
