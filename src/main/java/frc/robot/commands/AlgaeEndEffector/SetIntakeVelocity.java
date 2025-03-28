// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeEndEffector;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.AlgaeEndEffector;
import frc.robot.subsystem.AlgaeEndEffector.IntakeVelocity;

public class SetIntakeVelocity extends InstantCommand {
  private final AlgaeEndEffector cmdAlgaeEndEffector;
  private final IntakeVelocity cmdIntakeVelocity;

  public SetIntakeVelocity(AlgaeEndEffector algaeEndEffector, IntakeVelocity intakeVelocity) {
    addRequirements(algaeEndEffector);
    cmdAlgaeEndEffector = algaeEndEffector;
    cmdIntakeVelocity = intakeVelocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmdAlgaeEndEffector.setIntakeVelocity(cmdIntakeVelocity);
  }

}
