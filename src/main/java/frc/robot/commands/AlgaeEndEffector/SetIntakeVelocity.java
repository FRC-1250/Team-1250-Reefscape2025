// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeEndEffector;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Intake.IntakeVelocity;

public class SetIntakeVelocity extends InstantCommand {
  private final Intake cmdIntake;
  private final IntakeVelocity cmdIntakeVelocity;

  public SetIntakeVelocity(Intake Intake, IntakeVelocity intakeVelocity) {
    addRequirements(Intake);
    cmdIntake = Intake;
    cmdIntakeVelocity = intakeVelocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmdIntake.setIntakeVelocity(cmdIntakeVelocity);
  }

}
