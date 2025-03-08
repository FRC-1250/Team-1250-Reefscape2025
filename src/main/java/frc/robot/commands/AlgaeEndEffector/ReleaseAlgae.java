// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeEndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.AlgaeEndEffector;
import frc.robot.subsystem.AlgaeEndEffector.IntakeVelocity;

public class ReleaseAlgae extends Command {

  private final AlgaeEndEffector cmdAlgaeEndEffector;

  public ReleaseAlgae(AlgaeEndEffector algaeEndEffector) {
    addRequirements(algaeEndEffector);
    cmdAlgaeEndEffector = algaeEndEffector;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmdAlgaeEndEffector.setIntakeVelocity(IntakeVelocity.RELEASE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cmdAlgaeEndEffector.stopIntake();
  }
}