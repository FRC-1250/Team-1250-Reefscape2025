// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeEndEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.AlgaeEndEffector;
import frc.robot.subsystem.AlgaeEndEffector.IntakeVelocity;

public class IntakeAlgaeWithDelayedStop extends Command {

  private final AlgaeEndEffector cmdAlgaeEndEffector;
  private final Timer cmdTimer;
  private final double cmdDelayBeforeStop;
  private boolean cmdStateTransition = false;

  public IntakeAlgaeWithDelayedStop(AlgaeEndEffector algaeEndEffector) {
    addRequirements(algaeEndEffector);
    cmdAlgaeEndEffector = algaeEndEffector;
    cmdTimer = new Timer();
    cmdDelayBeforeStop = 0.02;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmdAlgaeEndEffector.setIntakeVelocity(IntakeVelocity.INTAKE);
  }

  @Override
  public void execute() {
    if (cmdAlgaeEndEffector.hasAlgae() && !cmdStateTransition) {
      cmdTimer.start();
      cmdStateTransition = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cmdAlgaeEndEffector.holdPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmdTimer.advanceIfElapsed(cmdDelayBeforeStop);
  }
}