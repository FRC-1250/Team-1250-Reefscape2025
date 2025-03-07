// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  @Logged(name = "RobotContainer")
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    DriverStation.startDataLog(DataLogManager.getLog());
    Epilogue.bind(this);
    DriverStation.silenceJoystickConnectionWarning(true);

    CommandScheduler.getInstance().onCommandInitialize(
        command -> DataLogManager.log(
            String.format("Command init: %s, with requirements: %s", command.getName(), command.getRequirements())));

    CommandScheduler.getInstance().onCommandFinish(
        command -> DataLogManager.log(String.format("Command finished: %s", command.getName())));

    CommandScheduler.getInstance().onCommandInterrupt(
        command -> DataLogManager.log(String.format("Command interrupted: %s", command.getName())));
  }

  @Override
  public void robotInit() {
    FollowPathCommand.warmupCommand().schedule();
    Pathfinding.setPathfinder(new LocalADStar());
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.determineMaxSpeed();
    m_robotContainer.controlFactory.addLimelightVisionMeasurements();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.controlFactory.displaySubsystemErrorState().schedule();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
