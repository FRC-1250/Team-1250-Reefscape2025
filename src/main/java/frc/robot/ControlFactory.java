// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystem.CommandSwerveDrivetrain;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.EndEffector;
import frc.robot.subsystem.SystemLights;
import frc.robot.subsystem.Elevator.Position;
import frc.robot.subsystem.SystemLights.PresetColor;
import frc.robot.util.HealthStatus;

/** Add your docs here. */
public class ControlFactory {

    private final CommandSwerveDrivetrain swerveDrivetrain;
    private final Elevator elevator;
    private final EndEffector endEffector;
    private final SystemLights systemLights;
    private final Translation2d blueReef = new Translation2d(4.490, 4);
    private final Translation2d redReef = new Translation2d(13.05, 4);
    private final double[] highAlgaeAprilTags = { 6, 8, 10, 17, 19, 21 };
    private final double[] lowAlgaeAprilTags = { 7, 9, 11, 18, 20, 22 };

    public ControlFactory(CommandSwerveDrivetrain swerveDrivetrain, Elevator elevator, EndEffector endEffector,
            SystemLights systemLights) {
        this.swerveDrivetrain = swerveDrivetrain;
        this.elevator = elevator;
        this.endEffector = endEffector;
        this.systemLights = systemLights;
    }

    public Command lockElevator(double lockDurationInSeconds) {
        return Commands.idle(
                elevator)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .until(() -> endEffector.hasCoral())
                .withTimeout(lockDurationInSeconds);
    }

    public Command reefHighDealgaePrep() {
        return Commands.sequence(
                elevator.cmdSetPosition(Elevator.Position.HIGH_ALGAE_PREP),
                endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.DEPLOYED));
    }

    public Command reefHighDealgae() {
        return Commands.sequence(
                elevator.cmdSetPosition(Elevator.Position.HIGH_ALGAE),
                endEffector.cmdDealgae());
    }

    public Command reefHighDealgaeUndo(Position pos) {
        return Commands.sequence(
                elevator.cmdSetPosition(Elevator.Position.HIGH_ALGAE_PREP),
                endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.HOME),
                elevator.cmdSetPosition(pos));
    }

    public Command reefLowDealgaePrep() {
        return Commands.sequence(
                elevator.cmdSetPosition(Elevator.Position.LOW_ALGAE_PREP),
                endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.DEPLOYED));
    }

    public Command reefLowDealgae() {
        return Commands.sequence(
                elevator.cmdSetPosition(Elevator.Position.LOW_ALGAE),
                endEffector.cmdDealgae());
    }

    public Command reefLowDealgaeUndo(Position pos) {
        return Commands.sequence(
                elevator.cmdSetPosition(Elevator.Position.LOW_ALGAE_PREP),
                endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.HOME),
                elevator.cmdSetPosition(pos));
    }

    public boolean hasLowAlgae() {
        return (idInArray(lowAlgaeAprilTags, LimelightHelpers.getFiducialID("limelight")));
    }

    public boolean hasHighAlgae() {
        return (idInArray(highAlgaeAprilTags, LimelightHelpers.getFiducialID("limelight")));
    }

    public Rotation2d determineHeadingToReef() {
        if (swerveDrivetrain.getOperatorForwardDirection().getDegrees() == 0) {
            return Rotation2d.fromRadians(
                    Math.atan2(
                            blueReef.getY() - swerveDrivetrain.getState().Pose.getY(),
                            blueReef.getX() - swerveDrivetrain.getState().Pose.getX()));
        } else { // 180 degrees
            return Rotation2d.fromRadians(
                    Math.atan2(
                            swerveDrivetrain.getState().Pose.getY() - redReef.getY(),
                            swerveDrivetrain.getState().Pose.getX() - redReef.getX()));
        }
    }

    public Command displaySubsystemErrorState() {
        return Commands.repeatingSequence(
                Commands.runOnce(
                        () -> {
                            systemLights.diagnosticColors.clear();
                            if (elevator.getHealthStatus() == HealthStatus.ERROR) {
                                systemLights.diagnosticColors.add(PresetColor.PURPLE);
                            }

                            if (endEffector.getHealthStatus() == HealthStatus.ERROR) {
                                systemLights.diagnosticColors.add(PresetColor.WHITE);
                            }

                            if (swerveDrivetrain.getHealthStatus() == HealthStatus.ERROR) {
                                systemLights.diagnosticColors.add(PresetColor.RED);
                            }

                            if (systemLights.diagnosticColors.size() == 0) {
                                systemLights.diagnosticColors.add(PresetColor.KELLY_GREEN);
                            }
                            systemLights.cmdSetLEDs(systemLights.diagnosticColors.get(0));
                        }, systemLights),
                Commands.run(
                        () -> {
                            systemLights.cycleDiagnosticColors();
                        }, systemLights).withTimeout(5))
                .withName("Diagnostic lights")
                .ignoringDisable(true);
    }

    private boolean idInArray(double[] arr, double id) {
        for (double i : arr) {
            if (Double.compare(i, id) == 0)
                return true;
        }
        return false;
    }

    public Command homeEndEffectorAndSetElevatorPosition(Position pos) {
        return Commands.sequence(
                endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.HOME),
                endEffector.cmdStopAlgaeMotor(),
                endEffector.cmdStopCoralMotor(),
                Commands.waitSeconds(0.5),
                elevator.cmdSetPosition(pos));

    }

}
