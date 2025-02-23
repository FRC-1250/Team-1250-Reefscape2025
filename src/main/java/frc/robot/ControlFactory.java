// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystem.Climber;
import frc.robot.subsystem.CommandSwerveDrivetrain;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.EndEffector;
import frc.robot.subsystem.Limelight;
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
    private final Climber climber;
    private final Limelight limelight;
    private final Translation2d blueReef = new Translation2d(4.490, 4);
    private final Translation2d redReef = new Translation2d(13.05, 4);
    private final double[] highAlgaeAprilTags = { 6, 8, 10, 17, 19, 21 };
    private final double[] lowAlgaeAprilTags = { 7, 9, 11, 18, 20, 22 };
    private final PathConstraints constraints = new PathConstraints(
            3.0,
            3.0,
            Units.degreesToRadians(270),
            Units.degreesToRadians(360));

    public ControlFactory(
            CommandSwerveDrivetrain swerveDrivetrain,
            Elevator elevator,
            EndEffector endEffector,
            Climber climber,
            Limelight limelight,
            SystemLights systemLights) {
        this.swerveDrivetrain = swerveDrivetrain;
        this.elevator = elevator;
        this.endEffector = endEffector;
        this.climber = climber;
        this.limelight = limelight;
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
                endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.DEPLOYED))
                .withName("High dealgae prep");
    }

    public Command reefHighDealgae() {
        return Commands.sequence(
                elevator.cmdSetPosition(Elevator.Position.HIGH_ALGAE),
                endEffector.cmdDealgae()).withName("High dealgae");
    }

    public Command reefHighDealgaeUndo(Position pos) {
        return Commands.sequence(
                elevator.cmdSetPosition(Elevator.Position.HIGH_ALGAE_PREP),
                endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.HOME),
                elevator.cmdSetPosition(pos).withName("High dealgae undo prep"));
    }

    public Command reefLowDealgaePrep() {
        return Commands.sequence(
                elevator.cmdSetPosition(Elevator.Position.LOW_ALGAE_PREP),
                endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.DEPLOYED))
                .withName("Low dealgae prep");
    }

    public Command reefLowDealgae() {
        return Commands.sequence(
                elevator.cmdSetPosition(Elevator.Position.LOW_ALGAE),
                endEffector.cmdDealgae().withName("Low dealgae"));
    }

    public Command reefLowDealgaeUndo(Position pos) {
        return Commands.sequence(
                elevator.cmdSetPosition(Elevator.Position.LOW_ALGAE_PREP),
                endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.HOME),
                elevator.cmdSetPosition(pos).withName("Low dealgae undo prep"));
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

                            if (climber.getHealthStatus() == HealthStatus.ERROR) {
                                systemLights.diagnosticColors.add(PresetColor.BLUE);
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
                elevator.cmdSetPosition(pos)).withName("Home");

    }

    public Command dynamicElevatorPosition(Position position) {
        return Commands.run(() -> {
            double rotationOffset = 0;

            // 0.43 meters is the min distance to the apriltag with bumpers to the reef..
            double distanceToTarget = limelight.getDistanceFromTarget() - 0.43;

            // 0.1016 meters is ~ 1 coral...
            if (distanceToTarget <= 0.125) {
                // Assume every 0.125 meters produced 2.5 rotations
                rotationOffset = distanceToTarget * 20.0;
            }

            // Set the position with the offset applied, if one exists
            elevator.setPosition(position.rotations + rotationOffset);

            if (elevator.isNearPositionAndTolerance(position.rotations, rotationOffset + 0.5)) {
                elevator.previousElevatorPosition = elevator.elevatorPosition;
                elevator.elevatorPosition = position;
            }
        },
                elevator)
                .withName(String.format("Elevator dynamic position - %s", position.name()));
    }

    public Command pathfindToPose(Pose2d targetPose, double goalVelocity) {
        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                goalVelocity // Goal end velocity in meters/sec
        ).withName("Pathfind to pose");
    }

    public void addLimelightVisionMeasurements() {
        var driveState = swerveDrivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        limelight.setRobotOrientation(headingDeg);
        var llMeasurement = limelight.getBotPoseEstimate_wpiBlue_MegaTag2();
        if (llMeasurement != null && llMeasurement.tagCount > 0 && llMeasurement.avgTagDist < 2 && omegaRps < 2.0) {
            swerveDrivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            swerveDrivetrain.addVisionMeasurement(llMeasurement.pose,
                    Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
        }
    }

}
