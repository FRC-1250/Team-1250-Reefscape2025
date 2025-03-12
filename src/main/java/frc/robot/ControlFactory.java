// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;

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
import frc.robot.commands.AlgaeEndEffector.AddWristRotations;
import frc.robot.commands.AlgaeEndEffector.IntakeAlgae;
import frc.robot.commands.AlgaeEndEffector.ReleaseAlgae;
import frc.robot.commands.AlgaeEndEffector.SetWristPosition;
import frc.robot.commands.Climber.ShallowClimb;
import frc.robot.commands.Elevator.AddElevatorRotations;
import frc.robot.commands.Elevator.ResetElevatorPosition;
import frc.robot.commands.Elevator.SetElevatorPosition;
import frc.robot.subsystem.AlgaeEndEffector;
import frc.robot.subsystem.Climber;
import frc.robot.subsystem.CommandSwerveDrivetrain;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.EndEffector;
import frc.robot.subsystem.Limelight;
import frc.robot.subsystem.SystemLights;
import frc.robot.subsystem.AlgaeEndEffector.IntakeVelocity;
import frc.robot.subsystem.AlgaeEndEffector.WristPosition;
import frc.robot.subsystem.Elevator.ElevatorPosition;
import frc.robot.subsystem.SystemLights.PresetColor;
import frc.robot.util.HealthStatus;

/**
 * Factory class intended to provide convenience commands that have some general
 * assumpions made to make button mapping easier. It also provides some helper
 * methods for commands or triggers.
 */
public class ControlFactory {

    private final CommandSwerveDrivetrain swerveDrivetrain;
    private final Elevator elevator;
    private final EndEffector endEffector;
    private final SystemLights systemLights;
    private final Climber climber;
    private final Limelight limelight;
    private final AlgaeEndEffector algaeEndEffector;
    private final Translation2d blueReef = new Translation2d(4.490, 4);
    private final Translation2d redReef = new Translation2d(13.05, 4);
    private final double[] lowAlgaeAprilTags = { 6, 8, 10, 17, 19, 21 };
    private final double[] highAlgaeAprilTags = { 7, 9, 11, 18, 20, 22 };

    // Use similar values to when we're apporaching a scoring position in a real
    // auto for waypointing
    private final PathConstraints adamsSmoothPathConstraints = new PathConstraints(
            1.2,
            3.0,
            Units.degreesToRadians(270),
            Units.degreesToRadians(720));

    public ControlFactory(
            CommandSwerveDrivetrain swerveDrivetrain,
            Elevator elevator,
            EndEffector endEffector,
            Climber climber,
            Limelight limelight,
            AlgaeEndEffector algaeEndEffector,
            SystemLights systemLights) {
        this.swerveDrivetrain = swerveDrivetrain;
        this.elevator = elevator;
        this.endEffector = endEffector;
        this.climber = climber;
        this.limelight = limelight;
        this.algaeEndEffector = algaeEndEffector;
        this.systemLights = systemLights;
    }

    /*
     * Climber
     */

    public Command cmdShallowClimb() {
        return new ShallowClimb(climber, Amps.of(80));
    }

    /*
     * Elevator
     */
    public Command cmdSetElevatorHome() {
        return new SetElevatorPosition(elevator, ElevatorPosition.HOME);
    }

    public Command cmdSetElevatorBarge() {
        return new SetElevatorPosition(elevator, ElevatorPosition.BARGE);
    }

    public Command cmdSetElevatorHighAlgae() {
        return new SetElevatorPosition(elevator, ElevatorPosition.HIGH_ALGAE);
    }

    public Command cmdSetElevatorLowAlgae() {
        return new SetElevatorPosition(elevator, ElevatorPosition.LOW_ALGAE);
    }

    public Command cmdSetElevatorPosition(ElevatorPosition position) {
        return new SetElevatorPosition(elevator, position);
    }

    public Command cmdAddElevatorRotations(double rotations) {
        return new AddElevatorRotations(elevator, rotations);
    }

    public Command cmdResetElevatorPosition() {
        return new ResetElevatorPosition(elevator);
    }

    /*
     * Algae intake
     */
    public Command cmdIntakeAlgaeFloor() {
        return Commands.sequence(
                new IntakeAlgae(algaeEndEffector, IntakeVelocity.INTAKE),
                new SetWristPosition(algaeEndEffector, WristPosition.FLOOR));
    }

    public Command cmdIntakeAlgaeReef() {
        return Commands.sequence(
                new IntakeAlgae(algaeEndEffector, IntakeVelocity.INTAKE),
                new SetWristPosition(algaeEndEffector, WristPosition.REEF));
    }

    public Command cmdReleaseAlgaeBarge() {
        return Commands.sequence(
                new SetWristPosition(algaeEndEffector, WristPosition.BARGE),
                new ReleaseAlgae(algaeEndEffector, IntakeVelocity.RELEASE));
    }

    public Command cmdSetWristHome() {
        return new SetWristPosition(algaeEndEffector, WristPosition.HOME);
    }

    public Command cmdIntakeAlgae() {
        return new IntakeAlgae(algaeEndEffector, IntakeVelocity.INTAKE);
    }

    public Command cmdReleaseAlgae() {
        return new ReleaseAlgae(algaeEndEffector, IntakeVelocity.RELEASE);
    }

    public Command cmdSetWristPosition(WristPosition position) {
        return new SetWristPosition(algaeEndEffector, position);
    }

    public Command cmdAddWristRotations(double rotations) {
        return new AddWristRotations(algaeEndEffector, rotations);
    }

    /*
     * Diag lights
     */

    public Command cmdDisplaySubsystemErrorState() {
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

    /*
     * Drive
     */

    public Command cmdPathfindToPose(Pose2d targetPose, double goalVelocity) {
        return AutoBuilder.pathfindToPose(
                targetPose,
                adamsSmoothPathConstraints,
                goalVelocity // Goal end velocity in meters/sec
        ).withName("Pathfind to pose");
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

    /*
     * Limelight
     */

    public boolean hasLowAlgae() {
        return (idInArray(lowAlgaeAprilTags, limelight.getFid()));
    }

    public boolean hasHighAlgae() {
        return (idInArray(highAlgaeAprilTags, limelight.getFid()));
    }

    private boolean idInArray(double[] arr, double id) {
        for (double i : arr) {
            if (Double.compare(i, id) == 0)
                return true;
        }
        return false;
    }

    /*
     * To be removed
     */

    public Command lockElevator(double lockDurationInSeconds) {
        return Commands.idle(
                elevator)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .until(() -> endEffector.hasCoral())
                .withTimeout(lockDurationInSeconds);
    }

    public Command reefHighDealgaePrep() {
        return Commands.sequence(
                elevator.cmdSetPosition(Elevator.ElevatorPosition.HIGH_ALGAE_PREP),
                endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.DEPLOYED))
                .withName("High dealgae prep");
    }

    public Command reefHighDealgae() {
        return Commands.sequence(
                elevator.cmdSetPosition(Elevator.ElevatorPosition.HIGH_ALGAE),
                endEffector.cmdDealgae()).withName("High dealgae");
    }

    public Command reefHighDealgaeUndo(ElevatorPosition pos) {
        return Commands.sequence(
                elevator.cmdSetPosition(Elevator.ElevatorPosition.HIGH_ALGAE_PREP),
                endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.HOME),
                elevator.cmdSetPosition(pos).withName("High dealgae undo prep"));
    }

    public Command reefLowDealgaePrep() {
        return Commands.sequence(
                elevator.cmdSetPosition(Elevator.ElevatorPosition.LOW_ALGAE_PREP),
                endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.DEPLOYED))
                .withName("Low dealgae prep");
    }

    public Command reefLowDealgae() {
        return Commands.sequence(
                elevator.cmdSetPosition(Elevator.ElevatorPosition.LOW_ALGAE),
                endEffector.cmdDealgae().withName("Low dealgae"));
    }

    public Command reefLowDealgaeUndo(ElevatorPosition pos) {
        return Commands.sequence(
                elevator.cmdSetPosition(Elevator.ElevatorPosition.LOW_ALGAE_PREP),
                endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.HOME),
                elevator.cmdSetPosition(pos).withName("Low dealgae undo prep"));
    }

    public Command homeEndEffectorAndSetElevatorPosition(ElevatorPosition pos) {
        return Commands.sequence(
                endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.HOME),
                endEffector.cmdStopAlgaeMotor(),
                endEffector.cmdStopCoralMotor(),
                Commands.waitSeconds(0.5),
                elevator.cmdSetPosition(pos)).withName("Home");

    }

    public Command dynamicElevatorPosition(ElevatorPosition position) {
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

}
