// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;

import java.util.Map;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.commands.AlgaeEndEffector.AddWristRotations;
import frc.robot.commands.AlgaeEndEffector.SetIntakeVelocity;
import frc.robot.commands.AlgaeEndEffector.SetWristPosition;
import frc.robot.commands.Climber.DeepClimb;
import frc.robot.commands.Climber.DeepClimbPostClimb;
import frc.robot.commands.Climber.DeepClimbPreclimb;
import frc.robot.commands.Climber.SetDeepClimbTorque;
import frc.robot.commands.Elevator.AddElevatorRotations;
import frc.robot.commands.Elevator.ResetElevatorPosition;
import frc.robot.commands.Elevator.SetElevatorPosition;
import frc.robot.commands.Elevator.StopElevator;
import frc.robot.subsystem.CommandSwerveDrivetrain;
import frc.robot.subsystem.DeepClimber;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.Limelight;
import frc.robot.subsystem.SystemLights;
import frc.robot.subsystem.Intake.IntakeVelocity;
import frc.robot.subsystem.Wrist.WristPosition;
import frc.robot.subsystem.DeepClimber.DeepClimberPhase;
import frc.robot.subsystem.Elevator.ElevatorPosition;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.SystemLights.PresetColor;
import frc.robot.subsystem.Wrist;
import frc.robot.util.HealthMonitor;
import frc.robot.util.HealthStatus;
import frc.robot.util.ReefScoringMap;

/**
 * Factory class intended to provide convenience commands that have some general
 * assumpions made to make button mapping easier. It also provides some helper
 * methods for commands or triggers.
 */
public class ControlFactory {

    private final CommandSwerveDrivetrain swerveDrivetrain;
    private final Elevator elevator;
    private final SystemLights systemLights;
    private final Wrist wrist;
    private final Intake intake;
    private final Limelight limelight;
    private final DeepClimber deepClimber;
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
            DeepClimber deepClimber,
            Wrist wrist,
            Intake intake,
            Limelight limelight,
            SystemLights systemLights) {
        this.swerveDrivetrain = swerveDrivetrain;
        this.elevator = elevator;
        this.deepClimber = deepClimber;
        this.limelight = limelight;
        this.wrist = wrist;
        this.intake = intake;
        this.systemLights = systemLights;
    }

    /*
     * Climber
     */

    public Command cmdDeepClimbRawTorque() {
        return new SetDeepClimbTorque(deepClimber, Amps.of(80));
    }

    public Command cmdDeepClimb() {
        return new DeepClimb(deepClimber, DeepClimberPhase.CLIMB, Amps.of(80))
                .withName("Climber: Deep climb");
    }

    private int DeepClimbSelector() {
        if (deepClimber.PreClimbFlag == true && deepClimber.PostClimbFlag == true) {
            return 1;

        } else if (deepClimber.PreClimbFlag == true && deepClimber.PostClimbFlag == false) {
            return 2;
        } else if (deepClimber.PreClimbFlag == false && deepClimber.PostClimbFlag == false) {
            return 3;
        } else {
            return 0;
        }

    }

    public Command cmdDeepClimbSelector() {
        return new SelectCommand<>(Map.ofEntries(
                Map.entry(2, new DeepClimbPostClimb(deepClimber, Amps.of(40))),
                Map.entry(3, new DeepClimbPreclimb(deepClimber, Amps.of(80)))),
                () -> DeepClimbSelector());
    }

    /*
     * Elevator
     */

    public Command cmdStopElevatorMotors() {
        return new StopElevator(elevator);
    }

    public Command cmdSetElevatorPosition(ElevatorPosition position) {
        return new SetElevatorPosition(elevator, position)
                .withName(String.format("Elevator: %s", position.name()));
    }

    public Command cmdAddElevatorRotations(double rotations) {
        return new AddElevatorRotations(elevator, rotations);
    }

    public Command cmdResetElevatorPosition() {
        return new ResetElevatorPosition(elevator);
    }

    public Command cmdSetElevatorToReefAlgae() {
        return new ConditionalCommand(
                cmdSetElevatorPosition(ElevatorPosition.HIGH_ALGAE),
                cmdSetElevatorPosition(ElevatorPosition.LOW_ALGAE),
                () -> hasHighAlgae());
    }

    /*
     * Algae intake
     */
    public Command cmdSetWristPosition(WristPosition position) {
        return new SetWristPosition(wrist, position);
    }

    public Command cmdAddWristRotations(double rotations) {
        return new AddWristRotations(wrist, rotations);
    }

    public Command cmdSetIntakeVelocity(IntakeVelocity velocity) {
        return new SetIntakeVelocity(intake, velocity);
    }

    public Command cmdHomeIntake() {
        return new ConditionalCommand(
                cmdHomeIntakeWithAlgae(),
                cmdHomeIntakeNoAlgae(),
                () -> intake.hasAlgae());
    }

    public Command cmdHomeIntakeNoAlgae() {
        return Commands.sequence(
                cmdSetIntakeVelocity(IntakeVelocity.STOP),
                cmdSetWristPosition(WristPosition.HOME));
    }

    public Command cmdHomeIntakeWithAlgae() {
        return Commands.sequence(
                cmdSetIntakeVelocity(IntakeVelocity.STOP),
                cmdSetWristPosition(WristPosition.ALGAE_CONTAINMENT));
    }

    public Command cmdIntakeAlgae(IntakeVelocity velocity, WristPosition position) {
        return Commands.sequence(
                cmdSetIntakeVelocity(velocity),
                cmdSetWristPosition(position),
                Commands.waitUntil(() -> intake.hasAlgae()),
                cmdHomeIntake())
                .unless(() -> intake.hasAlgae())
                .withName(String.format("Intake: %s", position.toString()));
    }

    public Command cmdIntakeAlgaeSelector() {
        return new SelectCommand<>(Map.ofEntries(
                Map.entry(1, cmdIntakeAlgae(IntakeVelocity.YOINK, WristPosition.FLOOR)),
                Map.entry(2, cmdIntakeAlgae(IntakeVelocity.YOINK, WristPosition.REEF))),
                () -> intakeAlgaeSelector());
    }

    private int intakeAlgaeSelector() {
        if (elevator.isNearPositionAndTolerance(ElevatorPosition.HOME.rotations, 5)) {
            return 1;
        } else {
            return 2;
        }
    }

    public Command cmdReleaseAlgae(WristPosition position) {
        return Commands.sequence(
                cmdSetWristPosition(position),
                cmdSetIntakeVelocity(IntakeVelocity.YEET))
                .withName(String.format("Intake: %s", position.toString()));
    }

    public Command cmdReleaseAlgaeSelector() {
        return new SelectCommand<>(Map.ofEntries(
                Map.entry(1, cmdReleaseAlgae(WristPosition.BARGE)),
                Map.entry(2, cmdReleaseAlgae(WristPosition.ALGAE_CONTAINMENT)),
                Map.entry(3, cmdSetIntakeVelocity(IntakeVelocity.YEET))),
                () -> releaseAlgaeSelector());
    }

    private int releaseAlgaeSelector() {
        if (elevator.isNearPositionAndTolerance(ElevatorPosition.BARGE.rotations, 5)) {
            return 1;
        } else if (wrist.isWristNearPosition(WristPosition.PROCESSOR)) {
            return 3;
        } else {
            return 2;
        }
    }

    /*
     * Diag lights
     */

    public Command cmdDisplaySubsystemErrorState() {
        return Commands.repeatingSequence(
                Commands.runOnce(
                        () -> {
                            HealthMonitor hm = HealthMonitor.getInstance();
                            systemLights.diagnosticColors.clear();
                            if (hm.getSubsystemStatus("Elevator") == HealthStatus.ERROR) {
                                systemLights.diagnosticColors.add(PresetColor.PURPLE);
                            }

                            if (hm.getSubsystemStatus("Drivetrain") == HealthStatus.ERROR) {
                                systemLights.diagnosticColors.add(PresetColor.RED);
                            }

                            if (hm.getSubsystemStatus("Climber") == HealthStatus.ERROR) {
                                systemLights.diagnosticColors.add(PresetColor.BLUE);
                            }

                            if (hm.getSubsystemStatus("AlgaeEndEffector") == HealthStatus.ERROR) {
                                systemLights.diagnosticColors.add(PresetColor.WHITE);
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

    public Command cmdFollowPath(String pathName) {
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
        } catch (Exception e) {
            DataLogManager.log(String.format("GatorBot: Not able to build path command, %s", e.getMessage()));
            return Commands.none();
        }
    }

    public Command cmdFollowPathToRedReefTag() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(6, cmdFollowPath(ReefScoringMap.getReefPathFromLimelightID(6))),
                        Map.entry(7, cmdFollowPath(ReefScoringMap.getReefPathFromLimelightID(7))),
                        Map.entry(8, cmdFollowPath(ReefScoringMap.getReefPathFromLimelightID(8))),
                        Map.entry(9, cmdFollowPath(ReefScoringMap.getReefPathFromLimelightID(9))),
                        Map.entry(10, cmdFollowPath(ReefScoringMap.getReefPathFromLimelightID(10))),
                        Map.entry(11, cmdFollowPath(ReefScoringMap.getReefPathFromLimelightID(11)))),
                () -> limelight.getFid()).withName("Drivetain: Follow path to red reef");
    }

    public Command cmdFollowPathToBlueReefTag() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(17, cmdFollowPath(ReefScoringMap.getReefPathFromLimelightID(17))),
                        Map.entry(18, cmdFollowPath(ReefScoringMap.getReefPathFromLimelightID(18))),
                        Map.entry(19, cmdFollowPath(ReefScoringMap.getReefPathFromLimelightID(19))),
                        Map.entry(20, cmdFollowPath(ReefScoringMap.getReefPathFromLimelightID(20))),
                        Map.entry(21, cmdFollowPath(ReefScoringMap.getReefPathFromLimelightID(21))),
                        Map.entry(22, cmdFollowPath(ReefScoringMap.getReefPathFromLimelightID(22)))),
                () -> limelight.getFid()).withName("Drivetain: Follow path to blue reef");
    }

    public Command cmdPathfindToPose(Pose2d targetPose) {
        return AutoBuilder.pathfindToPose(
                targetPose,
                adamsSmoothPathConstraints,
                0 // Goal end velocity in meters/sec
        ).withName("Pathfind to pose");
    }

    public Command cmdPathfindToRedReefTag() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(6, cmdPathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(6))),
                        Map.entry(7, cmdPathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(7))),
                        Map.entry(8, cmdPathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(8))),
                        Map.entry(9, cmdPathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(9))),
                        Map.entry(10, cmdPathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(10))),
                        Map.entry(11, cmdPathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(11)))),
                () -> limelight.getFid()).withName("Drivetain: Pathfind to red reef");
    }

    public Command cmdPathfindToBlueReefTag() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(17, cmdPathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(17))),
                        Map.entry(18, cmdPathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(18))),
                        Map.entry(19, cmdPathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(19))),
                        Map.entry(20, cmdPathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(20))),
                        Map.entry(21, cmdPathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(21))),
                        Map.entry(22, cmdPathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(22)))),
                () -> limelight.getFid()).withName("Drivetain: Pathfind to blue reef");
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
        if (!DriverStation.isDisabled()) {
            var driveState = swerveDrivetrain.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

            limelight.setRobotOrientation(headingDeg);
            var llMeasurement = limelight.getBotPoseEstimate_wpiBlue_MegaTag2();
            if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                swerveDrivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                swerveDrivetrain.addVisionMeasurement(llMeasurement.pose,
                        Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
            }
        }
    }

    public void addLimelightVisionMeasurementsV2() {
        if (DriverStation.isDisabled()) {
            PoseEstimate megaTag = limelight.getBotPoseEstimate_wpiBlue_MegaTag1();

            if (megaTag == null | megaTag.tagCount == 0)
                return;

            if (megaTag.tagCount < 1)
                return;

            swerveDrivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));
            swerveDrivetrain.addVisionMeasurement(megaTag.pose, Utils.fpgaToCurrentTime(megaTag.timestampSeconds));
        } else {
            SwerveDriveState driveState = swerveDrivetrain.getState();
            limelight.setRobotOrientation(driveState.Pose.getRotation().getDegrees());
            PoseEstimate megaTag = limelight.getBotPoseEstimate_wpiBlue_MegaTag2();
            double xTrust, yTrust = 10.0;

            if (Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond) > 2.0)
                return;

            if (megaTag == null | megaTag.tagCount == 0)
                return;

            if (areAnyTagsAmbiguous(megaTag.rawFiducials))
                return;

            if (areAnyTagsFar(megaTag.rawFiducials))
                return;

            if (isEstimateOutOfBounds(megaTag.pose))
                return;

            if (hasTeleported(megaTag.pose, driveState.Pose, 3.5))
                return;

            // Increase the trust value to reduce trust in vision measurements
            if (megaTag.tagCount > 1) {
                xTrust = 0.5;
                yTrust = 0.5;
            } else {
                xTrust = 0.8;
                yTrust = 0.8;
            }

            swerveDrivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(xTrust, yTrust, 9999999));
            swerveDrivetrain.addVisionMeasurement(megaTag.pose, Utils.fpgaToCurrentTime(megaTag.timestampSeconds));
        }
    }

    private boolean areAnyTagsAmbiguous(RawFiducial[] tags) {
        for (RawFiducial rawFiducial : tags) {
            if (rawFiducial.ambiguity > 0.8) {
                return true;
            }
        }
        return false;
    }

    private boolean areAnyTagsFar(RawFiducial[] tags) {
        for (RawFiducial rawFiducial : tags) {
            if (rawFiducial.distToRobot > 3) {
                return true;
            }
        }
        return false;
    }

    private boolean isEstimateOutOfBounds(Pose2d pose) {
        return pose.getX() > Units.feetToMeters(52) || pose.getY() > Units.feetToMeters(27);
    }

    private boolean hasTeleported(Pose2d visionPose, Pose2d currentPose, double teleportThreshold) {
        return currentPose.minus(visionPose).getTranslation().getNorm() > teleportThreshold;
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
}
