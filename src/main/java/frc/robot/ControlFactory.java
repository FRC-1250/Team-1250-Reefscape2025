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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.commands.AlgaeEndEffector.AddWristRotations;
import frc.robot.commands.AlgaeEndEffector.HoldIntakePosition;
import frc.robot.commands.AlgaeEndEffector.SetIntakeVelocity;
import frc.robot.commands.AlgaeEndEffector.SetWristPosition;
import frc.robot.commands.Climber.DeepClimb;
import frc.robot.commands.Climber.DeepClimbPostClimb;
import frc.robot.commands.Climber.DeepClimbPreclimb;
import frc.robot.commands.Climber.SetDeepClimbTorque;
import frc.robot.commands.Elevator.AddElevatorRotations;
import frc.robot.commands.Elevator.HomeElevatorBasedOnAmps;
import frc.robot.commands.Elevator.ResetElevatorPosition;
import frc.robot.commands.Elevator.SetElevatorPosition;
import frc.robot.commands.LEDs.DiagnosticLights;
import frc.robot.commands.LEDs.MatchLights;
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
import frc.robot.subsystem.Wrist;
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
                Map.entry(2, new DeepClimbPostClimb(deepClimber, Amps.of(20))),
                Map.entry(3, new DeepClimbPreclimb(deepClimber, Amps.of(20)))),
                () -> DeepClimbSelector());
    }

    /*
     * Elevator
     */

    public Command cmdHomeElevator() {
        return new HomeElevatorBasedOnAmps(elevator).andThen(new ResetElevatorPosition(elevator));
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

    public Command cmdHoldIntakePosition() {
        return new HoldIntakePosition(intake);
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
                cmdSetWristPosition(WristPosition.HOME),
                cmdHoldIntakePosition());
    }

    public Command cmdHomeIntakeWithAlgae() {
        return Commands.sequence(
                cmdSetIntakeVelocity(IntakeVelocity.STOP),
                cmdSetWristPosition(WristPosition.ALGAE_CONTAINMENT),
                cmdHoldIntakePosition());
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

    public Command cmdReleaseCoral() {
        return Commands.sequence(
                cmdSetWristPosition(WristPosition.L1),
                cmdSetIntakeVelocity(IntakeVelocity.YEET));
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
     * System lights
     */

    public Command cmdDisplaySubsystemErrorState() {
        return new DiagnosticLights(systemLights, 5).ignoringDisable(true).repeatedly();
    }

    // TODO: Add some logic for LEDs during a match
    public Command cmdDisplayMatchState() {
        return new MatchLights(systemLights);
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

    private double tagAmbiguous = 0;
    private double tagTooSmall = 0;
    private double resultOutOfBounds = 0;
    private double resultTeleported = 0;
    private double framesProcessed = 0;
    private double frameRejectionRate = 0;
    private double xTrust, yTrust = 10.0;

    private double maxRadiansPerSecond = 2;
    private double minAllowedViewableTagDecimal = 0.05; // 0 to 100 so 0.05 is 5%
    private double maxTeleportDistance = 4.5;
    private double maxAllowedTagAmbiguity = 0.6;
    private int maxFramesBeforeTeleport = 10;
    private int teleportFrameCounter = 0;
    private double fieldBuffer = Units.feetToMeters(0);
    private double fieldLength = Units.feetToMeters(52);
    private double fieldWidth = Units.feetToMeters(27);

    /*
     * Use MegaTag v1 to determine our starting position prior to
     * switching to MegaTag v2 in the rest of the match. Assuming we can see an
     * AprilTag at the start of the match, this helps reduce the impact of the
     * robot "teleporting" at the beginning of a match from (0,0) to wherever it
     * actually is on the field.
     */
    public void addLimelightVisionMeasurementsV2() {
        SwerveDriveState driveState = swerveDrivetrain.getState();
        limelight.setRobotOrientation(driveState.Pose.getRotation().getDegrees());
        PoseEstimate megaTag = limelight.getBotPoseEstimate_wpiBlue_MegaTag2();
        framesProcessed++;

        if (Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond) > maxRadiansPerSecond)
            return;

        if (megaTag == null | megaTag.tagCount == 0)
            return;

        if (areAnyTagsAmbiguous(megaTag.rawFiducials)) {
            tagAmbiguous++;
            SmartDashboard.putNumber("Ambiguious Tag", tagAmbiguous);
            return;
        }

        if (isTagTooSmall(megaTag.rawFiducials)) {
            tagTooSmall++;
            SmartDashboard.putNumber("Too Small", tagTooSmall);
            return;
        }

        if (isEstimateOutOfBounds(megaTag.pose)) {
            resultOutOfBounds++;
            SmartDashboard.putNumber("Went Out of Bounds", resultOutOfBounds);
            return;
        }

        if (hasTeleported(megaTag.pose, driveState.Pose, maxTeleportDistance)) {
            resultTeleported++;
            SmartDashboard.putNumber("Teleported", resultTeleported);
            return;

        }

        xTrust = yTrust = calculateTrust(megaTag);

        swerveDrivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(xTrust, yTrust, 9999999));
        swerveDrivetrain.addVisionMeasurement(megaTag.pose, Utils.fpgaToCurrentTime(megaTag.timestampSeconds));
        frameRejectionRate = (tagAmbiguous + tagTooSmall + resultOutOfBounds + resultTeleported)
                / framesProcessed;
        SmartDashboard.putNumber("Rejection rate", frameRejectionRate);

    }

    private double calculateTrust(PoseEstimate estimate) {
        double averageDistance = 0;
        for (RawFiducial tag : estimate.rawFiducials) {
            averageDistance += tag.distToRobot;
        }
        averageDistance /= estimate.tagCount;

        double trust = 0.1 + (0.2 * averageDistance);

        if (estimate.tagCount > 1) {
            return trust * 0.5;
        }

        return trust;
    }

    private boolean areAnyTagsAmbiguous(RawFiducial[] tags) {
        for (RawFiducial tag : tags) {
            if (tag.ambiguity > maxAllowedTagAmbiguity) {
                return true;
            }
        }
        return false;
    }

    private boolean isTagTooSmall(RawFiducial[] tags) {
        for (RawFiducial tag : tags) {
            if (tag.ta * 100 < minAllowedViewableTagDecimal) {
                SmartDashboard.putNumber("ta", tag.ta * 100);
                return true;
            }
        }
        return false;
    }

    private boolean isEstimateOutOfBounds(Pose2d pose) {
        if (pose.getX() < -fieldBuffer || pose.getX() > fieldLength + fieldBuffer) {
            return true;
        }
        if (pose.getY() < -fieldBuffer || pose.getY() > fieldWidth + fieldBuffer) {
            return true;
        }
        return false;
    }

    private boolean hasTeleported(Pose2d visionPose, Pose2d currentPose, double teleportThreshold) {
        double distance = currentPose.getTranslation().getDistance(visionPose.getTranslation());

        if (distance > teleportThreshold) {
            teleportFrameCounter++;
            // If we see the same "wrong" position for X frames,
            // it's probably real just let it jump.
            if (teleportFrameCounter > maxFramesBeforeTeleport) {
                teleportFrameCounter = 0;
                swerveDrivetrain.resetPose(visionPose);
                return false;
            }
            return true;
        }
        teleportFrameCounter = 0;
        return false;
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
