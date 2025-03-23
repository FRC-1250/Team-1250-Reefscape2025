// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;

import java.util.Map;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.commands.AlgaeEndEffector.AddWristRotations;
import frc.robot.commands.AlgaeEndEffector.SetIntakeVelocity;
import frc.robot.commands.AlgaeEndEffector.SetWristPosition;
import frc.robot.commands.Climber.DeepClimb;
import frc.robot.commands.Climber.SetDeepClimbTorque;
import frc.robot.commands.Climber.ShallowClimb;
import frc.robot.commands.Elevator.AddElevatorRotations;
import frc.robot.commands.Elevator.ResetElevatorPosition;
import frc.robot.commands.Elevator.SetElevatorPosition;
import frc.robot.commands.Elevator.StopElevator;
import frc.robot.subsystem.AlgaeEndEffector;
import frc.robot.subsystem.Climber;
import frc.robot.subsystem.CommandSwerveDrivetrain;
import frc.robot.subsystem.DeepClimber;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.Limelight;
import frc.robot.subsystem.SystemLights;
import frc.robot.subsystem.AlgaeEndEffector.IntakeVelocity;
import frc.robot.subsystem.AlgaeEndEffector.WristPosition;
import frc.robot.subsystem.DeepClimber.DeepClimberPhase;
import frc.robot.subsystem.Elevator.ElevatorPosition;
import frc.robot.subsystem.SystemLights.PresetColor;
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
    private final Climber climber;
    private final Limelight limelight;
    private final DeepClimber deepClimber;
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
            Climber climber,
            DeepClimber deepClimber,
            Limelight limelight,
            AlgaeEndEffector algaeEndEffector,
            SystemLights systemLights) {
        this.swerveDrivetrain = swerveDrivetrain;
        this.elevator = elevator;
        this.climber = climber;
        this.deepClimber = deepClimber;
        this.limelight = limelight;
        this.algaeEndEffector = algaeEndEffector;
        this.systemLights = systemLights;
    }

    /*
     * Climber
     */

    public Command cmdShallowClimb() {
        return new ShallowClimb(climber, Amps.of(80))
                .withName("Climber: Shallow climb");
    }

    public Command cmdDeepClimbRawTorque() {
        return new SetDeepClimbTorque(deepClimber, Amps.of(80));
    }

    public Command cmdDeepClimb() {
        return new DeepClimb(deepClimber, DeepClimberPhase.CLIMB, Amps.of(80))
                .withName("Climber: Deep climb");
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
        return new SetWristPosition(algaeEndEffector, position);
    }

    public Command cmdAddWristRotations(double rotations) {
        return new AddWristRotations(algaeEndEffector, rotations);
    }

    public Command cmdSetIntakeVelocity(IntakeVelocity velocity) {
        return new SetIntakeVelocity(algaeEndEffector, velocity);
    }

    public Command cmdReturnIntakeBasedOnAlgae() {
        return new ConditionalCommand(
                cmdResetScoringPosition(),
                cmdHomeIntake(),
                () -> algaeEndEffector.hasAlgae());
    }

    public Command cmdHomeIntake() {
        return Commands.sequence(
                cmdSetIntakeVelocity(IntakeVelocity.STOP),
                cmdSetWristPosition(WristPosition.HOME));
    }

    public Command cmdResetScoringPosition() {
        return Commands.sequence(cmdSetIntakeVelocity(IntakeVelocity.STOP),
                cmdSetWristPosition(WristPosition.ALGAE_CONTAINMENT));
    }

    public Command cmdIntakeAlgae(IntakeVelocity velocity, WristPosition position) {
        return Commands.sequence(
                cmdSetIntakeVelocity(velocity),
                cmdSetWristPosition(position),
                Commands.waitUntil(() -> algaeEndEffector.hasAlgae()),
                cmdSetIntakeVelocity(IntakeVelocity.STOP),
                cmdSetWristPosition(WristPosition.ALGAE_CONTAINMENT))
                .unless(() -> algaeEndEffector.hasAlgae())
                .withName(String.format("Intake: %s", position.toString()));
    }

    public Command cmdIntakeAlgaeBasedOnElevatorPosition() {
        return new ConditionalCommand(
                cmdIntakeAlgae(IntakeVelocity.YOINK, WristPosition.FLOOR),
                cmdIntakeAlgae(IntakeVelocity.YOINK, WristPosition.REEF),
                () ->  elevator.isNearPositionAndTolerance(ElevatorPosition.HOME.rotations,5));
    }

    public Command cmdIntakeAlgaeSelector() {
        return new SelectCommand<>(Map.ofEntries(
                Map.entry(1, cmdIntakeAlgae(IntakeVelocity.YOINK, WristPosition.FLOOR)),
                Map.entry(2, cmdIntakeAlgae(IntakeVelocity.YOINK, WristPosition.REEF))),
                () -> intakeAlgaeSelector());
    }

    private int intakeAlgaeSelector() {
        if(elevator.isNearPositionAndTolerance(ElevatorPosition.HOME.rotations,5)) {
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

    public Command cmdReleaseAlgaeBasedOnElevatorPosition() {
        return new ConditionalCommand(
                cmdReleaseAlgae(WristPosition.BARGE),
                cmdReleaseAlgae(WristPosition.ALGAE_CONTAINMENT),
                () -> elevator.isNearPositionAndTolerance(ElevatorPosition.BARGE.rotations, 5));
    }

    public Command cmdReleaseAlgaeSelector() {
        return new SelectCommand<>(Map.ofEntries(
                Map.entry(1, cmdReleaseAlgae(WristPosition.BARGE)),
                Map.entry(2, cmdReleaseAlgae(WristPosition.ALGAE_CONTAINMENT)),
                Map.entry(3, cmdSetIntakeVelocity(IntakeVelocity.YEET))),
                () -> releaseAlgaeSelector());
    }

    private int releaseAlgaeSelector() {
        if(elevator.isNearPositionAndTolerance(ElevatorPosition.BARGE.rotations, 5)) {
            return 1;
        } else if (algaeEndEffector.isWristNearPosition(WristPosition.PROCESSOR)) {
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

                            if (hm.getSubsystemStatus("Climber") == HealthStatus.ERROR
                             ) {
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
}
