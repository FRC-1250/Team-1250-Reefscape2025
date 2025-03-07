// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystem.Climber;
import frc.robot.subsystem.CommandSwerveDrivetrain;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.EndEffector;
import frc.robot.subsystem.Limelight;
import frc.robot.subsystem.SystemLights;
import frc.robot.subsystem.Elevator.Position;
import frc.robot.subsystem.EndEffector.AlgaeServoPosition;
import frc.robot.subsystem.EndEffector.HeadPosition;
import frc.robot.subsystem.SystemLights.PresetColor;
import frc.robot.util.ReefScoringMap;

public class RobotContainer {
    // kSpeedAt12Volts desired top speed
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    // 3/4 of a rotation per second, max angular velocity
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(5, 0, 0);

    private final Telemetry logger = new Telemetry();
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private final CommandXboxController joystick = new CommandXboxController(0);
    private CommandPS4Controller devJoystick;

    @Logged(name = "End effector")
    public final EndEffector endEffector = new EndEffector();

    @Logged(name = "Elevator")
    public final Elevator elevator = new Elevator();

    @Logged(name = "System lights")
    public final SystemLights systemLights = new SystemLights();

    @Logged(name = "Climber")
    public final Climber climber = new Climber();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Limelight limelight = new Limelight();
    public final ControlFactory controlFactory = new ControlFactory(
            drivetrain,
            elevator,
            endEffector,
            climber,
            limelight,
            systemLights);

    // From control factory
    private final Trigger reefHasHighAlgae = new Trigger(() -> controlFactory.hasHighAlgae());
    private final Trigger reefHasLowAlgae = new Trigger(() -> controlFactory.hasLowAlgae());

    // From end effector
    private final Trigger hasCoral = new Trigger(() -> endEffector.hasCoral());
    private final Trigger hasAlgae = new Trigger(() -> endEffector.hasAlgae());

    // From Driverstation
    private final Trigger isTeleop = new Trigger(() -> DriverStation.isTeleopEnabled());

    // From elevator
    private final Trigger isAtLowAlgaePrepPosition = new Trigger(
            () -> elevator.isAtPosition(Elevator.Position.LOW_ALGAE_PREP));
    private final Trigger isAtHighAlgaePrepPosition = new Trigger(
            () -> elevator.isAtPosition(Elevator.Position.HIGH_ALGAE_PREP));
    private final Trigger isAtAlgaeContainmentPositon = new Trigger(
            () -> elevator.isAtPosition(Elevator.Position.CONTAIN_ALGAE));
    private final Trigger isAtHighAlgaePosition = new Trigger(
            () -> elevator.isAtPosition(Elevator.Position.HIGH_ALGAE));
    private final Trigger isAtLowAlgaePosition = new Trigger(
            () -> elevator.isAtPosition(Elevator.Position.LOW_ALGAE));
    private final Trigger isAtL1 = new Trigger(
            () -> elevator.isAtPosition(Elevator.Position.L1));
    private final Trigger isAtL2 = new Trigger(
            () -> elevator.isAtPosition(Elevator.Position.L2));
    private final Trigger isAtL3 = new Trigger(
            () -> elevator.isAtPosition(Elevator.Position.L3));
    private final Trigger isAtL4 = new Trigger(
            () -> elevator.isAtPosition(Elevator.Position.L4));
    private final Trigger isAtHome = new Trigger(
            () -> elevator.isAtPosition(Elevator.Position.STARTING_CONFIGURATION));

    // From limelight
    private final Trigger isId6 = new Trigger(() -> limelight.isBestTagSeen(6));
    private final Trigger isId7 = new Trigger(() -> limelight.isBestTagSeen(7));
    private final Trigger isId8 = new Trigger(() -> limelight.isBestTagSeen(8));
    private final Trigger isId9 = new Trigger(() -> limelight.isBestTagSeen(9));
    private final Trigger isId10 = new Trigger(() -> limelight.isBestTagSeen(10));
    private final Trigger isId11 = new Trigger(() -> limelight.isBestTagSeen(11));
    private final Trigger isId17 = new Trigger(() -> limelight.isBestTagSeen(17));
    private final Trigger isId18 = new Trigger(() -> limelight.isBestTagSeen(18));
    private final Trigger isId19 = new Trigger(() -> limelight.isBestTagSeen(19));
    private final Trigger isId20 = new Trigger(() -> limelight.isBestTagSeen(20));
    private final Trigger isId21 = new Trigger(() -> limelight.isBestTagSeen(21));
    private final Trigger isId22 = new Trigger(() -> limelight.isBestTagSeen(22));

    private final boolean devController = false;
    private final boolean driveEnabled = true;
    private final boolean automationEnabled = true;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(16);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(16);

    public RobotContainer() {
        configureBindings();
        configureSmartDashboardBindings();
        configureNamedCommands();
        configureAutoCommands();
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void determineMaxSpeed() {
        if (elevator.isAbovePosition(Position.L3)) {
            MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 3;
        } else if (elevator.isAbovePosition(Position.L2)) {
            MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 1.5;
        } else {
            MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        }
    }

    private void configureBindings() {
        // More complicated triggers go to the top
        joystick.rightTrigger()
                .and(isAtL1.or(isAtL2).or(isAtL3).or(isAtL4))
                .onTrue(endEffector.cmdAddCoralRotations(20));
        joystick.rightTrigger()
                .and(isAtHighAlgaePrepPosition.or(isAtHighAlgaePosition))
                .whileTrue(controlFactory.reefHighDealgae());
        joystick.rightTrigger()
                .and(isAtLowAlgaePrepPosition.or(isAtLowAlgaePosition))
                .whileTrue(controlFactory.reefLowDealgae());
        joystick.rightTrigger()
                .and(isAtAlgaeContainmentPositon)
                .whileTrue(endEffector.cmdFireAlgae());

        joystick.leftTrigger()
                .and(reefHasHighAlgae)
                .onTrue(controlFactory.reefHighDealgaePrep());
        joystick.leftTrigger()
                .and(reefHasLowAlgae)
                .onTrue(controlFactory.reefLowDealgaePrep());

        joystick.povDown()
                .and(isAtL2.or(isAtL3).or(isAtL4).or(isAtHome).or(isAtAlgaeContainmentPositon))
                .and(hasAlgae.negate())
                .onTrue(elevator.cmdSetPosition((Elevator.Position.L1)));
        joystick.povDown()
                .and(isAtHighAlgaePrepPosition.or(isAtLowAlgaePrepPosition))
                .and(hasAlgae.negate())
                .onTrue(controlFactory.homeEndEffectorAndSetElevatorPosition((Elevator.Position.L1)));
        joystick.povDown()
                .and(isAtHighAlgaePosition)
                .and(hasAlgae.negate())
                .onTrue(controlFactory.reefHighDealgaeUndo((Elevator.Position.L1)));
        joystick.povDown()
                .and(isAtLowAlgaePosition)
                .and(hasAlgae.negate())
                .onTrue(controlFactory.reefLowDealgaeUndo((Elevator.Position.L1)));

        joystick.povRight()
                .and(isAtL1.or(isAtL3).or(isAtL4).or(isAtHome).or(isAtAlgaeContainmentPositon))
                .onTrue(elevator.cmdSetPosition((Elevator.Position.L2)));
        joystick.povRight()
                .and(isAtHighAlgaePrepPosition.or(isAtLowAlgaePrepPosition))
                .onTrue(controlFactory.homeEndEffectorAndSetElevatorPosition((Elevator.Position.L2)));
        joystick.povRight().and(isAtHighAlgaePosition)
                .onTrue(controlFactory.reefHighDealgaeUndo((Elevator.Position.L2)));
        joystick.povRight().and(isAtLowAlgaePosition)
                .onTrue(controlFactory.reefLowDealgaeUndo((Elevator.Position.L2)));

        joystick.povLeft()
                .and(isAtL1.or(isAtL2).or(isAtL4).or(isAtHome).or(isAtAlgaeContainmentPositon))
                .onTrue(elevator.cmdSetPosition((Elevator.Position.L3)));
        joystick.povLeft()
                .and(isAtHighAlgaePrepPosition.or(isAtLowAlgaePrepPosition))
                .onTrue(controlFactory.homeEndEffectorAndSetElevatorPosition((Elevator.Position.L3)));
        joystick.povLeft().and(isAtHighAlgaePosition)
                .onTrue(controlFactory.reefHighDealgaeUndo((Elevator.Position.L3)));
        joystick.povLeft().and(isAtLowAlgaePosition)
                .onTrue(controlFactory.reefLowDealgaeUndo((Elevator.Position.L3)));

        joystick.povUp()
                .and(isAtL1.or(isAtL2).or(isAtL3).or(isAtHome).or(isAtAlgaeContainmentPositon))
                .onTrue(elevator.cmdSetPosition((Elevator.Position.L4)));
        joystick.povUp()
                .and(isAtHighAlgaePrepPosition.or(isAtLowAlgaePrepPosition))
                .onTrue(controlFactory.homeEndEffectorAndSetElevatorPosition((Elevator.Position.L4)));
        joystick.povUp().and(isAtHighAlgaePosition)
                .onTrue(controlFactory.reefHighDealgaeUndo((Elevator.Position.L4)));
        joystick.povUp().and(isAtLowAlgaePosition)
                .onTrue(controlFactory.reefLowDealgaeUndo((Elevator.Position.L4)));

        joystick.b()
                .and(isAtL1.or(isAtL2).or(isAtL3).or(isAtL4).or(isAtAlgaeContainmentPositon).or(isAtHome))
                .and(hasAlgae.negate())
                .onTrue(controlFactory
                        .homeEndEffectorAndSetElevatorPosition((Elevator.Position.STARTING_CONFIGURATION)));
        joystick.b()
                .and(isAtHighAlgaePrepPosition.or(isAtLowAlgaePrepPosition))
                .and(hasAlgae.negate())
                .onTrue(controlFactory
                        .homeEndEffectorAndSetElevatorPosition((Elevator.Position.STARTING_CONFIGURATION)));
        joystick.b()
                .and(isAtHighAlgaePosition)
                .and(hasAlgae.negate())
                .onTrue(controlFactory.reefHighDealgaeUndo((Elevator.Position.STARTING_CONFIGURATION)));
        joystick.b()
                .and(isAtLowAlgaePosition)
                .and(hasAlgae.negate())
                .onTrue(controlFactory.reefLowDealgaeUndo((Elevator.Position.STARTING_CONFIGURATION)));

        joystick.b()
                .and(hasAlgae)
                .onTrue(controlFactory.homeEndEffectorAndSetElevatorPosition((Elevator.Position.CONTAIN_ALGAE)));

        joystick.back().onTrue(endEffector.cmdSetHeadRotation(EndEffector.HeadPosition.CENTER));
        joystick.rightBumper().onTrue(endEffector.cmdSetHeadRotation(EndEffector.HeadPosition.CENTER_RIGHT));
        joystick.leftBumper().onTrue(endEffector.cmdSetHeadRotation(EndEffector.HeadPosition.CENTER_LEFT));
        joystick.y().onTrue(climber.cmdSetTorque(Amps.of(80)));

        if (driveEnabled) {
            drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> drive
                            .withVelocityX(yLimiter.calculate(-joystick.getLeftY() * MaxSpeed))
                            .withVelocityY(xLimiter.calculate(-joystick.getLeftX() * MaxSpeed))
                            .withRotationalRate(-joystick.getRightX() * MaxAngularRate)).withName("Field centric swerve"));

            joystick.x().whileTrue(drivetrain.applyRequest(
                    () -> driveFacingAngle
                            .withVelocityX(yLimiter.calculate(-joystick.getLeftY() * MaxSpeed))
                            .withVelocityY(xLimiter.calculate(-joystick.getLeftX() * MaxSpeed))
                            .withTargetDirection(controlFactory.determineHeadingToReef())).withName("Auto turn to reef"));

            // Red
            joystick.a().and(isId6).whileTrue(controlFactory.pathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(6), 0));
            joystick.a().and(isId7).whileTrue(controlFactory.pathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(7), 0));
            joystick.a().and(isId8).whileTrue(controlFactory.pathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(8), 0));
            joystick.a().and(isId9).whileTrue(controlFactory.pathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(9), 0));
            joystick.a().and(isId10).whileTrue(controlFactory.pathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(10), 0));
            joystick.a().and(isId11).whileTrue(controlFactory.pathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(11), 0));

            // Blue
            joystick.a().and(isId17).whileTrue(controlFactory.pathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(17), 0));
            joystick.a().and(isId18).whileTrue(controlFactory.pathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(18), 0));
            joystick.a().and(isId19).whileTrue(controlFactory.pathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(19), 0));
            joystick.a().and(isId20).whileTrue(controlFactory.pathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(20), 0));
            joystick.a().and(isId21).whileTrue(controlFactory.pathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(21), 0));
            joystick.a().and(isId22).whileTrue(controlFactory.pathfindToPose(ReefScoringMap.getReefPoseFromLimelightID(22), 0));

            joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()).withName("Reseed drive"));
        }

        if (automationEnabled) {
            isTeleop.and(hasCoral.negate()).onTrue(Commands.waitSeconds(0.2).andThen(endEffector.cmdSetHeadRotation(EndEffector.HeadPosition.CENTER)));

            isTeleop.and(hasCoral).onTrue(endEffector.cmdAddCoralRotations(5)
                    .andThen(endEffector.cmdSetHeadRotation(EndEffector.HeadPosition.IDLE))
                    .andThen(Commands.waitSeconds(0.2))
                    .andThen(elevator.cmdSetPosition(Position.L1))
                    .withName("Coral index and prep"));

            isTeleop.and(hasCoral.negate()).onTrue(systemLights.cmdSetLEDs(PresetColor.KELLY_GREEN));
            isTeleop.and(hasCoral.negate()).and(isAtHome).whileTrue(endEffector.cmdSetCoralDutyCycleOut(.05));
            isTeleop.and(hasCoral).onTrue(systemLights.cmdSetLEDs(PresetColor.RED));
        }
      
        if (devController) {
            devJoystick = new CommandPS4Controller(1);

            // End effector head, left and right
            devJoystick.axisGreaterThan(0, 0.75)
                    .onTrue(endEffector.cmdSetHeadRotation(EndEffector.HeadPosition.CENTER_RIGHT));
            devJoystick.axisLessThan(0, -0.75)
                    .whileTrue(endEffector.cmdSetHeadRotation(EndEffector.HeadPosition.CENTER_LEFT));
            devJoystick.L3().onTrue(endEffector.cmdSetHeadRotation(EndEffector.HeadPosition.CENTER));

            // End effector algae intake, in and out
            devJoystick.axisGreaterThan(5, 0.75)
                    .onTrue(endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.DEPLOYED));
            devJoystick.axisLessThan(5, -0.75)
                    .onTrue(endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.HOME));
            devJoystick.R3().onTrue(endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.MIDDLE));

            // Scoring options
            devJoystick.R1().onTrue(endEffector.cmdAddCoralRotations(20));
            devJoystick.R2().whileTrue(endEffector.cmdSetAlgaeDutyCycleOut(0.5));
            devJoystick.L1().onTrue(climber.cmdSetTorque(Amps.of(80)));
            devJoystick.L2().whileTrue(endEffector.cmdSetAlgaeDutyCycleOut(-0.5));

            // Elevator duty cycle out, up and down
            devJoystick.cross().onTrue(elevator.cmdAddRotations(-1));
            devJoystick.square().onTrue(elevator.cmdAddRotations(-5));
            devJoystick.triangle().onTrue(elevator.cmdAddRotations(1));
            devJoystick.circle().onTrue(elevator.cmdAddRotations(5));
        } else {
                devJoystick = new CommandPS4Controller(1);

                devJoystick.cross().onTrue(elevator.cmdAddRotations(-1));
                devJoystick.square().onTrue(elevator.cmdAddRotations(-5));
                devJoystick.triangle().onTrue(elevator.cmdAddRotations(1));
                devJoystick.circle().onTrue(elevator.cmdAddRotations(5));

                devJoystick.R1().whileTrue(endEffector.cmdSetAlgaeDutyCycleOut(0.5));
                devJoystick.R2().whileTrue(endEffector.cmdSetAlgaeDutyCycleOut(-0.5));

                devJoystick.L1().onTrue(endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.DEPLOYED));
                devJoystick.L2().onTrue(endEffector.cmdSetAlgaeIntakePostion(EndEffector.AlgaeServoPosition.HOME));

                devJoystick.povUp().whileTrue(elevator.cmdSetDutyCycleOut(0.30));
                devJoystick.povDown().whileTrue(elevator.cmdSetDutyCycleOut(-0.20));
        }
    }

    private void configureSmartDashboardBindings() {
        // Servo replacement commands
        SmartDashboard.putData(endEffector.cmdBumpHead(true));
        SmartDashboard.putData(endEffector.cmdBumpHead(false));
        SmartDashboard.putData(endEffector.cmdJumpHead(true));
        SmartDashboard.putData(endEffector.cmdJumpHead(false));
        SmartDashboard.putData(endEffector.cmdSetHeadRotation(HeadPosition.LOGICAL_CENTER).withName("Head, center"));

        SmartDashboard.putData(endEffector.cmdBumpAlgaeIntake(true));
        SmartDashboard.putData(endEffector.cmdBumpAlgaeIntake(false));
        SmartDashboard.putData(endEffector.cmdJumpAlgaeIntake(true));
        SmartDashboard.putData(endEffector.cmdJumpAlgaeIntake(false));
        SmartDashboard.putData(
                endEffector.cmdSetAlgaeIntakePostion(AlgaeServoPosition.MIDDLE).withName(" Intake, middle"));

        // Elevator
        SmartDashboard.putData(elevator.cmdAddRotations(5));
        SmartDashboard.putData(elevator.cmdAddRotations(1));
        SmartDashboard.putData(elevator.cmdAddRotations(-5));
        SmartDashboard.putData(elevator.cmdAddRotations(-1));

        SmartDashboard.putData(elevator.cmdResetPos());
    }

    private void addPathAuto(String name, String pathName) {
        try {
            autoChooser.addOption(name, new PathPlannerAuto(pathName));
        } catch (Exception e) {
            // Exceptions are now caught in the PathPlannerAuto constructor and this should
            // never run. Leaving it in place to catch any edge cases.
            DataLogManager.log(String.format("GatorBot: Not able to build auto routines! %s", e.getMessage()));
        }
    }

    private void configureAutoCommands() {
        /*
         * Do nothing as default is a human safety condition, this should always be the
         * default
         */
        autoChooser.setDefaultOption("Do nothing", new WaitCommand(15));
        addPathAuto("CenterSingleCoral", "CenterSingleCoral");
        addPathAuto("LeftDoubleCoral", "LeftDoubleCoral");
        addPathAuto("RightDoubleCoral", "RightDoubleCoral");
        addPathAuto("RightSingleCoral", "RightSingleCoral");
        addPathAuto("LeftSingleCoral", "LeftSingleCoral");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("L4", elevator.cmdSetPosition(Position.L4));
        NamedCommands.registerCommand("Home", elevator.cmdSetPosition(Position.STARTING_CONFIGURATION));
        NamedCommands.registerCommand("RotateHeadLeft", endEffector.cmdSetHeadRotation(HeadPosition.CENTER_LEFT));
        NamedCommands.registerCommand("RotateHeadRight", endEffector.cmdSetHeadRotation(HeadPosition.CENTER_RIGHT));
        NamedCommands.registerCommand("RotateHeadIdle", endEffector.cmdSetHeadRotation(HeadPosition.IDLE));
        NamedCommands.registerCommand("ScoreCoral", endEffector.cmdAddCoralRotations(20));
        NamedCommands.registerCommand("DealgaeHigh", controlFactory.reefHighDealgae().withTimeout(2));
        NamedCommands.registerCommand("DealgaeLow", controlFactory.reefLowDealgae().withTimeout(2));
        NamedCommands.registerCommand("AutoIndexer", endEffector.cmdAutoIndexer().andThen(endEffector.cmdAddCoralRotations(5)));
        NamedCommands.registerCommand("HighAlgeaPrep", elevator.cmdSetPosition(Position.HIGH_ALGAE_PREP));
        //NamedCommands.registerCommand("LowAlgeaPrep", elevator.cmdSetPosition(Position.LOW_ALGAE_PREP));

        // Complex commands
        NamedCommands.registerCommand(
                "ScoreRightL4AndHome",
                elevator.cmdSetPosition(Position.L4)
                        .andThen(endEffector.cmdSetHeadRotation(HeadPosition.CENTER_RIGHT))
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(endEffector.cmdAddCoralRotations(20))
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(elevator.cmdSetPosition(Position.STARTING_CONFIGURATION)));

        NamedCommands.registerCommand(
                "ScoreRightL4AndHighAlgeaPrep",
                elevator.cmdSetPosition(Position.L4)
                        .andThen(endEffector.cmdSetHeadRotation(HeadPosition.CENTER_RIGHT))
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(endEffector.cmdAddCoralRotations(20))
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(elevator.cmdSetPosition(Position.HIGH_ALGAE_PREP)));

        NamedCommands.registerCommand(
                "ScoreRightL4AndLowAlgeaPrep",
                elevator.cmdSetPosition(Position.L4)
                        .andThen(endEffector.cmdSetHeadRotation(HeadPosition.CENTER_RIGHT))
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(endEffector.cmdAddCoralRotations(20))
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(elevator.cmdSetPosition(Position.LOW_ALGAE_PREP)));

        NamedCommands.registerCommand(
                "ScoreLeftL4AndHome",
                elevator.cmdSetPosition(Position.L4)
                        .andThen(endEffector.cmdSetHeadRotation(HeadPosition.CENTER_LEFT))
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(endEffector.cmdAddCoralRotations(20))
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(elevator.cmdSetPosition(Position.STARTING_CONFIGURATION)));

        NamedCommands.registerCommand(
                "ScoreLeftL4AndHighAlgeaPrep",
                elevator.cmdSetPosition(Position.L4)
                        .andThen(endEffector.cmdSetHeadRotation(HeadPosition.CENTER_LEFT))
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(endEffector.cmdAddCoralRotations(20))
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(elevator.cmdSetPosition(Position.HIGH_ALGAE_PREP)));

        NamedCommands.registerCommand(
                "ScoreLeftL4AndLowAlgeaPrep",
                elevator.cmdSetPosition(Position.L4)
                        .andThen(endEffector.cmdSetHeadRotation(HeadPosition.CENTER_LEFT))
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(endEffector.cmdAddCoralRotations(20))
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(elevator.cmdSetPosition(Position.LOW_ALGAE_PREP)));

                        NamedCommands.registerCommand(
                                "ScoreRightL2AndHome",
                                elevator.cmdSetPosition(Position.L2)
                                        .andThen(endEffector.cmdSetHeadRotation(HeadPosition.CENTER_RIGHT))
                                        .andThen(Commands.waitSeconds(0.5))
                                        .andThen(endEffector.cmdAddCoralRotations(20))
                                        .andThen(Commands.waitSeconds(0.5))
                                        .andThen(elevator.cmdSetPosition(Position.STARTING_CONFIGURATION)));

                                        NamedCommands.registerCommand(
                                                "ScoreLeftL2AndHome",
                                                elevator.cmdSetPosition(Position.L2)
                                                        .andThen(endEffector.cmdSetHeadRotation(HeadPosition.CENTER_LEFT))
                                                        .andThen(Commands.waitSeconds(0.5))
                                                        .andThen(endEffector.cmdAddCoralRotations(20))
                                                        .andThen(Commands.waitSeconds(0.5))
                                                        .andThen(elevator.cmdSetPosition(Position.STARTING_CONFIGURATION)));
    }

    
}
