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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystem.CommandSwerveDrivetrain;
import frc.robot.subsystem.DeepClimber;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.Limelight;
import frc.robot.subsystem.SystemLights;
import frc.robot.subsystem.Wrist;
import frc.robot.subsystem.Intake.IntakeVelocity;
import frc.robot.subsystem.Wrist.WristPosition;
import frc.robot.subsystem.Elevator.ElevatorPosition;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Limelight.LimeLightPipeline;

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

    private final CommandXboxController primaryDriverJoystick = new CommandXboxController(0);
    private CommandPS4Controller operatorJoystick;

    @Logged(name = "Elevator")
    public final Elevator elevator = new Elevator();

    @Logged(name = "System lights")
    public final SystemLights systemLights = new SystemLights();

    @Logged(name = "Deep climber")
    public final DeepClimber deepClimber = new DeepClimber();

    @Logged(name = "Wrist")
    public final Wrist wrist = new Wrist();

    @Logged(name = "Intake")
    public final Intake intake = new Intake();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Limelight limelight = new Limelight();
    public final ControlFactory controlFactory = new ControlFactory(
            drivetrain,
            elevator,
            deepClimber,
            wrist,
            intake,
            limelight,
            systemLights);

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(14, -18, 0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(14, -18, 0);

    private final Trigger isBlue = new Trigger(
            () -> Double.compare(drivetrain.getOperatorForwardDirection().getDegrees(), 0) == 0);

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
        if (elevator.isAbovePosition(ElevatorPosition.HIGH_ALGAE)) {
            MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 6;
            MaxAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond);
        } else if (elevator.isAbovePosition(ElevatorPosition.LOW_ALGAE)) {
            MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 4;
            MaxAngularRate = RotationsPerSecond.of(0.50).in(RadiansPerSecond);
        } else {
            MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
            MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        }
    }

    private void configureBindings() {
        /*
         * Primary driver controls
         */

        // Drive
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(yLimiter.calculate(-primaryDriverJoystick.getLeftY() * MaxSpeed))
                        .withVelocityY(xLimiter.calculate(-primaryDriverJoystick.getLeftX() * MaxSpeed))
                        .withRotationalRate(-primaryDriverJoystick.getRightX() * MaxAngularRate))
                        .withName("Field centric swerve"));

        primaryDriverJoystick.x().whileTrue(drivetrain.applyRequest(
                () -> driveFacingAngle
                        .withVelocityX(yLimiter.calculate(-primaryDriverJoystick.getLeftY() * MaxSpeed))
                        .withVelocityY(xLimiter.calculate(-primaryDriverJoystick.getLeftX() * MaxSpeed))
                        .withTargetDirection(Rotation2d.fromDegrees(
                                drivetrain.getOperatorForwardDirection().getDegrees())))
                .withName("Forward and strafe"));

        primaryDriverJoystick.a().and(isBlue).whileTrue(controlFactory.cmdFollowPathToBlueReefTag());
        primaryDriverJoystick.a().and(isBlue.negate()).whileTrue(controlFactory.cmdFollowPathToRedReefTag());
        primaryDriverJoystick.start()
                .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()).withName("Reseed drive"));

        // Elevator
        // Allow elevator to free fall against brake mode
        primaryDriverJoystick.back().onTrue(controlFactory.cmdStopElevatorMotors());
        primaryDriverJoystick.b().onTrue(controlFactory.cmdSetElevatorPosition(ElevatorPosition.HOME));
        primaryDriverJoystick.rightBumper().onTrue(controlFactory.cmdSetElevatorPosition(ElevatorPosition.BARGE));
        primaryDriverJoystick.leftBumper().onTrue(controlFactory.cmdSetElevatorToReefAlgae());

        // Intake
        primaryDriverJoystick.rightTrigger()
                .whileTrue(controlFactory.cmdReleaseAlgaeSelector())
                .whileFalse(controlFactory.cmdHomeIntake());
        primaryDriverJoystick.leftTrigger()
                .whileTrue(controlFactory.cmdIntakeAlgaeSelector())
                .whileFalse(controlFactory.cmdHomeIntake());

        // Climber
        primaryDriverJoystick.y().onTrue(controlFactory.cmdDeepClimbSelector());

        primaryDriverJoystick.povRight().whileTrue(controlFactory.cmdDeepClimbRawTorque());
        primaryDriverJoystick.povDown().onTrue(controlFactory.cmdSetWristPosition(WristPosition.PROCESSOR));
    }

    @SuppressWarnings("unused")
    private void configureOpController() {
        operatorJoystick = new CommandPS4Controller(1);
        operatorJoystick.axisLessThan(1, -0.5).whileTrue(controlFactory.cmdSetIntakeVelocity(IntakeVelocity.RELEASE))
                .onFalse(controlFactory.cmdSetIntakeVelocity(IntakeVelocity.STOP));
        operatorJoystick.axisGreaterThan(1, 0.5).whileTrue(controlFactory.cmdSetIntakeVelocity(IntakeVelocity.YOINK))
                .onFalse(controlFactory.cmdSetIntakeVelocity(IntakeVelocity.STOP));

        operatorJoystick.povUp().onTrue(controlFactory.cmdAddElevatorRotations(5));
        operatorJoystick.povLeft().onTrue(controlFactory.cmdAddElevatorRotations(1));
        operatorJoystick.povDown().onTrue(controlFactory.cmdAddElevatorRotations(-5));
        operatorJoystick.povRight().onTrue(controlFactory.cmdAddElevatorRotations(-1));

        operatorJoystick.triangle().onTrue(controlFactory.cmdAddWristRotations(0.1));
        operatorJoystick.cross().onTrue(controlFactory.cmdAddWristRotations(-0.1));
        operatorJoystick.square().onTrue(controlFactory.cmdAddWristRotations(0.05));
        operatorJoystick.circle().onTrue(controlFactory.cmdAddWristRotations(-0.05));

        operatorJoystick.L1().onTrue(controlFactory.cmdSetElevatorPosition(ElevatorPosition.HOME));
        operatorJoystick.L2().onTrue(controlFactory.cmdSetElevatorPosition(ElevatorPosition.L1));

        operatorJoystick.R1().onTrue(controlFactory.cmdSetWristPosition(WristPosition.REEF));
        operatorJoystick.R2().onTrue(controlFactory.cmdSetWristPosition(WristPosition.HOME));
    }

    private void configureSmartDashboardBindings() {
        // Limelight
        SmartDashboard.putData(limelight.switchPipeline(LimeLightPipeline.MORNING));
        SmartDashboard.putData(limelight.switchPipeline(LimeLightPipeline.EVENING));
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
        addPathAuto("CenterAlgae", "CenterAlgea");
        addPathAuto("LeftMiddleCageBargeScore", "LeftMiddleCageBargeScore");
        addPathAuto("RightMiddle_To_Processor", "RightMiddle_To_Processor");
        addPathAuto("GetOffLineLeft", "GetOffLineLeft");
        addPathAuto("GetOffLineRight", "GetOffLineRight");
        addPathAuto("GetOffLineCenter", "GetOffLineCenter");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureNamedCommands() {
        double releaseTimeoutTime = 1;

        NamedCommands.registerCommand("HomeElevator", controlFactory.cmdSetElevatorPosition(ElevatorPosition.HOME));
        NamedCommands.registerCommand("Release", controlFactory.cmdReleaseAlgae(WristPosition.PROCESSOR));
        // Complex

        NamedCommands.registerCommand(
                "DealgaeHigh",
                Commands.sequence(
                        controlFactory.cmdSetElevatorPosition(ElevatorPosition.HIGH_ALGAE),
                        controlFactory.cmdIntakeAlgae(IntakeVelocity.INTAKE, WristPosition.REEF),
                        controlFactory.cmdSetWristPosition(WristPosition.ALGAE_CONTAINMENT)));

        NamedCommands.registerCommand(
                "DealgaeHighGoHome",
                Commands.sequence(
                        controlFactory.cmdSetElevatorPosition(ElevatorPosition.HIGH_ALGAE),
                        controlFactory.cmdIntakeAlgae(IntakeVelocity.INTAKE, WristPosition.REEF),
                        controlFactory.cmdSetWristPosition(WristPosition.ALGAE_CONTAINMENT),
                        controlFactory.cmdSetElevatorPosition(ElevatorPosition.HOME)));

        NamedCommands.registerCommand(
                "DealgaeLow",
                Commands.sequence(
                        controlFactory.cmdSetElevatorPosition(ElevatorPosition.LOW_ALGAE),
                        controlFactory.cmdIntakeAlgae(IntakeVelocity.INTAKE, WristPosition.REEF),
                        controlFactory.cmdSetWristPosition(WristPosition.ALGAE_CONTAINMENT)));

        NamedCommands.registerCommand(
                "DealgaeLowGoHome",
                Commands.sequence(
                        controlFactory.cmdSetElevatorPosition(ElevatorPosition.LOW_ALGAE),
                        controlFactory.cmdIntakeAlgae(IntakeVelocity.INTAKE, WristPosition.REEF),
                        controlFactory.cmdSetWristPosition(WristPosition.ALGAE_CONTAINMENT),
                        controlFactory.cmdSetElevatorPosition(ElevatorPosition.HOME)));

        NamedCommands.registerCommand(
                "L1Coral",
                Commands.sequence(
                        controlFactory.cmdSetElevatorPosition(ElevatorPosition.L1),
                        controlFactory.cmdReleaseAlgae(WristPosition.L1).withTimeout(releaseTimeoutTime),
                        controlFactory.cmdSetWristPosition(WristPosition.HOME)));

        NamedCommands.registerCommand(
                "ScoreBargeGoHome",
                Commands.sequence(
                        controlFactory.cmdSetElevatorPosition(ElevatorPosition.BARGE),
                        controlFactory.cmdReleaseAlgae(WristPosition.BARGE).withTimeout(releaseTimeoutTime),
                        controlFactory.cmdSetWristPosition(WristPosition.HOME),
                        controlFactory.cmdSetElevatorPosition(ElevatorPosition.HOME)));
    }
}