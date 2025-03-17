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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystem.AlgaeEndEffector;
import frc.robot.subsystem.Climber;
import frc.robot.subsystem.CommandSwerveDrivetrain;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.Limelight;
import frc.robot.subsystem.SystemLights;
import frc.robot.subsystem.Elevator.ElevatorPosition;
import frc.robot.subsystem.Limelight.LimeLightPipeline;

public class RobotContainer {
    public Alliance alliance;

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
    private final CommandPS4Controller operatorJoystick = new CommandPS4Controller(1);

    @Logged(name = "Elevator")
    public final Elevator elevator = new Elevator();

    @Logged(name = "System lights")
    public final SystemLights systemLights = new SystemLights();

    @Logged(name = "Climber")
    public final Climber climber = new Climber();

    @Logged(name = "Algae end effector")
    public final AlgaeEndEffector algaeEndEffector = new AlgaeEndEffector();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Limelight limelight = new Limelight();
    public final ControlFactory controlFactory = new ControlFactory(
            drivetrain,
            elevator,
            climber,
            limelight,
            algaeEndEffector,
            systemLights);

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(16);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(16);

    private final Trigger isBlue = new Trigger(() -> drivetrain.getOperatorForwardDirection().getDegrees() == 0);

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
            MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 3;
        } else if (elevator.isAbovePosition(ElevatorPosition.LOW_ALGAE)) {
            MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 1.5;
        } else {
            MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
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
                        .withVelocityX(0)
                        .withVelocityY(xLimiter.calculate(-primaryDriverJoystick.getLeftX() * MaxSpeed))
                        .withTargetDirection(Rotation2d.fromDegrees(
                                drivetrain.getOperatorForwardDirection().getDegrees())))
                .withName("Forward and strafe"));

        primaryDriverJoystick.a().and(isBlue).whileTrue(controlFactory.cmdPathfindToBlueReefTag());
        primaryDriverJoystick.a().and(isBlue).negate().whileTrue(controlFactory.cmdPathfindToRedReefTag());
        primaryDriverJoystick.start()
                .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()).withName("Reseed drive"));

        // Elevator
        primaryDriverJoystick.b().onTrue(controlFactory.cmdSetElevatorToHome());
        primaryDriverJoystick.rightBumper().onTrue(controlFactory.cmdSetElevatorToBarge());
        primaryDriverJoystick.leftBumper().onTrue(controlFactory.cmdSetElevatorToReefAlgae());

        // Intake
        primaryDriverJoystick.rightTrigger().whileTrue(controlFactory.cmdReleaseAlgaeBasedOnElevator());
        primaryDriverJoystick.leftTrigger().whileTrue(controlFactory.cmdIntakeAlgaeBasedOnElevator());

        // Climber
        primaryDriverJoystick.y().onTrue(controlFactory.cmdShallowClimb());

        /*
         * Operator controls
         */
        operatorJoystick.R1().whileTrue(controlFactory.cmdIntakeAlgae());
        operatorJoystick.L1().whileTrue(controlFactory.cmdReleaseAlgae());

        operatorJoystick.povUp().onTrue(controlFactory.cmdAddElevatorRotations(5));
        operatorJoystick.povDown().onTrue(controlFactory.cmdAddElevatorRotations(-5));

        operatorJoystick.triangle().onTrue(controlFactory.cmdAddWristRotations(0.2));
        operatorJoystick.cross().onTrue(controlFactory.cmdAddWristRotations(-0.2));
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
        addPathAuto("CenterToBarge", "CenterToBarge");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureNamedCommands() {
        double intakeTimeoutTime = 0.5;
        double releaseTimeoutTime = 1;

        // Elevator
        NamedCommands.registerCommand("Home", controlFactory.cmdSetElevatorToHome());
        NamedCommands.registerCommand("HighAlgae", controlFactory.cmdSetElevatorHighAlgae());
        NamedCommands.registerCommand("LowAlgae", controlFactory.cmdSetElevatorLowAlgae());
        NamedCommands.registerCommand("Barge", controlFactory.cmdSetElevatorToBarge());

        // Intake
        NamedCommands.registerCommand("IntakeReef", controlFactory.cmdIntakeAlgaeReef());
        NamedCommands.registerCommand("ReleaseBarge", controlFactory.cmdReleaseAlgaeBarge());
        NamedCommands.registerCommand("Release",controlFactory.cmdReleaseAlgae());

        // Complex
        NamedCommands.registerCommand(
                "HighDealgae",
                controlFactory.cmdSetElevatorHighAlgae()
                        .andThen(controlFactory.cmdIntakeAlgaeReef().withTimeout(intakeTimeoutTime)
                        .andThen(controlFactory.cmdSetWristHome()
                        .andThen(controlFactory.cmdSetElevatorToHome()))));

        NamedCommands.registerCommand(
                "LowDealgae",
                controlFactory.cmdSetElevatorLowAlgae()
                        .andThen(controlFactory.cmdIntakeAlgaeReef().withTimeout(intakeTimeoutTime)
                        .andThen(controlFactory.cmdSetWristHome()
                        .andThen(controlFactory.cmdSetElevatorToHome()))));

        NamedCommands.registerCommand(
                "ScoreBarge",
                controlFactory.cmdSetElevatorToBarge()
                        .andThen(controlFactory.cmdReleaseAlgaeBarge().withTimeout(releaseTimeoutTime)
                        .andThen(controlFactory.cmdSetWristHome()
                        .andThen(controlFactory.cmdSetElevatorToHome()))));

    }
}
