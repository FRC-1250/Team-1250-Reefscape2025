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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.SeekAprilTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystem.Climber;
import frc.robot.subsystem.CommandSwerveDrivetrain;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.EndEffector;
import frc.robot.subsystem.SystemLights;
import frc.robot.subsystem.Elevator.Position;
import frc.robot.subsystem.EndEffector.AlgaeServoPosition;
import frc.robot.subsystem.EndEffector.HeadPosition;
import frc.robot.subsystem.SystemLights.PresetColor;

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
            .withHeadingPID(12, 0, 0);

    private final SeekAprilTag seekAprilTag = new SeekAprilTag()
            .withDriveRequestType(DriveRequestType.Velocity);

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
    public final ControlFactory controlFactory = new ControlFactory(drivetrain, elevator, endEffector, climber, systemLights);

    // From control factory
    private final Trigger reefHasHighAlgae = new Trigger(() -> controlFactory.hasHighAlgae());
    private final Trigger reefHasLowAlgae = new Trigger(() -> controlFactory.hasLowAlgae());

    // From end effector
    private final Trigger hasCoral = new Trigger(() -> endEffector.hasCoral());
    private final Trigger hasAlgae = new Trigger(() -> endEffector.hasAlgae());

    // System Lights Triggers
    private final Trigger isEnabled = new Trigger(() -> DriverStation.isEnabled());

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
    private final Trigger isTeleop = new Trigger(() -> DriverStation.isTeleopEnabled());

    private final boolean devController = true;
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
        } else if (elevator.isAbovePosition(Position.L1)) {
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
                .whileTrue(endEffector.cmdSetAlgaeDutyCycleOut(1));

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
                .and(isAtL1.or(isAtL2).or(isAtL3).or(isAtL4).or(isAtAlgaeContainmentPositon))
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
        joystick.y().whileTrue(climber.cmdSetTorque(Amps.of(80)));

        if (driveEnabled) {
            drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> drive
                            .withVelocityX(yLimiter.calculate(-joystick.getLeftY() * MaxSpeed))
                            .withVelocityY(xLimiter.calculate(-joystick.getLeftX() * MaxSpeed))
                            .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

            joystick.x().whileTrue(drivetrain.applyRequest(
                    () -> driveFacingAngle
                            .withVelocityX(yLimiter.calculate(-joystick.getLeftY() * MaxSpeed))
                            .withVelocityY(xLimiter.calculate(-joystick.getLeftX() * MaxSpeed))
                            .withTargetDirection(controlFactory.determineHeadingToReef())));

            joystick.a().whileTrue(drivetrain.applyRequest(
                    () -> seekAprilTag
                            .withRobotPose(drivetrain.getState().Pose)));

            joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        }

        if (automationEnabled) {
            hasCoral.negate().and(isTeleop).onTrue(endEffector.cmdSetHeadRotation(EndEffector.HeadPosition.CENTER));
            hasCoral.onTrue(endEffector.cmdAddCoralRotations(5)
                    .andThen(endEffector.cmdSetHeadRotation(EndEffector.HeadPosition.CENTER_LEFT))
                    .andThen(Commands.waitSeconds(0.5))
                    .andThen(elevator.cmdSetPosition(Position.L1)));
        }

        isEnabled.and(hasCoral.negate()).onTrue(systemLights.cmdSetLEDs(PresetColor.KELLY_GREEN));
        isEnabled.and(hasCoral.negate()).and(isAtHome).whileTrue(endEffector.cmdSetCoralDutyCycleOut(.05));
        hasCoral.whileTrue(systemLights.cmdSetLEDs(PresetColor.RED));

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
            // Placeholder for climber -> devJoystick.L1().onTrue();
            devJoystick.L2().whileTrue(endEffector.cmdSetAlgaeDutyCycleOut(-0.5));

            // Elevator duty cycle out, up and down
            devJoystick.cross().whileTrue(elevator.cmdSetDutyCycleOut(-0.2));
            devJoystick.triangle().whileTrue(elevator.cmdSetDutyCycleOut(0.2));

            // Elevator positions
            devJoystick.povDown().whileTrue(elevator.cmdSetDutyCycleOut(-0.2));
            devJoystick.povUp().whileTrue(elevator.cmdSetDutyCycleOut(0.2));

            devJoystick.povDown().onTrue(elevator.cmdSetPosition(Elevator.Position.L1));
            devJoystick.povRight().onTrue(elevator.cmdSetPosition(Elevator.Position.L2));
            devJoystick.povLeft().onTrue(elevator.cmdSetPosition(Elevator.Position.L3));
            devJoystick.povUp().onTrue(elevator.cmdSetPosition(Elevator.Position.L4));
            devJoystick.circle().onTrue(elevator.cmdSetPosition(Elevator.Position.STARTING_CONFIGURATION));
            devJoystick.square().onTrue(elevator.cmdSetPosition(Elevator.Position.CONTAIN_ALGAE));
        }
    }

    private void configureSmartDashboardBindings() {
        // Sys ID for swerve
        SmartDashboard.putData(drivetrain.sysIdDynamic(Direction.kForward).withName("Dynamic forward"));
        SmartDashboard.putData(drivetrain.sysIdDynamic(Direction.kReverse).withName("Dynamic reverse"));
        SmartDashboard.putData(drivetrain.sysIdQuasistatic(Direction.kForward).withName("Quasistatic forward"));
        SmartDashboard.putData(drivetrain.sysIdQuasistatic(Direction.kReverse).withName("Quasistatic reverse"));

        // Servo replacement commands
        SmartDashboard.putData(endEffector.cmdBumpHead(true).withName("Head, bump right"));
        SmartDashboard.putData(endEffector.cmdBumpHead(false).withName("Head, bump left"));
        SmartDashboard.putData(endEffector.cmdJumpHead(true).withName("Head, jump right"));
        SmartDashboard.putData(endEffector.cmdJumpHead(false).withName("Head, jump left"));
        SmartDashboard.putData(endEffector.cmdSetHeadRotation(HeadPosition.LOGICAL_CENTER).withName("Head, center"));

        SmartDashboard.putData(endEffector.cmdBumpAlgaeIntake(true).withName("Algae intake, bump in"));
        SmartDashboard.putData(endEffector.cmdBumpAlgaeIntake(false).withName("Algae intake, bump out"));
        SmartDashboard.putData(endEffector.cmdJumpAlgaeIntake(true).withName("Algae intake, jump in"));
        SmartDashboard.putData(endEffector.cmdJumpAlgaeIntake(false).withName("Algae intake, jump out"));
        SmartDashboard.putData(
                endEffector.cmdSetAlgaeIntakePostion(AlgaeServoPosition.MIDDLE).withName("Algae intake, middle"));
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
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("L4", elevator.cmdSetPosition(Position.L4));
        NamedCommands.registerCommand("Home", elevator.cmdSetPosition(Position.STARTING_CONFIGURATION));
        NamedCommands.registerCommand("RotateHeadLeft", endEffector.cmdSetHeadRotation(HeadPosition.CENTER_LEFT));
        NamedCommands.registerCommand("RotateHeadRight", endEffector.cmdSetHeadRotation(HeadPosition.CENTER_RIGHT));
        NamedCommands.registerCommand("ScoreCoral", endEffector.cmdAddCoralRotations(20).withTimeout(2));
        NamedCommands.registerCommand("DealgaeHigh", controlFactory.reefHighDealgae().withTimeout(2));
        NamedCommands.registerCommand("DealgaeLow", controlFactory.reefLowDealgae().withTimeout(2));

    }

}
