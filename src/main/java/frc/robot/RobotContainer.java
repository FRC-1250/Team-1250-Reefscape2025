// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystem.CommandSwerveDrivetrain;
import frc.robot.subsystem.DeepClimber;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.Limelight;
import frc.robot.subsystem.SystemLights;
import frc.robot.subsystem.Wrist;
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

    private final Telemetry logger = new Telemetry();
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private final CommandXboxController primaryDriverJoystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    @Logged(name = "Robot state")
    private RobotState robotState;

    public enum RobotState {
        INTAKE,
        CLIMB,
        HIGH_ALGAE,
        LOW_ALGAE,
        BARGE,
        PROCESSOR,
        SINGLE_PLAYER
    }

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
    private final EventLoop singlePlayerLoop = new EventLoop();
    private final EventLoop twoPlayerLoop = new EventLoop();

    public RobotContainer() {
        configureSinglePlayerBindings();
        configureTwoPlayerBindings();
        configureDefaultCommands();
        changeEventLoop(singlePlayerLoop);
        configureSmartDashboardBindings();
        configureNamedCommands();
        configureAutoCommands();
        robotState = RobotState.SINGLE_PLAYER;
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void determineMaxSpeed() {
        if (elevator.isAbovePosition(ElevatorPosition.HIGH_ALGAE)) {
            MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 5;
            MaxAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond);
        } else if (elevator.isAbovePosition(ElevatorPosition.LOW_ALGAE)) {
            MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 3;
            MaxAngularRate = RotationsPerSecond.of(0.50).in(RadiansPerSecond);
        } else {
            MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
            MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        }
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(yLimiter.calculate(-primaryDriverJoystick.getLeftY() * MaxSpeed))
                        .withVelocityY(xLimiter.calculate(-primaryDriverJoystick.getLeftX() * MaxSpeed))
                        .withRotationalRate(-primaryDriverJoystick.getRightX() * MaxAngularRate))
                        .withName("Field centric swerve"));
    }

    private void configureDefaultBindings(EventLoop loop) {
        primaryDriverJoystick.start(loop)
                .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()).withName("Reseed drive"));

        primaryDriverJoystick.b(loop).onTrue(controlFactory.cmdSetElevatorPosition(ElevatorPosition.HOME));
    }

    private void configureSinglePlayerBindings() {
        configureDefaultBindings(singlePlayerLoop);
        // Elevator
        primaryDriverJoystick.back(singlePlayerLoop).onTrue(controlFactory.cmdStopElevatorMotors());
        primaryDriverJoystick.rightBumper(singlePlayerLoop)
                .onTrue(controlFactory.cmdSetElevatorPosition(ElevatorPosition.BARGE));
        primaryDriverJoystick.leftBumper(singlePlayerLoop)
                .onTrue(controlFactory.cmdSetWristPosition(WristPosition.PROCESSOR));
        primaryDriverJoystick.a(singlePlayerLoop)
                .onTrue(controlFactory.cmdSetElevatorPosition(ElevatorPosition.HIGH_ALGAE));
        primaryDriverJoystick.x(singlePlayerLoop)
                .onTrue(controlFactory.cmdSetElevatorPosition(ElevatorPosition.LOW_ALGAE));

        // Intake
        primaryDriverJoystick.rightTrigger(0.5, singlePlayerLoop)
                .whileTrue(controlFactory.cmdReleaseAlgaeSelector())
                .whileFalse(controlFactory.cmdHomeIntake());
        primaryDriverJoystick.leftTrigger(0.5, singlePlayerLoop)
                .whileTrue(controlFactory.cmdIntakeAlgaeSelector())
                .whileFalse(controlFactory.cmdHomeIntake());

        // Climber
        primaryDriverJoystick.y(singlePlayerLoop).onTrue(controlFactory.cmdDeepClimbSelector());
    }

    private void configureTwoPlayerBindings() {
        configureDefaultBindings(twoPlayerLoop);

        Trigger bargeState = new Trigger(() -> isRobotState(RobotState.BARGE));
        Trigger highAlgaeState = new Trigger(() -> isRobotState(RobotState.HIGH_ALGAE));
        Trigger lowAlgaeState = new Trigger(() -> isRobotState(RobotState.LOW_ALGAE));
        Trigger climbState = new Trigger(() -> isRobotState(RobotState.CLIMB));
        Trigger processorState = new Trigger(() -> isRobotState(RobotState.PROCESSOR));
        Trigger intakeState = new Trigger(() -> isRobotState(RobotState.INTAKE));

        // Intake mode
        intakeState.and(primaryDriverJoystick.leftTrigger(0.5, twoPlayerLoop))
                .onTrue(controlFactory.cmdSetElevatorPosition(ElevatorPosition.HOME));

        // Barge
        bargeState.and(primaryDriverJoystick.leftTrigger(0.5, twoPlayerLoop))
                .onTrue(controlFactory.cmdSetElevatorPosition(ElevatorPosition.BARGE));

        // High
        highAlgaeState.and(primaryDriverJoystick.leftTrigger(0.5, twoPlayerLoop))
                .onTrue(controlFactory.cmdSetElevatorPosition(ElevatorPosition.HIGH_ALGAE));

        // Low
        lowAlgaeState.and(primaryDriverJoystick.leftTrigger(0.5, twoPlayerLoop))
                .onTrue(controlFactory.cmdSetElevatorPosition(ElevatorPosition.LOW_ALGAE));

        // Processor
        processorState.and(primaryDriverJoystick.leftTrigger(0.5, twoPlayerLoop))
                .onTrue(controlFactory.cmdSetElevatorPosition(ElevatorPosition.HOME));

        // Climb
        climbState.and(primaryDriverJoystick.leftTrigger(0.5, twoPlayerLoop))
                .onTrue(controlFactory.cmdSetElevatorPosition(ElevatorPosition.HOME));

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
        addPathAuto("LeftStartToBargeDouble", "LeftStartToBargeDouble");
        addPathAuto("RightStartToProcessor", "RightStartToProcessor");
        addPathAuto("CenterStartToBargeDouble", "CenterStartToBargeDouble");
        autoChooser.addOption("GetOffLine",
                Commands.sequence(
                        Commands.runOnce(
                                () -> drivetrain.resetRotation(drivetrain.getOperatorForwardDirection()),
                                drivetrain),
                        drivetrain.applyRequest(() -> drive.withVelocityX(-1)).withTimeout(2)));
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureNamedCommands() {
        double releaseTimeoutTime = 0.5;

        NamedCommands.registerCommand("ElevatorBarge", controlFactory.cmdSetElevatorPosition(ElevatorPosition.BARGE));
        NamedCommands.registerCommand("ElevatorHome", controlFactory.cmdSetElevatorPosition(ElevatorPosition.HOME));
        NamedCommands.registerCommand("ElevatorHighAlgae",
                controlFactory.cmdSetElevatorPosition(ElevatorPosition.HIGH_ALGAE));
        NamedCommands.registerCommand("ElevatorLowAlgae",
                controlFactory.cmdSetElevatorPosition(ElevatorPosition.LOW_ALGAE));
        NamedCommands.registerCommand("ElevatorL1", controlFactory.cmdSetElevatorPosition(ElevatorPosition.L1));
        NamedCommands.registerCommand("ReleaseAlgae",
                controlFactory.cmdReleaseAlgaeSelector().withTimeout(releaseTimeoutTime));
        NamedCommands.registerCommand("ReleaseCoral", Commands.none().withTimeout(releaseTimeoutTime));
        NamedCommands.registerCommand("IntakeAlgae", controlFactory.cmdIntakeAlgaeSelector());
    }

    public void changeEventLoop(EventLoop loop) {
        CommandScheduler.getInstance().setActiveButtonLoop(loop);
    }

    public boolean isCurrentEventLoop(EventLoop loop) {
        return CommandScheduler.getInstance().getActiveButtonLoop().equals(loop);
    }

    public void setRobotState() {
        boolean isBarge = false;
        boolean isHighAlgae = false;
        boolean isLowAlgae = false;
        boolean isClimb = false;
        boolean isProcessor = false;

        if (isCurrentEventLoop(singlePlayerLoop)) {
            robotState = RobotState.SINGLE_PLAYER;
        } else {
            isBarge = operatorJoystick.b(twoPlayerLoop).getAsBoolean();
            isHighAlgae = operatorJoystick.a(twoPlayerLoop).getAsBoolean();
            isLowAlgae = operatorJoystick.x(twoPlayerLoop).getAsBoolean();
            isClimb = operatorJoystick.y(twoPlayerLoop).getAsBoolean();
            isProcessor = operatorJoystick.leftBumper(twoPlayerLoop).getAsBoolean();
            if (isLowAlgae) {
                robotState = RobotState.LOW_ALGAE;
            } else if (isHighAlgae) {
                robotState = RobotState.HIGH_ALGAE;
            } else if (isClimb) {
                robotState = RobotState.CLIMB;
            } else if (isProcessor) {
                robotState = RobotState.PROCESSOR;
            } else if (isBarge) {
                robotState = RobotState.BARGE;
            } else {
                robotState = RobotState.INTAKE;
            }
        }        
    }

    public boolean isRobotState(RobotState state) {
        return robotState == state;
    }
}