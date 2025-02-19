// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystem.Elevator.Position;
import frc.robot.util.HealthStatus;
import frc.robot.util.TalonHealthChecker;

public class EndEffector extends SubsystemBase {

  public enum AlgaeServoPosition {
    DEPLOYED(.1),
    MIDDLE(0.5),
    HOME(.89);

    public final double value;

    AlgaeServoPosition(double value) {
      this.value = value;
    }
  }

  public enum ServoOffset {
    BUMP(0.01),
    JUMP(0.1);

    public final double value;

    ServoOffset(double value) {
      this.value = value;
    }
  }

  public static class HeadPosition {
    private final static double TURE_CENTER = 0.45;
    private final static double CORAL_BRANCH_SERVO_OFFSET = 0.115;
    private final static double NINTY_DEGREE_OFFSET = 0.35;

    public final static double LOGICAL_CENTER = 0.5;
    public final static double CENTER = TURE_CENTER;
    public final static double CENTER_LEFT = TURE_CENTER - CORAL_BRANCH_SERVO_OFFSET;
    public final static double LEFT = TURE_CENTER - NINTY_DEGREE_OFFSET;
    public final static double CENTER_RIGHT = TURE_CENTER + CORAL_BRANCH_SERVO_OFFSET;
    public final static double RIGHT = TURE_CENTER + NINTY_DEGREE_OFFSET;
  }

  private TalonFX algaeMotor = new TalonFX(21);
  private DutyCycleOut algaeDutyCycleOut = new DutyCycleOut(0);
  private DigitalInput algaeSensor = new DigitalInput(4);
  private Servo algaeIntakeArm = new Servo(1);

  private TalonFX coralMotor = new TalonFX(20);
  private DutyCycleOut coralDutyCycleOut = new DutyCycleOut(0);
  private PositionVoltage coralPositionControl = new PositionVoltage(0);
  private DigitalInput coralSensor = new DigitalInput(3);
  private Servo headRotate = new Servo(0);
  private double currentPosition = 0;

  private final boolean healthCheckEnabled = true;
  private TalonHealthChecker algaeMotorCheck;
  private TalonHealthChecker coralMotorCheck;
  private HealthStatus healthStatus = HealthStatus.IS_OK;
  private AlgaeServoPosition algaeIntakeState = AlgaeServoPosition.HOME;

  /** Creates a new EndEffector. */
  public EndEffector() {
    Slot0Configs positionPIDConfigs = new Slot0Configs()
        .withKG(0)
        .withKS(0)
        .withKP(3)
        .withKI(0)
        .withKD(0)
        .withKV(0);

    TalonFXConfiguration coralTalonConfiguration = new TalonFXConfiguration();
    coralTalonConfiguration.Slot0 = positionPIDConfigs;
    coralTalonConfiguration.CurrentLimits.SupplyCurrentLimit = 25;
    coralTalonConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    coralTalonConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    coralTalonConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    coralMotor.getConfigurator().apply(coralTalonConfiguration);

    TalonFXConfiguration algaeTalonConfiguration = new TalonFXConfiguration();
    algaeTalonConfiguration.CurrentLimits.SupplyCurrentLimit = 25;
    algaeTalonConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    algaeTalonConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    algaeTalonConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    algaeMotor.getConfigurator().apply(algaeTalonConfiguration);

    headRotate.setBoundsMicroseconds(2500, 1500, 1500, 1500, 500);
    algaeIntakeArm.setBoundsMicroseconds(2500, 1500, 1500, 1500, 500);

    setAlgaeIntakePostion(AlgaeServoPosition.HOME);
    setHeadPosition(HeadPosition.CENTER);

    if (healthCheckEnabled) {
      algaeMotorCheck = new TalonHealthChecker(algaeMotor, getName());
      coralMotorCheck = new TalonHealthChecker(coralMotor, getName());
    }
  }

  public Command cmdStopCoralMotor() {
    return Commands.runOnce(() -> stopCoralMotor(), this);
  }

  public Command cmdSetCoralPosition(double position) {
    return new FunctionalCommand(
        () -> {
        },
        () -> setCoralPosition(position),
        interrupted -> stopCoralMotor(),
        () -> isCoralNearPosition(position),
        this);
  }

  public Command cmdAddCoralRotations(double rotations) {
    return new FunctionalCommand(
        () -> currentPosition = getCoralMotorPosition(),
        () -> setCoralPosition(currentPosition + rotations),
        interrupted -> stopCoralMotor(),
        () -> isCoralNearPosition(currentPosition + rotations),
        this);
  }

  public Command cmdSetCoralDutyCycleOut(double output) {
    return Commands.runEnd(
        () -> setCoralDutyCycleOut(output),
        () -> stopCoralMotor(),
        this);
  }

  public Command cmdSetHeadRotation(double value) {
    return Commands.runOnce(() -> setHeadPosition(value), this);
  }

  public Command cmdBumpHead(boolean moveRight) {
    if (moveRight) {
      return Commands.runOnce(() -> setHeadPosition(getHeadPosition() + ServoOffset.BUMP.value), this);
    } else {
      return Commands.runOnce(() -> setHeadPosition(getHeadPosition() - ServoOffset.BUMP.value), this);
    }
  }

  public Command cmdJumpHead(boolean moveRight) {
    if (moveRight) {
      return Commands.runOnce(() -> setHeadPosition(getHeadPosition() + ServoOffset.JUMP.value), this);
    } else {
      return Commands.runOnce(() -> setHeadPosition(getHeadPosition() - ServoOffset.JUMP.value), this);
    }
  }

  public Command cmdStopAlgaeMotor() {
    return Commands.runOnce(() -> algaeMotor.stopMotor(), this);
  }

  public Command cmdSetAlgaeDutyCycleOut(double output) {
    return Commands.runEnd(
        () -> setAlgaeDutyCycleOut(output),
        () -> stopAlgaeMotor(),
        this);
  }

  public Command cmdDealgae() {
    return Commands.runEnd(
        () -> {
          setAlgaeDutyCycleOut(-1);
          setAlgaeIntakePostion(AlgaeServoPosition.HOME);
        },
        () -> {
          stopAlgaeMotor();
        }, this);
  }

  public Command cmdSetAlgaeIntakePostion(AlgaeServoPosition value) {
    return Commands.runOnce(() -> {
      setAlgaeIntakePostion(value);
      algaeIntakeState = value;
    }, this);
  }

  public Command cmdBumpAlgaeIntake(boolean moveRight) {
    if (moveRight) {
      return Commands.runOnce(() -> setAlgaeIntakePostion(getAlgaeIntakeArmPosition() + ServoOffset.BUMP.value), this);
    } else {
      return Commands.runOnce(() -> setAlgaeIntakePostion(getAlgaeIntakeArmPosition() - ServoOffset.BUMP.value), this);
    }
  }

  public Command cmdJumpAlgaeIntake(boolean moveRight) {
    if (moveRight) {
      return Commands.runOnce(() -> setAlgaeIntakePostion(getAlgaeIntakeArmPosition() + ServoOffset.JUMP.value), this);
    } else {
      return Commands.runOnce(() -> setAlgaeIntakePostion(getAlgaeIntakeArmPosition() - ServoOffset.JUMP.value), this);
    }
  }

  @Override
  public void periodic() {
    if (healthCheckEnabled) {
      if (!algaeMotorCheck.isDeviceHealthy() |
          !coralMotorCheck.isDeviceHealthy()) {
        healthStatus = HealthStatus.ERROR;
      } else {
        healthStatus = HealthStatus.IS_OK;
      }
    }
  }

  @Logged(name = "Health status")
  public HealthStatus getHealthStatus() {
    return healthStatus;
  }

  @Logged(name = "Has coral")
  public boolean hasCoral() {
    return !coralSensor.get();
  }

  @Logged(name = "Has algae")
  public boolean hasAlgae() {
    return !algaeSensor.get();
  }

  @Logged(name = "Algae intake position")
  public double getAlgaeIntakeArmPosition() {
    return algaeIntakeArm.getPosition();
  }

  @Logged(name = "Head rotate position")
  public double getHeadPosition() {
    return headRotate.getPosition();
  }

  @Logged(name = "Coral motor rotations")
  public double getCoralMotorPosition() {
    return coralMotor.getPosition().getValueAsDouble();
  }

  public boolean isAlgaeIntakeAtPosition(AlgaeServoPosition position) {
    return position == algaeIntakeState;
  }

  private void stopCoralMotor() {
    coralMotor.stopMotor();
  }

  private void setCoralDutyCycleOut(double output) {
    coralMotor.setControl(coralDutyCycleOut.withOutput(output));
  }

  private void setCoralPosition(double position) {
    coralMotor.setControl(coralPositionControl.withPosition(position));
  }

  private boolean isCoralNearPosition(double position) {
    return MathUtil.isNear(position, getCoralMotorPosition(), 1);
  }

  private void setHeadPosition(double position) {
    headRotate.setPosition(position);
  }

  private void stopAlgaeMotor() {
    algaeMotor.stopMotor();
  }

  private void setAlgaeDutyCycleOut(double output) {
    algaeMotor.setControl(algaeDutyCycleOut.withOutput(output));
  }

  private void setAlgaeIntakePostion(double position) {
    algaeIntakeArm.setPosition(position);
  }

  private void setAlgaeIntakePostion(AlgaeServoPosition position) {
    setAlgaeIntakePostion(position.value);
  }

}
