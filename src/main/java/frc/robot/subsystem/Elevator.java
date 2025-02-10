// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public enum Position {
    HOME(0),
    SENSOR(0.5),
    CONTAIN_ALGAE(0),
    L1(0.0),
    L2(0),
    L2_5(0),
    L3(0),
    L3_5(0),
    L4(0),
    PEAK(68);

    public final double rotations;

    Position(double rotations) {
      this.rotations = rotations;
    }
  }

  private TalonFX leftMotor = new TalonFX(30);
  private TalonFX rightMotor = new TalonFX(31);
  private DigitalInput coralSensor = new DigitalInput(0);
  private DigitalInput homeSensor = new DigitalInput(1);
  private MotionMagicVoltage motionMagicPostionControl = new MotionMagicVoltage(0).withEnableFOC(false);
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(false);
  private boolean homeFound = false;
  private boolean previousHomeSensor = isAtHome();

  public Elevator() {
    Slot0Configs positionPIDConfigs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKG(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
        .withKS(0)
        .withKV(0.01)
        .withKP(1.3)
        .withKI(0)
        .withKD(0.013);

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(RotationsPerSecond.of(20))
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(40))
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

    SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
    softwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
    softwareLimitSwitchConfigs.ForwardSoftLimitThreshold = Position.PEAK.rotations;
    softwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
    softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = Position.HOME.rotations;

    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.SupplyCurrentLimit = 25;

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    VoltageConfigs voltageConfigs = new VoltageConfigs();
    voltageConfigs.PeakForwardVoltage = 8;
    voltageConfigs.PeakReverseVoltage = -4;

    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.SoftwareLimitSwitch = softwareLimitSwitchConfigs;
    talonFXConfiguration.Slot0 = positionPIDConfigs;
    talonFXConfiguration.MotionMagic = motionMagicConfigs;
    talonFXConfiguration.CurrentLimits = currentLimitsConfigs;
    talonFXConfiguration.MotorOutput = motorOutputConfigs;
    talonFXConfiguration.Voltage = voltageConfigs;

    leftMotor.getConfigurator().apply(talonFXConfiguration);
    rightMotor.getConfigurator().apply(talonFXConfiguration);
  }

  public Command cmdManualHome() {
      return cmdSetDutyCycleOut(-0.05).until(() -> homeFound);
  }

  public Command cmdSetPosition(Position position) {
    return new FunctionalCommand(
        () -> {
        },
        () -> setPosition(position.rotations),
        interrupted -> {
          if (position == Position.HOME || position == Position.CONTAIN_ALGAE) {
            stopMotors();
          }
        },
        () -> isNearPosition(position.rotations),
        this);
  }

  public Command cmdSetPosition(double position) {
    return new FunctionalCommand(
        () -> {
        },
        () -> setPosition(position),
        interrupted -> stopMotors(),
        () -> isNearPosition(position),
        this);
  }

  public Command cmdSetDutyCycleOut(double output) {
    return Commands.runEnd(
        () -> setDutyCycleOut(output),
        () -> stopMotors(),
        this);
  }

  public Command cmdStop() {
    return Commands.runOnce(() -> stopMotors(), this);
  }

  @Override
  public void periodic() {
    if (!homeFound && previousHomeSensor != isAtHome()) {
      homeFound = true;
      resetMotorPositionToSensor();
    }
    previousHomeSensor = isAtHome();
  }

  @Logged(name = "Home found")
  public boolean getHomeFound() {
    return homeFound;
  }

  @Logged(name = "Has coral")
  public boolean hasCoralInChute() {
    // TODO: Implement in position commands when we have more understanding of the
    // bots scoring workflow
    return !coralSensor.get();
  }

  @Logged(name = "Right rotations")
  public double getRightMotorPosition() {
    return rightMotor.getPosition().getValueAsDouble();
  }

  @Logged(name = "Left rotations")
  public double getLeftMotorPosition() {
    return leftMotor.getPosition().getValueAsDouble();
  }

  public boolean isNearCoralScoringPosition() {
    return (isNearPosition(Elevator.Position.L1)
        || isNearPosition(Elevator.Position.L2)
        || isNearPosition(Elevator.Position.L3)
        || isNearPosition(Elevator.Position.L4));
  }

  public boolean isNearReefAlgaePosition() {
    return (isNearPosition(Elevator.Position.L2_5)
        || isNearPosition(Elevator.Position.L3_5));
  }

  public boolean isNearAlgaeContainmentPosition() {
    return isNearPosition(Elevator.Position.CONTAIN_ALGAE);
  }

  @Logged(name = "Home")
  public boolean isAtHome() {
    return !homeSensor.get();
  }

  private void setPosition(double position) {
    motionMagicPostionControl.Position = position;
    leftMotor.setControl(motionMagicPostionControl);
    rightMotor.setControl(motionMagicPostionControl);
  }

  private void setDutyCycleOut(double output) {
    dutyCycleOut.Output = output;
    leftMotor.setControl(dutyCycleOut);
    rightMotor.setControl(dutyCycleOut);
  }

  private void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  private void resetMotorPositionToSensor() {
    leftMotor.setPosition(Position.SENSOR.rotations);
    rightMotor.setPosition(Position.SENSOR.rotations);
  }

  private boolean isNearPosition(double position) {
    // TODO: Tune tolerance value to something representative of real life
    return MathUtil.isNear(position, getLeftMotorPosition(), 10)
        || MathUtil.isNear(position, getRightMotorPosition(), 10);
  }

  private boolean isNearPosition(Position position) {
    return isNearPosition(position.rotations);
  }
}
