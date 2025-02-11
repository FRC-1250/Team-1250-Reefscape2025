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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public enum Position {
    STARTING_CONFIGURATION(0),
    SENSOR(0.5),
    CONTAIN_ALGAE(0),
    CORAL_STATION(0),
    L1(5),
    L2(10),
    L2_5(12),
    L3(15),
    L3_5(18),
    L4(20),
    PEAK(25);

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
  private final Timer timer = new Timer();
  private final double timerDuration = 2;
  private final boolean tuningMode = true;

  public Elevator() {

    if (tuningMode) {
      timer.start();
      SmartDashboard.putNumber("G", 0);
      SmartDashboard.putNumber("S", 0);
      SmartDashboard.putNumber("V", 0);
      SmartDashboard.putNumber("P", 0);
      SmartDashboard.putNumber("D", 0);
    }

    Slot0Configs positionPIDConfigs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKG(0.3)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
        .withKS(0.2)
        .withKV(0.25)
        .withKP(10)
        .withKI(0)
        .withKD(0);

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(RotationsPerSecond.of(75))
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(300))
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0));

    SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
    softwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
    softwareLimitSwitchConfigs.ForwardSoftLimitThreshold = Position.PEAK.rotations;
    softwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
    softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = Position.STARTING_CONFIGURATION.rotations;

    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.SupplyCurrentLimit = 25;

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    VoltageConfigs voltageConfigs = new VoltageConfigs();
    voltageConfigs.PeakForwardVoltage = 12;
    voltageConfigs.PeakReverseVoltage = -6;

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
      resetMotorPositionToPosition(Position.SENSOR.rotations);
    }
    previousHomeSensor = isAtHome();

    if (timer.hasElapsed(timerDuration)) {
      if (tuningMode) {
        Slot0Configs positionPIDConfigs = new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(SmartDashboard.getNumber("G", 0))
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withKS(SmartDashboard.getNumber("S", 0))
            .withKV(SmartDashboard.getNumber("V", 0))
            .withKP(SmartDashboard.getNumber("P", 0))
            .withKI(0)
            .withKD(SmartDashboard.getNumber("D", 0));

        leftMotor.getConfigurator().apply(positionPIDConfigs);
        rightMotor.getConfigurator().apply(positionPIDConfigs);
      }
      timer.reset();
    }
  }

  @Logged(name = "Home found")
  public boolean getHomeFound() {
    return homeFound;
  }

  @Logged(name = "Has coral")
  public boolean hasCoralInChute() {
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

  public boolean isAbovePosition(Position position) {
    return position.rotations < getLeftMotorPosition() || position.rotations < getRightMotorPosition();
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

  private void resetMotorPositionToPosition(double rotations) {
    leftMotor.setPosition(rotations);
    rightMotor.setPosition(rotations);
  }

  private boolean isNearPosition(double position) {
    return MathUtil.isNear(position, getLeftMotorPosition(), 0.5)
        || MathUtil.isNear(position, getRightMotorPosition(), 0.5);
  }

  private boolean isNearPosition(Position position) {
    return isNearPosition(position.rotations);
  }
}
