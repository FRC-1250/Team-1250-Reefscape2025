// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public enum Position {
    HOME(0),
    L1(0.0),
    L2(0),
    L3(0),
    L4(0),
    PEAK(0);

    public final double rotations;

    Position(double rotations) {
      this.rotations = rotations;
    }
  }

  private TalonFX leftMotor = new TalonFX(10);
  private TalonFX rightMotor = new TalonFX(11);
  private DigitalInput coralSensor = new DigitalInput(0);
  private DigitalInput homeSensor = new DigitalInput(1);
  private MotionMagicVoltage motionMagicPostionControl = new MotionMagicVoltage(0).withEnableFOC(false);
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public Elevator() {
    Slot0Configs positionPIDConfigs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKG(0)
        .withKS(0)
        .withKV(0.01)
        .withKP(1.3)
        .withKI(0)
        .withKD(0.013);

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(80)
        .withMotionMagicAcceleration(160)
        .withMotionMagicJerk(1600);

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
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

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

  public Command cmdSetPosition(double position) {
    return Commands.run(() -> {
      motionMagicPostionControl.Position = position;
      leftMotor.setControl(motionMagicPostionControl);
      rightMotor.setControl(motionMagicPostionControl);
    }, this)
        .until(() -> isNearPosition(position))
        .andThen(cmdStop());
  }

  public Command cmdSetDutyCycleOut(double output) {
    return Commands.run(() -> {
      dutyCycleOut.Output = output;
      leftMotor.setControl(dutyCycleOut);
      rightMotor.setControl(dutyCycleOut);
    }, this);
  }

  public Command cmdStop() {
    return Commands.run(() -> {
      leftMotor.stopMotor();
      rightMotor.stopMotor();
    }, this);
  }

  @Override
  public void periodic() {

  }

  private boolean getCoralSensor() {
    return !coralSensor.get();
  }

  private void resetMotorPositionToHome() {
    leftMotor.setPosition(0);
    rightMotor.setPosition(0);
  }

  private double getRightMotorPosition() {
    return rightMotor.getPosition().getValueAsDouble();
  }

  private double getLeftMotorPosition() {
    return leftMotor.getPosition().getValueAsDouble();
  }

  private boolean isNearPosition(double position) {
    // TODO: Tune tolerance value to something representative of real life
    return MathUtil.isNear(position, getLeftMotorPosition(), 10)
        || MathUtil.isNear(position, getRightMotorPosition(), 10);
  }

  private boolean isAtHome() {
    return !homeSensor.get();
  }
}
