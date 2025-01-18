// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public enum Position {
    L1(0.0);

    public final double rotations;

    Position(double rotations) {
      this.rotations = rotations;
    }
  }

  private TalonFX leader = new TalonFX(10);
  private TalonFX follower = new TalonFX(11);
  private DigitalInput coralSensor = new DigitalInput(0);
  private DigitalInput homeSensor = new DigitalInput(1);
  private DigitalInput peakSensor = new DigitalInput(2);
  private MotionMagicVoltage motionMagicPostionControl = new MotionMagicVoltage(0).withEnableFOC(false);
  private TorqueCurrentFOC torqueCurrentControl = new TorqueCurrentFOC(0);
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public Elevator() {
    Slot0Configs positionPIDConfigs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKG(0)
        .withKS(0)
        .withKV(0)
        .withKP(0)
        .withKI(0)
        .withKD(0);

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(0)
        .withMotionMagicAcceleration(0)
        .withMotionMagicJerk(0);

    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.Slot0 = positionPIDConfigs;
    talonFXConfiguration.MotionMagic = motionMagicConfigs;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 25;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonFXConfiguration.Voltage.PeakForwardVoltage = 8;
    talonFXConfiguration.Voltage.PeakReverseVoltage = -4;

    leader.getConfigurator().apply(talonFXConfiguration);
    follower.getConfigurator().apply(talonFXConfiguration);
  }

  public Command cmdSetTorque() {
    return Commands.none();
  }

  public Command cmdSetPosition() {
    return Commands.none();
  }

  public Command cmdSetDutyCycleOut() {
    return Commands.none();
  }

  public Command cmdStop() {
    return Commands.none();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Digital Input", getDigitalInput());
  }

  private boolean getDigitalInput() {
    return !coralSensor.get();
  }

  private void stop() {

  }

  private void setPosition() {
    // Using MotionMagic control
  }

  private double getPosition() {
    return 0;
  }

  private void setTorqueCurrent() {
    // Using Current control
  }

  private double getTorqueCurrent() {
    return 0;
  }

  private void setDutyCycleOut() {
    // Using DutyCycleOut
  }

  private double getDutyCycleOut() {
    return 0;
  }

  private boolean isNearForwardLimit() {
    return false;
  }

  private boolean isNearReverseLimit() {
    return false;
  }

  private boolean isAtHome() {
    return false;
  }
}
