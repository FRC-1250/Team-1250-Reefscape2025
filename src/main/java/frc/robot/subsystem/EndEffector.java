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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {

  public enum AlgaeServoPosition {
    DEPLOYED(1), 
    HOME(0);

    public final double value;

    AlgaeServoPosition(double value) {
      this.value = value;
    }
  }

  private TalonFX algaeMotor = new TalonFX(21);
  private DutyCycleOut algaeDutyCycleOut = new DutyCycleOut(0);
  private DigitalInput algaeSensor = new DigitalInput(4);
  private Servo algaeIntakePosition = new Servo(1);

  private TalonFX coralMotor = new TalonFX(20);
  private DutyCycleOut coralDutyCycleOut = new DutyCycleOut(0);
  private PositionVoltage coralPositionControl = new PositionVoltage(0);
  private DigitalInput coralSensor = new DigitalInput(3);
  private Servo headRotate = new Servo(0);

  /** Creates a new EndEffector. */
  public EndEffector() {
    Slot0Configs positionPIDConfigs = new Slot0Configs()
        .withKG(0)
        .withKS(0)
        .withKP(0)
        .withKI(0)
        .withKD(0)
        .withKV(0);

    TalonFXConfiguration coralTalonConfiguration = new TalonFXConfiguration();
    coralTalonConfiguration.Slot0 = positionPIDConfigs;
    coralTalonConfiguration.CurrentLimits.SupplyCurrentLimit = 25;
    coralTalonConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    coralTalonConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    coralTalonConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    coralMotor.getConfigurator().apply(coralTalonConfiguration);

    TalonFXConfiguration algaeTalonConfiguration = new TalonFXConfiguration();
    algaeTalonConfiguration.CurrentLimits.SupplyCurrentLimit = 25;
    algaeTalonConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    algaeTalonConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    algaeTalonConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    algaeMotor.getConfigurator().apply(algaeTalonConfiguration);

    headRotate.setBoundsMicroseconds(2500, 1500, 1500, 1500, 500);
    algaeIntakePosition.setBoundsMicroseconds(2500, 1500, 1500, 1500, 500);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command cmdStopCoralMotor() {
    return Commands.run(() -> {
      coralMotor.stopMotor();
    }, this);
  }

  public Command cmdSetCoralPosition(double position) {
    return Commands.run(() -> {
      coralPositionControl.Position = position;
      coralMotor.setControl(coralPositionControl);
    }, this);
  }

  public Command cmdSetCoralDutyCycleOut(double output) {
    return Commands.run(() -> {
      coralDutyCycleOut.Output = output;
      coralMotor.setControl(coralDutyCycleOut);
    }, this);
  }

  public Command cmdStopAlgaeMotor() {
    return Commands.run(() -> {
      algaeMotor.stopMotor();
    }, this);
  }

  public Command cmdSetAlgaeDutyCycleOut(double output) {
    return Commands.run(() -> {
      algaeDutyCycleOut.Output = output;
      algaeMotor.setControl(algaeDutyCycleOut);
    }, this);
  }

  public Command cmdSetAlgaeIntakePostion(double value) {
    return Commands.runOnce(() -> {
      algaeIntakePosition.setPosition(value);
    }, this);
  }

  public Command cmdSetHeadRotation(double value) {
    return Commands.runOnce(() -> {
      headRotate.setPosition(value);
    }, this);
  }

  private void stopCoralMotor() {

  }

  private void setCoralPosition() {

  }

  private double getCoralPosition() {
    return 0;
  }

  private void setCoralDutyCycleOut() {
    // Using DutyCycleOut
  }

  private boolean getCoralSensor() {
    return true;
  }

  private void stopAlgaeMotor() {

  }

  private void setAlgaeDutyCycleOut() {
    // Using DutyCycleOut
  }

  private boolean getAlgaeSensor() {
    return true;
  }

  private void setAlgaeIntakePostion() {

  }

}
